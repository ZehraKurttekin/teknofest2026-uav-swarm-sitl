#!/usr/bin/env python3
"""
distributed_follower.py — Dağıtık Takipçi Node

Dağıtık mimari:
  Lider drone `/swarm/leader_state` topic'ine kendi konumunu,
  formasyon tipini ve yaw'ı yayınlar.

  Her takipçi bu topic'i dinler ve:
    1. Lider konumunu alır
    2. Kendi indeksine göre offset'i KENDISI hesaplar
    3. Hedefe KENDISI gider

  swarm_mission.py artık takipçilere "şuraya git" demez.
  Sadece formasyon tipini ve mesafeyi günceller.
  Takipçiler reaktif olarak lideri takip eder.

Bu yapı dağıtık sayılır çünkü:
  - Her drone bağımsız karar veriyor
  - Merkezi koordinatör (swarm_mission) devre dışı kalsa bile
    takipçiler lideri takip etmeye devam eder
  - Sadece lider pozisyonu + formasyon tipi paylaşılıyor
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition
import json
import math
import threading
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))
from formation import formation_positions, compute_yaw_to_target, MIN_SAFE_DIST
from apf_repulsion import apply_repulsion
from px4_msgs.msg import VehicleLocalPosition


# Takipçinin formasyon içindeki sabit indeksi
# Lider = index 0, Takipçi_2 = index 1, Takipçi_3 = index 2
FOLLOWER_INDEX = {
    2: 1,   # drone_id=2 → formasyon index 1
    3: 2,   # drone_id=3 → formasyon index 2
}

CRUISE_SPEED = 2.0


class DistributedFollower(Node):
    """
    Dağıtık takipçi.
    Lider konumunu topic'ten alır, kendi offset'ini hesaplar, hareket eder.
    swarm_mission'dan komut beklemez.
    """

    def __init__(self, drone_id: int, drone_ctrl):
        super().__init__(f'distributed_follower_{drone_id}')
        self.drone_id = drone_id
        self.ctrl     = drone_ctrl
        self.index    = FOLLOWER_INDEX.get(drone_id, 1)

        # Mevcut formasyon durumu
        self._formation_type = 'OKBASI'
        self._formation_dist = 8.0
        self._active_drones  = [1, 2, 3]
        self._enabled        = True   # False ise (ayrılma sırasında) takip etme

        self._lock = threading.Lock()

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # Lider state topic'ini dinle
        self.create_subscription(
            String, '/swarm/leader_state',
            self._leader_state_cb, qos_reliable)

        # Formasyon parametre güncellemelerini dinle (swarm_mission'dan)
        self.create_subscription(
            String, '/swarm/formation_params',
            self._params_cb, qos_reliable)

        # Tüm dronların anlık pozisyonlarını dinle (APF için)
        self._drone_positions = {}   # {drone_id: (x, y, z)}
        self._pos_lock        = threading.Lock()

        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        for did in [1, 2, 3]:
            if did == self.drone_id:
                continue   # Kendi pozisyonunu drone_ctrl'den alıyoruz
            # İki versiyonu da dinle — PX4 build'ine göre biri aktif olacak
            self.create_subscription(
                VehicleLocalPosition,
                f'/px4_{did}/fmu/out/vehicle_local_position_v1',
                lambda msg, d=did: self._other_pos_cb(msg, d),
                qos_px4)
            self.create_subscription(
                VehicleLocalPosition,
                f'/px4_{did}/fmu/out/vehicle_local_position',
                lambda msg, d=did: self._other_pos_cb(msg, d),
                qos_px4)

        self.get_logger().info(
            f'DistributedFollower {drone_id}: '
            f'index={self.index}, APF aktif, diger dronlar dinleniyor')

    def _other_pos_cb(self, msg, drone_id):
        with self._pos_lock:
            self._drone_positions[drone_id] = (msg.x, msg.y, -msg.z)

    def _get_other_positions(self):
        with self._pos_lock:
            return [p for p in self._drone_positions.values()]

    def _leader_state_cb(self, msg):
        """
        Lider pozisyon + formasyon bilgisi geldi.
        Kendi hedef pozisyonumu hesapla ve git.
        """
        with self._lock:
            if not self._enabled:
                return
            if self.drone_id not in self._active_drones:
                return

        try:
            state = json.loads(msg.data)
        except Exception:
            return

        lx     = state.get('x', 0.0)
        ly     = state.get('y', 0.0)
        lz     = state.get('z', 8.0)
        yaw    = state.get('yaw', 0.0)
        tip    = state.get('formation_type', self._formation_type)
        mesafe = state.get('distance', self._formation_dist)
        active = state.get('active_drones', self._active_drones)

        if self.drone_id not in active:
            return

        # Aktif drone'lar içindeki indeksimi bul
        try:
            my_idx = active.index(self.drone_id)
        except ValueError:
            return

        # Formasyon pozisyonunu KENDIM hesapla
        positions = formation_positions(lx, ly, lz, tip, len(active), mesafe, yaw)

        if my_idx < len(positions):
            x, y, z = positions[my_idx]

            # APF itme kuvveti ekle
            my_pos = self.ctrl.get_position()  # (x, y, alt)
            others = self._get_other_positions()
            new_x, new_y, new_z = apply_repulsion((x, y, z), my_pos, others)

            self.ctrl.goto_local(new_x, new_y, new_z,
                                 yaw=yaw, max_speed=CRUISE_SPEED)

    def _params_cb(self, msg):
        """Formasyon parametrelerini güncelle."""
        try:
            p = json.loads(msg.data)
            with self._lock:
                self._formation_type = p.get('formation_type', self._formation_type)
                self._formation_dist = p.get('distance', self._formation_dist)
                self._active_drones  = p.get('active_drones', self._active_drones)
        except Exception:
            pass

    def enable(self):
        """Takip etmeye başla."""
        with self._lock:
            self._enabled = True

    def disable(self):
        """Takibi durdur (ayrılma sırasında)."""
        with self._lock:
            self._enabled = False


class LeaderStatePublisher(Node):
    """
    Lider drone'un konumunu ve formasyon parametrelerini yayınlar.
    swarm_mission tarafından kullanılır.
    10 Hz yayın.
    """

    def __init__(self, get_leader_pos_fn):
        super().__init__('leader_state_publisher')
        self.get_pos = get_leader_pos_fn

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.state_pub  = self.create_publisher(String, '/swarm/leader_state',  qos)
        self.params_pub = self.create_publisher(String, '/swarm/formation_params', qos)

        # Mevcut durum
        self._yaw            = 0.0
        self._formation_type = 'OKBASI'
        self._formation_dist = 8.0
        self._active_drones  = [1, 2, 3]
        self._lock           = threading.Lock()

        # 10 Hz yayın timer'ı
        self.create_timer(0.1, self._publish_state)

    def _publish_state(self):
        """10 Hz — lider konumunu yayınla."""
        x, y, z = self.get_pos()
        with self._lock:
            state = {
                'x': x, 'y': y, 'z': z,
                'yaw':            self._yaw,
                'formation_type': self._formation_type,
                'distance':       self._formation_dist,
                'active_drones':  self._active_drones,
            }
        self.state_pub.publish(String(data=json.dumps(state)))

    def update(self, yaw: float, formation_type: str,
               formation_dist: float, active_drones: list):
        """Formasyon parametrelerini güncelle."""
        with self._lock:
            self._yaw            = yaw
            self._formation_type = formation_type
            self._formation_dist = formation_dist
            self._active_drones  = active_drones

        # Params topic'ine de yayınla
        self.params_pub.publish(String(data=json.dumps({
            'formation_type': formation_type,
            'distance':       formation_dist,
            'active_drones':  active_drones,
        })))
