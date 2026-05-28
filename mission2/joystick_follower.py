#!/usr/bin/env python3
"""
joystick_follower.py — SANCAK Görev 2 · Dağıtık Takipçi

Her takipçi drone (2 veya 3) için ayrı bir instance çalıştırılır:
  python3 joystick_follower.py 2
  python3 joystick_follower.py 3

Görev:
  1. Kendi PX4 kontrolcüsü ile bağlantı kurar
  2. /swarm/leader_state topic'ini dinler
  3. Lider pozisyonuna + formasyon parametrelerine göre KENDİ hedefini hesaplar
  4. Manevra (pitch/roll) eğimini kendi Z offset'ine kendisi dönüştürür
  5. goto_local ile hedefe uçar

Bu yapı dağıtıktır:
  - Merkezi bir kontrol node'u takipçilere "şuraya git" demiyor
  - Her takipçi lider bilgisini okuyup kendi kararını veriyor
  - Lider düşerse takipçiler durur (çarpışma olmaz)
  - Her drone kendi PX4'ünü ayrı ayrı sürüyor
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

import threading
import time
import math
import json
import sys
import os
import signal

sys.path.insert(0, os.path.dirname(__file__))
_mission_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'mission')
sys.path.insert(0, _mission_dir)
from drone_controller2 import DroneController
from formation2 import get_formation_offsets

DRONE_COUNT = 3
TAKEOFF_ALT = 8.0


class JoystickFollower(Node):
    """
    Tek bir takipçi drone için.
    /swarm/leader_state dinler, kendi hedefini hesaplar.
    """

    def __init__(self, drone_id: int):
        super().__init__(f'joystick_follower_{drone_id}')
        self.drone_id = drone_id

        # Bu drone'un PX4 kontrolü
        self.ctrl = DroneController(drone_id)

        # Executor
        self.mt_executor = MultiThreadedExecutor()
        self.mt_executor.add_node(self.ctrl)
        self.mt_executor.add_node(self)
        threading.Thread(target=self.mt_executor.spin, daemon=True).start()

        # Leader state dinle
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(
            String, '/swarm/leader_state',
            self._on_leader_state, qos_be)

        # Durum
        self.last_state = None
        self.has_taken_off = False
        self.initial_pos = None   # Kalkış başlangıç pozisyonu

        self.shutting_down = False
        signal.signal(signal.SIGINT,  self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)

        # Drone index'i — sorted key order: 1=lider, 2 ve 3 takipçi
        # formation_positions fonksiyonu 3 drone için offset döndürür:
        #   index 0 = lider (0, 0)
        #   index 1, 2 = takipçiler
        # Bizim drone_id 2 ise index 1, drone_id 3 ise index 2
        self.my_index = drone_id - 1

        print("=" * 60)
        print(f"  TAKİPÇİ {drone_id} (bağımsız node)")
        print(f"  /swarm/leader_state dinleniyor")
        print(f"  Index (formation): {self.my_index}")
        print("=" * 60)

    def _sig_handler(self, sig, frame):
        if self.shutting_down:
            return
        self.shutting_down = True
        print(f"\n[FOLLOWER {self.drone_id}] Ctrl+C")
        try:
            self.ctrl.land()
        except Exception:
            pass
        time.sleep(2)
        sys.exit(0)

    # ─── Leader state callback ──────────────────────
    def _on_leader_state(self, msg):
        try:
            state = json.loads(msg.data)
        except Exception:
            return

        self.last_state = state

        armed = bool(state.get('armed', False))

        # Lider armed değilse bir şey yapma
        if not armed:
            self.has_taken_off = False
            return

        # Kalkış: lider armed olduğunda takipçi de kalkmalı
        if armed and not self.has_taken_off:
            self._do_takeoff_self()
            return

        # Formasyon aktif mi?
        formation_active = bool(state.get('active', False))

        if not formation_active:
            # Formasyon henüz kurulmadı — mevcut konumu koru
            # (Drone bulunduğu yerde hover eder)
            x, y, _ = self.ctrl.get_position()
            alt = float(state.get('leader_alt', TAKEOFF_ALT))
            self.ctrl.goto_local(x, y, alt)
            return

        # ─── Formasyon aktif — kendi hedefini hesapla ───
        leader_x  = float(state.get('leader_x', 0.0))
        leader_y  = float(state.get('leader_y', 0.0))
        leader_z  = float(state.get('leader_alt', TAKEOFF_ALT))
        yaw       = float(state.get('yaw', 0.0))
        formation = state.get('formation', 'V')
        dist      = float(state.get('dist', 7.0))
        pitch_eg  = float(state.get('pitch_egim', 0.0))
        roll_eg   = float(state.get('roll_egim', 0.0))

        # Formasyon offset'lerini hesapla (formation2.py)
        offsets = get_formation_offsets(formation, DRONE_COUNT, dist, yaw)

        if self.my_index >= len(offsets):
            return

        dx, dy, dz = offsets[self.my_index]

        # Lider merkezli hedef
        target_x = leader_x + dx
        target_y = leader_y + dy
        target_z = leader_z + dz

        # ─── MANEVRA: Z eğimini KENDİ hesapla ────────
        # Body frame'e çevir (yaw ters rotasyonu)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        body_x =  dx * cos_y + dy * sin_y   # ileri/geri yönünde
        body_y = -dx * sin_y + dy * cos_y   # sağ/sol yönünde

        # Pitch eğimi: ileri offset × pitch_egim / dist
        # Roll eğimi:  sağ offset  × roll_egim  / dist
        # (Dist = 0 koruma)
        d_safe = max(dist, 0.001)
        z_offset_pitch = pitch_eg * (body_x / d_safe)
        z_offset_roll  = roll_eg  * (body_y / d_safe)

        target_z = target_z + z_offset_pitch + z_offset_roll

        # Hedefe git
        self.ctrl.goto_local(target_x, target_y, max(2.0, target_z), yaw)

    # ─── Takipçi kendi kalkışını başlatır ───────────
    def _do_takeoff_self(self):
        """
        Lider armed olduğunu gördüğünde takipçi de kalkar.
        Bulunduğu (x, y) konumundan dikey TAKEOFF_ALT'a çıkar.
        """
        self.has_taken_off = True
        print(f"[FOLLOWER {self.drone_id}] Lider armed — takipçi de kalkıyor")

        # Anlık pozisyonu oku
        x, y, _ = self.ctrl.get_position()
        self.ctrl.target_x = x
        self.ctrl.target_y = y
        self.ctrl.takeoff(TAKEOFF_ALT)


def main():
    if len(sys.argv) < 2:
        print("Kullanım: python3 joystick_follower.py <drone_id>")
        print("  drone_id: 2 veya 3")
        sys.exit(1)

    try:
        drone_id = int(sys.argv[1])
    except ValueError:
        print(f"Geçersiz drone_id: {sys.argv[1]}")
        sys.exit(1)

    if drone_id not in (2, 3):
        print(f"drone_id 2 veya 3 olmalı, verilen: {drone_id}")
        sys.exit(1)

    rclpy.init()
    node = JoystickFollower(drone_id)
    try:
        while rclpy.ok() and not node.shutting_down:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
