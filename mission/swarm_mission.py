#!/usr/bin/env python3
"""
swarm_mission.py — SANCAK Sürü İHA — Dağıtık Mimari

Dağıtık yapı:
  Lider → /swarm/leader_state → Takipçiler kendi offset'ini kendisi hesaplar

  swarm_mission artık takipçilere "şuraya git" demez.
  Sadece formasyon tipini/mesafesini günceller.
  Takipçiler reaktif olarak lideri takip eder.

  Bu yapı dağıtık sayılır:
  - Her drone bağımsız karar veriyor
  - Sadece lider pozisyonu paylaşılıyor (bilgi minimumu)
  - Merkezi koordinatör sadece görev mantığı için
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
from typing import List

sys.path.insert(0, os.path.dirname(__file__))
from drone_controller import DroneController
from qr_reader import QRReader
from formation import (formation_positions, compute_yaw_to_target,
                       check_min_separation, interpolate_formation, MIN_SAFE_DIST)
from pad_scout import PadScout, PadCoordinator
from distributed_follower import DistributedFollower, LeaderStatePublisher
from apf_repulsion import apply_repulsion
from set_px4_params import set_params_for_drone, PARAMS as PX4_TUNING_PARAMS

DRONE_COUNT    = 3
TAKEOFF_ALT    = 8.0
QR_SCAN_ALT    = 2.5
QR_SCAN_TIMEOUT = 20.0
PAD_ALT        = 5.0
FORMATION_DIST = 7.0     # 7 metre — her komşu arası, sabit ve korunur

CRUISE_SPEED    = 4.5
APPROACH_SPEED  = 2.5
LAND_SPEED      = 0.8
FORMATION_SPEED = 3.5

N_INTERP  = 6
INTERP_DT = 0.4

# ═══ EKSEN DÖNÜŞÜMÜ ═══════════════════════════════
# Gazebo ENU (X=doğu, Y=kuzey) ↔ PX4 NED (X=kuzey, Y=doğu)
# Gazebo X ↔ NED Y,  Gazebo Y ↔ NED X
# Ayrıca spawn offseti nedeniyle NED X'te 3m kayma gözlemleniyor.
NED_X_OFFSET = -3.0   # Gözlemsel ofset — testlerle ayarlanmıştır

def gazebo_to_ned(gazebo_x, gazebo_y):
    """Gazebo (x, y) → PX4 NED (x, y) dönüşümü."""
    ned_x = gazebo_y + NED_X_OFFSET
    ned_y = gazebo_x
    return (ned_x, ned_y)

# World dosyasındaki Gazebo koordinatları (okunması kolay, dünya ile eşleşiyor)
QR_POSITIONS_GAZEBO = {
    1: (25.00,   9.00),    # qr_plaka_1
    2: (25.00,  17.66),    # qr_plaka_2
    3: (15.00,  0.66),    # qr_plaka_3
    4: (10.00,   9.00),    # qr_plaka_4
    5: (15.00,  17.66),    # qr_plaka_5
    6: (25.00,  0.66),    # qr_plaka_6
}

# NED'e çevrilmiş — drone hedefleri bu
QR_POSITIONS = {qid: gazebo_to_ned(gx, gy)
                for qid, (gx, gy) in QR_POSITIONS_GAZEBO.items()}

HOME_POS     = (0.0, 0.0)
SPAWN_NED    = {1: (0.0, 0.0), 2: (0.0, -4.0), 3: (0.0, 4.0)}
SPAWN_CENTER = (0.0, 0.0)

UAV_NS  = {1: 'uav1', 2: 'uav2', 3: 'uav3'}
TEAM_ID = 'team_1'


class SwarmMission:

    def __init__(self):
        rclpy.init()

        # Drone kontrolcüleri
        self.drones = {i: DroneController(i) for i in range(1, DRONE_COUNT+1)}

        # QR okuma (sadece lider)
        self.qr_data  = None
        self.qr_event = threading.Event()
        self.qr_reader = QRReader(UAV_NS[1], self._on_qr_read)

        # Pad tespiti — tüm dronlar aynı anda tarar
        self.pad_coordinator = PadCoordinator()
        self._sep_renk = {}
        self.pad_scouts = {}
        for i in range(1, DRONE_COUNT+1):
            self.pad_scouts[i] = PadScout(
                drone_id=i, uav_namespace=UAV_NS[i],
                get_drone_pos_fn=self.drones[i].get_position,
                get_drone_heading_fn=self.drones[i].get_heading,
                altitude_fn=lambda: self.current_altitude)

        # Dağıtık mimari
        # Lider state yayıncısı
        self.leader_pub = LeaderStatePublisher(self.drones[1].get_position)

        # Takipçi node'lar — kendi kararlarını kendileri verirler
        self.followers = {}
        for i in range(2, DRONE_COUNT+1):
            self.followers[i] = DistributedFollower(i, self.drones[i])

        # Executor
        self.executor = MultiThreadedExecutor()
        for d in self.drones.values():
            self.executor.add_node(d)
        for n in [self.qr_reader, self.pad_coordinator, self.leader_pub]:
            self.executor.add_node(n)
        for f in self.followers.values():
            self.executor.add_node(f)
        for s in self.pad_scouts.values():
            self.executor.add_node(s)

        threading.Thread(target=self.executor.spin, daemon=True).start()

        self.separated        = {}
        self.current_altitude = TAKEOFF_ALT
        self.current_formation = 'OKBASI'
        self.current_formation_dist = FORMATION_DIST
        self.current_yaw      = 0.0
        self._last_positions  = {}
        self._shutting_down   = False

        signal.signal(signal.SIGINT,  self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        print("=" * 55)
        print("  SANCAK — Dağıtık Sürü İHA Sistemi")
        print("  Lider → /swarm/leader_state → Takipçiler")
        print("  Ctrl+C → Mevcut konumda güvenli iniş")
        print("=" * 55)

    def _signal_handler(self, sig, frame):
        if self._shutting_down:
            return
        self._shutting_down = True
        print("\n[SANCAK] Ctrl+C — mevcut konumda iniş...")
        for drone in self.drones.values():
            try:
                drone.emergency_land()
            except Exception:
                pass
        time.sleep(2.0)
        self.shutdown()
        sys.exit(0)

    def _on_qr_read(self, data):
        self.qr_data = data
        # Çözümlenen JSON'u detaylı log'a yaz
        self.log("┌─── QR ÇÖZÜMLENDİ ────────────────")
        for line in json.dumps(data, indent=2, ensure_ascii=False).split('\n'):
            self.log(f"│ {line}")
        self.log("└──────────────────────────────────")
        self.qr_event.set()

    def log(self, msg):
        print(f"[SANCAK] {msg}")

    def spin(self, s):
        time.sleep(s)

    def _leader_goto_with_apf(self, x, y, z, yaw, max_speed):
        """Lider için APF itme kuvveti uygulanmış goto_local."""
        my_pos = self.drones[1].get_position()
        others = []
        for i in [2, 3]:
            if not self.separated.get(i, False):
                others.append(self.drones[i].get_position())
        new_x, new_y, new_z = apply_repulsion((x, y, z), my_pos, others)
        self.drones[1].goto_local(new_x, new_y, new_z, yaw=yaw, max_speed=max_speed)

    def _update_leader_state(self, yaw=None, tip=None, mesafe=None, active=None):
        """Lider state'i güncelle — takipçiler otomatik tepki verir."""
        self.leader_pub.update(
            yaw=yaw if yaw is not None else self.current_yaw,
            formation_type=tip if tip is not None else self.current_formation,
            formation_dist=mesafe if mesafe is not None else self.current_formation_dist,
            active_drones=active if active is not None else self._active_drones()
        )

    def _active_drones(self):
        return [i for i in range(1, DRONE_COUNT+1)
                if not self.separated.get(i, False)]

    # ─── FORMASYON ───────────────────────────────────────
    def send_formation(self, leader_x, leader_y, altitude,
                       tip, mesafe, active_drones=None,
                       target_x=None, target_y=None,
                       interpolate=True):
        """
        Dağıtık formasyon geçişi.
        1. Lider hedef pozisyona gider
        2. leader_state güncellenir → takipçiler reaktif olarak takip eder
        3. Interpolasyon: lider kademeli hareket eder,
           takipçiler her adımda lider pozisyonunu okuyup tepki verir
        """
        if active_drones is None:
            active_drones = self._active_drones()

        cur_x, cur_y, _ = self.drones[1].get_position()
        if target_x is not None and target_y is not None:
            yaw = compute_yaw_to_target(cur_x, cur_y, target_x, target_y)
        else:
            yaw = self.current_yaw
        self.current_yaw = yaw

        # Collision check
        adj = mesafe
        for _ in range(5):
            pos = formation_positions(leader_x, leader_y, altitude,
                                      tip, len(active_drones), adj, yaw)
            ok, min_d = check_min_separation(pos, MIN_SAFE_DIST)
            if ok:
                break
            adj *= (MIN_SAFE_DIST / min_d) * 1.2

        target_pos = formation_positions(leader_x, leader_y, altitude,
                                         tip, len(active_drones), adj, yaw)

        self.log(f"Formasyon: {tip} | {adj:.1f}m | "
                 f"({leader_x:.1f},{leader_y:.1f},{altitude:.1f}m) | "
                 f"Yaw:{math.degrees(yaw):.1f}°")

        # Formasyon parametrelerini güncelle
        self.current_formation      = tip
        self.current_formation_dist = adj

        # Lider state güncelle → takipçiler otomatik tepki verir
        self._update_leader_state(
            yaw=yaw, tip=tip, mesafe=adj, active=active_drones)

        if interpolate and self._last_positions:
            # Mevcut pozisyondan interpolasyon — lider kademeli hareket
            last = [self._last_positions.get(d, self.drones[d].get_position())
                    for d in active_drones]

            for step in range(1, N_INTERP + 1):
                t = step / N_INTERP
                interp = interpolate_formation(last, target_pos, t)

                # Lider hareket et (APF'li)
                if interp:
                    x, y, z = interp[0]
                    self._leader_goto_with_apf(
                        x, y, z, yaw=yaw, max_speed=FORMATION_SPEED)

                # leader_state güncelle → takipçiler reaktif olarak takip eder
                # (Lider topic'i 10Hz yayınlıyor, takipçiler anlık tepki verir)
                time.sleep(INTERP_DT)
        else:
            if target_pos:
                x, y, z = target_pos[0]
                self._leader_goto_with_apf(
                    x, y, z, yaw=yaw, max_speed=FORMATION_SPEED)

        self.drones[1].wait_until_reached(threshold=1.5, timeout=30)
        self.drones[1].wait_until_stable(max_wait=2.0)

        # Son pozisyonları kaydet
        time.sleep(0.3)
        for i in self._active_drones():
            self._last_positions[i] = self.drones[i].get_position()

    # ─── NAVİGASYON ──────────────────────────────────────
    def _navigate_first_qr(self, qr_id):
        """İlk QR'a her drone spawn offseti ile gitsin — formasyon kurma."""
        if qr_id not in QR_POSITIONS:
            return
        qr_x, qr_y = QR_POSITIONS[qr_id]

        # Yaw hesapla: SPAWN_CENTER'dan QR1'e
        cur_x, cur_y, _ = self.drones[1].get_position()
        yaw = compute_yaw_to_target(cur_x, cur_y, qr_x, qr_y)
        self.current_yaw = yaw

        self.log(f"İlk QR#{qr_id} → NED({qr_x:.1f},{qr_y:.1f}) — "
                 f"spawn dizilimi korunuyor")

        # Takipçileri askıya al — kendi başlarına gidecekler
        for follower in self.followers.values():
            follower.disable()

        # Her drone: (hedef - spawn_center) offseti kendi spawn'ına eklensin
        # Yani her drone kendi spawn offsetiyle QR1'e paralel gider
        sx_c, sy_c = SPAWN_CENTER
        dx = qr_x - sx_c
        dy = qr_y - sy_c

        for i in range(1, DRONE_COUNT+1):
            sx, sy = SPAWN_NED[i]
            tx = sx + dx
            ty = sy + dy
            self.drones[i].goto_local(tx, ty, self.current_altitude,
                                      yaw=yaw, max_speed=FORMATION_SPEED)
            self.log(f"  Drone {i}: spawn({sx:.1f},{sy:.1f}) → "
                     f"({tx:.1f},{ty:.1f})")

        # Lider hedefe ulaştığında hepsi ulaşmış olacak (aynı offset)
        self.drones[1].wait_until_reached(threshold=1.5, timeout=45)
        self.spin(2.0)

        # Takipçileri tekrar etkinleştir (sonraki formasyon komutları için)
        for fid, follower in self.followers.items():
            if not self.separated.get(fid, False):
                follower.enable()

        # Son pozisyonları kaydet
        for i in range(1, DRONE_COUNT+1):
            self._last_positions[i] = self.drones[i].get_position()

    def navigate_to_qr(self, qr_id):
        if qr_id not in QR_POSITIONS:
            return
        qr_x, qr_y = QR_POSITIONS[qr_id]
        active = self._active_drones()
        self.log(f"QR #{qr_id} → NED({qr_x},{qr_y}) | {self.current_formation}")
        self.send_formation(
            qr_x, qr_y, self.current_altitude,
            self.current_formation, self.current_formation_dist,
            active, target_x=qr_x, target_y=qr_y)

    # ─── QR OKUMA — SADECE LİDER ALÇALIR ─────────────────
    def scan_qr(self, qr_id):
        qr_x, qr_y = QR_POSITIONS[qr_id]

        # Aşama 1: Hızlı yaklaşma — takipçiler normal takipte
        self.drones[1].goto_local(
            qr_x, qr_y, self.current_altitude,
            yaw=self.current_yaw, max_speed=CRUISE_SPEED)
        self.drones[1].wait_until_reached(threshold=3.0, timeout=30)

        # Aşama 2: Hassas yaklaşma — threshold gevşek, QR plakası 120x120cm
        self.drones[1].goto_local(
            qr_x, qr_y, self.current_altitude,
            yaw=self.current_yaw, max_speed=APPROACH_SPEED)
        if not self.drones[1].wait_until_reached(threshold=1.0, timeout=10):
            self.log("Info: QR üstüne geldi, küçük osilasyon olabilir — okuma başlıyor")

        # ─── TAKİPÇİLERİ ASKIYA AL (sadece lider alçalacak) ───
        self.log("Takipçiler pozisyonu koruyor — sadece lider alçalıyor")
        for fid, follower in self.followers.items():
            follower.disable()
        # Takipçiler için son hedefi kaydet — pozisyonu hover ile koruyacaklar
        for fid in self.followers:
            fx, fy, fz = self.drones[fid].get_position()
            self.drones[fid].goto_local(fx, fy, self.current_altitude,
                                        yaw=self.current_yaw, max_speed=0.5)

        # Lider alçalıyor + stabilize — threshold gevşek
        self.drones[1].goto_local(
            qr_x, qr_y, QR_SCAN_ALT,
            yaw=self.current_yaw, max_speed=APPROACH_SPEED)
        self.drones[1].wait_until_reached(threshold=0.8, timeout=12)
        self.drones[1].wait_until_stable(max_wait=2.0)

        self.qr_event.clear()
        self.qr_reader.reset(expected_id=qr_id)
        self.log(f"QR #{qr_id} okunuyor...")

        qr_ok = self.qr_event.wait(timeout=QR_SCAN_TIMEOUT)

        # Lider geri yükseliyor — takipçi irtifasına döner
        self.drones[1].goto_local(
            qr_x, qr_y, self.current_altitude,
            yaw=self.current_yaw, max_speed=CRUISE_SPEED)
        self.drones[1].wait_until_reached(threshold=1.5, timeout=10)

        # ─── TAKİPÇİLERİ TEKRAR ETKİNLEŞTİR ───────────────
        self.log("Takipçiler tekrar takip modunda")
        for fid, follower in self.followers.items():
            if not self.separated.get(fid, False):
                follower.enable()

        if qr_ok:
            self.log(f"QR #{qr_id} ✓")
            self.spin(2.0)
            return self.qr_data
        else:
            self.log(f"QR #{qr_id} okunamadı!")
            return None

    # ─── PX4 PARAMETRE UYUMLAMASI ────────────────────────
    def tune_px4_params(self):
        """Tüm dronlar için aynı hız/ivme parametrelerini MAVLink ile set et.
        Bu sayede 3 drone da eşit karakteristikte hareket eder."""
        self.log("PX4 parametreleri ayarlanıyor (3 drone için)...")
        for drone_id in [1, 2, 3]:
            try:
                ok = set_params_for_drone(drone_id, PX4_TUNING_PARAMS)
                if ok:
                    self.log(f"  Drone {drone_id}: parametreler ✓")
                else:
                    self.log(f"  Drone {drone_id}: parametreler EKSİK")
            except Exception as e:
                self.log(f"  Drone {drone_id}: parametre hatası — {e}")
        self.spin(1.0)

    # ─── KALKIŞ ──────────────────────────────────────────
    def takeoff_all(self):
        # PX4 parametreleri ROMFS/airframe içinde kalıcı — otomatik uygulanır
        self.log(f"Kalkış → {TAKEOFF_ALT}m (spawn konumlarını koruyarak)")

        # Kalkıştan ÖNCE QR1 yönünü hesapla
        cx, cy = SPAWN_CENTER
        qr1_x, qr1_y = QR_POSITIONS[1]
        init_yaw = compute_yaw_to_target(cx, cy, qr1_x, qr1_y)
        self.current_yaw = init_yaw
        self.log(f"Kalkış yönü: {math.degrees(init_yaw):.1f}° (QR1'e)")

        # Her drone KENDİ spawn konumunda dikey kalkış yapsın — formasyon kurmaz
        for i, drone in self.drones.items():
            drone.prepare_takeoff(TAKEOFF_ALT, initial_yaw=init_yaw)
            sx, sy = SPAWN_NED[i]
            drone.target_x   = sx
            drone.target_y   = sy
            drone.target_yaw = init_yaw

        # Leader state yayınlamaya başla (formasyon parametreleri boş)
        # Takipçiler kalkışta formasyon oluşturmasın — active listesi boş tutalım
        # Böylece leader_state callback tetiklenmez ve takipçi kendi kendine kalkar
        self._update_leader_state(
            yaw=init_yaw, tip='OKBASI',
            mesafe=FORMATION_DIST, active=[])   # ← boş: takipçi hareket etmez

        self.spin(3.0)
        for drone in self.drones.values():
            drone.arm_and_offboard()
            time.sleep(0.1)

        self.spin(15.0)
        self.log("Dikey kalkış tamam ✓ (her drone spawn konumunda)")
        self.spin(3.0)

        # _last_positions'ı doldur — sonraki formasyon geçişleri için başlangıç
        for i in range(1, DRONE_COUNT+1):
            self._last_positions[i] = self.drones[i].get_position()
        self.log("Başlangıç dikey kalkış hazır ✓")

    # ─── GÖREV ───────────────────────────────────────────
    def execute_mission(self, qr_data, qr_id, next_qr_x, next_qr_y):
        gorev  = qr_data.get('gorev', {})
        self.log(f"─── Görev #{qr_id} ───")
        active = self._active_drones()

        lx, ly, _ = self.drones[1].get_position()
        next_yaw  = compute_yaw_to_target(lx, ly, next_qr_x, next_qr_y)
        self.current_yaw = next_yaw

        # 1. Formasyon
        form = gorev.get('formasyon', {})
        if form.get('aktif'):
            tip    = form.get('tip', self.current_formation)
            mesafe = max(float(form.get('ajanlar_arasi_mesafe_m',
                                        self.current_formation_dist)), FORMATION_DIST)
            self.log(f"Formasyon → {tip} | {mesafe}m")
            lx, ly, _ = self.drones[1].get_position()
            self.send_formation(lx, ly, self.current_altitude, tip, mesafe,
                                active, target_x=next_qr_x,
                                target_y=next_qr_y, interpolate=True)
            self.spin(5.0)

        # 2. Pitch/Roll
        man = gorev.get('manevra_pitch_roll', {})
        if man.get('aktif'):
            self._execute_pitch_roll(
                float(man.get('pitch_deg', 0)),
                float(man.get('roll_deg', 0)), active)

        # 3. İrtifa
        irt = gorev.get('irtifa_degisimi', {}) or gorev.get('irtifa_degisim', {})
        if irt.get('aktif') and irt.get('deger'):
            new_alt = float(irt['deger'])
            if new_alt > 0:
                self.current_altitude = new_alt
                self.log(f"İrtifa → {new_alt}m")
                x, y, _ = self.drones[1].get_position()
                self.drones[1].goto_local(x, y, new_alt, yaw=next_yaw)
                # Takipçiler lider topic'inden z değişimini anlayacak
                self._update_leader_state()
                self.spin(5.0)

        # 4. Bekleme
        bekleme = float(gorev.get('bekleme_suresi_s', 0))
        if bekleme > 0:
            self.log(f"Bekleme: {bekleme}s")
            self.spin(bekleme)

        # 5. Ayrılma
        ayrilma = qr_data.get('suruden_ayrilma', {})
        if ayrilma.get('aktif'):
            self._execute_separation(ayrilma, active)

        # 6. Katılma
        katilma = qr_data.get('suruye_katilma', {})
        if katilma.get('aktif'):
            self._execute_rejoin(katilma)

    def _execute_pitch_roll(self, pitch, roll, active):
        if len(active) < 2:
            return
        lx, ly, lz = self.drones[1].get_position()
        m = self.current_formation_dist

        if pitch != 0:
            dz = m * math.sin(abs(pitch) * math.pi / 180)
            if pitch < 0:
                self.drones[1].goto_local(lx, ly, lz-dz, yaw=self.current_yaw)
                for i, did in enumerate(active[1:], 1):
                    x, y, z = self.drones[did].get_position()
                    self.drones[did].goto_local(x, y, z+dz*i/len(active),
                                                yaw=self.current_yaw)
            else:
                self.drones[1].goto_local(lx, ly, lz+dz, yaw=self.current_yaw)
                for i, did in enumerate(active[1:], 1):
                    x, y, z = self.drones[did].get_position()
                    self.drones[did].goto_local(x, y, z-dz*i/len(active),
                                                yaw=self.current_yaw)
            self.spin(3.0)
            self.drones[1].goto_local(lx, ly, self.current_altitude,
                                      yaw=self.current_yaw)
            self.spin(2.0)

        if roll != 0:
            dz = m * math.sin(abs(roll) * math.pi / 180)
            for i, did in enumerate(active):
                x, y, z = self.drones[did].get_position()
                if i == 0:
                    self.drones[did].goto_local(x, y, z, yaw=self.current_yaw)
                else:
                    sign = 1 if (i%2==1)==(roll>0) else -1
                    self.drones[did].goto_local(x, y, z+sign*dz,
                                                yaw=self.current_yaw)
            self.spin(3.0)
            for did in active:
                x, y, _ = self.drones[did].get_position()
                self.drones[did].goto_local(x, y, self.current_altitude,
                                            yaw=self.current_yaw)
            self.spin(2.0)

    # ─── PAD İNİŞ — Görsel Servoing + Yükselerek Arama ──
    def _do_land_on_pad(self, drone_id, pad_x, pad_y):
        """
        Hedef renk pad'ine görsel servoing ile iner.
        - Kamera pad'i her gördüğünde anlık merkezi okur ve oraya gider
        - Pad görünmezse yükselerek spiral arama yapar
        - Bulduğu anda aşağı doğru insin
        """
        d     = self.drones[drone_id]
        renk  = self._sep_renk.get(drone_id, '')
        scout = self.pad_scouts.get(drone_id)

        self.log(f"Drone {drone_id}: {renk} pad'ine iniş başladı")

        # ─── Aşama 1: Yaklaşma (PAD_ALT = 5m) ───────────
        d.goto_local(pad_x, pad_y, PAD_ALT, max_speed=2.0)
        d.wait_until_reached(threshold=1.0, timeout=20)

        # ─── Aşama 2: Görsel takip ile aşağı iniş ───────
        # İrtifa adım adım düşürülür, her adımda pad anlık takip edilir
        target_x, target_y = pad_x, pad_y   # ilk tahmin

        inis_adimlari = [5.0, 3.5, 2.5, 1.5, 1.0]   # irtifalar
        for hedef_alt in inis_adimlari:
            # Bu adım için anlık kamera konumu oku
            if scout is not None:
                inst = scout.get_instantaneous_pad(renk)
                if inst:
                    ix, iy, area = inst
                    # Makul alan görülüyor mu?
                    if area > 1500:
                        # Anlık merkezi kullan
                        target_x, target_y = ix, iy
                        self.log(f"  [{hedef_alt:.1f}m] {renk} görünüyor: "
                                 f"({ix:.2f},{iy:.2f}) alan={area:.0f}")
                    else:
                        self.log(f"  [{hedef_alt:.1f}m] {renk} zayıf görülüyor "
                                 f"(alan={area:.0f}) — son hedef kullanılıyor")
                else:
                    self.log(f"  [{hedef_alt:.1f}m] {renk} görünmüyor — "
                             f"yükselerek arama")
                    bulundu = self._spiral_search_rising(
                        drone_id, target_x, target_y, renk)
                    if bulundu:
                        target_x, target_y = bulundu
                        self.log(f"  {renk} bulundu: "
                                 f"({target_x:.2f},{target_y:.2f})")
                    else:
                        self.log(f"  UYARI: {renk} hâlâ bulunamadı")

            # Hedef pozisyona ve yeni irtifaya in
            d.goto_local(target_x, target_y, hedef_alt, max_speed=0.6)
            d.wait_until_reached(threshold=0.3, timeout=10)
            self.spin(0.5)

        # ─── Aşama 3: Son iniş — doğrudan yere ──────────
        # Son anlık konum güncellemesi
        if scout is not None:
            inst = scout.get_instantaneous_pad(renk)
            if inst and inst[2] > 1500:
                target_x, target_y = inst[0], inst[1]

        d.goto_local(target_x, target_y, 0.5, max_speed=0.5)
        d.wait_until_reached(threshold=0.3, timeout=10)

        d.land()
        ok = d.wait_for_disarm(timeout=30.0)
        self.log(f"Drone {drone_id}: {'Disarm ✓' if ok else 'Timeout'}")
        return ok

    def _spiral_search_rising(self, drone_id, center_x, center_y, renk):
        """
        Pad kaybedildiğinde: yükselerek spiral arama.
        Yükseklik arttıkça kamera görüş alanı büyür.
        """
        d     = self.drones[drone_id]
        scout = self.pad_scouts.get(drone_id)
        if scout is None:
            return None

        # 3 yükseklik × 4 yön = 12 nokta maksimum
        yukseklik_adimlari = [6.0, 8.0, 10.0]
        yaricaplar         = [1.0, 2.5, 4.0]

        for alt, radius in zip(yukseklik_adimlari, yaricaplar):
            # Önce merkeze in ama yükseklikte
            d.goto_local(center_x, center_y, alt, max_speed=1.5)
            d.wait_until_reached(threshold=0.5, timeout=8)
            self.spin(0.5)

            # Merkezden kontrol et
            inst = scout.get_instantaneous_pad(renk)
            if inst and inst[2] > 1000:
                return (inst[0], inst[1])

            # Spiral: 4 yön, yarıçap kadar uzakta
            for angle_deg in [0, 90, 180, 270]:
                sx = center_x + radius * math.cos(math.radians(angle_deg))
                sy = center_y + radius * math.sin(math.radians(angle_deg))
                d.goto_local(sx, sy, alt, max_speed=1.5)
                d.wait_until_reached(threshold=0.5, timeout=6)
                self.spin(0.4)

                inst = scout.get_instantaneous_pad(renk)
                if inst and inst[2] > 1000:
                    return (inst[0], inst[1])

        return None

    def _do_land_home(self, drone_id):
        d = self.drones[drone_id]
        x, y, _ = d.get_position()
        d.goto_local(x, y, 0.5, yaw=self.current_yaw, max_speed=0.5)
        d.wait_until_reached(threshold=0.3, timeout=15)
        d.land()
        d.wait_for_disarm(timeout=30.0)

    def _execute_separation(self, ayrilma, active):
        key   = ayrilma.get('ayrilacak_drone_id', '')
        renk  = ayrilma.get('hedef_renk', '')
        bekle = float(ayrilma.get('bekleme_suresi_s', 5))

        try:
            sep_id = int(key.split('_')[1])
        except Exception:
            self.log(f"Geçersiz: {key}")
            return
        if sep_id not in self.drones:
            return

        self.log(f"Drone {sep_id} ayrılıyor → {renk}")
        self.separated[sep_id] = True
        self._sep_renk[sep_id] = renk

        # Ayrılan dronun takibini durdur
        if sep_id in self.followers:
            self.followers[sep_id].disable()

        # Leader state güncelle — aktif dronlar listesini güncelle
        active_now = self._active_drones()
        self._update_leader_state(active=active_now)

        coord = self.pad_coordinator.get_pad(renk)
        if coord:
            px, py = coord
            ok = self._do_land_on_pad(sep_id, px, py)
        else:
            # Koordinat yoksa drone'un mevcut konumundan başlat — pad_scout
            # _do_land_on_pad içinde yükselerek arayacak
            self.log(f"UYARI: {renk} pad konumu bilinmiyor — "
                     f"mevcut konumdan görsel arama başlatılıyor")
            dx, dy, _ = self.drones[sep_id].get_position()
            ok = self._do_land_on_pad(sep_id, dx, dy)

        self.log(f"Drone {sep_id}: {bekle}s yerde bekliyor (motorlar kapalı)...")
        self.spin(bekle)

        # Tekrar arm + kalkış
        self.log(f"Drone {sep_id}: Kalkıyor...")
        d = self.drones[sep_id]
        cur_x, cur_y, _ = d.get_position()
        d.target_x   = cur_x
        d.target_y   = cur_y
        d.target_z   = -self.current_altitude
        d.target_yaw = self.current_yaw
        d.arm_and_offboard()
        self.spin(3.0)

        # Takibi yeniden başlat
        if sep_id in self.followers:
            self.followers[sep_id].enable()

        self.separated[sep_id] = False
        self.log(f"Drone {sep_id}: Sürüye katıldı ✓")

        # Leader state güncelle
        active_now = self._active_drones()
        self._update_leader_state(active=active_now)
        lx, ly, _ = self.drones[1].get_position()
        self.send_formation(lx, ly, self.current_altitude,
                            self.current_formation, self.current_formation_dist,
                            active_now)

    def _execute_rejoin(self, katilma):
        key = katilma.get('katilacak_drone_id', '')
        try:
            jid = int(key.split('_')[1])
        except Exception:
            return
        if jid not in self.drones or not self.separated.get(jid, False):
            return
        self.separated[jid] = False
        if jid in self.followers:
            self.followers[jid].enable()
        active = self._active_drones()
        self._update_leader_state(active=active)
        lx, ly, _ = self.drones[1].get_position()
        self.send_formation(lx, ly, self.current_altitude,
                            self.current_formation, self.current_formation_dist,
                            active)

    # ─── ANA DÖNGÜ ───────────────────────────────────────
    def run(self):
        try:
            self.takeoff_all()

            current_qr = 1
            visited    = set()
            success    = True

            first_qr = True
            while current_qr != 0 and current_qr not in visited:
                visited.add(current_qr)
                self.log(f"\n{'═'*45}\n  Hedef: QR #{current_qr}\n{'═'*45}")

                if first_qr:
                    # İlk QR'a dronlar spawn konumunu koruyarak gitsin
                    # Formasyon KURMA — her drone kendi offset'iyle hareket
                    self._navigate_first_qr(current_qr)
                    first_qr = False
                else:
                    self.navigate_to_qr(current_qr)

                qr_data = self.scan_qr(current_qr)

                if qr_data is None:
                    success = False
                    break

                sonraki  = qr_data.get('sonraki_qr', {})
                next_id  = sonraki.get(TEAM_ID, 0)
                nx, ny   = QR_POSITIONS.get(next_id, HOME_POS)

                self.log(f"Sonraki: QR#{next_id} → ({nx:.1f},{ny:.1f})")
                self.execute_mission(qr_data, current_qr, nx, ny)
                current_qr = next_id

            all_d = list(range(1, DRONE_COUNT+1))
            self.log("🏁 Tamamlandı!" if success else "⚠️ Başarısız")

            # Eve dön — her drone KENDI SPAWN noktasına gitsin
            self.log("Dönüş: her drone kendi kalkış noktasına")
            for i in all_d:
                sx, sy = SPAWN_NED[i]
                self.drones[i].goto_local(sx, sy, TAKEOFF_ALT,
                                          yaw=self.current_yaw, max_speed=2.0)
            # Hepsi için bekle
            self.spin(10.0)
            for i in all_d:
                self.drones[i].wait_until_reached(threshold=1.0, timeout=20)
            self.spin(2.0)

            # Eş zamanlı iniş — her drone KENDI spawn noktasında
            self.log("Eş zamanlı iniş (kalkış noktalarına)...")
            for i in all_d:
                sx, sy = SPAWN_NED[i]
                self.drones[i].goto_local(sx, sy, 0.5, yaw=self.current_yaw,
                                          max_speed=0.5)
            self.spin(5.0)

            for i in all_d:
                self.drones[i].land()
                time.sleep(0.1)

            threads = [threading.Thread(target=self.drones[i].wait_for_disarm,
                                        args=(30.0,), daemon=True)
                       for i in all_d]
            for t in threads:
                t.start()
            for t in threads:
                t.join()
            self.log("Tüm dronlar indi ✓")

        except KeyboardInterrupt:
            pass
        finally:
            if not self._shutting_down:
                self.shutdown()

    def shutdown(self):
        for lst in [list(self.drones.values()),
                    [self.qr_reader, self.pad_coordinator, self.leader_pub],
                    list(self.followers.values()),
                    list(self.pad_scouts.values())]:
            for n in lst:
                try:
                    n.destroy_node()
                except Exception:
                    pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.log("Sistem kapatıldı.")


def main():
    SwarmMission().run()


if __name__ == '__main__':
    main()
