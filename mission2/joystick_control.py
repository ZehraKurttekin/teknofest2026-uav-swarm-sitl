#!/usr/bin/env python3
"""
joystick_control.py — SANCAK Görev 2 · Yarı Otonom Sürü

keyboard_swarm.py'nin mantığı ama klavye yerine WebSocket üzerinden
gelen /swarm/joystick_cmd topic'inden tetiklenir.

Farklar:
  - Kalkışta formasyon KURULMAZ. Dronlar BULUNDUĞU konumdan dikey 8m'ye çıkar.
    (Görev 1 sonunda dronlar rastgele yerlere inmiş olabilir — her biri
     kendi anlık pozisyonundan kalkar.)
  - Formasyon değiştirme tuşu (1/2/3) veya web butonu basıldığında kurulur.
  - İniş: her drone MEVCUT konumunda dikey alçalır.

Kullanılan modüller:
  - drone_controller2.py (basit PX4 controller)
  - formation2.py        (OKBASI/V/CIZGI offset fonksiyonu)
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
from drone_controller2 import DroneController
from formation2 import get_formation_offsets

# ─── SABİTLER ──────────────────────────────────────
DRONE_COUNT   = 3
TAKEOFF_ALT   = 8.0
# NOT: SPAWN_NED ve SPAWN_CENTER kaldırıldı — artık KULLANILMIYOR.
# Kalkış/iniş her drone'un ANLIK pozisyonundan başlar. Görev 1 sonunda
# dronlar rastgele bir yerde inmiş olabilir, sorun değil.
FORMATION_DIST = 7.0

# Hareket parametreleri (keyboard_swarm.py'deki ile benzer)
HAREKET_HIZ   = 1.0     # metre / tuş
YAW_HIZ       = math.radians(5.0)   # 5° per tuş
IRTIFA_HIZ    = 0.5
MANEVRA_EGIM  = 2.0     # metre / adım (max ±6m)

MOD_HAREKET = 'HAREKET'
MOD_MANEVRA = 'MANEVRA'


class JoystickSwarm(Node):

    def __init__(self):
        super().__init__('joystick_swarm')

        # Drone controller'ları
        self.drones = {i: DroneController(i) for i in range(1, DRONE_COUNT+1)}

        # Executor — isim 'executor' değil çünkü Node base class ile çakışabiliyor
        self.mt_executor = MultiThreadedExecutor()
        for d in self.drones.values():
            self.mt_executor.add_node(d)
        self.mt_executor.add_node(self)
        threading.Thread(target=self.mt_executor.spin, daemon=True).start()

        # ROS2 subs/pubs
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            String, '/swarm/joystick_cmd',
            self._on_cmd, qos)
        self.feedback_pub = self.create_publisher(
            String, '/swarm/joystick_feedback', qos)

        # Durum
        self.armed      = False
        self.mod        = MOD_HAREKET
        self.formasyon  = 'V'
        self.mesafe     = FORMATION_DIST
        self.swarm_yaw  = 0.0
        # Sürü merkezi — kalkışta dronların anlık pozisyonundan hesaplanır
        self.cx         = 0.0
        self.cy         = 0.0
        self.cz         = TAKEOFF_ALT
        self.pitch_egim = 0.0
        self.roll_egim  = 0.0

        # Basılı tuşlar — her update'te işlenir
        self.pressed_keys = set()
        self._pressed_lock = threading.Lock()

        # Kalkıştan sonra formasyon oluştu mu? Önce FALSE.
        # Formasyon butonu basılana kadar dronlar BULUNDUĞU yerde hover.
        self.formation_active = False

        # Update loop
        self.update_timer = self.create_timer(0.05, self._update_loop)  # 20 Hz

        self.shutting_down = False
        signal.signal(signal.SIGINT,  self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)

        print("=" * 60)
        print("  SANCAK — GÖREV 2 · Yarı Otonom Sürü Kontrolü")
        print("  /swarm/joystick_cmd dinleniyor")
        print("  HTML arayüzü için joystick_bridge.py gerekli")
        print("=" * 60)

    # ─── Log / Feedback ────────────────────────────
    def _feedback(self, msg):
        m = String()
        m.data = msg
        self.feedback_pub.publish(m)
        print(f"[MISSION2] {msg}")

    def _sig_handler(self, sig, frame):
        if self.shutting_down:
            return
        self.shutting_down = True
        print("\n[MISSION2] Ctrl+C — güvenli iniş...")
        for d in self.drones.values():
            try:
                d.land()
            except Exception:
                pass
        time.sleep(2)
        sys.exit(0)

    # ─── Joystick komut callback ───────────────────
    def _on_cmd(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        t = data.get('type', '')

        if t == 'takeoff':
            threading.Thread(target=self._do_takeoff, daemon=True).start()

        elif t == 'land':
            threading.Thread(target=self._do_land, daemon=True).start()

        elif t == 'mode_change':
            self.mod = data.get('mode', self.mod)
            self.pitch_egim = 0.0
            self.roll_egim  = 0.0
            self._feedback(f'Mod → {self.mod}')

        elif t == 'formation_change':
            self._set_formasyon(data.get('formation', self.formasyon))

        elif t == 'key_down':
            key = data.get('key', '').lower()
            if key:
                with self._pressed_lock:
                    self.pressed_keys.add(key)

        elif t == 'key_up':
            key = data.get('key', '').lower()
            with self._pressed_lock:
                self.pressed_keys.discard(key)

        elif t == 'key_press':
            # Tek seferlik (V → sıfırla)
            key = data.get('key', '').lower()
            if key == 'v':
                self.pitch_egim = 0.0
                self.roll_egim  = 0.0
                if self.formation_active:
                    self._apply_formation()
                self._feedback('Manevra sıfırlandı')

    # ─── Formasyon Uygulamaları ────────────────────
    def _positions_with_formation(self):
        offsets = get_formation_offsets(
            self.formasyon, DRONE_COUNT, self.mesafe, self.swarm_yaw)
        return [(self.cx + dx, self.cy + dy, self.cz + dz)
                for dx, dy, dz in offsets]

    def _apply_formation(self):
        """Normal formasyon — tüm dronlar offset'te."""
        positions = self._positions_with_formation()
        for i, did in enumerate(sorted(self.drones.keys())):
            if i < len(positions):
                x, y, z = positions[i]
                self.drones[did].goto_local(x, y, max(2.0, z), self.swarm_yaw)

    def _apply_free_hover(self):
        """
        Kalkıştan hemen sonra: her drone ANLIK (X, Y) konumunda
        yalnızca irtifa değiştirerek hover eder. Pozisyon kaymaları
        olmaz — başlangıç dizilimi (rastgele olabilir) korunur.
        """
        for did, drone in self.drones.items():
            # Her drone kendi ANLIK X, Y'sinde kalsın, sadece Z hedef irtifa
            x, y, _ = drone.get_position()
            drone.goto_local(x, y, max(2.0, self.cz), self.swarm_yaw)

    def _apply_manevra_pitch(self):
        """Merkez sabit, formasyon pitch ekseninde eğilir."""
        offsets = get_formation_offsets(
            self.formasyon, DRONE_COUNT, self.mesafe, self.swarm_yaw)
        cos_y = math.cos(self.swarm_yaw)
        sin_y = math.sin(self.swarm_yaw)
        for i, did in enumerate(sorted(self.drones.keys())):
            if i >= len(offsets):
                continue
            dx, dy, dz = offsets[i]
            # Body x yönündeki offset — pitch ile z değişir
            body_x = dx * cos_y + dy * sin_y
            egim   = self.pitch_egim * (body_x / (self.mesafe + 0.001))
            x = self.cx + dx
            y = self.cy + dy
            z = self.cz + dz + egim
            self.drones[did].goto_local(x, y, max(2.0, z), self.swarm_yaw)

    def _apply_manevra_roll(self):
        """Merkez sabit, formasyon roll ekseninde eğilir."""
        offsets = get_formation_offsets(
            self.formasyon, DRONE_COUNT, self.mesafe, self.swarm_yaw)
        cos_y = math.cos(self.swarm_yaw)
        sin_y = math.sin(self.swarm_yaw)
        for i, did in enumerate(sorted(self.drones.keys())):
            if i >= len(offsets):
                continue
            dx, dy, dz = offsets[i]
            body_y = -dx * sin_y + dy * cos_y
            egim   = self.roll_egim * (body_y / (self.mesafe + 0.001))
            x = self.cx + dx
            y = self.cy + dy
            z = self.cz + dz + egim
            self.drones[did].goto_local(x, y, max(2.0, z), self.swarm_yaw)

    def _apply_current(self):
        """Mevcut duruma göre doğru fonksiyonu çağır."""
        if not self.armed:
            return
        if not self.formation_active:
            self._apply_free_hover()
            return
        if self.mod == MOD_MANEVRA:
            if self.pitch_egim != 0.0:
                self._apply_manevra_pitch()
            elif self.roll_egim != 0.0:
                self._apply_manevra_roll()
            else:
                self._apply_formation()
        else:
            self._apply_formation()

    # ─── Formasyon değiştirme ──────────────────────
    def _set_formasyon(self, tip):
        if tip not in ('V', 'OKBASI', 'CIZGI'):
            self._feedback(f'Bilinmeyen formasyon: {tip}')
            return
        self.formasyon   = tip
        self.pitch_egim  = 0.0
        self.roll_egim   = 0.0
        first_time = not self.formation_active
        self.formation_active = True
        if first_time:
            self._feedback(f'Formasyon kuruluyor → {tip}')
        else:
            self._feedback(f'Formasyon → {tip}')
        if self.armed:
            self._apply_formation()

    # ─── Ana döngü — basılı tuşlara göre güncelle ──
    def _update_loop(self):
        if not self.armed or self.shutting_down:
            return

        with self._pressed_lock:
            keys = set(self.pressed_keys)

        if not keys:
            return

        changed = False

        # 20 Hz döngüde her tick için "hız × 0.05 sn" kadar katkı
        dt = 0.05
        cos_y = math.cos(self.swarm_yaw)
        sin_y = math.sin(self.swarm_yaw)

        # Hız çarpanları — m/s ya da rad/s cinsinden
        VEL_XY   = 2.0    # m/s
        VEL_Z    = 1.5    # m/s
        RATE_YAW = math.radians(45.0)  # rad/s
        RATE_MAN = 3.0    # m/s (manevra eğim oranı)

        if self.mod == MOD_HAREKET:
            if 'w' in keys:
                self.cx += cos_y * VEL_XY * dt
                self.cy += sin_y * VEL_XY * dt
                changed = True
            if 's' in keys:
                self.cx -= cos_y * VEL_XY * dt
                self.cy -= sin_y * VEL_XY * dt
                changed = True
            if 'd' in keys:
                self.cx += sin_y * VEL_XY * dt
                self.cy -= cos_y * VEL_XY * dt
                changed = True
            if 'a' in keys:
                self.cx -= sin_y * VEL_XY * dt
                self.cy += cos_y * VEL_XY * dt
                changed = True
        else:  # MANEVRA
            if self.formation_active:
                if 'w' in keys:
                    self.pitch_egim = min(6.0, self.pitch_egim + RATE_MAN * dt)
                    changed = True
                if 's' in keys:
                    self.pitch_egim = max(-6.0, self.pitch_egim - RATE_MAN * dt)
                    changed = True
                if 'd' in keys:
                    self.roll_egim = min(6.0, self.roll_egim + RATE_MAN * dt)
                    changed = True
                if 'a' in keys:
                    self.roll_egim = max(-6.0, self.roll_egim - RATE_MAN * dt)
                    changed = True

        # Yaw + irtifa — her iki modda aktif
        if 'e' in keys:
            self.swarm_yaw += RATE_YAW * dt
            changed = True
        if 'q' in keys:
            self.swarm_yaw -= RATE_YAW * dt
            changed = True
        if 'r' in keys:
            self.cz = min(30.0, self.cz + VEL_Z * dt)
            changed = True
        if 'f' in keys:
            self.cz = max(2.0, self.cz - VEL_Z * dt)
            changed = True

        if changed:
            self._apply_current()

    # ─── KALKIŞ — her drone ANLIK pozisyondan, formasyon KURMAZ ──
    def _do_takeoff(self):
        if self.armed:
            self._feedback('Zaten havada')
            return

        self._feedback('KALKIŞ başlıyor (bulunulan konumdan)...')
        self.swarm_yaw   = 0.0
        self.pitch_egim  = 0.0
        self.roll_egim   = 0.0
        self.cz          = TAKEOFF_ALT
        self.formation_active = False   # ← formasyon YOK

        # Her drone'un anlık pozisyonundan kalk
        # Önce birkaç saniye bekle ki local_pos subscriber'a mesaj gelsin
        self._feedback('Pozisyon okumaları alınıyor...')
        time.sleep(2.0)

        # Sürü merkezini dronların ortalaması olarak hesapla
        xs, ys = [], []
        for did, drone in self.drones.items():
            x, y, _ = drone.get_position()
            xs.append(x); ys.append(y)
            self._feedback(f'  Drone {did}: mevcut konum ({x:.1f}, {y:.1f})')
        self.cx = sum(xs) / len(xs)
        self.cy = sum(ys) / len(ys)
        self._feedback(f'Sürü merkezi: ({self.cx:.1f}, {self.cy:.1f})')

        # Her drone kendi anlık konumundan dikey kalksın
        threads = []
        for did, drone in self.drones.items():
            x, y, _ = drone.get_position()
            t = threading.Thread(
                target=self._takeoff_one,
                args=(drone, x, y, TAKEOFF_ALT),
                daemon=True)
            t.start()
            threads.append(t)
        for t in threads:
            t.join()

        self._feedback('Arm + Offboard gönderildi, yükseliyor...')
        time.sleep(15.0)

        self.armed = True
        self._feedback('Dikey kalkış tamam ✓ Mevcut dizilim korunuyor')
        self._feedback('Formasyon kurmak için: V, OKBASI veya CIZGI butonuna basın')

        self._apply_free_hover()

    def _takeoff_one(self, drone, x, y, alt):
        """Bir drone için kalkış — mevcut X,Y konumunda dikey."""
        drone.target_x = x
        drone.target_y = y
        drone.takeoff(alt)

    # ─── İNİŞ — her drone MEVCUT konumunda alçalır ─────────
    def _do_land(self):
        if not self.armed:
            self._feedback('Zaten yerde')
            return
        self._feedback('İNİŞ başlıyor — her drone mevcut konumunda')
        self.armed = False

        # Her drone mevcut X, Y'sinde 0.5m'ye alçalsın
        for did, drone in self.drones.items():
            x, y, _ = drone.get_position()
            drone.goto_local(x, y, 0.5, self.swarm_yaw)
        time.sleep(5.0)

        # NAV_LAND
        for d in self.drones.values():
            d.land()
            time.sleep(0.1)

        time.sleep(10.0)
        self.formation_active = False
        self._feedback('Tüm dronlar indi ✓')


def main():
    rclpy.init()
    node = JoystickSwarm()
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
