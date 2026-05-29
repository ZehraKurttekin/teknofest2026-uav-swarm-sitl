#!/usr/bin/env python3
"""
pad_scout.py — Hızlı Pad Tespiti

Eski yaklaşım:
  Peak detection — drone pad üzerinden geçip uzaklaşınca tepe tespit edilir.
  Sorun: Çok yavaş. Drone pad üzerindeyken zaten en iyi konumdayken bekleniyor.

Yeni yaklaşım — 2 katmanlı hızlı tespit:
  1. ANLIK (Instant): Alan > INSTANT_CONFIRM_AREA → hemen koordinatı kaydet
     (kesin değil ama çok hızlı, drone yakınken güncellenir)
  2. EN İYİ (Best): Tüm ölçümler içinde en büyük alan anındaki koordinat
     (drone pad üzerindeyken en büyük alan = en doğru merkez)

  Konfirmasyon: En iyi alan CONFIRM_AREA'yı geçince "kesin" kabul edilir.
  Peak beklenmez — en büyük alan anındaki koordinat direkt kullanılır.

  EMA hâlâ kullanılır ama sadece gürültü filtresi için, peak tespiti için değil.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import math
import threading

# ─── Kamera ──────────────────────────────────────────────
CAMERA_FOV = 1.0471975   # 60° radyan

# ─── HSV Renk Aralıkları ─────────────────────────────────
HSV_RED_LOWER1 = np.array([0,   150, 80])
HSV_RED_UPPER1 = np.array([10,  255, 255])
HSV_RED_LOWER2 = np.array([170, 150, 80])
HSV_RED_UPPER2 = np.array([180, 255, 255])
HSV_BLUE_LOWER = np.array([105, 150, 80])
HSV_BLUE_UPPER = np.array([125, 255, 255])

# ─── Hızlı Tespit Eşikleri ───────────────────────────────
MIN_DETECT_AREA    = 800    # Bu alanın altı yok say
INSTANT_UPDATE_AREA = 1500  # Alan küçük olsa da anında güncelle (yukarıdan tarama)
CONFIRM_AREA       = 6000   # Bu alanı geçince "kesin" kabul et
EMA_ALPHA          = 0.5    # Hızlı EMA (0.3'ten artırıldı)


class PadTracker:
    """
    Tek renk için hızlı pad takibi.
    Peak beklenmez — en büyük alan anındaki koordinat kullanılır.
    """

    def __init__(self, renk: str):
        self.renk = renk
        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        with self.lock:
            self._ema_area  = 0.0
            self._best_area = 0.0    # Şimdiye kadar en büyük alan
            self._best_x    = None
            self._best_y    = None
            self._confirmed = False
            self._conf_x    = None
            self._conf_y    = None
            # Anlık (en son güvenilir) koordinat
            self._instant_x = None
            self._instant_y = None
            self._instant_a = 0.0

    def update(self, raw_area: float, x: float, y: float):
        """
        Yeni ölçüm. Hemen koordinatı güncelle.
        Returns: (confirmed_yeni_mi, cx, cy)
        """
        with self.lock:
            if self._confirmed:
                # Kesinleşti ama anlık koordinatı güncellemeye devam et
                # (iniş sırasında daha doğru merkez için)
                if raw_area > MIN_DETECT_AREA:
                    self._instant_x = x
                    self._instant_y = y
                    self._instant_a = raw_area
                return (False, None, None)

            # EMA smoothing
            if self._ema_area == 0.0:
                self._ema_area = raw_area
            else:
                self._ema_area = EMA_ALPHA * raw_area + (1 - EMA_ALPHA) * self._ema_area

            smoothed = self._ema_area

            # Anlık koordinat — INSTANT_UPDATE_AREA üstündeyse hemen güncelle
            if smoothed >= INSTANT_UPDATE_AREA:
                self._instant_x = x
                self._instant_y = y
                self._instant_a = smoothed

            # En iyi koordinat güncelle
            if smoothed > self._best_area:
                self._best_area = smoothed
                self._best_x    = x
                self._best_y    = y

            # Konfirmasyon — peak bekleme yok
            # En iyi alan CONFIRM_AREA'yı geçtiyse hemen onayla
            if (self._best_area >= CONFIRM_AREA and
                    self._best_x is not None):
                self._confirmed = True
                self._conf_x    = self._best_x
                self._conf_y    = self._best_y
                return (True, self._conf_x, self._conf_y)

            return (False, None, None)

    def lost(self):
        """Pad bu frame'de görünmüyor."""
        with self.lock:
            if not self._confirmed:
                self._ema_area = 0.0   # EMA sıfırla, tekrar görününce temiz başla

    def is_confirmed(self) -> bool:
        with self.lock:
            return self._confirmed

    def get_best(self):
        """En iyi koordinat (kesinleşmemiş olabilir)."""
        with self.lock:
            if self._best_x is not None:
                return (self._best_x, self._best_y, self._best_area)
        return None

    def get_instant(self):
        """Anlık kamera koordinatı (en güncel)."""
        with self.lock:
            if self._instant_x is not None:
                return (self._instant_x, self._instant_y, self._instant_a)
        return None

    def get_confirmed(self):
        with self.lock:
            if self._confirmed:
                return (self._conf_x, self._conf_y)
        return None


class PadScout(Node):
    """
    Drone kamerasını dinler.
    Tüm aktif dronlar aynı anda tarar — ilk bulan yayınlar.
    """

    def __init__(self, drone_id: int, uav_namespace: str,
                 get_drone_pos_fn, get_drone_heading_fn, altitude_fn):
        super().__init__(f'pad_scout_{drone_id}')
        self.drone_id    = drone_id
        self.get_pos     = get_drone_pos_fn
        self.get_heading = get_drone_heading_fn
        self.get_alt     = altitude_fn
        self.bridge      = CvBridge()
        self.active      = True

        self.trackers = {
            'KIRMIZI': PadTracker('KIRMIZI'),
            'MAVI':    PadTracker('MAVI'),
        }

        self._current_areas = {'KIRMIZI': 0.0, 'MAVI': 0.0}
        self._areas_lock    = threading.Lock()

        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_pub = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=10)

        topic = f'/{uav_namespace}/down_camera/down_camera/image_raw'
        self.create_subscription(Image, topic, self._image_cb, qos_sub)
        self.pub = self.create_publisher(String, '/swarm/pad_found', qos_pub)
        self.get_logger().info(f'PadScout {drone_id}: {topic}')

    def _image_cb(self, msg):
        if not self.active:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception:
            return

        h, w = frame.shape[:2]
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red1  = cv2.inRange(hsv, HSV_RED_LOWER1, HSV_RED_UPPER1)
        red2  = cv2.inRange(hsv, HSV_RED_LOWER2, HSV_RED_UPPER2)
        masks = {
            'KIRMIZI': cv2.bitwise_or(red1, red2),
            'MAVI':    cv2.inRange(hsv, HSV_BLUE_LOWER, HSV_BLUE_UPPER),
        }

        for renk, mask in masks.items():
            area, cx_px, cy_px = self._process_mask(mask, w, h)

            with self._areas_lock:
                self._current_areas[renk] = area

            if area >= MIN_DETECT_AREA and cx_px is not None:
                pad_x, pad_y = self._pixel_to_ned(cx_px, cy_px, w, h)
                if pad_x is None:
                    continue

                confirmed, fx, fy = self.trackers[renk].update(area, pad_x, pad_y)

                if confirmed:
                    self.get_logger().info(
                        f'PadScout {self.drone_id}: {renk} KESİN ✓ '
                        f'NED({fx:.2f},{fy:.2f}) [area={area:.0f}]')
                    self.pub.publish(String(data=json.dumps({
                        'renk': renk, 'x': fx, 'y': fy,
                        'alan': area, 'drone_id': self.drone_id, 'kesin': True
                    })))
                else:
                    # Anlık güncelleme — hemen yayınla (kesin olmadan da)
                    best = self.trackers[renk].get_best()
                    if best and best[2] >= INSTANT_UPDATE_AREA:
                        bx, by, ba = best
                        self.pub.publish(String(data=json.dumps({
                            'renk': renk, 'x': bx, 'y': by,
                            'alan': ba, 'drone_id': self.drone_id, 'kesin': False
                        })))
            else:
                self.trackers[renk].lost()

    def _process_mask(self, mask, w, h):
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0, None, None

        largest = max(contours, key=cv2.contourArea)
        area    = cv2.contourArea(largest)
        if area < MIN_DETECT_AREA:
            return area, None, None

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return area, None, None

        return area, M['m10']/M['m00'], M['m01']/M['m00']

    def _pixel_to_ned(self, cx_px, cy_px, w, h):
        cx_norm = (cx_px - w/2) / (w/2)
        cy_norm = (cy_px - h/2) / (h/2)

        dx, dy, dz = self.get_pos()
        altitude = abs(dz) if dz < 0 else dz
        if altitude < 0.5:
            altitude = self.get_alt()
        if altitude < 0.5:
            return None, None

        half_w  = math.tan(CAMERA_FOV / 2) * altitude
        heading = self.get_heading()

        lf = -cy_norm * half_w
        lr =  cx_norm * half_w

        pad_x = dx + lf * math.cos(heading) - lr * math.sin(heading)
        pad_y = dy + lf * math.sin(heading) + lr * math.cos(heading)
        return pad_x, pad_y

    def get_instantaneous_pad(self, renk: str):
        return self.trackers[renk].get_instant()

    def get_current_area(self, renk: str) -> float:
        with self._areas_lock:
            return self._current_areas.get(renk, 0.0)

    def verify_color_on_camera(self, renk: str, min_area: float = 2000):
        area = self.get_current_area(renk)
        return area >= min_area, area

    def deactivate(self):
        self.active = False

    def activate(self):
        self.active = True
        for t in self.trackers.values():
            t.reset()
        with self._areas_lock:
            self._current_areas = {'KIRMIZI': 0.0, 'MAVI': 0.0}


class PadCoordinator(Node):
    """
    /swarm/pad_found dinler.
    Kesinleşmiş: değişmez.
    Ham: en büyük alan ile güncellenir (daha yakın = daha iyi).
    """

    def __init__(self):
        super().__init__('pad_coordinator')
        self.found_pads = {}
        self.lock = threading.Lock()

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(String, '/swarm/pad_found', self._cb, qos)
        self.get_logger().info('PadCoordinator: dinleniyor')

    def _cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        renk  = data.get('renk')
        x, y  = data.get('x'), data.get('y')
        alan  = data.get('alan', 0)
        kesin = data.get('kesin', False)

        if not renk or x is None:
            return

        with self.lock:
            cur = self.found_pads.get(renk)
            if cur and cur.get('kesin'):
                return   # Kesinleşti, üzerine yazma
            if kesin or not cur or alan > cur.get('alan', 0):
                self.found_pads[renk] = {'x': x, 'y': y, 'alan': alan, 'kesin': kesin}
                if kesin:
                    self.get_logger().info(
                        f'PadCoordinator: {renk} KESİN → NED({x:.2f},{y:.2f})')

    def get_pad(self, renk: str):
        with self.lock:
            p = self.found_pads.get(renk)
            return (p['x'], p['y']) if p else None

    def is_confirmed(self, renk: str) -> bool:
        with self.lock:
            return self.found_pads.get(renk, {}).get('kesin', False)

    def all_pads(self):
        with self.lock:
            return dict(self.found_pads)
