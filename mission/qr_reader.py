#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyzbar import pyzbar
import json
import threading


class QRReader(Node):

    def __init__(self, uav_namespace: str, callback):
        super().__init__(f'qr_reader_{uav_namespace}')

        self.bridge = CvBridge()
        self.callback = callback
        self.last_qr_id = None
        self.expected_id = None
        self.lock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        topic = f'/{uav_namespace}/down_camera/down_camera/image_raw'
        self.create_subscription(Image, topic, self._image_cb, qos)

    def _image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except:
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        codes = pyzbar.decode(gray)

        for code in codes:
            raw = code.data.decode('utf-8').strip()
            try:
                data = json.loads(raw)
            except:
                continue

            qr_id = data.get('qr_id')

            with self.lock:
                if qr_id == self.last_qr_id:
                    return
                self.last_qr_id = qr_id

            self.callback(data)
            break

    def reset(self, expected_id=None):
        """Yeni QR'a geçişte son QR ID'yi sıfırla."""
        with self.lock:
            self.last_qr_id = None
            self.expected_id = expected_id

    def _image_cb_check(self, data):
        """Beklenen QR ID kontrolü."""
        qr_id = data.get('qr_id')
        if self.expected_id is not None and qr_id != self.expected_id:
            return False  # Yanlış QR, yoksay
        return True
