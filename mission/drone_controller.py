#!/usr/bin/env python3
"""
drone_controller.py

Osilasyon azaltma:
  1. wait_until_stable() — hız 0.15 m/s altına düşene kadar max 3s bekle
  2. prepare_takeoff initial_yaw — kalkıştan doğru yön, ekstra dönüş yok
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
import math

from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint,
    VehicleCommand, VehicleLocalPosition, VehicleStatus,
)


class DroneController(Node):

    def __init__(self, drone_id: int):
        super().__init__(f'drone_controller_{drone_id}')
        self.drone_id = drone_id
        ns = f'/px4_{drone_id}'

        qos_pub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', qos_pub)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', qos_pub)
        self.command_pub = self.create_publisher(
            VehicleCommand, f'{ns}/fmu/in/vehicle_command', qos_pub)

        self.local_pos      = None
        self.vehicle_status = None

        # İki versiyonu da dinle — hangi PX4 build'i olursa olsun çalışsın
        self.create_subscription(VehicleLocalPosition,
            f'{ns}/fmu/out/vehicle_local_position_v1', self._pos_cb, qos_sub)
        self.create_subscription(VehicleLocalPosition,
            f'{ns}/fmu/out/vehicle_local_position', self._pos_cb, qos_sub)
        # Status topic — farklı PX4 versiyonlarında farklı suffix
        self.create_subscription(VehicleStatus,
            f'{ns}/fmu/out/vehicle_status_v3', self._status_cb, qos_sub)
        self.create_subscription(VehicleStatus,
            f'{ns}/fmu/out/vehicle_status_v1', self._status_cb, qos_sub)
        self.create_subscription(VehicleStatus,
            f'{ns}/fmu/out/vehicle_status', self._status_cb, qos_sub)

        self.target_x    = 0.0
        self.target_y    = 0.0
        self.target_z    = -8.0
        self.target_yaw  = 0.0
        self.max_speed   = 1.5

        self._land_state = 'none'
        self._land_sent  = False

        self.heartbeat_count = 0
        self.timer = self.create_timer(0.1, self._heartbeat)
        self.get_logger().info(f'DroneController {drone_id} başlatıldı (ns={ns})')

    def _pos_cb(self, msg):
        self.local_pos = msg

    def _status_cb(self, msg):
        self.vehicle_status = msg

    def _heartbeat(self):
        if self._land_state == 'landing':
            return

        ts = int(self.get_clock().now().nanoseconds / 1000)

        ocm = OffboardControlMode()
        ocm.position = True; ocm.velocity = False
        ocm.acceleration = False; ocm.attitude = False
        ocm.body_rate = False; ocm.timestamp = ts

        sp = TrajectorySetpoint()
        sp.position  = [self.target_x, self.target_y, self.target_z]
        sp.yaw       = self.target_yaw
        sp.velocity  = [float('nan'), float('nan'), float('nan')]
        sp.timestamp = ts

        self.offboard_pub.publish(ocm)
        self.setpoint_pub.publish(sp)
        self.heartbeat_count += 1

    def _send_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.command = command
        msg.target_system = self.drone_id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        for k, v in kwargs.items():
            setattr(msg, k, float(v))
        self.command_pub.publish(msg)

    def arm(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def set_offboard_mode(self):
        self._land_state = 'none'
        self._land_sent  = False
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                           param1=1.0, param2=6.0)

    def set_stabilized_mode(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                           param1=1.0, param2=1.0)

    def prepare_takeoff(self, altitude_m: float = 8.0, initial_yaw: float = 0.0):
        """
        Kalkış hazırlığı.
        initial_yaw: Kalkıştan itibaren bakılacak yön.
        Bu sayede kalkış sonrası ekstra yaw dönüşü olmaz → osilasyon azalır.
        """
        self._land_state = 'none'
        self._land_sent  = False
        if self.local_pos:
            self.target_x = self.local_pos.x
            self.target_y = self.local_pos.y
        else:
            self.target_x = 0.0
            self.target_y = float(self.drone_id - 1) * 3.0
        self.target_z   = -abs(altitude_m)
        self.target_yaw = initial_yaw

    def arm_and_offboard(self):
        self._land_state = 'none'
        self._land_sent  = False
        self.set_offboard_mode()
        time.sleep(0.3)
        self.arm()
        time.sleep(0.3)
        self.arm()

    def goto_local(self, x, y, z_m, yaw=0.0, max_speed=None):
        self._land_state = 'none'
        self._land_sent  = False
        self.target_x    = float(x)
        self.target_y    = float(y)
        self.target_z    = -abs(float(z_m))
        self.target_yaw  = float(yaw)
        if max_speed is not None:
            self.max_speed = max_speed

    def land(self):
        if not self._land_sent:
            # 1) Önce mevcut pozisyonda hover — trajectory setpoint çakışmasını önle
            if self.local_pos:
                self.target_x = self.local_pos.x
                self.target_y = self.local_pos.y
            # 2) Heartbeat'i durdur
            self._land_sent  = True
            self._land_state = 'landing'
            time.sleep(0.3)  # Son heartbeat'in gitmesini bekle
            # 3) NAV_LAND komutunu gönder (birkaç kez — reliability için)
            for _ in range(3):
                self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                time.sleep(0.1)
            self.get_logger().info(
                f'Drone {self.drone_id}: NAV_LAND, heartbeat durdu')

    def emergency_land(self):
        self.land()

    def get_speed(self) -> float:
        """Mevcut yatay hız (m/s). Osilasyon tespiti için."""
        if self.local_pos is None:
            return 0.0
        vx = getattr(self.local_pos, 'vx', 0.0)
        vy = getattr(self.local_pos, 'vy', 0.0)
        return math.sqrt(vx**2 + vy**2)

    def wait_until_stable(self, speed_threshold: float = 0.15,
                          max_wait: float = 3.0) -> bool:
        """
        Drone stabilize olana kadar bekle — osilasyon önleme.

        Hız speed_threshold (0.15 m/s) altına düşene kadar bekler.
        max_wait (3s) sonra zaten devam eder.

        QR okuma öncesi ve formasyon değişimi öncesi çağrılır.
        Hakemler osilasyon görürse -10 puan → bu bunu önler.
        """
        start = time.time()
        while time.time() - start < max_wait:
            time.sleep(0.15)
            if self.get_speed() < speed_threshold:
                return True
        return False

    def wait_for_disarm(self, timeout: float = 30.0) -> bool:
        start         = time.time()
        fallback_sent = False
        self.get_logger().info(f'Drone {self.drone_id}: Disarm bekleniyor...')

        while time.time() - start < timeout:
            time.sleep(0.3)
            if self.vehicle_status is not None:
                try:
                    if self.vehicle_status.arming_state == 1:
                        elapsed = time.time() - start
                        self.get_logger().info(
                            f'Drone {self.drone_id}: Disarm ✓ ({elapsed:.1f}s)')
                        self._land_state = 'none'
                        return True
                except AttributeError:
                    pass

            if time.time() - start > 10.0 and not fallback_sent:
                fallback_sent = True
                self.get_logger().warn(
                    f'Drone {self.drone_id}: 10s → STABILIZED + disarm')
                self.set_stabilized_mode()
                time.sleep(0.5)
                self.disarm()
                time.sleep(0.5)
                self.disarm()

        self.get_logger().warn(f'Drone {self.drone_id}: Timeout ({timeout}s)')
        self._land_state = 'none'
        self.disarm()
        return False

    def get_position(self):
        if self.local_pos:
            return (self.local_pos.x, self.local_pos.y, -self.local_pos.z)
        return (0.0, float(self.drone_id - 1) * 3.0, 0.0)

    def get_altitude(self):
        if self.local_pos:
            return -self.local_pos.z
        return 0.0

    def get_heading(self):
        if self.local_pos and hasattr(self.local_pos, 'heading'):
            return self.local_pos.heading
        return self.target_yaw

    def distance_to_target(self):
        x, y, z = self.get_position()
        return math.sqrt((x-self.target_x)**2 +
                         (y-self.target_y)**2 +
                         (z-(-self.target_z))**2)

    def wait_until_reached(self, threshold: float = 2.0,
                           timeout: float = 60.0) -> bool:
        start = time.time()
        while time.time() - start < timeout:
            time.sleep(0.2)
            if self.local_pos and self.distance_to_target() < threshold:
                self.get_logger().info(
                    f'Drone {self.drone_id}: Hedefe ulaşıldı '
                    f'(mesafe={self.distance_to_target():.2f}m)')
                return True
        self.get_logger().warn(
            f'Drone {self.drone_id}: Hedefe ulaşılamadı! (timeout={timeout}s)')
        return False

    def is_armed(self):
        return self.vehicle_status and self.vehicle_status.arming_state == 2

    def is_disarmed(self):
        return self.vehicle_status and self.vehicle_status.arming_state == 1

    def is_in_offboard(self):
        return self.vehicle_status and self.vehicle_status.nav_state == 14
