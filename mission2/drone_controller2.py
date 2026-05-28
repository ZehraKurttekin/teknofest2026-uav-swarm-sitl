#!/usr/bin/env python3
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
        super().__init__(f"drone_controller_{drone_id}")
        self.drone_id = drone_id
        ns = f"/px4_{drone_id}"
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.offboard_pub = self.create_publisher(OffboardControlMode, f"{ns}/fmu/in/offboard_control_mode", qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, f"{ns}/fmu/in/trajectory_setpoint", qos)
        self.command_pub  = self.create_publisher(VehicleCommand, f"{ns}/fmu/in/vehicle_command", qos)
        self.local_pos = None
        self.vehicle_status = None
        self.create_subscription(VehicleLocalPosition, f"{ns}/fmu/out/vehicle_local_position", self._pos_cb, qos)
        self.create_subscription(VehicleStatus, f"{ns}/fmu/out/vehicle_status", self._status_cb, qos)
        self.target_x   = 0.0
        self.target_y   = 0.0
        self.target_z   = -5.0
        self.target_yaw = 0.0
        self.create_timer(0.1, self._heartbeat)
        self.get_logger().info(f"DroneController {drone_id} hazir.")

    def _pos_cb(self, msg):    self.local_pos = msg
    def _status_cb(self, msg): self.vehicle_status = msg

    def _heartbeat(self):
        ts = int(self.get_clock().now().nanoseconds / 1000)
        m = OffboardControlMode()
        m.position = True; m.velocity = False; m.acceleration = False; m.timestamp = ts
        self.offboard_pub.publish(m)
        sp = TrajectorySetpoint()
        sp.position = [self.target_x, self.target_y, self.target_z]
        sp.yaw = self.target_yaw; sp.timestamp = ts
        self.setpoint_pub.publish(sp)

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
        self.get_logger().info(f"Drone {self.drone_id}: ARM")

    def disarm(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def set_offboard_mode(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info(f"Drone {self.drone_id}: OFFBOARD")

    def takeoff(self, altitude_m=5.0):
        if self.local_pos:
            self.target_x = self.local_pos.x
            self.target_y = self.local_pos.y
        self.target_z = -abs(altitude_m)
        self.get_logger().info(f"Drone {self.drone_id}: Heartbeat bekleniyor...")
        time.sleep(3.0)
        self.set_offboard_mode()
        time.sleep(1.0)
        self.arm()
        time.sleep(0.5)
        self.arm()
        self.get_logger().info(f"Drone {self.drone_id}: Takeoff -> {altitude_m}m")

    def goto_local(self, x, y, z_m, yaw=0.0):
        self.target_x   = x
        self.target_y   = y
        self.target_z   = -abs(z_m)
        self.target_yaw = yaw

    def land(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info(f"Drone {self.drone_id}: LAND")

    def get_position(self):
        if self.local_pos:
            return (self.local_pos.x, self.local_pos.y, -self.local_pos.z)
        return (0.0, 0.0, 0.0)

    def distance_to_target(self):
        x, y, z = self.get_position()
        return math.sqrt(
            (x - self.target_x)**2 +
            (y - self.target_y)**2 +
            (z - (-self.target_z))**2
        )

    def wait_until_reached(self, threshold=2.0, timeout=120.0):
        start = time.time()
        while time.time() - start < timeout:
            time.sleep(0.1)
            if self.local_pos and self.distance_to_target() < threshold:
                return True
        self.get_logger().warn(f"Drone {self.drone_id}: Hedefe ulasilamadi!")
        return False
