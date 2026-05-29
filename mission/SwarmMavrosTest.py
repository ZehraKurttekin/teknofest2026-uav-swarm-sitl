#!/usr/bin/env python3

import time
import rclpy
from drone_controller import DroneController


DRONE_COUNT = 3


class SwarmMavrosTest:

    def __init__(self):
        rclpy.init()

        self.drones = {
            i: DroneController(i) for i in range(1, DRONE_COUNT + 1)
        }

        print("\n=== SWARM MAVROS TEST BAŞLADI ===\n")

    # ---------------- CONNECT TEST ----------------
    def check_connections(self):
        print("[TEST] MAVROS bağlantı kontrolü...")

        for i, d in self.drones.items():
            pos = d.get_position()
            if pos is None:
                print(f"  Drone {i}: ❌ BAĞLANTI YOK")
            else:
                print(f"  Drone {i}: ✓ BAĞLI | POS: {pos}")

    # ---------------- ARM TEST ----------------
    def arm_test(self):
        print("\n[TEST] ARM kontrolü...")

        for i, d in self.drones.items():
            try:
                d.arm_and_offboard()
                print(f"  Drone {i}: ARM gönderildi ✓")
            except Exception as e:
                print(f"  Drone {i}: ARM HATA → {e}")

        time.sleep(3)

    # ---------------- MOVE TEST ----------------
    def move_test(self):
        print("\n[TEST] Hareket kontrolü...")

        for i, d in self.drones.items():
            x, y, z = d.get_position()
            d.goto_local(x + 1.0, y + 1.0, z, max_speed=1.5)
            print(f"  Drone {i}: +1m hareket gönderildi")

        time.sleep(5)

    # ---------------- STATE MONITOR ----------------
    def monitor(self):
        print("\n[TEST] State monitoring (5 sn)...")

        for _ in range(5):
            for i, d in self.drones.items():
                pos = d.get_position()
                hdg = d.get_heading()
                print(f"  Drone {i} | POS: {pos} | HDG: {hdg}")
            print("----")
            time.sleep(1)

    # ---------------- RUN ----------------
    def run(self):
        try:
            self.check_connections()
            self.arm_test()
            self.move_test()
            self.monitor()

            print("\n=== TEST TAMAMLANDI ===")

        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self):
        for d in self.drones.values():
            try:
                d.land()
            except:
                pass

        rclpy.shutdown()
        print("[SHUTDOWN] Sistem kapatıldı")


def main():
    SwarmMavrosTest().run()


if __name__ == "__main__":
    main()
