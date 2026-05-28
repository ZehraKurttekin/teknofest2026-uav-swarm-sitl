#!/usr/bin/env python3
import math

def get_formation_offsets(tip, n_drones, mesafe, yaw=0.0):
    offsets = []

    if tip == "OKBASI":
        offsets.append((0.0, 0.0, 0.0))
        angles = [math.pi + math.pi/4, math.pi - math.pi/4]
        for i in range(min(n_drones - 1, len(angles))):
            a = angles[i] + yaw
            offsets.append((mesafe * math.cos(a), mesafe * math.sin(a), 0.0))

    elif tip == "V":
        offsets.append((0.0, 0.0, 0.0))
        for i in range(1, n_drones):
            side = 1 if i % 2 == 1 else -1
            step = (i + 1) // 2
            dx = -mesafe * step * 1.5 * math.cos(yaw) - mesafe * step * side * 0.3 * math.sin(yaw)
            dy = -mesafe * step * 1.5 * math.sin(yaw) + mesafe * step * side * 1.2 * math.cos(yaw)
            offsets.append((dx, dy, 0.0))

    elif tip == "CIZGI":
        # Tum dronelari esit aralikli diz
        # n=3 icin: -mesafe, 0, +mesafe
        toplam = n_drones - 1
        for i in range(n_drones):
            offset_idx = i - toplam / 2.0
            dx = offset_idx * mesafe * math.cos(yaw)
            dy = offset_idx * mesafe * math.sin(yaw)
            offsets.append((dx, dy, 0.0))

    elif tip == "UCGEN":
        offsets.append((0.0, 0.0, 0.0))
        angles = [math.pi/2, math.pi*7/6, math.pi*11/6]
        for i in range(min(n_drones - 1, len(angles))):
            a = angles[i] + yaw
            offsets.append((mesafe * math.cos(a), mesafe * math.sin(a), 0.0))

    else:
        for i in range(n_drones):
            offsets.append((0.0, 0.0, float(i)))

    return offsets[:n_drones]


def formation_positions(lx, ly, lz, tip, n, mesafe, yaw=0.0):
    offsets = get_formation_offsets(tip, n, mesafe, yaw)
    return [(lx + dx, ly + dy, lz + dz) for dx, dy, dz in offsets]


if __name__ == "__main__":
    for tip in ["OKBASI", "V", "CIZGI"]:
        pos = formation_positions(0, 0, 8, tip, 3, 5.0)
        print(f"{tip}: {pos}")
