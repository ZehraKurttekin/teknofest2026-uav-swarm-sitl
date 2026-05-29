#!/usr/bin/env python3
"""
formation.py

Mesafe güncellendi:
  MIN_SAFE_DIST = 4.0m  (çarpışma tampon mesafesi)
  Varsayılan formasyon mesafesi swarm_mission'da 8.0m olarak ayarlanacak.
"""
import math
from typing import List, Tuple

MIN_SAFE_DIST = 4.0   # Minimum güvenli mesafe — çarpışma önleme
SQRT3_2 = math.sqrt(3) / 2


def rotate(dx: float, dy: float, yaw: float) -> Tuple[float, float]:
    return (
        dx * math.cos(yaw) - dy * math.sin(yaw),
        dx * math.sin(yaw) + dy * math.cos(yaw)
    )


def get_formation_offsets(tip: str, n_drones: int,
                          mesafe: float, yaw: float = 0.0) -> List[Tuple[float, float, float]]:
    d = max(mesafe, MIN_SAFE_DIST)
    h = d * SQRT3_2

    if tip == "OKBASI":
        # Lider geride, takipçiler önde — eşkenar üçgen
        raw = [(0.0, 0.0), (+h, +d/2), (+h, -d/2)]
    elif tip == "V":
        # Lider önde, takipçiler arkada — eşkenar üçgen
        raw = [(0.0, 0.0), (-h, +d/2), (-h, -d/2)]
    elif tip == "CIZGI":
        # Lider ortada, Drone2 ve Drone3 ayni satirda (cizgi)
        # rotate(+d, 0, yaw) -> yaw yonunde yan yana
        # yaw=0: Drone1(0,0), Drone2(+d,0), Drone3(-d,0)
        raw = [(0.0, 0.0), (+d, 0.0), (-d, 0.0)]
    else:
        return [(0.0, 0.0, float(i)*2.0) for i in range(n_drones)]

    offsets = []
    for rdx, rdy in raw[:n_drones]:
        rx, ry = rotate(rdx, rdy, yaw)
        offsets.append((rx, ry, 0.0))
    return offsets


def formation_positions(leader_x: float, leader_y: float, leader_z: float,
                        tip: str, n_drones: int, mesafe: float,
                        yaw: float = 0.0) -> List[Tuple[float, float, float]]:
    offsets = get_formation_offsets(tip, n_drones, mesafe, yaw)
    return [(leader_x + dx, leader_y + dy, leader_z + dz)
            for dx, dy, dz in offsets]


def interpolate_formation(current: List[Tuple[float, float, float]],
                          target: List[Tuple[float, float, float]],
                          t: float) -> List[Tuple[float, float, float]]:
    result = []
    for (cx, cy, cz), (tx, ty, tz) in zip(current, target):
        result.append((cx + (tx-cx)*t, cy + (ty-cy)*t, cz + (tz-cz)*t))
    return result


def check_min_separation(positions: List[Tuple[float, float, float]],
                         min_dist: float = MIN_SAFE_DIST) -> Tuple[bool, float]:
    min_found = float('inf')
    for i in range(len(positions)):
        for j in range(i+1, len(positions)):
            x1, y1, z1 = positions[i]
            x2, y2, z2 = positions[j]
            d = math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
            if d < min_found:
                min_found = d
    return (min_found >= min_dist, min_found)


def compute_yaw_to_target(cx: float, cy: float,
                          tx: float, ty: float) -> float:
    return math.atan2(ty - cy, tx - cx)
