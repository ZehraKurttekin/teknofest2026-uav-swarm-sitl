#!/usr/bin/env python3
"""
apf_repulsion.py — Artificial Potential Field çarpışma önleme

Her drone'un formasyon hedefine ek olarak,
diğer dronlardan gelen itici kuvvet hesaplanır.

İtme formülü:
  d < d_safe  →  F_rep = K_REP * (1/d - 1/d_safe) / d²
  d >= d_safe →  F_rep = 0

K_REP ve d_safe parametreleri ayarlanabilir.
Yumuşak itme için max ivme sınırlaması vardır.
"""

import math

# Parametreler
SAFE_DIST     = 5.0   # Dronlar arası güvenli mesafe (m)
INFLUENCE     = 7.0   # Bu mesafenin altında itme devreye girer
K_REP         = 15.0   # İtme kuvveti kazancı
MAX_PUSH      = 3.0   # Maksimum itme mesafesi (m) — güvenlik


def compute_repulsion(my_pos, other_positions):
    """
    my_pos           : (x, y, z) — bu drone'un pozisyonu
    other_positions  : [(x, y, z), ...] — diğer dronların pozisyonları
    
    Returns: (dx, dy, dz) — hedefe eklenecek offset vektörü
    """
    if not other_positions:
        return (0.0, 0.0, 0.0)

    push_x, push_y, push_z = 0.0, 0.0, 0.0
    mx, my, mz = my_pos

    for ox, oy, oz in other_positions:
        dx = mx - ox
        dy = my - oy
        dz = mz - oz
        d  = math.sqrt(dx*dx + dy*dy + dz*dz)

        if d < 0.01:  # Aynı nokta — sayısal koruma
            continue
        if d >= INFLUENCE:  # Yeterince uzak, itme yok
            continue

        # İtme büyüklüğü — yakınlık arttıkça hızla büyür
        # Clamp: d < 0.5m olsa bile patlamasın
        d_eff = max(d, 0.5)
        rep   = K_REP * (1.0/d_eff - 1.0/INFLUENCE) / (d_eff * d_eff)

        # Normalize yön
        nx = dx / d
        ny = dy / d
        nz = dz / d

        push_x += rep * nx
        push_y += rep * ny
        push_z += rep * nz * 0.3  # Z ekseninde itme daha zayıf (irtifa karışmasın)

    # Maksimum itme sınırı
    push_mag = math.sqrt(push_x**2 + push_y**2 + push_z**2)
    if push_mag > MAX_PUSH:
        scale    = MAX_PUSH / push_mag
        push_x  *= scale
        push_y  *= scale
        push_z  *= scale

    return (push_x, push_y, push_z)


def apply_repulsion(target_pos, my_pos, other_positions):
    """
    Formasyon hedefine itme eklenmiş son hedefi döner.
    
    target_pos : (x, y, z) — formasyon hedefi
    my_pos     : (x, y, z) — drone'un gerçek anlık pozisyonu
    other_positions : [(x, y, z), ...] — diğer dronlar
    
    Returns: (new_x, new_y, new_z)
    """
    tx, ty, tz = target_pos
    px, py, pz = compute_repulsion(my_pos, other_positions)
    return (tx + px, ty + py, tz + pz)
