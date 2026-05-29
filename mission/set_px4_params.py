#!/usr/bin/env python3
"""
set_px4_params.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Overshoot düzeltmesi için PX4 parametrelerini MAVLink üzerinden ayarlar.
SITL çalışırken bu scripti ayrı bir terminalde çalıştır.

Kullanım:
  python3 set_px4_params.py          # Tüm drone'lar (1,2,3)
  python3 set_px4_params.py --drone 1  # Sadece drone 1

Gereksinim:
  pip3 install pymavlink --break-system-packages
"""

import argparse
import time
from pymavlink import mavutil

# ─── Overshoot için önerilen parametre değerleri ──────────
# Orijinal değerlerden daha düşük kazançlar = daha yavaş/yumuşak hareket
PARAMS = {
    # ─── Yatay (XY) hareket ───────────────────────────
    'MPC_XY_P':           0.5,   # pozisyon kazancı — yumuşak
    'MPC_XY_VEL_P_ACC':   1.2,   # hız P
    'MPC_XY_VEL_I_ACC':   0.2,   # hız I
    'MPC_XY_VEL_D_ACC':   0.1,   # hız D
    'MPC_XY_VEL_MAX':     2.0,   # max yatay hız (m/s)
    'MPC_JERK_MAX':       4.0,   # max jerk
    'MPC_ACC_HOR':        2.0,   # yatay ivme limiti
    'MPC_DECEL_HOR_SLOW': 5.0,   # fren ivmesi — overshoot azalır

    # ─── Dikey (Z) hareket ────────────────────────────
    'MPC_Z_P':            1.0,   # pozisyon kazancı (varsayılan)
    'MPC_Z_VEL_MAX_DN':   1.5,   # max aşağı hız (m/s) — zıplama azalır
    'MPC_Z_VEL_MAX_UP':   2.0,   # max yukarı hız (m/s)
    'MPC_ACC_DOWN_MAX':   2.5,   # aşağı ivme limiti
    'MPC_ACC_UP_MAX':     3.0,   # yukarı ivme limiti

    # ─── İniş ─────────────────────────────────────────
    'MPC_LAND_SPEED':     0.5,   # son iniş hızı (m/s) — zıplama önleyici
    'MPC_LAND_ALT1':      4.0,   # bu irtifanın altında yavaşlamaya başla
    'MPC_LAND_ALT2':      1.0,   # bu irtifanın altında MPC_LAND_SPEED
    'LNDMC_XY_VEL_MAX':   0.3,   # iniş tespitinde izin verilen yatay hız
}

# Her drone'un UDP portu (SITL multi-vehicle)
DRONE_PORTS = {
    1: 14540,
    2: 14541,
    3: 14542,
}


def set_params_for_drone(drone_id: int, params: dict):
    port = DRONE_PORTS[drone_id]
    print(f"\n{'─'*45}")
    print(f"Drone {drone_id} bağlanıyor → UDP port {port}...")

    try:
        mav = mavutil.mavlink_connection(f'udp:localhost:{port}')
        mav.wait_heartbeat(timeout=10)
        print(f"Drone {drone_id}: Bağlantı kuruldu ✓")
    except Exception as e:
        print(f"Drone {drone_id}: Bağlantı hatası! {e}")
        print(f"  → SITL çalışıyor mu? Port {port} açık mı?")
        return False

    success_count = 0
    for param_name, param_value in params.items():
        # Parametre gönder
        mav.mav.param_set_send(
            mav.target_system,
            mav.target_component,
            param_name.encode('utf-8'),
            param_value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        time.sleep(0.1)

        # ACK bekle
        msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if msg and msg.param_id.rstrip('\x00') == param_name:
            print(f"  ✓ {param_name} = {param_value}")
            success_count += 1
        else:
            print(f"  ✗ {param_name} — ACK gelmedi, tekrar deniyor...")
            # Bir kez daha dene
            mav.mav.param_set_send(
                mav.target_system,
                mav.target_component,
                param_name.encode('utf-8'),
                param_value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            time.sleep(0.2)
            msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
            if msg:
                print(f"  ✓ {param_name} = {param_value} (2. denemede)")
                success_count += 1
            else:
                print(f"  ✗ {param_name} — ayarlanamadı!")

    print(f"Drone {drone_id}: {success_count}/{len(params)} parametre ayarlandı.")
    mav.close()
    return success_count == len(params)


def main():
    parser = argparse.ArgumentParser(description='PX4 overshoot parametre düzeltmesi')
    parser.add_argument('--drone', type=int, choices=[1, 2, 3],
                        help='Sadece belirtilen drone (yoksa tümü)')
    args = parser.parse_args()

    print("=" * 45)
    print("  PX4 Overshoot Düzeltme Parametreleri")
    print("=" * 45)
    print("\nAyarlanacak parametreler:")
    for k, v in PARAMS.items():
        print(f"  {k}: {v}")

    if args.drone:
        drones = [args.drone]
    else:
        drones = [1, 2, 3]

    results = {}
    for drone_id in drones:
        results[drone_id] = set_params_for_drone(drone_id, PARAMS)

    print(f"\n{'='*45}")
    print("Sonuç:")
    for drone_id, ok in results.items():
        status = "✓ Başarılı" if ok else "✗ Hata"
        print(f"  Drone {drone_id}: {status}")

    print("\nNOT: Parametreler sadece bu oturum için geçerli.")
    print("Kalıcı yapmak için ROMFS init dosyasına ekleyin.")
    print("  ~/Desktop/SANCAK/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/")


if __name__ == '__main__':
    main()
