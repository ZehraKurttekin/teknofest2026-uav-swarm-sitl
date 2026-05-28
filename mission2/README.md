# SANCAK — Görev 2 · Yarı Otonom Sürü Kontrolü (DAĞITIK MİMARİ)

TEKNOFEST 2026 şartnamesi bölüm 5.3:
> "Dağıtık sürü algoritması kullanılması gerekmektedir.
> Merkezi sürü algoritmaları eksik puan olarak değerlendirilecektir."

Bu mimari bu gereksinimi karşılar.

## Mimari

```
┌──────────────────────────┐
│  Tarayıcı (Hakem)        │  Web arayüz — TEK komut kaynağı
│  joystick_control.html   │  ws://localhost:8765
└──────────┬───────────────┘
           │ WebSocket
           ▼
┌──────────────────────────┐
│ joystick_bridge.py       │  WebSocket ↔ ROS2 köprüsü
└──────────┬───────────────┘
           │ /swarm/joystick_cmd
           ▼
┌──────────────────────────────────────┐
│ joystick_control.py                  │  LİDER NODE
│  • SADECE Drone 1 kontrolü           │  (Drone 1 sürücüsü)
│  • /swarm/leader_state yayını (10Hz) │
└──────────┬───────────────────────────┘
           │ /swarm/leader_state
           │ (lider pozisyonu + formasyon + eğim)
           ▼
┌──────────────────────┐    ┌──────────────────────┐
│ joystick_follower.py │    │ joystick_follower.py │
│  Drone 2 (bağımsız)  │    │  Drone 3 (bağımsız)  │
│  • Lider state oku   │    │  • Lider state oku   │
│  • Kendi offset hesap│    │  • Kendi offset hesap│
│  • Kendi PX4'ünü sür │    │  • Kendi PX4'ünü sür │
└──────────────────────┘    └──────────────────────┘
```

## Neden Bu Dağıtık?

**Merkezi mimari (yasak):**
- Tek bir node 3 dronun hedeflerini hesaplar
- Her drona doğrudan "şuraya git" komutu gönderir
- O node düşerse tüm sistem çöker

**Dağıtık mimari (bu proje):**
- Lider sadece kendi hareketini hesaplar
- Takipçiler lider bilgisini okuyup **kendi kararlarını verir**
- Her takipçi ayrı bir ROS2 node — ayrı process
- Lider düşerse takipçiler son komutla hover eder (PX4 failsafe)
- Manevra eğimi (pitch/roll) takipçinin **kendi** hesapladığı Z offset

## Dosyalar

- `joystick_control.html` — Web arayüzü (tüm kontroller)
- `joystick_bridge.py` — WebSocket ↔ ROS2 köprüsü
- `joystick_control.py` — **LİDER** node (sadece Drone 1)
- `joystick_follower.py` — **TAKİPÇİ** node (Drone 2 veya 3)
- `run_mission2.sh` — Bridge + HTTP başlatıcı

`mission/` klasöründen kullanılanlar:
- `drone_controller2.py` — PX4 kontrol arayüzü
- `formation2.py` — Formasyon offset hesabı (V, OKBASI, CIZGI)

## Kontrol Haritası

| Tuş | HAREKET Modu | MANEVRA Modu |
|-----|--------------|---------------|
| **T** | Kalk / İn | — |
| **H / M** | Mod değiştir | — |
| **1 / 2 / 3** | V / OKBASI / CIZGI | Aynı |
| **W / S** | Sürü ileri / geri | Formasyon pitch eğim |
| **A / D** | Sürü sol / sağ | Formasyon roll eğim |
| **Q / E** | Yaw sol / sağ | Yaw sol / sağ |
| **R / F** | İrtifa ± | İrtifa ± |
| **V** | — | Manevra sıfırla |

## Kurulum

```bash
# Bağımlılık
pip3 install websockets --break-system-packages

# mission/ altında drone_controller2.py ve formation2.py olduğundan emin ol
ls ~/Desktop/SANCAK/mission/drone_controller2.py
ls ~/Desktop/SANCAK/mission/formation2.py
```

## Çalıştırma

**5 terminal gerekli:**

### Terminal 1 — MicroXRCEAgent
```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 2 — PX4 SITL + Gazebo
```bash
cd ~/Desktop/SANCAK/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Desktop/SANCAK/models
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 3 -m iris -w sancak_suru
```

### Terminal 3 — Bridge + HTTP + Tarayıcı
```bash
cd ~/Desktop/SANCAK/mission2
chmod +x run_mission2.sh
./run_mission2.sh
```

### Terminal 4 — LİDER (Drone 1 kontrolcüsü)
```bash
cd ~/Desktop/SANCAK/mission2
source /opt/ros/humble/setup.bash
source ~/Desktop/SANCAK/px4_ws/install/setup.bash
python3 joystick_control.py
```

### Terminal 5 — TAKİPÇİLER (Drone 2 ve 3 ayrı node olarak)

**Seçenek A: İki ayrı terminal**
```bash
# Terminal 5a
cd ~/Desktop/SANCAK/mission2
source /opt/ros/humble/setup.bash
source ~/Desktop/SANCAK/px4_ws/install/setup.bash
python3 joystick_follower.py 2

# Terminal 5b
cd ~/Desktop/SANCAK/mission2
source /opt/ros/humble/setup.bash
source ~/Desktop/SANCAK/px4_ws/install/setup.bash
python3 joystick_follower.py 3
```

**Seçenek B: Tek komutla arka plan**
```bash
cd ~/Desktop/SANCAK/mission2
source /opt/ros/humble/setup.bash
source ~/Desktop/SANCAK/px4_ws/install/setup.bash
python3 joystick_follower.py 2 &
python3 joystick_follower.py 3 &
wait
```

## Kalkış Davranışı

1. Tarayıcıda **Kalk** butonuna bas
2. LİDER (`joystick_control.py`): Drone 1'i bulunduğu konumdan 8m'ye çıkarır
3. LİDER `/swarm/leader_state` topic'ine `armed=True` yayınlamaya başlar
4. TAKİPÇİLER (`joystick_follower.py × 2`): `armed=True` gördüklerinde **her biri bağımsız olarak** kendi PX4'ünü kalkış için komutlar
5. 3 drone bulundukları yerde dikey 8m'ye çıkar — **formasyon kurulmaz**
6. Tarayıcıda **V/Okbaşı/Çizgi** butonuna basınca `formation_active=True` olur
7. Her takipçi `formation2.py`'den kendi offset'ini hesaplayıp lidere göre hedef pozisyonuna gider

## Dağıtık Olduğunu Nasıl Gösteriyoruz?

Jüriye bunu net göstermek için:

```bash
# Tüm node'ları listele
ros2 node list
```

Beklenen çıktı:
```
/drone_controller_1         ← Lider PX4 bağlantısı
/drone_controller_2         ← Takipçi 1 PX4 bağlantısı
/drone_controller_3         ← Takipçi 2 PX4 bağlantısı
/joystick_bridge           ← WebSocket köprüsü
/joystick_leader           ← Lider mantığı
/joystick_follower_2       ← BAĞIMSIZ takipçi node
/joystick_follower_3       ← BAĞIMSIZ takipçi node
```

**`joystick_follower_2` ve `joystick_follower_3` ayrı süreçlerde çalışır** — merkezi bir kontrolcüye bağımlı değildir. Leader state topic'ini dinler, kendi karar verir.

```bash
# Leader state yayınını izle
ros2 topic echo /swarm/leader_state
```

JSON içeriği:
```json
{
  "leader_x": 5.2, "leader_y": 3.1, "leader_alt": 8.0,
  "yaw": 0.5, "formation": "V", "dist": 7.0,
  "active": true,
  "pitch_egim": 0.0, "roll_egim": 0.0,
  "armed": true
}
```

Bu topic "ne yapılacak" emri değil, **bilgi paylaşımı**. Her takipçi bu bilgiyi okuyup kendi kararını veriyor.

## Sorun Giderme

**Takipçiler kalkmıyor:**
- `ros2 topic echo /swarm/leader_state` ile `armed: true` gelip gelmediğini kontrol et
- Takipçi node'lar çalışıyor mu? `ros2 node list | grep follower`

**Formasyon kurulmuyor:**
- Tarayıcıda formasyon butonuna basıldı mı?
- Leader state'te `active: true` var mı?

**Takipçiler yanlış yere gidiyor:**
- `formation2.py` import doğru mu?
- `my_index` hesabı (drone_id - 1) doğru mu?
