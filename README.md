# GM-SANCAK — Sürü İHA Sistemi

TEKNOFEST 2026 **Sürü İHA Yarışması** için **GM-SANCAK** takımı tarafından geliştirilen sürü İHA yazılım altyapısı.

Proje, ROS 2 Humble + PX4 SITL + Gazebo Classic üzerinde çalışan, **dağıtık (distributed)** mimarili bir sürü kontrol sistemidir. Şartnamenin iki ana görevini de kapsar:

- **Görev 1 — Dinamik Sürü Kabiliyeti (Otonom):** QR kod üzerinden görev bilgisi alıp dinamik olarak planlama, formasyon/manevra icrası, renkli pad'e görsel servoing ile iniş ve sürüye yeniden katılma.
- **Görev 2 — Yarı Otonom Sürü Kontrolü:** Tek bir web/joystick arayüzü üzerinden, formasyon bütünlüğü korunarak sürünün operatör girdilerine senkronize tepki vermesi.

> ⚠️ **Durum:** Bu depo geliştirme aşamasındadır. Bilinen açık noktalar ve yapılacak düzeltmeler [Bilinen Sınırlamalar](#bilinen-sınırlamalar-ve-yapılacaklar) bölümünde listelenmiştir.

---

## Takım

| İsim | Rol |
|------|-----|
| Zehra Kurttekin | Takım Kaptanı — Görev 1 (otonom sürü, QR/pad, dağıtık takip) |
| Rafet Tunahan Kayahan | Görev 2 (yarı otonom / sanal kumanda) |
| Hızır Demir | Elektrik & donanım — İHA fiziksel parçaları, güç sistemi |
| Mehmet Emin Demir | Araştırma & dokümantasyon desteği |
| Ebubekir Cihangir | Araştırma & dokümantasyon desteği |

---

## Mimari Felsefesi: Neden Dağıtık?

Şartname bölüm 5.3 net bir gereksinim koyar:

> "Dağıtık sürü algoritması kullanılması gerekmektedir. Merkezi sürü algoritmaları eksik puan olarak değerlendirilecektir."

Bu projede sürü **lider–takipçi** modeliyle çalışır, ancak merkezi bir node takipçilere "şuraya git" emri vermez:

- **Lider**, yalnızca kendi konumunu ve güncel formasyon parametrelerini (`/swarm/leader_state`) yayınlar.
- **Her takipçi**, ayrı bir ROS 2 düğümü (ayrı process) olarak bu bilgiyi dinler ve kendi indeksine göre gitmesi gereken hedef koordinatı ile manevra (pitch/roll/Z) offsetini **kendisi hesaplar**.
- Yayınlanan topic bir *komut* değil, bir *bilgi paylaşımıdır*. Karar takipçide alınır.

Bunu jüriye kanıtlamanın en hızlı yolu:

```bash
ros2 node list          # bağımsız follower node'larını gösterir
ros2 topic echo /swarm/leader_state   # paylaşılan bilginin sadece durum olduğunu gösterir
```

Lider süreci durdurulduğunda takipçiler son bilinen duruma göre PX4 üzerinde hover ederek bağımsızlıklarını korur.

---

## Klasör Yapısı

```
SANCAK/
├── mission/                  # GÖREV 1 — Otonom sürü
│   ├── swarm_mission.py          # Ana görev akışı (kalkış→QR→görev→iniş)
│   ├── drone_controller.py       # PX4 kontrol arayüzü (offboard, arm, goto, land)
│   ├── distributed_follower.py   # Dağıtık takipçi + LeaderStatePublisher
│   ├── formation.py              # Formasyon offsetleri (OKBASI / V / CIZGI)
│   ├── apf_repulsion.py          # APF tabanlı çarpışma önleme
│   ├── pad_scout.py              # Renkli pad tespiti (HSV) + koordinatör
│   ├── qr_reader.py              # pyzbar ile QR çözümleme
│   ├── set_px4_params.py         # MAVLink ile PX4 parametre ayarı
│   └── SwarmMavrosTest.py        # Bağlantı/arm/hareket test scripti
│
├── mission2/                 # GÖREV 2 — Yarı otonom / sanal kumanda
│   ├── joystick_control.html     # Web arayüzü (tüm kontroller)
│   ├── joystick_bridge.py        # WebSocket ↔ ROS 2 köprüsü
│   ├── joystick_control.py       # LİDER node (Drone 1 + leader_state yayını)
│   ├── joystick_follower.py      # TAKİPÇİ node (Drone 2 / 3, bağımsız)
│   ├── drone_controller2.py      # Sade PX4 kontrol arayüzü
│   ├── formation2.py             # Formasyon offsetleri (V / OKBASI / CIZGI)
│   └── run_mission2.sh           # Bridge + HTTP server başlatıcı
│
├── models/                   # Gazebo modelleri
│   ├── iris/                     # Aşağı bakan kameralı Iris İHA
│   ├── qr_plaka_1..6/            # 6 adet QR plakası (1.2m × 1.2m)
│   ├── blue_pad/  red_pad/       # Mavi / kırmızı iniş alanları (2m × 2m)
│   └── mavlink/
│
└── worlds/
    └── sancak_suru.world         # 6 QR + 2 pad içeren görev dünyası
```

---

## Gereksinimler

- Ubuntu 22.04
- ROS 2 Humble
- PX4-Autopilot (SITL) + Gazebo Classic
- Micro-XRCE-DDS-Agent
- `px4_msgs` (workspace içinde derlenmiş)
- Python paketleri: `opencv-python`, `pyzbar`, `cv_bridge`, `numpy`, `websockets`, `pymavlink`

```bash
pip3 install opencv-python pyzbar numpy websockets pymavlink --break-system-packages
sudo apt-get install libzbar0   # pyzbar için sistem bağımlılığı
```

İlk kez kuruyorsanız Gazebo'nun modelleri bulması için:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Desktop/SANCAK/models
```

---

## Ortak Adım: Simülasyonu Başlatma

Her iki görev de aynı simülasyon altyapısı üzerinde çalışır.

**Terminal 1 — Micro-XRCE-DDS Agent**

```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2 — PX4 SITL + Gazebo (3 Iris)**

```bash
cd ~/Desktop/SANCAK/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Desktop/SANCAK/models
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 3 -m iris -w sancak_suru
```

---

## Görev 1 — Dinamik Sürü Kabiliyeti (Otonom)

### Akış

1. Sürü, başlangıç konumlarından eş zamanlı dikey kalkış yapar.
2. İlk QR (QR1) noktasına ilerler.
3. Lider QR'ı görsel olarak okur (`qr_reader.py`), JSON görevini çözer.
4. Takım ID'sine göre sıralı görevleri icra eder: **formasyon → pitch/roll manevra → irtifa → bekleme → sürüden ayrılma/katılma**.
5. Bir sonraki QR'a *formasyon rotasyonu* ile yönelir.
6. Rota boyunca tüm dronlar mavi/kırmızı pad'leri tarar; konumları `PadCoordinator`'da biriktirilir.
7. Ayrılma görevinde ilgili drone, hedef renk pad'ine görsel servoing ile iner, disarm olur, bekler, yeniden arm olup sürüye katılır.
8. `sonraki_qr = 0` okununca son görev icra edilir, sürü home'a döner ve güvenli iniş yapar.

### QR Mesaj Formatı (örnek)

```json
{
  "qr_id": 1,
  "gorev": {
    "formasyon": { "aktif": true, "tip": "OKBASI" },
    "manevra_pitch_roll": { "aktif": false, "pitch_deg": "-10", "roll_deg": "0" },
    "irtifa_degisim": { "aktif": true, "deger": 20 },
    "bekleme_suresi_s": 3
  },
  "suruden_ayrilma": {
    "aktif": false, "ayrilacak_drone_id": null,
    "hedef_renk": null, "bekleme_suresi_s": null
  },
  "sonraki_qr": { "team_1": 4, "team_2": 3, "team_3": 5 }
}
```

### Çalıştırma

Simülasyon ayakta iken (Ortak Adım):

```bash
cd ~/Desktop/SANCAK/mission
source /opt/ros/humble/setup.bash
source ~/Desktop/SANCAK/px4_ws/install/setup.bash
python3 swarm_mission.py
```

> Görev başlangıcında PX4 hız/ivme parametrelerini eşitlemek için (opsiyonel, ayrı terminalde):
> ```bash
> python3 set_px4_params.py
> ```

### Görev 1 Bileşenleri

| Dosya | Görev |
|-------|-------|
| `swarm_mission.py` | Görev durum makinesi, kalkış/iniş, QR akışı, ayrılma/katılma |
| `drone_controller.py` | Tek İHA PX4 arayüzü; offboard heartbeat, arm, `goto_local`, `land`, stabilizasyon bekleme |
| `distributed_follower.py` | Takipçi düğüm — lider state'i dinler, kendi formasyon offsetini hesaplar, APF uygular |
| `formation.py` | OKBASI / V / CIZGI offset geometrisi, min. ayrım kontrolü, rotasyon |
| `apf_repulsion.py` | Yapay potansiyel alan ile dronlar arası itme kuvveti |
| `pad_scout.py` | HSV renk maskesi ile pad tespiti, piksel→NED dönüşümü, koordinatör |
| `qr_reader.py` | Kamera görüntüsünden pyzbar ile QR çözümleme |

---

## Görev 2 — Yarı Otonom Sürü Kontrolü

### Mimari

```
┌──────────────────────────┐
│ Tarayıcı (Operatör/Hakem) │  Tek komut kaynağı
│ joystick_control.html     │  ws://localhost:8765
└──────────┬────────────────┘
           │ WebSocket
           ▼
┌──────────────────────────┐
│ joystick_bridge.py        │  WebSocket ↔ ROS 2 köprüsü
└──────────┬────────────────┘
           │ /swarm/joystick_cmd
           ▼
┌────────────────────────────────────┐
│ joystick_control.py                │  LİDER NODE (Drone 1)
│  • /swarm/leader_state yayını       │
└──────────┬─────────────────────────┘
           │ /swarm/leader_state
   ┌───────┴────────┐
   ▼                ▼
┌────────────┐  ┌────────────┐
│ follower 2 │  │ follower 3 │  Bağımsız takipçi node'lar
│ (kendi     │  │ (kendi     │  Kendi offset + PX4 kontrolü
│  kararı)   │  │  kararı)   │
└────────────┘  └────────────┘
```

### Kontrol Modları

- **Hareket Modu:** Formasyon korunur; sürü merkezi ileri/geri, sağ/sol, yukarı/aşağı hareket eder, yaw ile döner.
- **Manevra Modu:** Sürü merkezi sabit kalır; formasyon pitch/roll ekseninde eğilir. Eğim, her takipçinin kendi Z offsetine dönüştürdüğü bir vektördür.

### Kontrol Haritası

| Tuş / Buton | Hareket Modu | Manevra Modu |
|-------------|--------------|--------------|
| **T** | Kalk / İn | — |
| **H / M** | Hareket / Manevra moduna geç | — |
| **1 / 2 / 3** | V / OKBASI / CIZGI formasyon | Aynı |
| **W / S** | İleri / geri | Pitch eğim ± |
| **A / D** | Sol / sağ | Roll eğim ± |
| **Q / E** | Yaw sol / sağ | Yaw sol / sağ |
| **R / F** | İrtifa ± | İrtifa ± |
| **V** | — | Manevrayı sıfırla |

### Çalıştırma (Görev 1'in üstüne, 3 terminal daha)

**Terminal 3 — Köprü + HTTP + Tarayıcı**

```bash
cd ~/Desktop/SANCAK/mission2
chmod +x run_mission2.sh
./run_mission2.sh
# Tarayıcı: http://localhost:8080/joystick_control.html
```

**Terminal 4 — Lider (Drone 1)**

```bash
cd ~/Desktop/SANCAK/mission2
source /opt/ros/humble/setup.bash
source ~/Desktop/SANCAK/px4_ws/install/setup.bash
python3 joystick_control.py
```

**Terminal 5 — Takipçiler (Drone 2 ve 3, bağımsız)**

```bash
cd ~/Desktop/SANCAK/mission2
source /opt/ros/humble/setup.bash
source ~/Desktop/SANCAK/px4_ws/install/setup.bash
python3 joystick_follower.py 2 &
python3 joystick_follower.py 3 &
wait
```

### Kalkış Davranışı

1. Tarayıcıda **Kalk** butonuna basılır.
2. Lider, Drone 1'i bulunduğu konumdan hedef irtifaya çıkarır ve `armed=true` yayınlar.
3. Takipçiler `armed=true` görünce her biri **bağımsız olarak** kendi PX4'ünü kalkış için komutlar.
4. 3 drone dikey olarak yükselir — kalkışta formasyon kurulmaz, başlangıç dizilimi korunur.
5. V / OKBASI / CIZGI butonuna basılınca formasyon kurulur ve takipçiler offsetlerine geçer.

### Leader State (paylaşılan bilgi)

```json
{
  "leader_x": 5.2, "leader_y": 3.1, "leader_alt": 8.0,
  "yaw": 0.5, "formation": "V", "dist": 7.0,
  "active": true, "pitch_egim": 0.0, "roll_egim": 0.0,
  "armed": true
}
```

---

## ROS 2 Topic'leri

| Topic | Tür | Açıklama |
|-------|-----|----------|
| `/swarm/leader_state` | `std_msgs/String` (JSON) | Lider konumu + formasyon parametreleri (dağıtık takip) |
| `/swarm/formation_params` | `std_msgs/String` (JSON) | Formasyon tipi/mesafe güncellemeleri (Görev 1) |
| `/swarm/pad_found` | `std_msgs/String` (JSON) | Tespit edilen pad koordinatları (Görev 1) |
| `/swarm/joystick_cmd` | `std_msgs/String` (JSON) | Web arayüzünden gelen kumanda komutları (Görev 2) |
| `/swarm/joystick_feedback` | `std_msgs/String` | Mission → Web geri bildirim (Görev 2) |
| `/px4_N/fmu/in/...` | `px4_msgs` | Offboard, setpoint, vehicle_command |
| `/px4_N/fmu/out/...` | `px4_msgs` | Local position, vehicle status |
| `/uavN/down_camera/.../image_raw` | `sensor_msgs/Image` | Aşağı bakan kamera (QR + pad) |

---

## Simülasyon Ortamı

- **6 QR plakası** (`sancak_suru.world`), 1.2m × 1.2m, sabit konumlarda.
- **2 iniş pad'i:** mavi (16, 8) ve kırmızı (20, 2), 2m × 2m.
- **Kamera:** aşağı bakan, 640×480, 60° FOV (şartname sınırı 90°'nin altında).

---

## Bilinen Sınırlamalar ve Yapılacaklar

Aşağıdaki maddeler bilinçli olarak listelenmiştir; geliştirme bunlar üzerinde devam etmektedir.

- [ ] **Görev 1 dağıtıklığını güçlendirmek:** `swarm_mission.py` içindeki manevra ve bazı iniş adımları takipçilere doğrudan komut veriyor. Bunları `distributed_follower` üzerinden geçirmek.
- [ ] **Hardcoded koordinatları kaldırmak:** `SPAWN_NED` ve `NED_X_OFFSET` simülasyona özgüdür; kalkışı anlık konumdan başlatıp QR konumlarını runtime config'ten okumak.
- [ ] **Kamera namespace tutarlılığı:** Statik `iris.sdf` kamera topic'inin her drone için ayrı namespace'e yayınladığından emin olmak.
- [ ] **Görev 2'ye çarpışma önleme + stabilizasyon** eklemek (formasyon geçişlerinde).
- [ ] **PX4 fail-safe parametrelerini** açıkça ayarlamak (RTL / Land — hakem direktifine göre).
- [ ] **N-drone jeneriklik:** Algoritmaları sabit 3 drone yerine değişken sayıya uyarlamak (şartname "istenilen sayıda İHA").

---

## Notlar

- Yazılım yalnızca görevi başlatma komutu dışında dış müdahale gerektirmeyecek şekilde tasarlanmıştır (şartname görev kuralları).
- Sahada her İHA için ayrı RC kumanda, ayrı pilot ve kill switch zorunluluğu donanım tarafında karşılanmalıdır.
- Açık kaynak kütüphaneler (PX4, ROS 2, OpenCV, pyzbar) kullanılmıştır; yasaklı kod/firmware bulunmamaktadır.

---

*GM-SANCAK — TEKNOFEST 2026 Sürü İHA Yarışması*
