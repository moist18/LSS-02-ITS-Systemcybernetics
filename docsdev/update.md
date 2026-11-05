# ASV-2026 — Update & Development Roadmap
> Berdasarkan analisis menyeluruh kode **ASV-2025** untuk Kontes Kapal Indonesia (KKI) 2026
>
> Ditulis: 18 Maret 2026

---

## 1. Ringkasan Kondisi Sistem Saat Ini

| Modul | Kondisi | Catatan |
|---|---|---|
| **Mission Planner** | ✅ Baik | Multi-misi, dual-trajectory, timer-based phase |
| **GPS Navigation** | ✅ Baik | Haversine + GeoPoseStamped, radius tolerance |
| **Vision Navigation** | ✅ Baik | PID yaw, RC override, LIDAR proximity trigger |
| **Perception (YOLO)** | ✅ Baik | RKNN multi-thread 3 NPU cores, model warm-up |
| **LIDAR** | ⚠️ Terbatas | Hanya output Bool, tidak ada distance/angle data |
| **Remote (Arduino)** | ✅ Baik | 10-byte serial protocol, waypoint logger + JSON GCS |
| **Compass Navigation** | ⚠️ Legacy | Masih ada, tetapi bukan navigasi utama |
| **Monitoring** | ⚠️ Minimal | Battery + CPU + GPS + Date, tidak ada alerting |
| **Underwater Camera** | ⚠️ Terpisah | Node ada di `perception.yaml` tapi kode utama belum terlihat |
| **Simulasi** | ❌ Belum ada | Tidak ada Gazebo/SITL integration |
| **Testing** | ⚠️ Minimal | Hanya 1 Detection.py di folder Testing |

### Arsitektur Komunikasi ROS 2 (Ringkas)

```
[Arduino/RC]──remote.cpp──→ /robot/mode, /robot/trajectory, /robot/arming
                            ↘
[Mission Planner] ──────→ /robot/control/{gps,compass,vision}
       ↑                              |
 /robot/position/waypoint     ←───────┼──── [GPS Nav] ──→ mavros/setpoint_position/global
                                      |
                               [Vision Nav] ──→ mavros/rc/override
                                      ↑
                               [Vision Node (YOLO RKNN)] → /robot/vision/raw_yaw
                               [LIDAR Node] → /robot/lidar (Bool)
```

---

## 2. Pengembangan Teknologi (Tech Upgrade)

### 2.1. 🤖 Perception — Model & Detection

#### **A. Upgrade Model YOLO ke YOLOv11 atau YOLOv12**
- **Kondisi saat ini**: Model `LESGO11s_x640_rknn_model` (YOLOv11s)
- **Usulan**: Coba fine-tune YOLOv11m atau YOLOv12n untuk keseimbangan speed/accuracy
- **Alasan**: YOLOv12 (rilis 2025) menggunakan attention-based backbone yang lebih akurat
  untuk objek kecil seperti buoy dari jarak jauh

```yaml
# Rekomendasi config baru di perception.yaml
vision:
  detection:
    confidence: 0.35        # Turunkan threshold (dari 0.4) untuk deteksi awal
    iou_threshold: 0.45     # Adjust NMS
    min_buoy_area: 300      # Turunkan untuk deteksi jarak lebih jauh (dari 416)
```

#### **B. Multi-Kamera dengan Stereo Depth Estimation**
- **Kondisi saat ini**: 1 kamera forward (surface) + 1 kamera underwater (terpisah)
- **Usulan**: Tambah kamera kedua dengan baseline ~15cm untuk depth estimation pasif
- **Benefit**: Estimasi jarak buoy tanpa LiDAR → failsafe yang lebih robust
- **Implementasi**: OpenCV `StereoSGBM` atau `DepthAI OAK-D Lite` (plug-and-play)

#### **C. Perbaiki Logika `BBoxVisualization.processor()`**
- **Kondisi saat ini**: `vision.py` memanggil `vis.processor()` dengan 15+ argumen — rawan bug
- **Usulan**: Refactor menjadi dataclass parameter:

```python
# Sekarang (fragile)
display_frame, raw_yaw, snapshot, _ = self.vis.processor(
    display_frame, fps, mission, docking, left_buoy, right_buoy,
    boxes, confs, cls_list, w, h, min_rc, mid_rc, max_rc, min_buoy, min_box
)

# Usulan (robust)
@dataclass
class VisionConfig:
    mission: str
    docking: str
    left_buoy: str
    right_buoy: str
    min_rc: int
    mid_rc: int
    max_rc: int
    min_buoy_area: int
    min_box_area: int

result = self.vis.processor(frame, config)
```

#### **D. Tambahkan Kalman Filter untuk Tracking Buoy**
- **Kondisi saat ini**: Deteksi per-frame, tidak ada tracking antar frame
- **Usulan**: Gunakan `cv2.KalmanFilter` atau OpenCV's `TrackerCSRT`
- **Benefit**: Lebih stabil saat buoy terhalang sesaat (gelombang, spray air)
- **Implementasi**: Wrap sebelum `vis.processor()` di `runner()` vision node

---

### 2.2. 🧭 Navigation — Peningkatan Akurasi & Robustness

#### **A. Tambahkan Anti-Integral Windup pada PID**
- **Kondisi saat ini** (`vision_nav.cpp`):
  ```cpp
  this->integral += error * dt;  // Tidak ada batas windup!
  ```
- **Usulan**: Tambahkan integral clamping:
  ```cpp
  // Tambahkan parameter batas
  this->declare_parameter<double>("vision.pid.yaw.integral_max", 0.5);

  // Di compute():
  this->integral = std::clamp(this->integral + error * dt, -integral_max, integral_max);
  ```
- **Alasan**: Saat kapal berputar lama (phase timer) integral bisa meledak dan
  menyebabkan overshoot parah

#### **B. RTK GPS — Tambahkan Status Quality Check**
- **Kondisi saat ini** (`gps_nav.cpp`): Navigasi tanpa cek status kualitas GPS
- **Usulan**: Subscribe ke `/mavros/global_position/global` dan cek `status.status >= 2`
  (RTK Fixed) sebelum navigasi

```cpp
// Tambahkan di gps_nav.cpp
if (this->current_position.status.status < sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
    RCLCPP_WARN_THROTTLE(..., "GPS quality insufficient for navigation (status: %d)");
    return;  // Jangan navigate saat GPS lemah
}
```

#### **C. Path Smoothing — Cubic Spline antar Waypoint**
- **Kondisi saat ini**: Navigasi langsung dari WP ke WP (garis lurus)
- **Usulan**: Implementasikan cubic spline interpolation antar waypoint untuk
  mengurangi turning agresif di dekat waypoint
- **Library**: `GeographicLib::Geodesic` sudah ada, tinggal extend

#### **D. Compass Navigation — Upgrade ke IMU + Sensor Fusion**
- **Kondisi saat ini**: `compass_success` dari external node, tidak transparan
- **Usulan**: Gunakan EKF (Extended Kalman Filter) melalui `robot_localization`
  ROS 2 package untuk fuse GPS + IMU + Compass

```bash
# Tambahkan ke rosdep
sudo apt install ros-humble-robot-localization
```

---

### 2.3. 🔴 LIDAR — Dari Bool ke Data Penuh

#### **A. Publish Distance + Angle (bukan hanya Bool)**
- **Kondisi saat ini** (`rplidar.cpp`): Hanya publish `Bool` ke `/robot/lidar`
- **Usulan**: Publish custom message dengan jarak dan sudut objek terdekat:

```cpp
// Buat custom msg: sensor/msg/LidarDetection.msg
# float32 distance
# int32 angle_deg
# bool detected

// Atau gunakan geometry_msgs/Point
this->lidar_point_pub->publish(closest_point);
```
- **Benefit**: Vision nav bisa gunakan jarak LIDAR untuk fine approach (bukan hanya trigger)

#### **B. LIDAR-based Obstacle Avoidance**
- **Kondisi saat ini**: LIDAR hanya sebagai proximity trigger untuk surface imaging
- **Usulan**: Tambahkan obstacle avoidance layer — jika scan mendeteksi objek
  di sudut ±45° ketika GPS navigation aktif, sementara override ke MANUAL
- **Implementasi**: Node baru `obstacle_avoidance.cpp` yang subscribe `/scan`
  dan publish `/robot/control/avoid`

---

### 2.4. 🖥️ Monitoring & Alerting

#### **A. Tambahkan Low Battery Alert**
- **Kondisi saat ini** (`battery.cpp`): Hanya publish float, tidak ada threshold alert
- **Usulan**:
  ```cpp
  // Tambahkan threshold alert di battery.cpp
  if (current_battery < 20.0f) {
      RCLCPP_WARN(..., "BATTERY LOW: %.1f%%", current_battery);
      // Publish ke /robot/alert/battery
  }
  if (current_battery < 10.0f) {
      RCLCPP_ERROR(..., "CRITICAL BATTERY: %.1f%% — Returning to base!", current_battery);
      // Trigger RTL / mission abort
  }
  ```

#### **B. Mission Health Watchdog**
- **Kondisi saat ini**: Tidak ada timeout detection jika sistem stuck di satu state
- **Usulan**: Tambahkan watchdog timer di mission planner — jika waypoint tidak
  berubah dalam N menit, trigger mission reset atau RTL

#### **C. Web Ground Control Station (GCS)**
- **Kondisi saat ini**: `remote.cpp` sudah publish ke `/robot/waypoints/update` (JSON)
- **Usulan**: Kembangkan web GCS (Next.js/Vue) yang display:
  - Peta real-time posisi + waypoints
  - Live video stream
  - Battery + mission status
  - Log panel
- **Tech**: `rosbridge_suite` sudah ada di launch file! Tinggal develop frontend

---

### 2.5. 🔧 Code Quality & Architecture

#### **A. Ganti `std::string` Mission State dengan Enum**
- **Kondisi saat ini**: Mission state adalah raw string `"Floating Ball"`, `"Surface Imaging"`, dll.
- **Usulan**: Gunakan enum class untuk type safety:

```cpp
enum class MissionState {
    IDLE,
    FLOATING_BALL,
    SURFACE_IMAGING,
    UNDERWATER_IMAGING,
    DOCKING,
    FINISH
};
```

#### **B. Unit Testing dengan `gtest` (C++) dan `pytest` (Python)**
- **Kondisi saat ini**: Tidak ada unit test untuk node C++
- **Usulan**: Tambahkan test untuk:
  - `calculate_distance()` dan `calculate_bearing()` di `gps_nav.cpp`
  - PID controller di `vision_nav.cpp`
  - `angle_in_range()` di `rplidar.cpp`

```cmake
# Tambahkan di CMakeLists.txt
ament_add_gtest(test_gps_nav test/test_gps_nav.cpp)
```

#### **C. Upgrade ke ROS 2 Iron/Jazzy (dari Humble)**
- **Kondisi saat ini**: ROS 2 Humble (EOL April 2027)
- **Usulan**: Migrate ke ROS 2 Jazzy Jalisco (LTS, release Mei 2024, EOL 2029)
- **Breaking changes minimal**: API ROS 2 Humble → Jazzy sangat kompatibel

---

## 3. Konfigurasi untuk Kontes Kapal Indonesia 2026

### 3.1. Pemahaman Misi KKI 2026

Berdasarkan struktur kode (misi 4/5/6 compass+vision+GPS), sistem dirancang untuk:

| # | Sub-misi | Navigasi Aktif | Flag Selesai |
|---|---|---|---|
| 0 | **Floating Ball** — melewati gate buoy | Vision (timer) → GPS | WP advance |
| 1 | **Floating Ball** (lanjutan) | Compass → Vision → GPS | WP advance |
| 2 | **Surface Imaging** — foto permukaan | Compass → Vision → Surface trigger → GPS | `is_surface = true` |
| 3 | **Underwater Imaging** — foto bawah | Compass → Vision → Underwater trigger → GPS | `is_underwater = true` |
| 4 | **Docking** — masuk dok | Vision → (finish timer) | `is_finish = true` |

### 3.2. ⚙️ Konfigurasi Waypoint (Pre-race)

#### **Persiapan H-1 sebelum kontes:**
1. **Survei lapangan** — tandai semua waypoint dengan GPS handheld
2. **Input waypoint** ke YAML dengan precision ≥ 8 desimal
3. **Set radius tolerance** sesuai medan:
   ```yaml
   navigation:
     radius: 2.0   # Kolam tenang: 1.5-2.0m
                   # Danau/laut: 3.0-4.0m
   ```
4. **Kalibrasi steering bias** per hull:
   ```yaml
   # Sesuaikan bias per kondisi hull (lihat konfig saat ini: -70, -45)
   vision.rc.steering_bias: [-70, -45]  # [Floating Ball, Surface Imaging]
   ```

#### **Template YAML Waypoint (gps_compass4):**
```yaml
/gps_navigation:
  ros__parameters:
    navigation:
      radius: 2.0
      waypoints_A:
        count: 4
        waypoint_0:
          latitude: -7.XXXXXXXX   # Start → Floating Ball Gate
          longitude: 110.XXXXXXXX
          altitude: 0.0
        waypoint_1:
          latitude: -7.XXXXXXXX   # Floating Ball → Surface Imaging Area
          longitude: 110.XXXXXXXX
          altitude: 0.0
        waypoint_2:
          latitude: -7.XXXXXXXX   # Surface Imaging → Underwater Imaging
          longitude: 110.XXXXXXXX
          altitude: 0.0
        waypoint_3:
          latitude: -7.XXXXXXXX   # Underwater Imaging → Docking Area
          longitude: 110.XXXXXXXX
          altitude: 0.0
```

### 3.3. 🎯 Strategi per Sub-Misi

#### **Sub-misi 1: Floating Ball (Gate Navigation)**
```
Strategi:
- Gunakan vision_nav.cpp dengan PID yaw untuk center kapal di antara buoy
- Phase timer 8-10s untuk vision sebelum switch ke GPS
- Set min_buoy_area = 300px untuk deteksi awal dari jarak jauh
- Trajectory B default (green-left, red-right) = layout paling umum KKI

Tuning PID Floating Ball:
  kp: 1.0   (base)
  ki: 0.05  (rendah, cegah windup)
  kd: 0.0001

RC values:
  throttle: 2000  (full speed approach)
  min_value: 1000
  max_value: 2000
```

#### **Sub-misi 2: Surface Imaging**
```
Strategi:
- LIDAR trigger: aktifkan use_lidar: true di config
- Set lidar.max_range: 1.5m untuk trigger foto saat cukup dekat
- Reverse timer (3-5s) untuk mundur setelah foto
- Gunakan camera_height + camera_tilt yang sudah dikalibrasi

Config kamera surface:
  distance:
    camera_height: 0.60     # Ukur fisik di kapal
    camera_tilt_deg: 45.0   # Sesuaikan angle kamera
    trigger_distance: 0.75  # 75cm dari panel

RC values Surface Imaging:
  throttle: 1700  (lebih lambat untuk positioning)
  min_value: 1100 (throttle stop)
  max_value: 1900
```

#### **Sub-misi 3: Underwater Imaging**
```
Strategi:
- Kamera underwater pastikan waterproof dan terpasang di keel
- enable_underwater_capture: true di mission planner
- Vision nav menjaga posisi over underwater panel
- Reverse timer untuk backing off setelah capture

Konfigurasi:
  phase_duration.reverse: 3.0-5.0s  (sesuaikan depth area)
```

#### **Sub-misi 4: Docking**
```
Strategi:
- Vision nav mengarahkan ke dok menggunakan deteksi warna dok
- Gunakan trajectory_switcher untuk memilih A/B dock
  (Trajectory A → red_dock, Trajectory B → green_dock)
- dock_duration timer: 3-5s untuk kapal masuk penuh

Tuning Docking:
  kp: 0.8   (lebih smooth dari floating ball)
  ki: 0.02
  kd: 0.001
  throttle: 1700  (pendekatan lambat)
  steering_bias: -45  (kompensasi hull)
```

### 3.4. 🔀 Manajemen Trajectory (A/B)

```
Trajectory dipilih via RC switch (channel part_5):
  part_5 ≤ 1500 → Trajectory A (red-left, green-right, red-dock)
  part_5 > 1500 → Trajectory B (green-left, red-right, green-dock) ← DEFAULT

Strategi kontes:
- Amati posisi buoy dan dok saat briefing
- Set trajectory via RC sebelum kapal dilepas
- Jangan ubah trajectory mid-mission!
```

### 3.5. 📡 Setup Hardware Race Day

#### **Checklist Pre-launch:**
- [ ] RTK GPS terlock (status GBAS_FIX = 2), tunggu minimal 2 menit
- [ ] ArduPilot mode = GUIDED untuk GPS nav, MANUAL untuk vision nav
- [ ] Kalibrasi kompas pada lokasi kontes (jauh dari besi)
- [ ] Test RC override di mode Manual sebelum AUTO
- [ ] Pastikan `/mavros/state` connected = true, armed = true
- [ ] Cek battery > 70% sebelum start
- [ ] Test kamera (surface + underwater) via `debug: true`
- [ ] LIDAR scan test via `/scan` topic di RViz
- [ ] Set waypoints di YAML sesuai koordinat survey
- [ ] RC switch trajectory dengan benar sebelum ARM

#### **Launch Sequence:**
```bash
# Terminal 1: Dependencies
source libs/install/setup.bash
source main/install/setup.bash

# Terminal 2: Full Mission
ros2 launch launcher mission/gps_compass4.launch.py

# Monitoring di terminal lain:
ros2 topic echo /robot/mission          # Status misi
ros2 topic echo /robot/position/waypoint # Waypoint saat ini
ros2 topic echo /robot/battery          # Battery level
ros2 topic echo /mavros/state           # FCU state
```

---

## 4. Prioritas Pengembangan (Roadmap)

### 🔴 Prioritas Tinggi (Harus untuk KKI 2026)

| # | Item | Estimasi Waktu | Impact |
|---|---|---|---|
| 1 | Anti-integral windup PID yaw | 1 hari | Stabilitas vision nav |
| 2 | GPS quality check sebelum navigate | 1 hari | Cegah navigasi saat GPS lemah |
| 3 | LIDAR publish distance + angle | 2 hari | Approach presisi surface/underwater |
| 4 | Low battery alert + mission abort | 1 hari | Keselamatan kapal |
| 5 | Kalman Filter / buoy tracker | 3 hari | Stabilitas deteksi buoy |
| 6 | Unit test untuk PID + Haversine | 2 hari | Verifikasi algo navigasi |

### 🟡 Prioritas Menengah (Sebaiknya sebelum KKI 2026)

| # | Item | Estimasi Waktu | Impact |
|---|---|---|---|
| 7 | Refactor BBoxVisualization signature | 2 hari | Maintainability |
| 8 | MissionState enum (ganti string) | 1 hari | Type safety |
| 9 | Web GCS (peta + video stream) | 1-2 minggu | Monitoring real-time |
| 10 | LIDAR obstacle avoidance | 1 minggu | Keselamatan navigasi |
| 11 | Path smoothing (spline WP) | 1 minggu | Navigasi lebih halus |

### 🟢 Prioritas Rendah (Long-term / Post-KKI 2026)

| # | Item | Estimasi Waktu | Impact |
|---|---|---|---|
| 12 | Stereo depth estimation | 2-3 minggu | Eliminasi LIDAR dependency |
| 13 | Upgrade ROS 2 Jazzy | 3-5 hari | Long-term support |
| 14 | Gazebo/SITL simulation | 2-4 minggu | Testing tanpa hardware |
| 15 | EKF sensor fusion (robot_localization) | 1 minggu | Akurasi posisi |
| 16 | Multi-vehicle coordination | 1-2 bulan | Future expansion |

---

## 5. Catatan Teknis Penting

### Quirks & Known Issues yang Ditemukan di Kode

1. **`gps_vision_planner.cpp`** — `compass_control_pub` di-declare tapi tidak ada
   compass di vision mode (baris 438 → `nullptr`). Pastikan tidak di-publish.

2. **`remote.cpp`** — baudrate 9600 bisa jadi bottleneck di 20ms loop.
   Pertimbangkan upgrade ke 115200:
   ```yaml
   arduino:
     baudrate: 115200
   ```

3. **`vision.py`** — Thread workers tidak punya queue throttling murni.
   `frame_buffer.popleft()` di worker bisa race dengan `popleft()` di processor.
   Gunakan `queue.Queue` Python yang thread-safe:
   ```python
   import queue
   self.frame_buffer = queue.Queue(maxsize=5)
   ```

4. **Waypoint file path** di `remote.cpp` menggunakan `__FILE__` compile-time path.
   Saat cross-compile atau pindah mesin, path bisa salah. Ganti dengan:
   ```cpp
   std::filesystem::path pkg_share = ament_index_cpp::get_package_share_directory("sensor");
   ```

5. **`gps_nav.cpp` logging bug** (baris 216-217):
   ```cpp
   RCLCPP_INFO(..., "GPS-NAV → Reached Waypoint %d", this->current_waypoint);
   RCLCPP_INFO(..., "GPS-NAV → Moving to Waypoint %d", this->current_waypoint + 1);
   // current_waypoint sudah di-increment, log +1 akan salah
   ```
   Simpan nilai lama sebelum increment.

---

## 6. Summary

ASV-2025 sudah memiliki fondasi yang **sangat solid** dengan arsitektur ROS 2 modular,
RKNN NPU acceleration, dual-trajectory mission planning, dan RC remote yang komprehensif.

Untuk **KKI 2026**, fokus utama adalah:
1. **Stabilitas navigasi** — PID anti-windup + GPS quality check
2. **Presisi proximity** — LIDAR dengan data distance bukan hanya Bool
3. **Buoy tracking** — Kalman filter untuk deteksi yang konsisten
4. **Monitoring** — Battery alert dan mission watchdog
5. **Konfigurasi lapangan** — Survei waypoint H-1 dan kalibrasi hardware

Dengan perbaikan ini, sistem akan jauh lebih robust dan reliable di kondisi kontes nyata.
