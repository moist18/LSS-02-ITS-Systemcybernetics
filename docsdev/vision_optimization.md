# Optimasi Vision & Navigation ASV-2025

## Daftar Isi
1. [Multi-Object Tracking dengan ByteTrack](#1-multi-object-tracking-dengan-bytetrack)
2. [Alternatif Model Detection](#2-alternatif-model-detection)
3. [Optimasi Pipeline Vision](#3-optimasi-pipeline-vision)
4. [Heading Control: Compass vs Alternatif](#4-heading-control-compass-vs-alternatif)
5. [Strategi Navigation Terpadu](#5-strategi-navigation-terpadu)

---

## 1. Multi-Object Tracking dengan ByteTrack

### Masalah Saat Ini

Sistem vision (`vision.py`) saat ini hanya melakukan **per-frame detection** tanpa tracking. Setiap frame, YOLO mendeteksi objek dari nol — tidak ada hubungan antara deteksi frame N dan frame N+1. Akibatnya:

- **ID flickering**: buoy yang sama bisa "hilang" dan "muncul" terus-menerus
- **Yaw jittery**: `raw_yaw` berubah drastis antar frame karena deteksi tidak stabil
- **False positive lolos**: tidak ada temporal filtering

### Mengapa ByteTrack?

| Tracker | Kecepatan | Akurasi | Cocok ARM/NPU | Kompleksitas |
|---------|:---------:|:-------:|:--------------:|:------------:|
| **ByteTrack** | ✅✅✅ Sangat cepat | ✅✅ | ✅ CPU-only | Rendah |
| DeepSORT | ❌ Lambat (perlu ReID) | ✅✅✅ | ❌ Berat | Tinggi |
| BoT-SORT | ⚠️ Sedang | ✅✅✅ | ⚠️ | Tinggi |
| OC-SORT | ✅✅ Cepat | ✅✅ | ✅ | Sedang |
| Norfair | ✅✅ Cepat | ✅ | ✅ | Rendah |

**ByteTrack adalah pilihan terbaik** karena:
- Tidak perlu model ReID tambahan (murni Kalman filter + IoU matching)
- Overhead CPU minimal (~1ms per frame)
- Kompatibel penuh dengan Orange Pi 5 Pro
- Memanfaatkan **deteksi low-confidence** yang biasanya dibuang

### Cara Kerja ByteTrack

```
Frame N: YOLO deteksi → [buoy_A conf:0.85, buoy_B conf:0.72, noise conf:0.35]
                              ↓
ByteTrack:
  1. Match high-conf deteksi (>0.5) dengan track yang ada → IoU matching
  2. Track yang tidak match → coba match dengan low-conf deteksi (0.1-0.5)
  3. Track yang masih tidak match → keep alive beberapa frame (grace period)
  4. Deteksi yang tidak match → buat track baru
                              ↓
Output: [Track_1: buoy_A (ID konsisten), Track_2: buoy_B (ID konsisten)]
```

### Implementasi di ASV-2025

#### Install dependency:
```bash
pip install bytetrack  # atau supervision yang include ByteTrack
# Alternatif ringan:
pip install supervision
```

#### Modifikasi `vision.py` — tambah tracker:

```python
# === TAMBAHAN IMPORT ===
import supervision as sv
# atau implementasi manual ByteTrack:
# from perception.utils.bytetrack import BYTETracker

class VisionNode(Node):
    def __init__(self):
        # ... existing code ...

        # === TAMBAHAN: ByteTrack Tracker ===
        self.tracker = sv.ByteTrack(
            track_activation_threshold=0.4,   # min conf untuk track baru
            lost_track_buffer=15,              # frame grace period sebelum hapus track
            minimum_matching_threshold=0.8,    # IoU threshold untuk matching
            frame_rate=20                      # sesuai FPS kamera
        )

        # Track history untuk smoothing
        self.track_positions = {}  # {track_id: deque of (x, y)}
        self.SMOOTH_WINDOW = 5    # jumlah frame untuk smoothing
```

#### Modifikasi `processor()` — gunakan tracking:

```python
def processor(self, input_frame):
    # ... existing frame buffer code ...

    with self.result_lock:
        result = self.latest_result

    if result and hasattr(result, 'boxes') and len(result.boxes) > 0:
        # === BARU: Convert ke supervision format ===
        detections = sv.Detections(
            xyxy=result.boxes.xyxy.cpu().numpy(),
            confidence=result.boxes.conf.cpu().numpy(),
            class_id=result.boxes.cls.cpu().numpy().astype(int)
        )

        # === BARU: Update tracker ===
        tracked = self.tracker.update_with_detections(detections)

        # === BARU: Gunakan tracked detections (bukan raw detections) ===
        if len(tracked) > 0:
            display_frame, raw_yaw, snapshot, _ = self.vis.processor(
                display_frame, self.fps, self.mission,
                self.docking, self.left_buoy, self.right_buoy,
                tracked.xyxy.tolist(),
                tracked.confidence.tolist(),
                tracked.class_id.tolist(),
                # ... parameter lainnya sama ...
            )

            # === BARU: Smooth yaw menggunakan track history ===
            raw_yaw = self.smooth_yaw(raw_yaw)
    # ... rest of code ...

def smooth_yaw(self, current_yaw, window=5):
    """Exponential Moving Average untuk stabilkan yaw"""
    if not hasattr(self, '_yaw_history'):
        self._yaw_history = []

    self._yaw_history.append(current_yaw)
    if len(self._yaw_history) > window:
        self._yaw_history.pop(0)

    # EMA: frame terbaru punya bobot lebih besar
    weights = np.exp(np.linspace(0, 1, len(self._yaw_history)))
    weights /= weights.sum()
    return int(np.dot(self._yaw_history, weights))
```

### Keuntungan Setelah ByteTrack

| Aspek | Sebelum | Sesudah |
|-------|---------|---------|
| Stabilitas deteksi | Flickering tiap frame | Konsisten dengan track ID |
| Yaw smoothness | Jittery ±50-100 RC | Smooth ±10-20 RC |
| Object lost handling | Langsung hilang | Grace period 15 frame |
| Buoy identification | Hanya per-frame | Temporal consistency |

---

## 2. Alternatif Model Detection

### Analisis Model Saat Ini

Sistem menggunakan **YOLOv8s** dikonversi ke RKNN untuk NPU Orange Pi 5 Pro. Konfigurasi: `imgsz=640, conf=0.4, iou=0.4`.

### Opsi Model Lain

#### A. YOLOv8n (Nano) — Lebih Cepat

```
Keuntungan: +40% FPS (dari ~20 FPS → ~28 FPS)
Kekurangan: -5-10% mAP
Cocok jika: FPS lebih penting dari akurasi (misal: Floating Ball dengan speed tinggi)
```

Cara konversi:
```bash
# Export dari Ultralytics ke ONNX
yolo export model=LESGOv8n.pt format=onnx imgsz=640

# Convert ONNX ke RKNN (gunakan rknn-toolkit2)
python3 convert_to_rknn.py --model LESGOv8n.onnx --target rk3588
```

#### B. YOLOv10s — Tanpa NMS

```
Keuntungan: Tidak perlu NMS post-processing → latency lebih rendah
Kekurangan: Belum mature di RKNN, perlu custom export
Cocok jika: Ingin reduce post-processing overhead
```

#### C. RT-DETR (Real-Time DETR) — Transformer-based

```
Keuntungan: Akurasi tinggi tanpa NMS, baik untuk objek kecil
Kekurangan: Lebih berat, belum optimal di NPU RK3588
Cocok jika: Akurasi prioritas utama dan hardware upgrade
```

#### D. YOLOv8s dengan Input Lebih Kecil (imgsz=416)

```
Keuntungan: +30% FPS dengan model yang sama, tanpa retrain
Kekurangan: -5% akurasi pada objek jauh
Cocok jika: Quick win tanpa effort retrain
```

Ubah di `perception.yaml`:
```yaml
vision:
  detection:
    image_size: 416   # dari 640 → 416
```

### Rekomendasi

| Skenario | Model | Input Size |
|----------|-------|:----------:|
| **Speed priority** (Floating Ball) | YOLOv8n RKNN | 416 |
| **Balanced** (default) | YOLOv8s RKNN | 640 |
| **Accuracy priority** (Docking) | YOLOv8s RKNN | 640 |

**Dual-model strategy**: Load 2 model berbeda, switch berdasarkan mission:
```python
if self.mission == "Floating Ball":
    model = self.model_fast   # YOLOv8n@416
else:
    model = self.model_accurate  # YOLOv8s@640
```

---

## 3. Optimasi Pipeline Vision

### A. Reduce Latency — Camera Pipeline

Saat ini kamera sudah cukup optimal (MJPEG, buffer=1). Tambahan:

```python
# Tambah di camera setup (vision.py __init__)
self.cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)    # Manual exposure
self.cam.set(cv2.CAP_PROP_EXPOSURE, -6)         # Fast exposure
self.cam.set(cv2.CAP_PROP_AUTO_WB, 0)           # Disable auto white balance
```

### B. ROI Cropping — Fokus Area Relevan

Buoy dan objek selalu di bagian tengah-bawah frame. Crop frame sebelum inference:

```python
def crop_roi(self, frame):
    h, w = frame.shape[:2]
    # Crop top 20% (langit/horizon) — tidak ada objek di sana
    return frame[int(h*0.2):, :, :]
```

### C. Frame Skip — Adaptive Processing

```python
def runner(self):
    success, frame = self.cam.read()
    if not success:
        return

    # Skip frame jika buffer penuh (GPU/NPU sibuk)
    if len(self.frame_buffer) >= 3:
        self.stats['dropped'] += 1
        return

    display_frame = self.processor(frame)
```

### D. Async Result Publishing

Pisahkan inference dan publishing ke thread terpisah agar tidak saling block.

---

## 4. Heading Control: Compass vs Alternatif

### Analisis Compass Saat Ini

`compass_nav.cpp` menggunakan **bang-bang control** (on/off) — bukan PID:

```cpp
// compass_nav.cpp line 182-188
if (error > 0.0) {
    yaw_output = this->rc_max_value;  // Full right
} else {
    yaw_output = this->rc_min_value;  // Full left
}
```

**Masalah:**
1. **Overshoot**: ASV selalu belok full → overshoot → koreksi full → oscillasi
2. **Heading sensor noise**: Magnetometer rentan interferensi motor/metal
3. **Blocking navigation**: Compass phase harus selesai sebelum vision mulai
4. **Tidak ada throttle control**: Hanya yaw, throttle = 1500 (netral/diam)

### Apakah Compass Masih Relevan?

**Jawaban: RELEVAN tapi perlu di-upgrade**, terutama saat:
- GPS → Vision transition: ASV perlu menghadap arah yang benar sebelum kamera bisa mendeteksi buoy
- Kondisi tanpa visual reference (tidak ada buoy terlihat)

**TIDAK relevan jika:**
- Transition time terlalu lama (menghambat total mission time)
- Heading bisa didapat dari GPS bearing (saat ASV bergerak)

### Alternatif 1: GPS-Based Heading (Tanpa Compass)

Saat ASV bergerak, heading bisa dihitung dari **2 titik GPS berurutan**:

```cpp
// Tambah di gps_nav.cpp
double calculate_current_heading() {
    // Gunakan 2 posisi GPS terakhir untuk hitung heading aktual
    if (prev_position.latitude == 0.0) return -1.0;

    double bearing = calculate_bearing(
        prev_position.latitude, prev_position.longitude,
        current_position.latitude, current_position.longitude
    );

    return fmod(bearing * 180.0 / M_PI + 360.0, 360.0);
}
```

**Keuntungan**: Tidak perlu compass hardware, heading akurat saat bergerak
**Kekurangan**: Tidak bekerja saat diam (perlu minimum speed ~0.5 m/s)

### Alternatif 2: Vision-Assisted Heading (Tanpa Compass Phase)

Hilangkan compass phase sepenuhnya. Setelah GPS waypoint tercapai, langsung masuk vision dengan **search pattern**:

```
GPS Navigation → Waypoint Reached → Vision Search Mode → Object Detected → Vision Track
```

```cpp
// Di vision_nav.cpp runner():
if (!this->objects_detected) {
    // SEARCH MODE: belok pelan ke satu arah sampai deteksi objek
    static int search_direction = 1;
    static int search_counter = 0;

    rc_yaw = this->mid_rc_value + (search_direction * 100);  // Belok pelan
    rc_throttle = this->rc_throttle[mission_index];

    search_counter++;
    if (search_counter > 100) {  // ~5 detik, balik arah
        search_direction *= -1;
        search_counter = 0;
    }
} else {
    // TRACK MODE: normal vision tracking
    // ... existing PID code ...
}
```

### Alternatif 3: Compass dengan PID (Upgrade)

Pertahankan compass tapi upgrade dari bang-bang ke PID:

```cpp
// Ganti compass_navigation() di compass_nav.cpp
bool compass_navigation(double target_heading, double tolerance) {
    double error = this->get_heading_error(target_heading);

    if (fabs(error) <= tolerance) {
        return true;  // Target reached
    }

    // PID instead of bang-bang
    double normalized_error = error / 180.0;  // [-1.0, 1.0]

    // Proportional control dengan deadband
    double kp = 0.7;
    double output = kp * normalized_error;
    output = std::max(-1.0, std::min(1.0, output));

    int yaw_output = this->rc_mid_value +
        static_cast<int>(output * (this->rc_max_value - this->rc_mid_value));

    this->override_control(yaw_output, 1500, 1600, 1500);  // Tambah throttle!
    return false;
}
```

### Alternatif 4: IMU Heading (Gyroscope Fusion)

MAVLink autopilot sudah punya IMU. Gunakan fused heading dari `/mavros/imu/data`:

```cpp
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/utils.h>

// Subscribe ke IMU
this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    "/mavros/imu/data", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                          msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw;
        tf2::getEulerYPR(q, yaw, pitch, roll);  // radians
        this->imu_heading = fmod(yaw * 180.0 / M_PI + 360.0, 360.0);
    }
);
```

**Keuntungan**: Lebih stabil dari magnetometer, update rate tinggi (50-100Hz)
**Kekurangan**: Drift jangka panjang (perlu GPS correction)

### Perbandingan Semua Alternatif

| Metode | Akurasi | Kecepatan Transisi | Saat Diam | Noise | Effort |
|--------|:-------:|:-----------------:|:---------:|:-----:|:------:|
| Compass (saat ini) | ⚠️ Oscillasi | ❌ Lambat | ✅ | ❌ Tinggi | — |
| Compass + PID | ✅ Baik | ✅ Cepat | ✅ | ⚠️ Sedang | Rendah |
| GPS Bearing | ✅ Baik | ✅ Cepat | ❌ | ✅ Rendah | Rendah |
| Vision Search | ✅ Langsung | ✅✅ Paling Cepat | ✅ | ✅ | Sedang |
| IMU Heading | ✅✅ Terbaik | ✅ Cepat | ✅ | ✅ | Sedang |

### Rekomendasi: Hybrid Approach

```
GPS Phase → GPS Bearing untuk heading (saat bergerak)
         → Waypoint tercapai
         → Cek: apakah vision sudah deteksi objek?
            ├─ YA → langsung Vision Track (skip compass)
            └─ TIDAK → Vision Search Mode (sweep pelan)
                       → Objek terdeteksi → Vision Track
```

**Ini menghilangkan compass phase sepenuhnya** dan menghemat 3-8 detik per waypoint.

---

## 5. Strategi Navigation Terpadu

### Masalah Speed Navigation Saat Ini

1. **Sequential phases**: GPS → Compass → Vision (serial, bukan parallel)
2. **Transition delay**: Menunggu compass selesai sebelum vision
3. **Throttle tidak adaptive**: Speed konstan, tidak adjust berdasarkan jarak
4. **Tidak ada look-ahead**: ASV tidak mulai mencari objek sampai compass selesai

### Solusi: Concurrent Navigation Pipeline

```
┌─────────────────────────────────────────────────────┐
│              PROPOSED: CONCURRENT PIPELINE           │
│                                                      │
│  GPS Navigation  ─────────────────────►  Waypoint    │
│       ↕ (parallel)                      Reached     │
│  Vision Detection ─────────────────►  Object Found?  │
│                                        ├─ YES → Track│
│                                        └─ NO → Search│
│                                                      │
│  Compass/IMU: hanya sebagai heading reference,       │
│  BUKAN sebagai phase terpisah                        │
└─────────────────────────────────────────────────────┘
```

### Implementasi: Unified Navigation Node

Gabungkan logika GPS + Vision menjadi satu state machine:

```cpp
enum class NavState {
    GPS_CRUISE,       // Menuju waypoint dengan GPS
    GPS_APPROACH,     // Dekat waypoint, mulai scan vision
    VISION_SEARCH,    // Sweep mencari objek
    VISION_TRACK,     // Lock pada objek, navigasi vision
    MISSION_EXECUTE,  // Surface/Underwater imaging, docking
    COMPLETE
};
```

**State Transitions:**

```
GPS_CRUISE (jarak > 5m)
  → GPS setpoint + throttle tinggi
  → Vision SUDAH aktif di background (warm detection)

GPS_APPROACH (jarak < 5m)
  → Kurangi throttle
  → Prioritaskan vision jika objek terdeteksi
  → if (objects_detected) → VISION_TRACK

VISION_SEARCH (objek tidak terlihat setelah waypoint)
  → Sweep pattern (belok pelan kiri-kanan)
  → Throttle rendah
  → if (objects_detected) → VISION_TRACK

VISION_TRACK (objek terdeteksi)
  → PID yaw dari vision
  → Throttle adaptive berdasarkan jarak ke objek
  → if (proximity_trigger) → MISSION_EXECUTE

MISSION_EXECUTE
  → Surface/Underwater imaging logic
  → Docking logic
  → if (complete) → COMPLETE atau next waypoint
```

### Adaptive Throttle

```cpp
int calculate_adaptive_throttle(double distance_to_target, int mission_index) {
    // Semakin dekat target, semakin pelan
    int max_throttle = this->rc_throttle[mission_index];
    int min_throttle = this->min_rc_value[mission_index];

    if (distance_to_target > 5.0) {
        return max_throttle;  // Full speed
    } else if (distance_to_target > 2.0) {
        // Linear interpolation
        double ratio = (distance_to_target - 2.0) / 3.0;
        return min_throttle + static_cast<int>(ratio * (max_throttle - min_throttle));
    } else {
        return min_throttle + 100;  // Crawl speed
    }
}
```

---

## Ringkasan Prioritas Implementasi

| # | Perubahan | Effort | Impact | Prioritas |
|---|-----------|:------:|:------:|:---------:|
| 1 | ByteTrack tracking | Rendah | ✅✅✅ Tinggi | **P0** |
| 2 | Yaw EMA smoothing | Rendah | ✅✅ Sedang | **P0** |
| 3 | Compass → PID upgrade | Rendah | ✅✅ Sedang | **P1** |
| 4 | Vision Search Mode | Sedang | ✅✅✅ Tinggi | **P1** |
| 5 | Input size 416 untuk speed | Rendah | ✅ Sedang | **P1** |
| 6 | Skip compass (hybrid nav) | Sedang | ✅✅✅ Tinggi | **P2** |
| 7 | Adaptive throttle | Sedang | ✅✅ Sedang | **P2** |
| 8 | Dual model strategy | Sedang | ✅ Sedang | **P3** |
| 9 | Unified nav state machine | Tinggi | ✅✅✅ Tinggi | **P3** |
| 10 | IMU heading | Sedang | ✅✅ Sedang | **P3** |

**Quick Wins (bisa selesai 1-2 jam):** Item 1, 2, 3, 5
**Medium Term (1-2 hari):** Item 4, 6, 7
**Long Term (1 minggu):** Item 8, 9, 10
