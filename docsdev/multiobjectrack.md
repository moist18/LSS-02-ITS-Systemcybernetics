# Multi-Object Tracking (MOT) Integration — ASV-2025

## Ringkasan

Dokumen ini mendefinisikan rencana integrasi **motcpp** (Modern C++ Multi-Object Tracking Library) ke dalam sistem robot ASV-2025 untuk menggantikan per-frame detection yang saat ini digunakan di `vision.py`.

> **Referensi library**: [github.com/Geekgineer/motcpp](https://github.com/Geekgineer/motcpp)

---

## Mengapa Multi-Object Tracking?

### Masalah Saat Ini (Per-Frame Detection)

| Problem | Impact | Contoh |
|---------|--------|--------|
| **ID flickering** | Buoy sama hilang-muncul | green_buoy terdeteksi frame 1, hilang frame 2, muncul frame 3 |
| **Yaw jittery** | Steering oscillasi ±50-100 RC | ASV belok kanan-kiri tanpa alasan |
| **False positive lolos** | Noise dianggap objek nyata | Pantulan air terdeteksi sebagai buoy |
| **No temporal consistency** | Tidak bisa lacak objek spesifik | Tidak tahu mana buoy "terdekat" yang sedang dikejar |

### Solusi: Multi-Object Tracking

```
Frame N:   YOLO → [buoy_A(0.85), buoy_B(0.72), noise(0.35)]
                    ↓
ByteTrack: IoU matching + Kalman prediction
                    ↓
Output:    [Track_1: buoy_A (ID=1, konsisten), Track_2: buoy_B (ID=2, konsisten)]
```

---

## Mengapa motcpp (Bukan Python ByteTrack)?

| Aspek | Python (supervision) | **motcpp (C++)** |
|-------|:--------------------:|:----------------:|
| ByteTrack FPS | ~45 FPS | **1100 FPS** |
| OC-SORT FPS | ~32 FPS | **850 FPS** |
| Overhead | GIL + Python interp | Native C++ |
| Dependencies | pip install | CMake FetchContent |
| Cocok ARM/NPU | ⚠️ Python overhead | ✅ Optimized |
| Algorithms | 1-2 | **10 SOTA** |
| Test coverage | Varies | **>90% GTest** |

**motcpp 24× lebih cepat dari Python** dan langsung compatible dengan C++ navigation nodes yang sudah ada di ASV-2025.

---

## Tracker yang Direkomendasikan

### Primary: ByteTrack (Motion-Only)

- **0 dependency tambahan** — tidak perlu ONNX/ReID model
- **Overhead ~0.9ms per frame** — negligible vs kamera 20 FPS (50ms/frame)
- **Two-stage association** — memanfaatkan deteksi low-confidence yang biasanya dibuang
- **Kalman filter prediction** — objek yang hilang sementara tetap di-track

### Alternative: OC-SORT

- Lebih robust saat objek saling berdekatan (occlusion)
- Sedikit lebih lambat (850 FPS, masih sangat cepat)
- Cocok jika ada masalah buoy crossing

### Tidak Direkomendasikan (untuk saat ini)

- **ReID trackers** (BoostTrack, StrongSORT, dll) — butuh ONNX Runtime + model besar
- **DeepOC-SORT** — perlu appearance embeddings
- Bisa di-upgrade nanti jika pindah ke Jetson

---

## Arsitektur Integrasi

### Opsi Terpilih: C++ ROS Node (tracking_node)

```
┌──────────────┐    /detect/raw     ┌───────────────┐    /detect/tracked    ┌──────────────┐
│  vision.py   │ ──────────────────→│ tracking_node │───────────────────────→│ novisual.py  │
│  (YOLO RKNN) │  DetectionArray    │ (motcpp C++)  │   TrackedObjectArray  │ (yaw calc)   │
└──────────────┘                    └───────────────┘                        └──────────────┘
                                          │
                                          │ /detect/object/bool
                                          ↓
                                    ┌──────────────┐
                                    │vision_nav.cpp│
                                    │ (PID Control)│
                                    └──────────────┘
```

### Data Flow Detail

1. **vision.py** menjalankan YOLO inference → raw detections `[x1,y1,x2,y2,conf,class]`
2. Publish sebagai **DetectionArray** ke `/detect/raw`
3. **tracking_node** (C++) menerima detections:
   - Convert ke `Eigen::MatrixXf` (motcpp format)
   - `tracker.update(dets, frame)` → tracked objects dengan persistent ID
   - Convert kembali ke **TrackedObjectArray**
   - Publish ke `/detect/tracked`
4. **vision.py** subscribe `/detect/tracked` → gunakan tracked boxes untuk yaw calculation
5. **vision_nav.cpp** tetap menerima `/robot/vision/raw_yaw` — tidak perlu diubah

---

## Custom ROS Messages

### Detection.msg
```
float32 x1
float32 y1
float32 x2
float32 y2
float32 confidence
int32 class_id
```

### DetectionArray.msg
```
std_msgs/Header header
Detection[] detections
```

### TrackedObject.msg
```
float32 x1
float32 y1
float32 x2
float32 y2
int32 track_id
float32 confidence
int32 class_id
```

### TrackedObjectArray.msg
```
std_msgs/Header header
TrackedObject[] tracks
```

---

## Konfigurasi Tracker

```yaml
# tracking.yaml
/tracking_node:
  ros__parameters:
    tracking:
      enabled: true
      algorithm: "bytetrack"       # sort, bytetrack, ocsort
      det_thresh: 0.3              # Min confidence untuk track baru
      max_age: 30                  # Frame grace period (30 = 1.5s @20fps)
      min_hits: 3                  # Min detections sebelum track confirmed
      iou_threshold: 0.3           # IoU matching threshold
      per_class: true              # Track tiap class secara independen
      nr_classes: 6                # Jumlah class YOLO
      max_obs: 50                  # Max observations untuk velocity
```

---

## Unit Test Plan

### C++ GoogleTest Tests

| Test Suite | Test Case | What It Verifies |
|------------|-----------|------------------|
| **ByteTrackInit** | Default params init | Tracker creates without crash |
| **EmptyDetections** | Update with 0 detections | Returns empty, no crash |
| **SingleObjectTracking** | 10 frames, 1 object linear motion | Same track_id all frames |
| **MultiObjectTracking** | 5 frames, 3 objects | 3 distinct IDs maintained |
| **ObjectDisappearance** | Detected 5 frames, gone 30 frames | Track deleted after max_age |
| **ObjectReappearance** | Gone 10 frames (< max_age), back | Same track_id maintained |
| **PerClassTracking** | 2 green_buoy + 1 red_buoy, per_class=true | No cross-class ID mixing |
| **LowConfDetection** | Low confidence matched to existing track | ByteTrack second-stage works |
| **TrackerReset** | reset() → new detections | IDs restart from 0 |
| **ROSMsgToEigen** | Convert DetectionArray → MatrixXf | Correct dimensions & values |
| **EigenToROSMsg** | Convert MatrixXf → TrackedObjectArray | Correct field mapping |
| **EMASmoothing** | Jittery yaw values | Output variance < input |
| **StepResponse** | Yaw jump 1000→2000 | Smooth transition ~5 frames |

### Python Integration Tests (pytest)

| Test Case | What It Verifies |
|-----------|------------------|
| `test_publish_raw_detections` | vision.py publishes DetectionArray correctly |
| `test_receive_tracked_objects` | vision.py receives and processes TrackedObjectArray |
| `test_fallback_no_tracking` | Works when tracking.enabled=false |
| `test_tracking_node_offline` | vision.py functions if tracking_node isn't running |

---

## Timeline Implementasi

| Phase | Durasi | Deliverables |
|-------|:------:|-------------|
| **Phase 1: Foundation** | 2 hari | motcpp build, custom ROS msgs, basic tracking_node |
| **Phase 2: Integration** | 2 hari | vision.py mods, tracked output piping, config |
| **Phase 3: Testing** | 2 hari | All unit tests pass, performance benchmark |
| **Phase 4: Optimization** | 1 hari | Parameter tuning, aarch64 cross-compile, docs |

**Total: ~7 hari kerja**

---

## Referensi

- [motcpp GitHub](https://github.com/Geekgineer/motcpp) — Library source
- [ByteTrack Paper](https://arxiv.org/abs/2110.06864) — ECCV 2022
- [OC-SORT Paper](https://arxiv.org/abs/2203.14360) — CVPR 2023
- [vision_optimization.md](file:///home/aryaputra/Documents/ASV-CONTROL/ASV-2025/vision_optimization.md) — Analisis masalah existing
