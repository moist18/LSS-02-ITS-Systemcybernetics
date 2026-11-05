# Rencana Integrasi LiDAR 3D ke ASV-2025

Dokumen ini adalah **rencana perubahan** yang perlu dilakukan di seluruh kodebase ASV-2025 jika menambahkan sensor LiDAR 3D (misalnya Livox Mid-360, Ouster OS0, atau Velodyne VLP-16).

> **Perbedaan utama LiDAR 2D vs 3D:**
> - LiDAR 2D (RPLIDAR): `sensor_msgs/LaserScan` → satu bidang, publish `/scan`
> - LiDAR 3D: `sensor_msgs/PointCloud2` → point cloud tiga dimensi, publish `/lidar/points` (atau sejenisnya)

---

## Gambaran Sistem Saat Ini

```
RPLIDAR (2D)
    │ /dev/ttyUSB0 (serial)
    ▼
rplidar_ros (driver) → /scan (LaserScan)
    │
    ▼
rplidar.cpp (sensor pkg) → /robot/lidar (Bool)
    │
    ├──► vision.py (perception) — proximity trigger
    └──► vision_nav.cpp (navigation) — RC override
```

---

## Gambaran Setelah LiDAR 3D

```
LiDAR 3D (mis. Livox Mid-360)
    │ UDP / USB / PCIe
    ▼
lidar3d_driver (SDK Driver) → /lidar/points (PointCloud2)
    │
    ▼
lidar3d.cpp  [NODE BARU] (sensor pkg) → /robot/lidar (Bool)
                                      → /robot/lidar/distance (Float32)  [opsional]
                                      → /robot/lidar/pointcloud (PointCloud2) [opsional, filtered]
    │
    ├──► vision.py (perception) — proximity trigger (tidak berubah logikanya)
    └──► vision_nav.cpp (navigation) — bisa tambah obstacle avoidance
```

---

## Daftar File yang Perlu Diubah

### Prioritas 1 — WAJIB (core integration)

| # | File | Jenis | Perubahan |
|---|------|-------|-----------|
| 1 | `sensor/sensor/lidar3d.cpp` | **BARU** | Node C++ untuk proses PointCloud2 |
| 2 | `sensor/CMakeLists.txt` | **UBAH** | Tambah executable `lidar3d` + dependency `pcl_ros` |
| 3 | `sensor/package.xml` | **UBAH** | Tambah dependency `pcl_ros`, `pcl_conversions` |
| 4 | `launcher/config/sensor.yaml` | **UBAH** | Tambah parameter `/lidar3d_node` |
| 5 | `launcher/launch/standalone/lidar3d.launch.py` | **BARU** | Launch standalone untuk debug LiDAR 3D |
| 6 | `launcher/launch/mission/*.launch.py` | **UBAH** | Ganti node `rplidar_ros` + `rplidar` → driver 3D + `lidar3d` |

### Prioritas 2 — OPSIONAL (fitur lanjutan)

| # | File | Jenis | Perubahan |
|---|------|-------|-----------|
| 7 | `navigation/navigation/vision_nav.cpp` | **UBAH** | Gunakan data jarak detail dari LiDAR 3D |
| 8 | `navigation/navigation/gps_nav.cpp` | **UBAH** | Tambah obstacle avoidance saat navigasi GPS |
| 9 | `launcher/config/perception.yaml` | **UBAH** | Update parameter lidar untuk vision node |
| 10 | `lss_02/web_ui/src/app/page.tsx` | **UBAH** | Tampilkan data LiDAR 3D di Web GCS |

---

## Detail Perubahan Per File

---

### 1. `sensor/sensor/lidar3d.cpp` — **[FILE BARU]**

Node baru yang menggantikan peran `rplidar.cpp`, tapi mengkonsumsi `PointCloud2` dari LiDAR 3D.

**Topik yang disubscribe:**
- `/lidar/points` — `sensor_msgs/PointCloud2`

**Topik yang dipublish:**
- `/robot/lidar` — `std_msgs/Bool` (kompatibel dengan sistem saat ini, tidak perlu ubah subscriber)
- `/robot/lidar/distance` — `std_msgs/Float32` (jarak objek terdekat, opsional)

**Logika utama:**
```cpp
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

// Subscribe ke PointCloud2
this->cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar/points", rclcpp::SensorDataQoS(),
    std::bind(&Lidar3DNode::cloud_callback, this, std::placeholders::_1)
);

void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 1. Convert PointCloud2 → PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // 2. Filter berdasarkan area depan (X: positif = depan, Y: kiri-kanan, Z: ketinggian)
    //    X: [min_range, max_range]
    //    Y: [-lateral_width, +lateral_width]
    //    Z: [min_height, max_height]  ← ini keunggulan 3D vs 2D

    // 3. Hitung jumlah titik valid di ROI (Region of Interest)
    int valid_count = 0;
    float closest_distance = max_range;

    for (const auto& pt : cloud->points) {
        if (pt.x >= min_range && pt.x <= max_range &&
            std::abs(pt.y) <= lateral_width &&
            pt.z >= min_height && pt.z <= max_height) {
            valid_count++;
            closest_distance = std::min(closest_distance, pt.x);
        }
    }

    // 4. Publish deteksi (Bool) — backward compatible
    std_msgs::msg::Bool detect_msg;
    detect_msg.data = valid_count >= min_valid_point;
    lidar_pub->publish(detect_msg);

    // 5. Publish jarak terdekat (Float32) — fitur baru
    std_msgs::msg::Float32 dist_msg;
    dist_msg.data = closest_distance;
    distance_pub->publish(dist_msg);
}
```

**Parameter baru yang ditambah vs `rplidar.cpp`:**

| Parameter | Lama (2D) | Baru (3D) |
|-----------|-----------|-----------|
| `lidar.min_range` | ✅ ada | ✅ tetap ada |
| `lidar.max_range` | ✅ ada | ✅ tetap ada |
| `lidar.min_point` | ✅ ada | ✅ tetap ada |
| `lidar.min_angle` | ✅ ada (sudut 2D) | ❌ **dihapus** |
| `lidar.max_angle` | ✅ ada (sudut 2D) | ❌ **dihapus** |
| `lidar.lateral_width` | ❌ tidak ada | ✅ **baru** (lebar area Y) |
| `lidar.min_height` | ❌ tidak ada | ✅ **baru** (filter ketinggian Z) |
| `lidar.max_height` | ❌ tidak ada | ✅ **baru** (filter ketinggian Z) |

---

### 2. `sensor/CMakeLists.txt` — **[UBAH]**

```cmake
# Tambah dependency PCL
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Tambah executable baru
add_executable(lidar3d ${PROJECT_NAME}/lidar3d.cpp)
ament_target_dependencies(lidar3d
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
)
target_link_libraries(lidar3d ${PCL_LIBRARIES})

# Install executables
install(TARGETS
  remote
  rplidar      # ← tetap, untuk backward compat
  lidar3d      # ← tambah ini
  state
  DESTINATION lib/${PROJECT_NAME}
)
```

---

### 3. `sensor/package.xml` — **[UBAH]**

Tambahkan dependency berikut:

```xml
<!-- Tambahkan di bawah <depend>sensor_msgs</depend> -->
<depend>pcl_ros</depend>
<depend>pcl_conversions</depend>
```

---

### 4. `launcher/config/sensor.yaml` — **[UBAH]**

Tambahkan section baru untuk node LiDAR 3D:

```yaml
# === CONFIG LAMA (biarkan untuk backward compat) ===
/rplidar_node:
  ros__parameters:
    channel_type: 'serial'
    serial_port: '/dev/ttyUSB0'
    serial_baudrate: 256000
    frame_id: 'laser'
    inverted: false
    angle_compensate: true

/lidar_node:  # node rplidar.cpp lama
  ros__parameters:
    lidar:
      min_range: 0.2
      max_range: 2.2
      min_point: 5
      min_angle: 345
      max_angle: 15

# === CONFIG BARU LIDAR 3D ===
/lidar3d_node:
  ros__parameters:
    lidar:
      topic: '/lidar/points'     # topic PointCloud2 dari driver
      min_range: 0.3             # jarak minimum (meter)
      max_range: 3.0             # jarak maksimum (meter)
      min_point: 5               # minimum titik valid
      lateral_width: 0.6         # lebar area deteksi kiri-kanan (meter)
      min_height: -0.3           # ketinggian minimum titik (meter, relatif ke sensor)
      max_height: 0.5            # ketinggian maksimum titik (meter)

# Config driver LiDAR 3D (tergantung vendor)
# Contoh Livox Mid-360:
/livox_lidar_publisher:
  ros__parameters:
    xfer_format: 0           # 0=PointCloud2, 1=CustomMsg
    multi_topic: 0
    data_src: 0
    publish_freq: 10.0
    output_data_type: 0
    frame_id: 'livox_frame'
```

---

### 5. `launcher/launch/standalone/lidar3d.launch.py` — **[FILE BARU]**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sensor_param = os.path.join(
        get_package_share_directory('launcher'),
        'config', 'sensor.yaml'
    )

    return LaunchDescription([
        # Driver LiDAR 3D (sesuaikan package dengan vendor)
        # Contoh untuk Livox Mid-360:
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[sensor_param]
        ),
        # Node pemrosesan LiDAR 3D
        Node(
            package='sensor',
            executable='lidar3d',
            name='lidar3d_node',
            output='screen',
            emulate_tty=True,
            parameters=[sensor_param]
        ),
    ])
```

---

### 6. `launcher/launch/mission/*.launch.py` — **[UBAH]**

Di semua 5 mission launch file (`gps_vision.launch.py`, `gps_compass4.launch.py`, dst.), bagian SENSOR perlu diupdate:

**Sebelum (RPLIDAR 2D — saat ini dicomment):**
```python
# Node(
#     package='rplidar_ros',
#     executable='rplidar_node',
#     name='rplidar_node',
#     ...
# ),
# Node(
#     package='sensor',
#     executable='rplidar',
#     name='lidar_node',
#     ...
# ),
```

**Setelah (LiDAR 3D — uncomment dan ganti):**
```python
Node(
    package='livox_ros_driver2',   # ganti sesuai driver vendor
    executable='livox_ros_driver2_node',
    name='livox_lidar_publisher',
    output='screen',
    emulate_tty=True,
    parameters=[sensor_param]
),
Node(
    package='sensor',
    executable='lidar3d',          # ganti dari 'rplidar' ke 'lidar3d'
    name='lidar3d_node',
    output='screen',
    emulate_tty=True,
    parameters=[sensor_param]
),
```

---

### 7. `navigation/navigation/vision_nav.cpp` — **[OPSIONAL UBAH]**

Saat ini `vision_nav.cpp` hanya menerima `/robot/lidar` sebagai `Bool`. Dengan LiDAR 3D, bisa ditambahkan subscriber untuk data jarak:

```cpp
// Tambah subscriber untuk jarak (dari lidar3d.cpp)
#include <std_msgs/msg/float32.hpp>

// Di constructor:
this->lidar_distance_sub = this->create_subscription<std_msgs::msg::Float32>(
    "/robot/lidar/distance", rclcpp::SensorDataQoS(),
    std::bind(&VisionNavigation::lidar_distance_callback, this, std::placeholders::_1)
);

// Callback baru:
void lidar_distance_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->lidar_distance = msg->data;  // jarak objek terdekat
}

// Gunakan lidar_distance untuk kontrol throttle adaptif
// Semakin dekat → throttle semakin kecil (smooth stopping)
```

**Keuntungan:** ASV bisa melakukan *soft stop* proporsional terhadap jarak, bukan sekadar `true/false`.

---

### 8. `navigation/navigation/gps_nav.cpp` — **[OPSIONAL UBAH]**

Saat ini GPS navigation tidak mempertimbangkan rintangan. Dengan LiDAR 3D bisa ditambah **basic obstacle avoidance**:

```cpp
// Subscribe ke deteksi LiDAR
this->lidar_sub = this->create_subscription<std_msgs::msg::Bool>(
    "/robot/lidar", rclcpp::SensorDataQoS(),
    std::bind(&GPSNavigation::lidar_callback, this, std::placeholders::_1)
);

// Di runner(): Jika ada rintangan, pause/reroute GPS navigation
if (this->lidar_detected && this->gps_control) {
    // Hentikan sementara publish geopose setpoint
    // atau override throttle ke minimum
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "GPS-NAV → Obstacle detected! Halting...");
    return;
}
```

---

### 9. `launcher/config/perception.yaml` — **[UBAH MINOR]**

Tidak ada perubahan besar. Pastikan parameter `use_lidar` konsisten:

```yaml
/vision_node:
  ros__parameters:
    config:
      use_lidar: true   # ← aktifkan (sebelumnya false)
```

---

### 10. `lss_02/web_ui/src/app/page.tsx` — **[OPSIONAL UBAH]**

Tambahkan panel LiDAR di Web GCS untuk monitoring real-time:

```tsx
// Tambah state
const [lidarDetected, setLidarDetected] = useState(false);
const [lidarDistance, setLidarDistance] = useState<number | null>(null);

// Di connectToRos():
const lidarTopic = new ROSLIB.Topic({
  ros: r,
  name: '/robot/lidar',
  messageType: 'std_msgs/Bool'
});
lidarTopic.subscribe((msg: any) => setLidarDetected(msg.data));

const lidarDistTopic = new ROSLIB.Topic({
  ros: r,
  name: '/robot/lidar/distance',
  messageType: 'std_msgs/Float32'
});
lidarDistTopic.subscribe((msg: any) => setLidarDistance(msg.data));

// Di JSX — tambah panel baru:
<div className="bg-slate-900 rounded-2xl p-6 border border-slate-800">
  <h2 className="text-xl font-semibold mb-4">LiDAR 3D Status</h2>
  <div className="flex items-center justify-between">
    <span>Obstacle Detected</span>
    <span className={lidarDetected ? 'text-red-400' : 'text-emerald-400'}>
      {lidarDetected ? '⚠ YES' : '✓ CLEAR'}
    </span>
  </div>
  {lidarDistance !== null && (
    <div className="flex items-center justify-between mt-2">
      <span>Closest Object</span>
      <span className="font-mono text-amber-400">{lidarDistance.toFixed(2)} m</span>
    </div>
  )}
</div>
```

---

## Topik ROS 2 — Sebelum vs Sesudah

| Topic | Tipe | Sumber | Status |
|-------|------|--------|--------|
| `/scan` | `sensor_msgs/LaserScan` | rplidar_ros | ❌ **Dihapus** (ganti driver) |
| `/lidar/points` | `sensor_msgs/PointCloud2` | Driver LiDAR 3D | ✅ **Baru** |
| `/robot/lidar` | `std_msgs/Bool` | `lidar3d.cpp` | ✅ **Tetap** (backward compat) |
| `/robot/lidar/distance` | `std_msgs/Float32` | `lidar3d.cpp` | ✅ **Baru** |
| `/robot/lidar/pointcloud` | `sensor_msgs/PointCloud2` | `lidar3d.cpp` | ⚡ **Opsional** |

---

## Diagram Alur Data (Setelah LiDAR 3D)

```
┌──────────────────────────────────────────────────────────────┐
│                      HARDWARE                                │
│            LiDAR 3D (Livox / Ouster / Velodyne)              │
│              (UDP / USB, ~10Hz point cloud)                  │
└───────────────────────────┬──────────────────────────────────┘
                            │
                            ▼
┌──────────────────────────────────────────────────────────────┐
│              SDK Driver (vendor ROS2 package)                │
│           livox_ros_driver2 / ouster_ros / velodyne_ros      │
│                                                              │
│   Publish: /lidar/points (sensor_msgs/PointCloud2)           │
└───────────────────────────┬──────────────────────────────────┘
                            │ PointCloud2
                            ▼
┌──────────────────────────────────────────────────────────────┐
│                lidar3d.cpp  [NODE BARU]                      │
│              (package: sensor)                               │
│                                                              │
│  1. Convert PointCloud2 → PCL                                │
│  2. Passthrough filter:                                      │
│     - X: [0.3m, 3.0m]  (jarak depan)                       │
│     - Y: [-0.6m, +0.6m] (lebar kiri-kanan)                 │
│     - Z: [-0.3m, +0.5m] (ketinggian)                       │
│  3. Hitung titik valid                                       │
│  4. Hitung jarak objek terdekat                              │
│                                                              │
│  Publish: /robot/lidar (Bool)       ← backward compat       │
│  Publish: /robot/lidar/distance (Float32) ← BARU            │
└──────────────┬───────────────────────────────────────────────┘
               │
     ┌─────────┴─────────┐
     │                   │
     ▼                   ▼
┌──────────┐    ┌────────────────────────────┐
│vision.py │    │ vision_nav.cpp             │
│          │    │                            │
│proximity │    │ RC Override + Soft Stop    │
│trigger   │    │ (opsional: adaptive ctrl)  │
└──────────┘    └────────────────────────────┘
                           │
                           ▼
               ┌────────────────────┐
               │  gps_nav.cpp       │
               │  (opsional:        │
               │  obstacle halt)    │
               └────────────────────┘
```

---

## Checklist Implementasi

```
[ ] 1. Install ROS2 driver sesuai sensor LiDAR 3D yang dipakai
       - Livox: livox_ros_driver2
       - Ouster: ouster_ros
       - Velodyne: velodyne_ros

[ ] 2. Install PCL dependency
       sudo apt install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions

[ ] 3. Buat sensor/sensor/lidar3d.cpp (node baru)

[ ] 4. Update sensor/CMakeLists.txt (tambah pcl + executable lidar3d)

[ ] 5. Update sensor/package.xml (tambah depend pcl_ros)

[ ] 6. Update launcher/config/sensor.yaml (tambah /lidar3d_node params)

[ ] 7. Buat launcher/launch/standalone/lidar3d.launch.py

[ ] 8. Update 5 mission launch files (ganti node rplidar → lidar3d)

[ ] 9. [Opsional] Update vision_nav.cpp untuk adaptive distance control

[ ] 10. [Opsional] Update gps_nav.cpp untuk obstacle avoidance

[ ] 11. [Opsional] Update web_ui page.tsx untuk panel LiDAR 3D

[ ] 12. Build ulang workspace:
        colcon build --packages-select sensor
        colcon build --packages-select launcher

[ ] 13. Test standalone:
        ros2 launch launcher lidar3d.launch.py
        ros2 topic echo /robot/lidar
        ros2 topic echo /robot/lidar/distance
```

---

## Catatan Penting

1. **Backward Compatibility**: Topic `/robot/lidar` (Bool) tetap dipertahankan → `vision.py` dan `vision_nav.cpp` tidak perlu diubah untuk fungsionalitas dasar.

2. **Pemilihan Driver**: Sesuaikan `package` dan `executable` di launch file dengan sensor LiDAR 3D yang benar-benar dipakai (Livox / Ouster / Velodyne / dll).

3. **Frame ID & TF**: LiDAR 3D perlu konfigurasi `tf2` transform (posisi sensor relatif ke robot frame) agar koordinat point cloud akurat. Tambahkan static transform publisher di launch file.

4. **PCL di ARM**: Jika ASV menggunakan Raspberry Pi / Jetson / RK3588, pastikan PCL bisa dikompilasi di arsitektur ARM (biasanya bisa, tapi perlu build dari source jika pakai versi custom).

5. **Latensi**: PointCloud2 lebih berat dari LaserScan. Gunakan `rclcpp::SensorDataQoS()` dan tambahkan profiling jika ada lag.

---

## Pengukuran Jarak ke Buoy / Obstacle

LiDAR 3D mengukur jarak secara **langsung dari fisika** (Time of Flight) dengan akurasi ±2–5cm, bukan estimasi dari piksel kamera.

### Tiga Pendekatan Pengukuran

#### Pendekatan 1 — Closest Point (Sudah Ada di Rencana)
Ambil titik valid terdekat dari ROI (Region of Interest):

```cpp
float closest_distance = std::numeric_limits<float>::max();

for (const auto& pt : cloud->points) {
    if (pt.x >= min_range && pt.x <= max_range &&
        std::abs(pt.y) <= lateral_width &&
        pt.z >= min_height && pt.z <= max_height) {

        float dist = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
        closest_distance = std::min(closest_distance, dist);
    }
}

std_msgs::msg::Float32 dist_msg;
dist_msg.data = closest_distance;
distance_pub->publish(dist_msg);
```

**Cocok untuk:** proximity trigger Surface Imaging  
**Kekurangan:** rentan noise gelombang air

---

#### Pendekatan 2 — Cluster Centroid (Lebih Akurat)
Kelompokkan titik menjadi objek, hitung pusat cluster = posisi buoy:

```cpp
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
tree->setInputCloud(filtered_cloud);

std::vector<pcl::PointIndices> cluster_indices;
pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
ec.setClusterTolerance(0.15);   // titik dalam radius 15cm = 1 cluster
ec.setMinClusterSize(10);       // min 10 titik = objek valid
ec.setMaxClusterSize(500);      // max 500 titik (buoy tidak besar)
ec.setSearchMethod(tree);
ec.setInputCloud(filtered_cloud);
ec.extract(cluster_indices);

for (const auto& indices : cluster_indices) {
    float sum_x = 0, sum_y = 0, sum_z = 0;
    for (int idx : indices.indices) {
        sum_x += filtered_cloud->points[idx].x;
        sum_y += filtered_cloud->points[idx].y;
        sum_z += filtered_cloud->points[idx].z;
    }
    int n = indices.indices.size();
    float cx = sum_x / n;  // posisi X buoy (depan-belakang)
    float cy = sum_y / n;  // posisi Y buoy (kiri-kanan)

    float distance = std::sqrt(cx*cx + cy*cy);
    RCLCPP_INFO(get_logger(), "Buoy | X:%.2f Y:%.2f Dist:%.2f m", cx, cy, distance);
}
```

**Output tambahan yang dipublish:**

| Topic | Tipe | Isi |
|-------|------|-----|
| `/robot/lidar/buoy_x` | `Float32` | Jarak maju ke buoy (meter) |
| `/robot/lidar/buoy_y` | `Float32` | Posisi lateral buoy (kiri/kanan) |
| `/robot/lidar/distance` | `Float32` | Jarak Euclidean total |

> Dengan `buoy_y`, `vision_nav.cpp` bisa menggunakan **PID yaw dari LiDAR** sebagai backup/pengganti kamera — jauh lebih stabil di kondisi cahaya buruk atau silau.

---

#### Pendekatan 3 — RANSAC Sphere Fitting (Paling Akurat)
Karena buoy berbentuk bola/silinder, bisa fit model geometri langsung:

```cpp
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

pcl::SACSegmentation<pcl::PointXYZ> seg;
seg.setModelType(pcl::SACMODEL_SPHERE);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setDistanceThreshold(0.05);      // toleransi 5cm
seg.setRadiusLimits(0.1, 0.5);      // radius buoy 10–50cm
seg.setInputCloud(cluster_cloud);
seg.segment(*inliers, *coefficients);

// coefficients->values = [center_x, center_y, center_z, radius]
float buoy_radius = coefficients->values[3];
float cx = coefficients->values[0];
float cy = coefficients->values[1];
// Jarak ke permukaan buoy (bukan ke pusatnya)
float distance = std::sqrt(cx*cx + cy*cy) - buoy_radius;
```

### Perbandingan Pendekatan

| | Closest Point | Cluster Centroid | RANSAC Sphere |
|--|:--:|:--:|:--:|
| **Akurasi jarak** | ±10–20cm | ±5–10cm | **±2–5cm** |
| **Noise robustness** | ❌ Rentan | ✅ Baik | ✅✅ Sangat baik |
| **Posisi Y (kiri/kanan)** | ❌ | ✅ | ✅ |
| **Beban CPU** | Sangat ringan | Sedang | Berat |
| **Rekomendasi mission** | Surface Imaging trigger | **Floating Ball nav** | Precision docking |

### Catatan tentang Colorization

Colorization (pewarnaan point cloud berdasarkan ketinggian Z) adalah **murni untuk visualisasi** di RViz2 — tidak mempengaruhi akurasi pengukuran jarak sama sekali. Yang benar-benar meningkatkan akurasi adalah:

| Teknik | Efek |
|--------|------|
| Voxel Grid Downsampling | Noise berkurang |
| Statistical Outlier Removal | Hapus titik liar |
| **Z-height filter** | Abaikan ombak/noise air permukaan |
| Euclidean Cluster Extraction | Kelompokkan objek diskrit |

Untuk runtime/mission: matikan colorization publish. Untuk debug di lab: aktifkan colorization di RViz2.

---

## LiDAR SLAM — Menggantikan GPS

### Konsep Dasar

GPS memberikan **posisi global (lat/lon)**. LiDAR SLAM memberikan **posisi lokal relatif terhadap lingkungan** — tanpa sinyal satelit.

```
GPS:   "Kamu di koordinat -7.123456, 112.654321"
SLAM:  "Kamu 12.3m ke depan dan 4.1m ke kiri dari titik start"
```

Teknologi ini disebut **SLAM** *(Simultaneous Localization and Mapping)*.

### Cara Kerja LiDAR SLAM

```
Frame t=0.0s:          Frame t=0.1s:
  ●●●                     ●●●
●     ●   → ASV gerak → ●     ●
  ●●●                     ●●●
                         ↑ scan bergeser sedikit

Algoritma: "scan bergeser Δx=0.05m, Δy=0.01m, Δθ=0.5°"
→ Itu adalah pergerakan ASV
→ Akumulasi semua Δ = posisi ASV saat ini
```

**3 Langkah Utama:**

```
1. SCAN MATCHING
   Frame baru vs frame lama → hitung delta (Δx, Δy, Δθ)
   Algoritma: ICP, NDT, LOAM, KISS-ICP

2. POSE ESTIMATION
   Akumulasi Δx, Δy, Δθ → posisi saat ini (x, y, heading)

3. MAP BUILDING (opsional)
   Gabung semua scan → peta 3D lingkungan
```

### Perbandingan Algoritma SLAM

| Algoritma | Cara Kerja | Cocok untuk ASV | Beban CPU |
|-----------|-----------|:--:|:--:|
| **ICP** | Match titik per titik | ❌ Drift tinggi | Sedang |
| **NDT** | Match distribusi statistik | ✅ Stabil | Sedang |
| **LeGO-LOAM** | Feature-based (edge + plane) | ✅✅ Outdoor | Sedang |
| **KISS-ICP** | Simple ICP modern | ✅ Ringan | **Ringan** |
| **LIO-SAM** | LiDAR + IMU fusion | ✅✅✅ Terbaik | Berat |
| **FAST-LIO2** | LiDAR + IMU, efisien | ✅✅ | Sedang |

### Arsitektur Sistem — GPS vs LiDAR SLAM

**Sistem saat ini (GPS-based):**
```
GPS → /mavros/global_position/global (lat, lon)
             ↓
      gps_nav.cpp → hitung bearing ke waypoint GPS
             ↓
      mavros setpoint → ArduPilot → motor
```

**Sistem pengganti (LiDAR SLAM):**
```
LiDAR 3D → /lidar/points (PointCloud2)
             ↓
      SLAM Node (KISS-ICP / LIO-SAM)
             ↓
      /odom  → posisi lokal (x, y, z) meter
      /map   → peta 3D lingkungan (opsional)
             ↓
      lidar_nav.cpp  [NODE BARU]
             ↓
      hitung jarak ke waypoint lokal (meter, bukan GPS)
             ↓
      RC override → motor
```

### Waypoint System — GPS vs SLAM

**GPS waypoint** (saat ini di `navigation/*.yaml`):
```yaml
navigation:
  waypoints_A:
    waypoint_0:
      latitude:  -7.123456
      longitude: 112.654321
      altitude:  0.0
```

**SLAM waypoint** (setelah migrasi):
```yaml
navigation:
  waypoints_A:
    waypoint_0:
      x: 10.0   # 10 meter ke depan dari titik start
      y: 3.0    # 3 meter ke kanan
    waypoint_1:
      x: 20.0
      y: 0.0
```

### Node Baru: `lidar_nav.cpp`

Menggantikan peran `gps_nav.cpp`, subscribe ke odometry SLAM:

```cpp
#include <nav_msgs/msg/odometry.hpp>

class LidarNavigation : public rclcpp::Node {
public:
    LidarNavigation() : Node("lidar_navigation") {

        // Subscribe ke odometry dari SLAM
        this->odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&LidarNavigation::odom_callback, this, std::placeholders::_1)
        );

        // Logika waypoint dan RC override sama seperti gps_nav.cpp
        // tapi menggunakan x/y meter, bukan lat/lon
    }

private:
    void runner() {
        double curr_x = this->current_pose.x;
        double curr_y = this->current_pose.y;

        double wp_x = waypoints[current_waypoint].x;
        double wp_y = waypoints[current_waypoint].y;

        double dx = wp_x - curr_x;
        double dy = wp_y - curr_y;
        double distance = std::sqrt(dx*dx + dy*dy);

        // Heading yang dibutuhkan
        double desired_heading = std::atan2(dy, dx);
        double heading_error = desired_heading - current_heading;

        // Jika sudah dalam radius waypoint
        if (distance < this->radius) {
            current_waypoint++;
            RCLCPP_INFO(get_logger(), "LIDAR-NAV → Waypoint %d reached!", current_waypoint);
        }

        // → RC override (logika sama seperti compass_nav.cpp)
    }
};
```

### File yang Perlu Diubah untuk SLAM Navigation

| File | Jenis | Perubahan |
|------|-------|-----------|
| `navigation/navigation/lidar_nav.cpp` | **BARU** | Menggantikan `gps_nav.cpp`, subscribe `/odom` |
| `navigation/CMakeLists.txt` | **UBAH** | Tambah executable `lidar_nav` |
| `navigation/package.xml` | **UBAH** | Tambah `nav_msgs` |
| `launcher/config/navigation/*.yaml` | **UBAH** | Ganti waypoint `lat/lon` → `x/y` meter |
| `launcher/launch/mission/*.launch.py` | **UBAH** | Ganti `gps_nav` → `lidar_nav`, tambah node SLAM |
| `monitoring/monitoring/gps.cpp` | **UBAH** | Tambah publish pose dari SLAM |
| `lss_02/web_ui/src/app/page.tsx` | **UBAH** | Tampilkan peta lokal SLAM, bukan GPS coordinate |

### Stack ROS2 yang Direkomendasikan

```bash
# Opsi 1: Paling ringan — cocok untuk mini PC / RK3588
KISS-ICP
  ros2 pkg : kiss_icp
  output   : /odom (Odometry)
  install  : pip install kiss-icp

# Opsi 2: Paling akurat — butuh IMU
LIO-SAM
  ros2 pkg : lio_sam
  output   : /odom, /map (PointCloud2)
  repo     : github.com/TixiaoShan/LIO-SAM

# Opsi 3: Seimbang ringan + IMU
FAST-LIO2
  ros2 pkg : fast_lio
  output   : /odom, /cloud_registered
  repo     : github.com/hku-mars/FAST_LIO
```

### Keterbatasan LiDAR SLAM untuk ASV

> ⚠️ Pertimbangan penting sebelum mengganti GPS:

| Masalah | Penjelasan | Solusi |
|---------|-----------|--------|
| **Featureless environment** | Di tengah danau/laut terbuka tanpa struktur di sekitar, SLAM kehilangan referensi | Hanya cocok di area dengan dermaga/bangunan |
| **Dynamic surface** | Ombak = noise point cloud yang bergerak terus | Z-filter ketat + IMU fusion |
| **Sensor bergoyang** | ASV rolling/pitching → scan tidak stabil | **Wajib pakai IMU** (LIO-SAM/FAST-LIO2) |
| **Drift jangka panjang** | Error terakumulasi tanpa loop closure | Kombinasi LiDAR + IMU + kompas |
| **Jangkauan terbatas** | Efektif hanya dalam radius < 50–100m | Tidak cocok navigasi jarak jauh |

### Rekomendasi Penggunaan

```
Kompetisi di kolam / danau kecil dengan struktur:
  ✅ LiDAR SLAM bisa menggantikan GPS sepenuhnya
  ✅ Akurasi posisi: ±5–10cm (jauh lebih baik dari GPS ±1–3m)
  ✅ Stack: KISS-ICP (ringan) atau LIO-SAM (akurat)

Open water / navigasi jarak jauh:
  ❌ SLAM tidak cukup (tidak ada fitur untuk di-match)
  ✅ GPS tetap diperlukan sebagai sumber posisi global

TERBAIK — Sensor Fusion GPS + LiDAR SLAM:
  GPS  → posisi global kasar (±1–3m), tidak drift
  SLAM → koreksi posisi lokal halus (±5–10cm)
  →  Robot Localization package (EKF/UKF) untuk fuse keduanya
```

### Diagram Alur Data Lengkap (SLAM Mode)

```
┌────────────────────────────────────────────────────────────┐
│                     HARDWARE                               │
│            LiDAR 3D + IMU (wajib untuk SLAM)              │
└──────────────┬─────────────────────────────────────────────┘
               │ PointCloud2 + IMU data
               ▼
┌────────────────────────────────────────────────────────────┐
│          SLAM Node (KISS-ICP / LIO-SAM / FAST-LIO2)        │
│                                                            │
│  Input : /lidar/points + /imu/data                         │
│  Output: /odom  (posisi x,y,z + orientasi)                 │
│          /map   (peta 3D akumulatif)                       │
└──────────────┬─────────────────────────────────────────────┘
               │ /odom (Odometry)
               ▼
┌────────────────────────────────────────────────────────────┐
│          lidar_nav.cpp  [NODE BARU]                        │
│                                                            │
│  1. Baca posisi saat ini dari /odom                        │
│  2. Hitung jarak ke waypoint (x/y meter)                  │
│  3. Hitung heading error                                   │
│  4. Advance ke waypoint berikutnya jika dalam radius       │
│                                                            │
│  Output: /mavros/rc/override                               │
└──────────────┬─────────────────────────────────────────────┘
               │
       ┌───────┴───────┐
       │               │
       ▼               ▼
┌────────────┐  ┌────────────────┐
│ ArduPilot  │  │ vision_nav.cpp │
│ (motor)    │  │ (docking/buoy) │
└────────────┘  └────────────────┘
```

### Checklist Tambahan untuk SLAM Navigation

```
[ ] Install SLAM package yang dipilih (KISS-ICP / LIO-SAM / FAST-LIO2)

[ ] Pastikan IMU tersedia dan terpublish ke /imu/data
    (biasanya dari MAVROS: /mavros/imu/data)

[ ] Kalibrasi extrinsic LiDAR-IMU (posisi relatif sensor)

[ ] Buat navigation/navigation/lidar_nav.cpp

[ ] Update navigation/CMakeLists.txt dan package.xml

[ ] Update launcher/config/navigation/*.yaml
    (ganti lat/lon waypoint → x/y meter)

[ ] Update launcher/launch/mission/*.launch.py
    (tambah node SLAM, ganti gps_nav → lidar_nav)

[ ] Test di area kecil dengan struktur di sekitar:
    ros2 topic echo /odom
    rviz2 → tambah topic /map dan /odom path

[ ] Validasi: jalan lurus 10m, cek apakah /odom menunjukkan ~10m
```