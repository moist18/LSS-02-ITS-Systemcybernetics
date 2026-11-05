# Materi Kuliah: Workflow Sistem ASV-2025 (ROS 2)

Dokumen ini merangkum **workflow logic program** dan **teknologi** yang dipakai pada repository `ASV-2025`, khususnya paket-paket di `main/src/*` dan dependensi pihak ketiga di `libs/src/*`.

---

## 1) Gambaran Besar Sistem

Sistem dibangun dengan **ROS 2** dan konsep modular **node-node** yang saling berkomunikasi lewat **topic**.

Alur besar (end-to-end):

1. **MAVROS** menghubungkan autopilot (ArduPilot/PX4) ke ROS 2 melalui protokol **MAVLink**.
2. **Sensor & input operator** (node `remote`) mengirim perintah:
   - mode manual/auto,
   - arming/disarming,
   - pilihan trajectory A/B,
   - reset mission,
   - dan RC override (kalau manual).
3. **Mission planner** memilih “mode navigasi” yang aktif (GPS vs Compass vs Vision) dan    memilih jenis misi (Floating Ball / Surface Imaging / Underwater Imaging / Docking).
4. **Navigation** menjalankan kontrol sesuai mode:
   - GPS navigation: publish setpoint global ke MAVROS,
   - Compass navigation: RC override berdasarkan heading target,
   - Vision navigation: RC override berdasarkan hasil vision + PID.
5. **Perception** memproses kamera + ML (YOLO/RKNN) untuk deteksi objek dan publish sinyal kontrol (mis. `raw_yaw`, `object_detected`, `snapshot`) serta streaming image.
6. **Monitoring** mempublikasikan telemetry sederhana (CPU, battery, GPS ringkas, date).
7. (Opsional) **Rosbridge** membuka websocket untuk integrasi Web-GCS.

---

## 2) Teknologi yang Dipakai

- **ROS 2**
  - C++: `rclcpp`
  - Python: `rclpy`
  - Launch system: `launch` / `launch_ros`
- **MAVROS / MAVLink**
  - `mavros`, `mavros_msgs`, `libmavconn`, `mavlink`
- **OpenCV**
  - `opencv2` / `cv2` dan `cv_bridge` (bridge ROS Image ↔ OpenCV)
- **Computer Vision / AI**
  - `ultralytics` YOLO
  - Optimisasi RKNN/NPU: environment variables `RKNN_*` (target platform: `rk3588`)
- **Sensor Lidar**
  - `rplidar_ros` menghasilkan `/scan` (LaserScan)
  - node lokal `lidar_node` memfilter `/scan` menjadi trigger boolean `/robot/lidar`
- **Serial komunikasi**
  - library `serial` (package `serial-ros2`) untuk komunikasi Arduino (remote controller)
- **Linux OS interface**
  - CPU usage dihitung dari `/proc/stat`

---

## 3) Struktur Package Utama (`main/src/*`)

### 3.1 `launcher`
**Fungsi**: menjalankan sistem lewat ROS 2 launch file + menyediakan konfigurasi YAML.

- **Isi utama**:
  - `launch/mavros.launch.py`
    - include MAVROS (`mavros/launch/apm.launch`) dengan:
      - `fcu_url: /dev/ttyACM0:115200`
      - `gcs_url: tcp-l://0.0.0.0:5760`
    - include `rosbridge.launch.py`
    - `TimerAction(60s)` menjalankan `ros2 param set` untuk parameter autopilot (contoh `WP_RADIUS`, `WP_SPEED`, dll)
  - `launch/rosbridge.launch.py`
    - include `rosbridge_server` websocket (port 9090, bind 0.0.0.0)
    - menjalankan `rosapi_node` agar Web-GCS bisa query topic/service/param
  - `launch/mission/*.launch.py` (contoh: `gps_rtk.launch.py`, `gps_vision.launch.py`, `gps_compass5.launch.py`)
    - menjalankan kumpulan node: mission + monitoring + navigation + perception + sensor.
  - `launch/standalone/*.launch.py`
    - menjalankan subset node (mis. monitoring saja, vision saja, rplidar saja, remote saja)
  - `config/*.yaml`
    - `sensor.yaml`, `perception.yaml`, `mission/*.yaml`, `navigation/*.yaml`

**Konsep yang bisa diajarkan**:
- “Orkestrasi” sistem multi-node dengan launch.
- Parameterisasi node lewat YAML.
- Integrasi ROS dengan web (rosbridge).

---

### 3.2 `monitoring`
**Fungsi**: telemetry sederhana robot.

**Executable / Node** (C++):
- `battery` (`battery_node`)
  - Sub: `/mavros/state`, `/mavros/battery`
  - Pub: `/robot/battery` (Float32, %)
- `cpu` (`cpu_node`)
  - Input: Linux `/proc/stat`
  - Pub: `/robot/cpu` (Float32, %)
- `gps` (`gps_node`)
  - Sub: `/mavros/global_position/global`
  - Pub: `/robot/gps` ([lat, lon])
  - Pub: `/robot/cog` (Float64)
  - Pub: `/robot/sog` ([km/h, knot])
- `date` (`date_node`)
  - Input: system clock
  - Pub: `/robot/date` ([weekday, day, month, year, hour, minute, sec])

**Ciri implementasi**:
- Publish periodik (umumnya 1 Hz) via `create_wall_timer()`.
- Logging memakai `RCLCPP_INFO_THROTTLE`.

---

### 3.3 `sensor`
**Fungsi**: jembatan input (remote Arduino), state/arming/mode autopilot, dan trigger lidar.

#### A) `remote` (`remote_node`)
- **Teknologi**: komunikasi **Serial** ke Arduino (library `serial`).
- **Subscriber**:
  - `/mavros/global_position/global` (untuk “waypoint fetcher”, menyimpan GPS ke file)
- **Publisher**:
  - `/mavros/rc/override` (OverrideRCIn) → manual RC override ke autopilot
  - `/mission/reset` (Bool)
  - `/robot/arming` (Bool)
  - `/robot/mode` (Bool) → auto_mode true/false
  - `/robot/setmode` (String) → mode autopilot (contoh: `MANUAL`, `GUIDED`)
  - `/robot/trajectory` (String) → `A` atau `B`
  - `/detect/docking` (String)
  - `/detect/left_buoy` (String)
  - `/detect/right_buoy` (String)
  - `/robot/waypoints/update` (String) → JSON update untuk Web-GCS
- **Loop**: timer cepat `20ms` membaca serial buffer, parse 10 integer.
- **Waypoint fetcher**:
  - saat channel tertentu > 1500, setiap ≥3s akan mengambil posisi GPS terkini,
  - append ke file (format YAML) + publish JSON ke `/robot/waypoints/update`.

Parameter di `launcher/config/sensor.yaml`:
- `/remote_node.ros__parameters.arduino.port`, `baudrate`

#### B) `state` (`state_node` + `state_init`)
- **Peran**: meneruskan command arming/mode ke service MAVROS.
- **Subscriber**:
  - `/robot/arming` (Bool)
  - `/robot/setmode` (String)
- **Service client ke MAVROS**:
  - `/mavros/cmd/arming` (CommandBool)
  - `/mavros/set_mode` (SetMode)
- **Tambahan**: `wait_for_mavros()`
  - menunggu heartbeat `/mavros/state.connected` dan memastikan service tersedia,
  - menggunakan `MultiThreadedExecutor` dan `CallbackGroupType::Reentrant`.

Parameter di `launcher/config/sensor.yaml`:
- `/state_init.ros__parameters.mavros_init_timeout`

#### C) `rplidar` (`lidar_node`)
- **Input**: subscribe `/scan` (`sensor_msgs/LaserScan`)
- **Output**: publish `/robot/lidar` (`std_msgs/Bool`) sebagai trigger “ada objek dekat”
- **Logika**:
  - filter range valid (min/max range) dan filter sudut (wrap-around angle)
  - hitung jumlah titik valid; jika >= `min_point` maka `True`

Parameter di `launcher/config/sensor.yaml`:
- `/lidar_node.ros__parameters.lidar.{min_range,max_range,min_point,min_angle,max_angle}`

Catatan: `rplidar_ros` (package pihak ketiga) biasanya yang mem-publish `/scan`.

---

### 3.4 `mission`
**Fungsi**: high-level mission planner: memilih misi dan mengaktifkan mode navigasi yang sesuai.

Executable (C++):
- `gps_rtk_planner`
- `gps_vision_planner`
- `gps_compass4_planner`
- `gps_compass5_planner`
- `gps_compass6_planner`

**Pola umum planner**:
- **Subscriber** (umumnya):
  - `/mavros/state` (cek connected/armed/mode)
  - `/robot/mode` (auto/ manual)
  - `/robot/position/waypoint` (progress dari GPS navigation)
  - `/robot/compass/success` (untuk varian compass)
  - `/robot/mission/surface` dan `/robot/mission/underwater` (status dari perception)
  - `/detect/left_buoy`, `/detect/right_buoy`, `/robot/trajectory` (untuk konfigurasi misi)
  - `/mission/reset`
- **Publisher**:
  - `/robot/setmode` (String) → men-switch autopilot mode (MANUAL/GUIDED)
  - `/robot/control/gps` (Bool)
  - `/robot/control/compass` (Bool)
  - `/robot/control/vision` (Bool)
  - `/robot/mission` (String) → nama misi (mis. `Floating Ball`, `Surface Imaging`, `Underwater Imaging`, `Docking`)
  - `/detect/left_buoy`, `/detect/right_buoy` (String)

**Inti logika**:
- Planner berjalan cepat (`20ms`) tetapi state & parameter dapat di-reload tiap 1s.
- Planner memakai “state machine” berdasarkan **current waypoint**.
- Menggunakan timer/phase_duration untuk transisi (mis. reverse delay 3s, atau vision phase X detik).
- `enable_underwater_capture` memungkinkan mode “underwater done by sensor” vs “anggap selesai (timer)”.

Parameter mission:
- `launcher/config/mission/*.yaml` mengatur `phase_duration` dan `enable_underwater_capture`.

---

### 3.5 `navigation`
**Fungsi**: low-level navigation/control yang benar-benar menggerakkan robot.

Executable (C++):
- `gps_nav` (`gps_navigation`)
- `compass_nav` (`compass_navigation`)
- `vision_nav` (`vision_navigation`)

#### A) `gps_nav`
- **Subscriber**:
  - `/mavros/global_position/global` (NavSatFix)
  - `/robot/mode`
  - `/robot/control/gps`, `/robot/control/compass`, `/robot/control/vision`
  - `/robot/trajectory` (A/B)
  - `/mission/reset`
- **Publisher**:
  - `mavros/setpoint_position/global` (GeoPoseStamped) → target GPS setpoint
  - `/robot/position/waypoint` (Int32) → progress waypoint
- **Logika**:
  - membaca waypoints dari parameter YAML (`navigation.waypoints_A/B`)
  - menghitung jarak (Haversine) dan bearing untuk yaw
  - jika jarak < `radius`, increment waypoint
  - hanya aktif saat `auto_mode==true` dan `gps_control==true` (serta control lainnya false)

#### B) `compass_nav`
- **Subscriber**:
  - `/mavros/global_position/compass_hdg` (Float64)
  - `/robot/mode`
  - `/robot/control/*`
  - `/robot/position/waypoint` (untuk memilih target heading)
  - `/robot/trajectory` (A/B)
  - `/mission/reset`
- **Publisher**:
  - `/mavros/rc/override` (OverrideRCIn)
  - `/robot/compass/success` (Bool)
- **Logika**:
  - ambil target heading dari parameter (`compass.heading.A/B`)
  - hitung heading error, normalisasi ke [-180,180]
  - putar yaw (RC min/max) sampai error <= tolerance
  - publish success terus-menerus

#### C) `vision_nav`
- **Subscriber**:
  - `/robot/mode`
  - `/robot/control/*`
  - `/robot/lidar` dan `/robot/snapshot` (trigger proximity)
  - `/robot/mission` (String)
  - `/robot/mission/surface` dan `/robot/mission/underwater`
  - `/robot/vision/raw_yaw` (Int32)
  - `/detect/object/bool` (Bool)
  - `/mission/reset`
- **Publisher**:
  - `/mavros/rc/override` (OverrideRCIn)
- **Kontrol**:
  - PID internal (kelas `PID`) untuk menghaluskan kontrol yaw dari `raw_yaw`.
  - throttle & bias steering bisa beda per misi (index 0/1).
  - docking punya “timer finish”: setelah `vision.timer.docking` detik, stop (RC netral).

Parameter navigation ada di:
- `launcher/config/navigation/*.yaml`

---

### 3.6 `perception`
**Fungsi**: vision + imaging (surface/underwater) + recording.

Di package ini ada campuran C++ dan Python.

#### A) `vision.py` (`vision_node`, Python / rclpy)
- **Teknologi**:
  - OpenCV (`cv2`), `cv_bridge`
  - `ultralytics.YOLO`
  - Optimisasi RKNN/NPU (RK3588) via env vars `RKNN_*`
  - Multi-thread worker untuk inference (beberapa instance model)
- **Subscriber**:
  - `/robot/mode`
  - `/robot/control/gps`, `/robot/control/compass`, `/robot/control/vision`
  - `/robot/lidar`
  - `/robot/mission`
  - `/detect/docking`, `/detect/left_buoy`, `/detect/right_buoy`
  - `/mission/reset`
- **Publisher**:
  - `/robot/vision/navigation` (CompressedImage)
  - `/robot/image/surface` (CompressedImage)
  - `/robot/mission/surface` (Bool)
  - `/detect/object/bool` (Bool)
  - `/robot/vision/raw_yaw` (Int32)
  - `/robot/snapshot` (Bool)
- **Logika ringkas**:
  - baca frame kamera → masuk buffer → worker thread inference YOLO
  - hasil deteksi dipakai untuk:
    - menentukan `objects_detected`
    - menghasilkan `raw_yaw` untuk steering
    - menghasilkan `snapshot` (trigger)
  - publish streaming image untuk navigasi
  - jika misi `Surface Imaging` dan trigger terpenuhi → publish foto/stream ke `/robot/image/surface` dan menandai `is_surface`

Parameter penting ada di `launcher/config/perception.yaml`.

#### B) `underwater` (`underwater_node`, C++ / rclcpp)
- **Input**: kamera underwater (OpenCV VideoCapture)
- **Subscriber**:
  - `/robot/mode`, `/robot/control/*`, `/robot/lidar`, `/robot/snapshot`, `/robot/mission`, `/mission/reset`
- **Publisher**:
  - `/robot/image/underwater` (CompressedImage)
  - `/robot/mission/underwater` (Bool)
- **Logika**:
  - publish trigger underwater ketika misi `Underwater Imaging` dan proximity trigger aktif
  - untuk memastikan delivery, publish image beberapa kali (thread + repeat)

#### C) `recorder.py` (`recorder_node`, Python / rclpy)
- **Fungsi**: rekam video dari kamera ke file `.avi`.
- **Teknologi**: OpenCV VideoWriter.
- **Catatan**: recorder ini berdiri sendiri (bukan pipeline ROS image topic).

---

## 4) Topik Kunci (Interface antar Package)

Topic “komando” (dari remote/planner ke node lain):
- `/robot/mode` (Bool) → auto mode ON/OFF
- `/robot/setmode` (String) → mode autopilot (MANUAL/GUIDED)
- `/mission/reset` (Bool)
- `/robot/trajectory` (String) → `A`/`B`

Topic “switch navigasi” (planner → navigation/perception):
- `/robot/control/gps` (Bool)
- `/robot/control/compass` (Bool)
- `/robot/control/vision` (Bool)

Topic “hasil vision” (perception → navigation):
- `/detect/object/bool` (Bool)
- `/robot/vision/raw_yaw` (Int32)
- `/robot/snapshot` (Bool)

Topic “trigger lidar”:
- `/scan` (LaserScan) → dari `rplidar_ros`
- `/robot/lidar` (Bool) → hasil filter dari `sensor/lidar_node`

Topic “aktuasi ke autopilot”:
- `mavros/setpoint_position/global` (GeoPoseStamped) → untuk GPS navigation
- `/mavros/rc/override` (OverrideRCIn) → untuk compass/vision/manual

Telemetry internal:
- `/robot/battery`, `/robot/cpu`, `/robot/gps`, `/robot/cog`, `/robot/sog`, `/robot/date`

---

## 5) Cara Menjalankan (untuk praktikum)

Umumnya pakai `launcher`:

1. Jalankan MAVROS + rosbridge:
   - `ros2 launch launcher mavros.launch.py`
2. Jalankan misi:
   - `ros2 launch launcher gps_rtk.launch.py`
   - atau `ros2 launch launcher gps_vision.launch.py`
   - atau `ros2 launch launcher gps_compass5.launch.py`

Atau jalankan komponen tertentu:
- Monitoring: `ros2 launch launcher monitoring.launch.py`
- Vision: `ros2 launch launcher vision.launch.py`
- Remote+State: `ros2 launch launcher remote.launch.py`
- RPLidar pipeline: `ros2 launch launcher rplidar.launch.py`

---

## 6) Dependensi Pihak Ketiga (`libs/src/*`) (Ringkas untuk mahasiswa)

- **`mavros` (2.12.0)**
  - Node komunikasi MAVLink ↔ ROS 2.
  - Membuat topic `/mavros/*` dan service `/mavros/*`.
- **`mavros_msgs` (2.12.0)**
  - Message/service definition untuk MAVROS.
- **`libmavconn` (2.12.0)**
  - Library koneksi MAVLink (URL mapper, transport).
- **`mavlink` (2025.9.9)**
  - Definisi protokol & message MAVLink.
- **`rplidar_ros` (2.1.4)**
  - Driver ROS 2 untuk Slamtec RPLidar, publish `sensor_msgs/LaserScan`.
- **`serial` (1.2.1)**
  - Library C++ untuk komunikasi serial (dipakai node `remote`).
- **`vision_opencv` / `cv_bridge` / `image_geometry` (3.2.1)**
  - Bridge ROS image ↔ OpenCV, dan util geometri kamera.

---

## 7) Catatan Pedagogis (poin bahasan kelas)

- Arsitektur ROS 2: node-topic-QoS-timer.
- State machine mission: waypoint → fase → switching mode navigasi.
- Pemisahan:
  - planner (high-level) vs navigation (low-level control) vs perception (sensor processing).
- Integrasi autopilot:
  - setpoint global vs RC override.
- Pipeline sensor:
  - lidar raw (`/scan`) → boolean trigger (`/robot/lidar`) → dipakai oleh vision/nav.
- Konsep real-time praktis:
  - loop 20ms untuk kontrol,
  - throttle logging untuk mencegah spam.

---

## 8) Konfigurasi YAML & Hubungan dengan Web Control

Bagian ini menjelaskan bagaimana file-file YAML di `launcher/config/*.yaml` (sensor, perception, mission, navigation, dsb.) dipakai oleh node ROS 2 dan bagaimana nilainya bisa diubah/dikontrol dari Web Control (Web-GCS).

### 8.1 Peran YAML di ROS 2

- **Sumber nilai awal parameter**
  - Setiap node (mis. `remote_node`, `vision_node`, `planner_node`, `gps_navigation`, dll.) memiliki **parameter ROS 2**.
  - File YAML seperti:
    - `config/sensor.yaml`
    - `config/perception.yaml`
    - `config/mission/*.yaml`
    - `config/navigation/*.yaml`
    dibaca oleh **launch file** (di package `launcher`) dan isinya dimasukkan sebagai **nilai awal** parameter node.
- **Struktur umum**
  - Contoh `sensor.yaml`:
    - `/remote_node.ros__parameters.arduino.port`, `baudrate`
    - `/lidar_node.ros__parameters.lidar.{min_range,max_range,min_point,min_angle,max_angle}`
  - Contoh `perception.yaml`:
    - `/vision_node.ros__parameters.config.camera_device`, `width`, `height`, `fps`
    - `/vision_node.ros__parameters.vision.distance.trigger_distance`, dll.
  - Contoh `mission/gps_compass.yaml`:
    - `/planner_node.ros__parameters.phase_duration.{0,1,2,reverse}`
    - `/planner_node.ros__parameters.config.enable_underwater_capture`
- **Prinsip penting**
  - YAML **bukan** “file yang dibaca terus-menerus setiap loop 20ms”.
  - YAML berperan sebagai **template konfigurasi** yang di-load saat node start, lalu diakses sebagai **parameter**:
    - `this->get_parameter("config.camera_device", ...)`
    - `this->get_parameter("phase_duration.0", ...)`

### 8.2 Dari YAML ke Parameter yang Dipakai Node

- **Saat launch dijalankan**
  - Contoh: `ros2 launch launcher gps_vision.launch.py`
  - Launch file akan:
    - membuat node `vision_node`, `gps_navigation`, `gps_vision_planner`, dll.
    - melampirkan file YAML yang sesuai (sensor, perception, mission, navigation).
  - ROS 2 otomatis mengisi parameter node dari file YAML tersebut.
- **Di dalam kode node**
  - Node membaca parameter sekali saat **startup** dan/atau **secara periodik**:
    - Misalnya planner bisa punya timer 1s untuk:
      - membaca ulang parameter (via `get_parameter`),
      - sehingga perubahan parameter runtime (bukan isi file YAML) bisa terambil.
  - Untuk navigation:
    - `gps_nav` membaca parameter `navigation.waypoints_A/B` untuk daftar waypoint.
  - Untuk perception:
    - `vision_node` membaca parameter kamera (`camera_device`, `fps`, dsb.) dan parameter YOLO (`confidence`, `iou_threshold`, dsb.).

### 8.3 Peran Web Control (Web-GCS) terhadap YAML/Parameter

Web Control **tidak langsung mengedit file YAML** di disk, tetapi berinteraksi lewat **ROS topic/service** melalui **rosbridge**:

- **1) Mengubah mode misi & pilihan jalur (trajectory)**
  - Web-GCS publish ke topic:
    - `/robot/trajectory` (`String`: `A`/`B`)
    - `/robot/mission` (`String`: nama misi)
    - `/robot/mode` (`Bool`: auto/manual)
  - Node **mission planner** dan **navigation** memakai kombinasi:
    - parameter dari YAML (mis. `phase_duration`, `heading`, waypoint A/B)
    - input runtime dari Web-GCS (topic di atas)
  - Jadi:
    - **YAML** menentukan angka-angka dasar (durasi fase, radius waypoint, heading, dsb.),
    - **Web Control** memilih skenario mana yang aktif (A/B, tipe misi) lewat topic.

- **2) Waypoint: integrasi YAML ↔ Web Control**
  - `remote_node` punya fitur “waypoint fetcher”:
    - saat channel RC tertentu aktif, node:
      - baca GPS saat itu (`/mavros/global_position/global`),
      - **append** ke file YAML waypoints,
      - publish JSON ke `/robot/waypoints/update`.
  - Web-GCS berlangganan ke topic `/robot/waypoints/update` via rosbridge:
    - menampilkan daftar waypoint di web.
    - sinkron dengan file YAML yang dipakai `gps_nav`.
  - Alur praktis:
    - Operator di lapangan “rekam” waypoint lewat RC → `remote_node` menulis YAML.
    - Web-GCS langsung tahu update-nya via `/robot/waypoints/update`.

- **3) Kamera & parameter vision (camera_device, trigger_distance, dll.)**
  - Nilai default diset di `perception.yaml`:
    - `camera_device`, `fps`, `width`, `height`, `trigger_distance`, dsb.
  - Untuk praktikum/web control, ada dua pola:
    - **Pola A (umum)**:
      - Web-GCS mengirim **command sederhana**:
        - contoh: pilih kamera atas/bawah lewat topic tertentu (mis. `camera_select`),
        - node vision menerjemahkan ke perubahan `VideoCapture` tanpa mengubah YAML.
    - **Pola B (advanced, lewat rosbridge + service parameter)**:
      - Web-GCS memanggil service ROS 2 `/<node>/set_parameters` via rosbridge:
        - misalnya mengganti `config.camera_device` dari 0 ke 2,
        - atau mengganti `vision.distance.trigger_distance`.
      - Node yang sudah mendukung **dynamic parameter** akan menerima event parameter change dan memakai nilai baru tanpa restart.
  - Di materi kelas, fokusnya:
    - paham bahwa:
      - **YAML** = nilai awal,
      - **Web Control** = override runtime (via topic/service) bila dibutuhkan.

- **4) Mission timing & underwater capture**
  - Di `mission/gps_compass.yaml`, parameter:
    - `phase_duration` (tiap fase misi)
    - `enable_underwater_capture` (boleh/tidak underwater pakai sensor sebenarnya)
  - Web-GCS tidak perlu mengedit YAML langsung:
    - Cukup mengirim perintah misi (topic) dan status sensor (lidar, snapshot).
    - Planner memakai kombinasi:
      - parameter durasi dari YAML,
      - sinyal runtime (topic) untuk menentukan kapan pindah fase/selesai misi.

### 8.4 Inti Konsep untuk Praktikum

- **YAML = konfigurasi statis (default)**:
  - Disiapkan sebelum misi: kamera mana yang aktif secara default, daftar waypoint awal, durasi fase, dll.
- **Web Control = konfigurasi dinamis (runtime)**:
  - Mengubah:
    - pilihan mission (GPS/Compass/Vision),
    - jalur A/B,
    - reset mission,
    - perintah perekaman waypoint,
    - dan (bila diaktifkan) sebagian parameter via service `set_parameters`.
- **Node = penghubung keduanya**:
  - Saat start: baca YAML → set parameter.
  - Saat jalan: dengarkan topic dari Web-GCS + (opsional) event perubahan parameter.
  - Karena itu, mahasiswa bisa melihat:
    - bagaimana **file konfigurasi** dan **web UI** saling melengkapi,
    - bukan saling menggantikan.