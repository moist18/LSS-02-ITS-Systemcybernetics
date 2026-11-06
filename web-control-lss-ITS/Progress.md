# Progress Log: Integrasi Backend FastAPI ke ROS 2 (LSS-02 USV)

Dokumen ini mencatat rencana, arsitektur, dan rekam jejak cicilan implementasi integrasi antara Backend Gateway (FastAPI) dan ekosistem ROS 2 pada USV LSS-02 ITS Systemcybernetics.

---

## 1. Arsitektur & Target Spesifikasi

### Target Node ROS 2:
- **Node**: `/gps_navigation` (berasal dari `main/src/navigation/navigation/gps_nav.cpp`)
- **Mekanisme Parameter**:
  - `navigation.radius` (*double*): Radius batas toleransi waypoint (default: 2.0 meter).
  - `navigation.waypoints_<A|B>.count` (*int*): Jumlah total waypoint aktif.
  - `navigation.waypoints_<A|B>.waypoint_<index>.latitude` (*double*): Koordinat lintang (-90 s.d. 90).
  - `navigation.waypoints_<A|B>.waypoint_<index>.longitude` (*double*): Koordinat bujur (-180 s.d. 180).
  - `navigation.waypoints_<A|B>.waypoint_<index>.altitude` (*double*): Ketinggian (default 0.0).
- **Service Endpoint ROS 2**:
  - `/<target_node>/set_parameters` (`rcl_interfaces/srv/SetParameters`)
  - `/<target_node>/get_parameters` (`rcl_interfaces/srv/GetParameters`)

### Alur Data:
```
[Web UI / Cockpit]
        │ POST /api/mission/waypoints (JSON payload)
        ▼
[FastAPI Backend - api/mission.py]
        │
        ▼
[MissionService & ROS2WaypointParamClient]
        │ Call Service /gps_navigation/set_parameters
        ▼
[ROS 2 Node: /gps_navigation] ──► [MAVROS / Pixhawk Autopilot]
```

---

## 2. Status Checklist Implementasi

- [x] **Inisiasi & Analisis Arsitektur**: Analisis skema parameter di `gps_vision.yaml` dan `gps_nav.cpp`.
- [x] **Cicilan 1: ROS 2 Waypoint Parameter Client (`ros_parameter_client.py`)**
  - [x] Implementasi class `ROS2WaypointParamClient` dengan service client `SetParameters`.
  - [x] Validasi range geografis WGS84 (Lat: [-90, 90], Lon: [-180, 180]).
  - [x] Mekanisme in-memory fallback saat ROS 2 offline/simulated mode.
  - [x] Dukungan switch trajectory (`A` atau `B`) dan update radius parameter.
  - [x] Export modul di `web-control-lss-ITS/backend/ros/__init__.py`.
- [x] **Cicilan 2: Kompatibilitas Dinamis di Node C++ (`gps_nav.cpp`)**
  - [x] Tambahkan `allow_undeclared_parameters(true)` dan `automatically_declare_parameters_from_overrides(true)` pada constructor `GPSNavigation`.
  - [x] Perbarui `load_waypoints` dan `load_parameter` menggunakan `get_parameter_or` agar penambahan parameter baru secara dinamis via service tidak memicu exception `ParameterNotDeclaredException`.
- [x] **Cicilan 3: Integrasi Service & REST API FastAPI**
  - [x] Hubungkan `mission_service.py` (`set_waypoints` & `sync_waypoints_to_ros`) ke `ROS2WaypointParamClient`.
  - [x] Update endpoint `POST /api/mission/waypoints` untuk parameter upload trajectory `A`/`B` dan `radius`.
  - [x] Tambah endpoint `GET /api/mission/waypoints/ros` untuk inspeksi parameter waypoint aktif & status sinkronisasi.
  - [x] Tambah endpoint `POST /api/mission/waypoints/sync` untuk sinkronisasi ulang manual ke ROS 2.
- [x] **Cicilan 4: Validasi Logika & Script Pengujian**
  - [x] Sintaks python terkompilasi bersih (`py_compile` lolos tanpa error).
  - [x] Pengujian modul `ROS2WaypointParamClient` dalam mode simulasi & offline cache sukses.
  - [x] Menyusun panduan command pengujian end-to-end untuk dijalankan di device target.

---

## 3. Log Catatan Progres Harian

### [Sesi 1] - Inisialisasi & Persiapan Arsitektur
- Meninjau implementasi `gps_nav.cpp` dan deklarasi parameter yang ada.
- Memastikan kompatibilitas Python 3.10 dengan ROS 2 Humble `rclpy`.
- Menyusun format payload parameter yang sesuai dengan ROS 2 standard `rcl_interfaces`.

### [Sesi 2] - Implementasi Modul ROS 2 Parameter Client & C++ Node Update
1. Dibuat file baru: `web-control-lss-ITS/backend/ros/ros_parameter_client.py`
   - Berisi singleton `ros2_waypoint_param_client`.
   - Menggunakan service ROS 2 `/{target_node}/set_parameters` tipe `rcl_interfaces/srv/SetParameters`.
   - Menghasilkan parameter:
     - `navigation.waypoints_<trajectory>.count`
     - `navigation.waypoints_<trajectory>.waypoint_<index>.latitude`
     - `navigation.waypoints_<trajectory>.waypoint_<index>.longitude`
     - `navigation.waypoints_<trajectory>.waypoint_<index>.altitude`
     - `navigation.radius`
   - Dilengkapi validasi WGS84, timeout handling (3 detik), dan auto fallback jika ROS 2 belum siap.
2. Dimodifikasi: `main/src/navigation/navigation/gps_nav.cpp`
   - Node constructor kini mengaktifkan `allow_undeclared_parameters(true)` agar waypoint berapapun jumlahnya bisa di-set saat runtime.
   - Menggunakan `get_parameter_or` untuk pembacaan nilai yang aman terhadap runtime updates.
3. Dimodifikasi: `web-control-lss-ITS/backend/services/mission_service.py` & `api/mission.py`
   - Menambahkan integrasi parameter sync saat `POST /api/mission/waypoints` dipanggil.
   - Menambahkan rute inspeksi `GET /api/mission/waypoints/ros`.

---

## 4. Panduan Pengujian di Device Target (Testing Steps & Commands)

Berikut adalah urutan command untuk pengujian di device (tanpa build di remote agent):

### Langkah 1: Compile Node C++ di Device Target
Jalankan di terminal robot (device asli):
```bash
cd ~/Documents/LSS_02/LSS-02-ITS-Systemcybernetics/main
source /opt/ros/humble/setup.bash
colcon build --packages-select navigation
source install/setup.bash
```

### Langkah 2: Jalankan Node `gps_navigation` di Terminal 1
```bash
source ~/Documents/LSS_02/LSS-02-ITS-Systemcybernetics/main/install/setup.bash
ros2 run navigation gps_nav
```

### Langkah 3: Jalankan Backend Gateway di Terminal 2
```bash
cd ~/Documents/LSS_02/LSS-02-ITS-Systemcybernetics/web-control-lss-ITS/backend
chmod +x start_backend.sh
./start_backend.sh
```

### Langkah 4: Uji Coba Upload Waypoint via REST API di Terminal 3
Kirimkan HTTP POST ke backend untuk mengunggah waypoint:
```bash
curl -X POST http://localhost:8000/api/mission/waypoints \
  -H "Content-Type: application/json" \
  -d '{
    "trajectory": "A",
    "radius": 2.5,
    "sync_to_ros": true,
    "waypoints": [
      {"lat": -7.2825, "lon": 112.7953, "alt": 0.0, "action": "SURVEY"},
      {"lat": -7.2830, "lon": 112.7960, "alt": 0.0, "action": "PASS"},
      {"lat": -7.2835, "lon": 112.7970, "alt": 0.0, "action": "DOCK"}
    ]
  }'
```

### Langkah 5: Verifikasi Parameter di ROS 2
Periksa apakah parameter sudah berhasil masuk ke node `/gps_navigation`:
```bash
# Cek semua parameter pada node gps_navigation
ros2 param dump /gps_navigation

# Atau cek nilai spesifik
ros2 param get /gps_navigation navigation.waypoints_A.count
ros2 param get /gps_navigation navigation.waypoints_A.waypoint_0.latitude
ros2 param get /gps_navigation navigation.waypoints_A.waypoint_0.longitude
ros2 param get /gps_navigation navigation.radius
```

### Langkah 6: Verifikasi via Endpoint Monitoring Backend
```bash
curl -X GET "http://localhost:8000/api/mission/waypoints/ros?trajectory=A"
```
