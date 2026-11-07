# Progress Log: Integrasi Backend FastAPI ke ROS 2 & Cockpit Map Planner (LSS-02 USV)

Dokumen ini mencatat rencana, arsitektur, dan rekam jejak cicilan implementasi integrasi antara Backend Gateway (FastAPI), Web Cockpit Frontend, dan ekosistem ROS 2 pada USV LSS-02 ITS Systemcybernetics.

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
[Map Viewport (LocationLiveCard)] ──(Klik Titik Peta)──► [TelemetryContext plannedWaypoints]
                                                                  │
                                                                  ▼
                                                      [Waypoint Plan Card (WeatherCard)]
                                                      (Menampilkan List Lat/Lon & Aksi)
                                                                  │
                                                      [Tombol "Upload ROS 2"]
                                                                  │
                                                                  ▼
                                                      [POST /api/mission/waypoints]
                                                                  │
                                                      [ROS2WaypointParamClient]
                                                                  │ Call Service SetParameters
                                                                  ▼
                                                      [ROS 2 Node: /gps_navigation]
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
- [x] **Cicilan 4: Validasi Logika & Backend Script Testing**
  - [x] Sintaks python terkompilasi bersih (`py_compile` lolos tanpa error).
  - [x] Pengujian modul `ROS2WaypointParamClient` dalam mode simulasi & offline cache sukses.
- [x] **Cicilan 5: Fitur Klik Peta & Sinkronisasi Real-Time ke Card Waypoint Plan**
  - [x] Shared state `plannedWaypoints` dan `isMapPlanningMode` pada `TelemetryContext.jsx`.
  - [x] Fitur klik pada Map (`LocationLiveCard.jsx`): Mendapatkan koordinat lat/lon dari klik event dan menggambar jalur garis GeoJSON secara dinamis.
  - [x] Marker bernomor interaktif dengan popup detail lat/lon di atas peta.
  - [x] Banner & kursor crosshair visual saat mode penambahan waypoint aktif.
  - [x] Sinkronisasi instan ke Card Waypoint Plan (`WeatherCard.jsx`): Menampilkan daftar waypoint berurutan dengan format Latitude & Longitude presisi 6 digit desimal.
  - [x] Tombol hapus satuan (delete per waypoint) dan reset rute.
  - [x] Tombol "Upload ROS 2" untuk deploy langsung parameter waypoint ke `/gps_navigation`.
  - [x] Vite build frontend terverifikasi berhasil (Exit code 0, 72 modules transformed).

---

## 3. Log Catatan Progres Harian

### [Sesi 1] - Inisialisasi & Persiapan Arsitektur
- Meninjau implementasi `gps_nav.cpp` dan deklarasi parameter yang ada.
- Memastikan kompatibilitas Python 3.10 dengan ROS 2 Humble `rclpy`.
- Menyusun format payload parameter yang sesuai dengan ROS 2 standard `rcl_interfaces`.

### [Sesi 2] - Implementasi Modul ROS 2 Parameter Client & C++ Node Update
1. Dibuat file: `web-control-lss-ITS/backend/ros/ros_parameter_client.py`
2. Dimodifikasi: `main/src/navigation/navigation/gps_nav.cpp` (Dynamic parameter declarations & `get_parameter_or`)
3. Dimodifikasi: `web-control-lss-ITS/backend/services/mission_service.py` & `api/mission.py`

### [Sesi 3] - Fitur Interaktif Klik Waypoint Map & Sinkronisasi Card Waypoint Plan
1. **Penyatuan State di `TelemetryContext.jsx`**:
   - Ditambahkan `plannedWaypoints`, `addPlannedWaypoint`, `removePlannedWaypoint`, `clearPlannedWaypoints`, dan `isMapPlanningMode`.
   - Fungsi `loadWaypoints` diperluas untuk menerima parameter `trajectory` ('A'/'B') dan `radius`.
2. **Implementasi Interaktif di `LocationLiveCard.jsx`**:
   - Menambahkan event listener `map.on('click')` yang secara otomatis menangkap latitude & longitude dan menyimpannya ke daftar jalur.
   - Mengubah kursor menjadi `crosshair` saat mode aktif.
   - Mengupdate GeoJSON `mission-route` layer `route-planned` (garis putus-putus beranimasi) secara dinamis dari posisi USV ke seluruh titik waypoint.
   - Merender marker bernomor dengan highlight warna emas dan tooltip koordinat lat/lon.
   - Menambahkan banner indikator: `KLIK PETA: Tambah Waypoint (X WP)`.
3. **Penyempurnaan Tampilan di `WeatherCard.jsx`**:
   - Menampilkan koordinat nyata `Lat: -7.xxxxxx, Lon: 112.xxxxxx` untuk setiap waypoint yang ditambahkan dari peta.
   - Dilengkapi tombol hapus (trash) per waypoint.
   - Tombol `+ Klik Map` untuk aktivasi mode perencanaan rute langsung dari card.
   - Tombol `Upload ROS 2` yang meneruskan seluruh array waypoint ke backend service `SetParameters`.

---

## 4. Panduan Pengujian di Device Target (Testing Steps & Commands)

### Langkah 1: Compile Node C++ di Device Target
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

### Langkah 4: Jalankan Frontend Dashboard di Terminal 3
```bash
cd ~/Documents/LSS_02/LSS-02-ITS-Systemcybernetics/web-control-lss-ITS
npm run dev
```
Buka browser di `http://localhost:5173`.

### Langkah 5: Uji Coba Fitur Klik Waypoint di Web Cockpit
1. Pada card **Location Live** (Map), klik tombol **`+ Klik Waypoint`** di pojok kanan atas map.
2. Kursor akan berubah menjadi `crosshair` dan muncul banner peringatan kuning di atas peta.
3. Klik di beberapa titik pada perairan/area misi:
   - Garis rute putus-putus biru akan otomatis terhubung ke setiap titik yang diklik.
   - Marker bernomor (1, 2, 3...) akan muncul di titik koordinat.
4. Perhatikan card **Waypoint Plan** di sebelah kanan:
   - Setiap titik yang diklik di peta langsung muncul secara real-time di card dengan format **`Lat: -7.xxxxxx, Lon: 112.xxxxxx`**.
   - Anda dapat menghapus titik tertentu menggunakan ikon tong sampah pada baris waypoint.
5. Klik tombol **`Upload ROS 2`** pada card Waypoint Plan.
6. Status akan berubah menjadi `SYNCED TO ROS2`.

### Langkah 6: Verifikasi Parameter di ROS 2 (Terminal 4)
Periksa apakah titik-titik yang baru saja diklik pada web masuk ke parameter node `/gps_navigation`:
```bash
# Cek semua parameter hasil upload dari dashboard web
ros2 param dump /gps_navigation

# Cek jumlah waypoint dan koordinat waypoint pertama
ros2 param get /gps_navigation navigation.waypoints_A.count
ros2 param get /gps_navigation navigation.waypoints_A.waypoint_0.latitude
ros2 param get /gps_navigation navigation.waypoints_A.waypoint_0.longitude
```
