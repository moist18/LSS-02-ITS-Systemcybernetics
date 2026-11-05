# 🌊 LSS-02 Marine Command Cockpit

Sistem Antarmuka **Oceanic Telemetry & Autonomous Surface Control (USV / ASV)** berbasis **React** dan **Tailwind CSS**. Dashboard ini dirancang khusus untuk memonitor dan mengontrol kapal tanpa awak (*Autonomous Surface Vehicle*) dengan standar kokpit kelautan modern, terinspirasi langsung dari rancangan antarmuka [`screen.png`](./screen.png) dan dikembangkan lebih lanjut dari kode dasar [`code.html`](./code.html) serta [`DESIGN.md`](./DESIGN.md).

---

## 📸 Tampilan Antarmuka

| Light Mode (Default Cockpit) | Dark Mode (Night Nav) |
| :---: | :---: |
| Bersih, fokus pada visualisasi telemetri siang hari | Kontras tinggi, nyaman untuk operasi malam hari |

---

## 🛠️ Tech Stack & Ekosistem

- **Framework**: [React 19](https://react.dev/) + [Vite](https://vitejs.dev/)
- **Styling**: [Tailwind CSS v3](https://tailwindcss.com/) dengan custom cockpit shadow & oceanic color palette
- **Tipografi**:
  - `Plus Jakarta Sans`: Tipografi modern UI display & labels
  - `JetBrains Mono`: Metrik data, koordinat geospasial, frekuensi, dan nilai telemetri
- **Ikonografi**: Google [Material Symbols Outlined](https://fonts.google.com/icons) & [Lucide React](https://lucide.dev/)
- **Protokol Terintegrasi**: ROS 2 (Robot Operating System), NMEA-0183/2000, RTK-GPS, Acoustic Ping Sonar

---

## 🗂️ Struktur Proyek

```text
control-lss-ITS/
├── index.html                  # HTML entry point dengan font preloading (Jakarta Sans, JetBrains Mono)
├── package.json                # Dependensi proyek & NPM scripts
├── postcss.config.js           # PostCSS configuration
├── tailwind.config.js          # Konfigurasi tema cockpit, font, bayangan, & dark mode
├── vite.config.js              # Vite server & build configuration
├── README.md                   # Dokumentasi lengkap proyek
├── screen.png                  # Desain acuan asli
├── code.html                   # Sumber kode dasar prototipe
├── DESIGN.md                   # Spesifikasi desain, warna, & token tipografi
└── src/
    ├── main.jsx                # React root mount point
    ├── App.jsx                 # Main cockpit layout (Grid 12-kolom & integrasi state)
    ├── index.css               # Base styling, custom scrollbars, dan range sliders
    └── components/             # Komponen modular UI
        ├── Sidebar.jsx             # Navigasi vertikal gelap kokpit kiri
        ├── Header.jsx              # Bar atas, ROS search, dark/light switch, profil
        ├── LocationLiveCard.jsx    # Feed kamera POV, peta kontur batimetri, & HUD
        ├── EmergencyCard.jsx       # Switch keamanan instan SIL-3 Disarm
        ├── WeatherCard.jsx         # Waypoint list & konfigurasi path planning via ROSBridge
        ├── PodCoolingCard.jsx      # Thermal Jetson Orin, fan speed, & pompa pendingin
        ├── RosTopicsCard.jsx       # Tabel feed live topik ROS 2 (/cmd_vel, /yolo, dll)
        ├── SonarPlayerCard.jsx     # Pemutar akustik Chirp Sonar 200kHz & visualizer
        ├── BaseGatewayCard.jsx     # Status koneksi menara pemancar darat (gateway)
        ├── PowerBankCard.jsx       # Telemetri baterai LiPo, tegangan, daya jangkau
        ├── ThrustHeadingCard.jsx   # Kontrol slider surge speed & RPM dual thruster
        └── AddTelemetryModal.jsx   # Dialog modal untuk menambahkan stream topik ROS
```

---

## 🚀 Fitur Unggulan & Detail Komponen

### 1. **Sidebar Navigasi Ikonik Kokpit (`Sidebar.jsx`)**
- Berlatar belakang gelap pekat (`#14171d`) menyerupai konsol navigasi kapal modern.
- Dilengkapi tombol navigasi dengan efek highlight melingkar:
  - 🌐 **Tactical Cockpit**: Tampilan utama telemetri dan kendali.
  - ⚙️ **Actuator Dynamics**: Diagnostik sistem aktuator dan thruster.
  - 📅 **Mission Schedule**: Manajemen waypoint dan jadwal pelayaran.
  - 🔧 **Vessel Config**: Pengaturan konfigurasi hardware kapal.
  - 🌊 **Bathymetry & Sonar**: Analisis data kedalaman laut dan profil sonar.
  - 📡 **Acoustic Sonar Link**: Pengaturan transponder akustik bawah air.
- Tombol **Disconnect / Safety Exit** di bagian bawah untuk pemutusan darurat.

### 2. **Header Bar Dinamis (`Header.jsx`)**
- **Command / Search Pill**: Input pencarian interaktif untuk memfilter topik ROS 2 atau perintah suara (*voice commands*).
- **Control Panel & Telemetry Action**: Akses cepat ke pengaturan sistem dan tombol `+ Add telemetry`.
- **Light / Dark Mode Switch**: Tombol beralih tema adaptif yang sinkron dengan seluruh komponen kartu.
- **Notification Badge**: Indikator lonceng dengan unread ping dot biru.
- **Operator Badge & Date**: Menampilkan profil *Capt. Alex* dan tanggal misi aktif (*22 May*).

### 3. **Location Live & Tactical Navigation Card (`LocationLiveCard.jsx`)**
- **3 Mode Tampilan**:
  1. *Optical Camera Feed*: Tampilan kamera depan USV waktu-nyata di laut.
  2. *Bathymetric Vector Map*: Peta kontur kedalaman laut interaktif dengan jalur waypoint.
  3. *Tactical HUD Navigation*: Overlay instrumen penerbangan/kelautan (*Artificial Horizon, Pitch Ladder, Heading Reticle*).
- **Kontrol Zoom (+/-)**: Pengaturan pembesaran tampilan secara fleksibel.
- **Deteksi AI Objek**: Penanda target otomatis `BUOY-04 [94%] • 18.4m` dengan radar pulse hijau.
- **Waypoint Tracker**: Menampilkan `WP-02 (3.1 nm)` dan bilah progres perjalanan menuju *Harbor Station North* (5.1 km).

### 4. **Emergency SIL-3 Disarm Card (`EmergencyCard.jsx`)**
- Kartu tombol darurat independen dengan status visual.
- Saklar toggle pengaman dengan konfirmasi instan sebelum mematikan seluruh daya thruster dan menurunkan jangkar pengaman.

### 5. **Waypoint & Path Planning Card (`WeatherCard.jsx`)**
- Menampilkan daftar waypoint dari konfigurasi rute aktif seperti `Harbor Survey Alpha`, `Coastal Perimeter`, dan `Bathymetry Grid`.
- Popup konfigurasi untuk memilih route, planner, frame ID, kecepatan jelajah, acceptance radius, dan mode loop.
- Menyiapkan preview payload ROSBridge `SetParametersAtomically` melalui namespace parameter path planner.
- Integrasi pengiriman ke ROSBridge masih disiapkan; endpoint default yang ditampilkan adalah `ws://localhost:9090`.

### 6. **Pod Cooling Management (`PodCoolingCard.jsx`)**
- Memonitor suhu komputasi *NVIDIA Jetson Orin* (46°C).
- Saklar toggle pendingin aktif.
- Visualisasi tingkat kecepatan kipas (*Fan RPM: 2,400*) dan sirkulasi pendingin cair (*Pump: 85%*).

### 7. **ROS 2 Topic Playlist (`RosTopicsCard.jsx`)**
- Ditampilkan di bagian atas kolom kanan dashboard dan terhubung dengan pencarian pada header.
- Monitoring feed data komunikasi robotika dengan 3 tab: `Live`, `Filtered`, dan `Alerts`.
- Menampilkan topik utama:
  - `/cmd_vel`: Data linier velocity dan angular yaw rate (100 Hz, 12ms).
  - `/yolo_detector`: Hasil inferensi deteksi pelampung dan rintangan AI (30 Hz, 18ms).
  - `/odom_filtered`: Estimasi posisi filter odometri terintegrasi (50 Hz, 08ms).
- Fitur penanda favorit (love toggle) dan status WebSocket bridge `ws://localhost:9090`.

### 8. **Chirp Sonar 200kHz Acoustic Player (`SonarPlayerCard.jsx`)**
- Modul pemantauan akustik berdesain gelap terintegrasi.
- Animasi gelombang laut dan *equalizer bar* yang membal aktif saat pemutaran berjalan.
- Penghitung waktu aktif dan kedalaman dasar laut (*DEPTH: 24.8m*).
- Pengatur penguatan sinyal (*Gain Level -10dB / +10dB*) dengan tampilan persentase.

### 9. **Base Gateway & Power Bank Sub-Cards**
- **Base Gateway (`BaseGatewayCard.jsx`)**: Pemantau link radio pangkalan darat, visual menara ganda dengan LED hijau, serta metrik latency (12ms) dan bandwidth (48.2M).
- **Power Bank (`PowerBankCard.jsx`)**: Baterai otomotif vertikal dengan indikator persentase hijau (96%), sisa jarak jelajah (48.2 nm), tegangan pack (25.2 V), total jarak pelayaran (631.2 nm), dan status termal LiPo (38°C).

### 10. **Thrust & Heading Alignment (`ThrustHeadingCard.jsx`)**
- Slider **Target Surge Limit** (0.0 m/s – 2.0 m/s):
  - Mengonversi kecepatan m/s ke knot secara langsung.
  - Secara cerdas menghitung dan menyelaraskan putaran RPM motor *Port (T1)* dan *Starboard (T2)* secara proporsional.
- Dilengkapi indikator arah haluan kapal `HDG 214° SW`.

### 11. **Add Telemetry Stream Modal (`AddTelemetryModal.jsx`)**
- Form interaktif untuk operator mendaftarkan sensor baru:
  - Nama topik ROS 2 (`/imu/data_raw`, `/sensors/depth`, dll.)
  - Tipe pesan interface (`sensor_msgs/NavSatFix`, `geometry_msgs/Twist`, dll.)
  - Frekuensi publikasi stream data (Hz).

---

## 💻 Panduan Menjalankan Aplikasi

### Kebutuhan Sistem:
- **Node.js**: v18.0.0 atau yang lebih baru
- **NPM**: v9.0.0 atau yang lebih baru

### 1. Instalasi Dependensi
Jika Anda baru mengkloning atau ingin memperbarui pustaka dependensi:
```bash
npm install
```

### 2. Menjalankan Server Pengembangan (Dev)
```bash
npm run dev
```
Setelah berjalan, buka browser di alamat:
👉 **`http://localhost:3000`**

### 3. Membangun Proyek untuk Produksi (Production Build)
```bash
npm run build
```
Hasil kompilasi dan optimasi berkas akan disimpan di dalam direktori `dist/`.

### 4. Meninjau Hasil Build (Preview)
```bash
npm run preview
```

---

## 🎨 Konfigurasi Desain & Penyesuaian

Jika Anda ingin menyesuaikan warna atau token desain cockpit, Anda dapat memodifikasi file [**`tailwind.config.js`**](./tailwind.config.js):

```javascript
colors: {
  cockpit: {
    bg: '#f1f3f6',      // Background kokpit terang
    dark: '#14171d',    // Background elemen gelap / sidebar
    card: '#ffffff',    // Latar kartu instrumen
    accent: '#0ea5e9',  // Aksen maritim (sky-500)
    border: '#e2e8f0',  // Garis pembatas kartu
  }
}
```

---

## 📄 Lisensi & Hak Cipta
Dikembangkan untuk sistem telemetri kendali laut **LSS ITS (Laboratorium Sistem Siber/Maritim)**.
Dilisensikan di bawah [ISC License](./package.json).
