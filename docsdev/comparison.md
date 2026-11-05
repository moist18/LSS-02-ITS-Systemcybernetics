# Analisis Teknologi dari `lss_02` untuk Diterapkan ke `ASV-2025`

Berdasarkan analisis file kode dalam folder `lss_02`, terdapat beberapa teknologi dan pendekatan sistem arsitektur yang sangat potensial untuk diadopsi ke dalam pengembangan kapal otonom `ASV-2025`.

## 1. Integrasi Web UI dengan ROS 2 via Topik String Sederhana (roslibjs)
Di dalam `lss_02/ros2_ws/src/lss_autonomy/lss_autonomy/mission_control.py`, terlihat pendekatan yang disederhanakan untuk mengintegrasikan Node ROS 2 dengan Web Dashboard.
- **Pendekatan `lss_02`:** Menggunakan publish/subscribe data berbasis `std_msgs/String` (contoh: status `'start'`, `'stop'`, `'restart'`) melalui topik `/autonomy/mission_manager/state_cmd` yang mempermudah kendali lewat *Web UI*. Ini menghindari keharusan membangun file *Custom Service* (SRV) khusus yang lebih kompleks untuk di-*bridge* menggunakan `roslibjs`.
- **Penerapan di `ASV-2025`:** Karena `ASV-2025` juga mungkin memiliki Control Dashboard / Mission Planner berbasis antarmuka web, pendekatan pengiriman kendali state mesin (start mission, stop mission, return to launch) memakai bridge JSON/String sederhana ini bisa mempermudah koneksi Next.js/React frontend ke sistem navigasi utama.

## 2. Komunikasi Serial Langsung (Direct Serial Communication) ke Aktuator
Modul kontrol `SteeringNode` pada `lss_02` menunjukkan mekanisme aktuasi diferensial (atau kemudi) yang dikirim langsung melalui USB ke mikrokontroler aktuator.
- **Pendekatan `lss_02`:** Menggunakan modul `pyserial` Python (`serial.Serial('/dev/ttyUSB0', baudrate=115200)`) untuk melakukan translate perintah `geometry_msgs/Twist` (`linear.x`, `angular.z`) menjadi representasi sinyal PWM (Range `1000 - 2000`). Data PWM ini kemudian ditransfer menggunakan protokol byte sederhana `b'its' + speed_bytes + angle_bytes`.
- **Penerapan di `ASV-2025`:** Jika ASV-2025 saat ini belum memiliki driver aktuator yang mapan, arsitektur *Direct-to-Serial Handler Node* seperti ini sangat efisien. Alur ini menjembatani hasil komputasi kontrol pada OS Tingkat Tinggi (Linux / ROS2) secara langsung ke MCU (seperti Arduino atau STM32) untuk Pulse Width Modulation ESC/Servo Motor di *thruster* kapal.

## 3. Komponen Web UI Modern (Next.js & Tailwind CSS)
Folder `lss_02/web_ui` mengungkap adanya sebuah tumpukan frontend yang digunakan.
- **Pendekatan `lss_02`:** Menggunakan ekosistem web modern (seperti yang terlihat dari `package.json`, `next.config.ts`, `postcss.config.mjs`) yang mengindikasikan penggunaan Next.js, React, dan Tailwind CSS.
- **Penerapan di `ASV-2025`:** Anda dapat memigrasi atau membangun Dashboard GCS (Ground Control Station) ASV-2025 menggunakan Next.js + Tailwind CSS. Hal ini memberikan tampilan yang responsif, estetika modern, serta kemudahan *deployment* untuk *monitoring* telemetri kapal, map *waypoints*, dan status sensor-sensor on-board secara *real-time*.

## 4. Pemisahan Autonomy dan Controller (Separation of Concerns)
Struktur ROS 2 pada `lss_02` memisahkan secara eksplisit node pengambil keputusan (*Autonomy/Mission Manager*) dengan eksekusi level bawah (*Controller*).
- **Pendekatan `lss_02`:** Paket `lss_autonomy` berfokus pada status misi kapal (`PathFollower` logic, mengecek apakah target WP terpenuhi), dan menghasilkan output referensi berupa instruksi kecepatan (*Twist*). Di tempat terpisah, paket `lss_controller` hanya memedulikan bagaimana mengatur persentase aktuator (`percent_to_pwm`) untuk mematuhi *Twist* referensi tersebut.
- **Penerapan di `ASV-2025`:** Sangat disarankan untuk mengamankan kemandirian paket di ASV-2025: satu sistem khusus algoritma Waypoint Navigation & Obstacle Avoidance, sementara package terpisah khusus *Hardware Abstract Layer* / Controller (mengurus PID, dinamika lambung, thruster). Jika hardware ASV-2025 nanti diganti, sistem Autonomy tidak perlu ditulis ulang.

## 5. Koreksi Navigasi Berbasis CTE (Cross Track Error)
Meskipun kelas `PathFollower` pada `lss_02` (dalam file `mission_control.py`) hanya memberikan kerangka kerja (*mock-up*) dengan nilai error yang mengembalikan `0.0`, gagasan dasar koreksi lintasannya sudah terlihat. Pengendalian ASV sangat cocok menggunakan perhitungan **Cross Track Error (CTE)**.
- **Konsep:** CTE mengukur jarak tegak lurus kapal terhadap garis lurus antara dua *waypoint*. Daripada kapal hanya sekadar menatap arah *waypoint* berikutnya (yang membuatnya rentan terbawa arus/angin menyimpang dari jalurnya), kapal akan terus memperbarui kemudi (koreksi *heading*) guna meminimalkan CTE ini.
- **Penerapan di `ASV-2025`:** 
  1. **LOS (Line of Sight) Guidance:** Anda dapat mengimplementasikan algoritma LOS berbasis *Lookahead Distance*, di mana referensi *heading* target dikalkulasi sedemikian agar meminimalkan CTE.
  2. **PID pada CTE:** Hasil selisih target *heading* dari algoritma LOS kemudian dilempar menjadi setpoint PID steering yang pada arsitektur `lss_02` ini dikeluarkan berupa `msg.angular.z = self.error`.
  3. Hal ini akan diteruskan ke node aktuasi diferensial/kemudi yang ada, menstabilkan perjalanan perahu tetap pada garis trek yang spesifik alih-alih mengayun ke kiri-kanan secara sembarangan.

## 6. Feedback Loop & Finite State Machine (FSM) Misi
Membedah `page.tsx` dari `web_ui`, terdapat status balikan *real-time* ke dasbor yang memberi tahu apakah mission sedang `running`, `stopped`, atau `unknown`.
- **Penerapan di `ASV-2025`:** Kelancaran misi ASV tidak hanya soal kapal mengikuti jalur, melainkan _situational awareness_. Anda diwajibkan menyusun **Finite State Machine (FSM)** di *Mission Manager* (misalnya state: `STANDBY`, `WAYPOINT_NAV`, `OBSTACLE_AVOIDANCE`, `RTL` / Return to Launch). State ini wajib dipublish secara rutin ke ROS topic (*feedback loop*), agar antarmuka Ground Control Station bisa bereaksi dan melarang operator mengirim perintah *start* ketika kapal masih berstatus *avoidance*.

## 7. Dynamic Parameter Tuning (Penyesuaian Parameter Langsung)
Antarmuka web di `lss_02` merancang slider khusus `Speed Target`. 
- **Penerapan di `ASV-2025`:** Sering kali arus laut memaksa kita menaikkan tenaga gas *(cruising speed)* secara mendadak atau melembutkan rotasi *PID* tanpa sempat me-*restart* Node ROS 2. Oleh karena itu, pasangkan fitur `rqt_reconfigure` atau buatlah penerima `/autonomy/parameters` pada node kontrol ASV Anda. Hal ini mengizinkan operator menggeser slider *speed* atau merombak nilai *Kp, Ki, Kd* (gain) mesin langsung dari UI *browser* saat sedang berlayar, sehingga koreksi pergerakan menjadi sangat halus.

## 8. Rekomendasi Struktur FSM Sistematis (Proposed FSM)
Berdasarkan analisis file `gps_vision_planner.cpp`, sistem Anda saat ini menggunakan nomor waypoint sebagai penanda state. Untuk meningkatkan reliabilitas (terutama saat terjadi eror atau butuh interupsi manual), berikut adalah draf struktur FSM berbasis `enum` yang direkomendasikan:

```cpp
// Definisi State Global untuk Kontrol Misi
enum class MissionState {
    STANDBY,            // Kapal diam, menunggu instruksi
    NAVIGATING_GPS,     // Sedang menuju waypoint menggunakan GPS
    VISION_ALIGNMENT,   // Sedang melakukan koreksi visual (misal: Floating Ball)
    SURFACE_IMAGING,    // State khusus pengambilan gambar permukaan
    UNDERWATER_IMAGING, // State khusus misi bawah air
    EMERGENCY_HOLD,     // Berhenti mendadak jika sensor bermasalah
    RETURN_TO_LAUNCH    // Kembali ke titik awal secara otonom
};

// Implementasi di PlannerNode
MissionState current_state = MissionState::STANDBY;

// Logika Transisi di mission_planner()
void mission_planner() {
    switch (current_state) {
        case MissionState::STANDBY:
            if (auto_mode && is_armed) current_state = MissionState::NAVIGATING_GPS;
            break;
            
        case MissionState::NAVIGATING_GPS:
            gps_navigation();
            // Transisi ke Vision jika mencapai waypoint tertentu (misal WP 0)
            if (current_waypoint == 0) current_state = MissionState::VISION_ALIGNMENT;
            break;
            
        case MissionState::VISION_ALIGNMENT:
            vision_navigation();
            // Kembali ke GPS jika durasi habis atau target tercapai
            if (elapsed_time >= phase_duration) current_state = MissionState::NAVIGATING_GPS;
            break;
            
        // ... penanganan state lainnya ...
    }
}
```

### Keuntungan Implementasi Ini:
1. **Readable:** Kode lebih mudah dibaca daripada sekadar angka `case 0`, `case 1`.
2. **Error Handling:** Anda bisa langsung melempar `current_state = MissionState::EMERGENCY_HOLD` jika sensor GPS atau Kamera mendeteksi anomali, tanpa peduli sedang berada di waypoint mana.
3. **Telemetry Dashboard:** Status `MissionState` ini bisa dipublish sebagai String ke `/robot/mission_status` agar Dashboard Next.js Anda bisa menampilkan teks "NAVIGATING" atau "EMERGENCY" secara eksplisit.

## Kesimpulan
Pendekatan praktis arsitektur ROS 2 yang terhubung *seamless* dengan antarmuka web modern (Next.js) yang ada di `lss_02` sangat ideal untuk `ASV-2025`. Kombinasi **ROSBridge+Next.js** untuk sisi komunikasi operator, serta **Python PySerial Node** sebagai jembatan *hardware level* (ESC/Servo), mempercepat laju iterasi pengembangan tanpa terjebak *overhead* sistem.
