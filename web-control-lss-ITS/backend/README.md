# LSS-02 USV Marine Command Cockpit - Backend

FastAPI Gateway yang menghubungkan UI Cockpit Web dengan subsistem ROS 2 Humble untuk USV LSS-02.

## Struktur Direktori

```text
backend/
├── main.py                     # Entry point FastAPI, Lifespan, CORS, and Routes
├── config.py                   # Pengaturan konfigurasi, port, ROS domain, dan batas aman
├── ros/                        # Integrasi ROS 2 Humble
│   ├── __init__.py
│   ├── ros_connection.py       # Node lifecycle & background executor
│   ├── ros_subscriber.py       # Topic subscriptions & telemetry caching
│   ├── ros_publisher.py        # Cmd_vel, mode, subsystem & emergency publishers
│   └── ros_parameter.py        # Runtime dynamic parameters
├── api/                        # REST API Endpoints
│   ├── __init__.py
│   ├── telemetry.py            # Endpoint sensor, daya, navigasi & diagnostik
│   ├── mission.py              # Endpoint autonomous waypoint & mission state
│   ├── control.py              # Endpoint kemudi manual, surge limit & disarm
│   └── configuration.py        # Endpoint pembacaan dan update parameter
├── websocket/                  # Real-time WebSocket Broadcaster
│   ├── __init__.py
│   └── telemetry_stream.py     # Stream telemetri 10 Hz ke UI cockpit
└── services/                   # Layanan Bisnis & Keamanan
    ├── __init__.py
    ├── mission_service.py      # State machine waypoint mission
    └── safety_service.py       # SIL-3 Emergency Stop & Watchdog keep-alive
```

## Cara Menjalankan Backend

### Cara Cepat (Rekomendasi)
Cukup jalankan script launcher otomatis yang sudah disediakan:
```bash
cd web-control-lss-ITS/backend
./start_backend.sh
```
Script ini akan otomatis menginisialisasi environment ROS 2 Humble dan menjalankan backend dengan Python yang sesuai.

### Cara Manual

#### 1. Menjalankan Langsung dengan ROS 2 Humble (Python 3.10)
```bash
cd web-control-lss-ITS/backend
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

#### 2. Menjalankan dalam Mode Simulasi / Standalone (Python Default)
```bash
cd web-control-lss-ITS/backend
python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

## Dokumentasi API Interaktif (Swagger UI)
Setelah backend berjalan, buka browser di:
- **Swagger UI**: [http://localhost:8000/docs](http://localhost:8000/docs)
- **ReDoc**: [http://localhost:8000/redoc](http://localhost:8000/redoc)
- **WebSocket Stream**: `ws://localhost:8000/ws/telemetry`
