from pydantic_settings import BaseSettings
from typing import List


class Settings(BaseSettings):
    PROJECT_NAME: str = "LSS-02 USV Marine Command Cockpit API"
    VERSION: str = "1.0.0"
    DEBUG: bool = True
    
    # Server Host & Port
    HOST: str = "0.0.0.0"
    PORT: int = 8000
    
    # CORS
    CORS_ORIGINS: List[str] = [
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "http://192.168.1.15:3000",
        "http://192.168.52.153:3000",
        "*"
    ]
    
    # ROS 2 Settings
    ROS_DOMAIN_ID: int = 0
    ROS_NODE_NAME: str = "usv_web_gateway"
    ROSBRIDGE_WS_URL: str = "ws://localhost:9090"
    
    # Telemetry Stream Rate (Hz)
    TELEMETRY_STREAM_HZ: float = 10.0
    
    # USV Defaults
    DEFAULT_CRUISE_SPEED: float = 2.0  # m/s
    DEFAULT_ACCEPTANCE_RADIUS: float = 3.5  # meters
    MAX_LINEAR_VELOCITY: float = 4.0  # m/s
    MAX_ANGULAR_VELOCITY: float = 1.5  # rad/s
    DEFAULT_FRAME_ID: str = "map"
    
    # Watchdog & Safety
    WATCHDOG_TIMEOUT_SEC: float = 2.0
    CRITICAL_BATTERY_PERCENT: float = 15.0

    class Config:
        env_file = ".env"
        extra = "ignore"


settings = Settings()
