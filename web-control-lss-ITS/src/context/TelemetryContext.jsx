import React, { createContext, useContext, useState, useEffect, useRef, useCallback } from 'react';

const TelemetryContext = createContext(null);

const DEFAULT_TELEMETRY = {
  timestamp: Date.now() / 1000,
  emergency_armed: false,
  mode: 'MANUAL',
  subsystems: {
    gps: true,
    compass: true,
    vision: true,
    lidar: true,
  },
  navigation: {
    latitude: -7.2825,
    longitude: 112.7953,
    altitude: 1.2,
    heading_deg: 214.0,
    heading_label: '214° SW',
    surge_velocity: 1.45,
    sway_velocity: 0.02,
    yaw_rate: -0.04,
    knots: 2.82,
    roll: 0.5,
    pitch: -0.8,
    yaw: -2.55,
    target_surge_limit: 1.45,
  },
  thrusters: {
    port_rpm: 1450,
    stbd_rpm: 1479,
    port_percent: 72,
    stbd_percent: 74,
    power_watts: 385.0,
  },
  battery: {
    voltage: 25.2,
    current: 15.3,
    percentage: 96.0,
    temperature: 32.4,
    type: 'LiPo 6S',
    capacity_ah: 22.0,
    health: 'Optimal',
  },
  environment: {
    water_temp_c: 28.4,
    ambient_temp_c: 31.0,
    pod_temp_c: 36.2,
    humidity_percent: 68.5,
    ph_level: 7.82,
    cooling_flow_lpm: 3.4,
    cooling_status: 'NORMAL',
  },
  sonar: {
    depth_meters: 4.85,
    frequency_khz: 200,
    bottom_echo_db: -14.2,
    profile_status: 'CHIRP_LOCKED',
  },
  detections: {
    active_target: 'Buoy_04',
    confidence: 0.94,
    target_distance_m: 12.8,
    target_bearing_deg: -15.2,
  },
  mission: {
    current_id: 'NONE',
    status: 'IDLE',
    progress_percent: 0.0,
    active_waypoint_index: 0,
    total_waypoints: 4,
    distance_to_wp_m: 18.4,
  },
};

const DEFAULT_TOPICS = {
  '/cmd_vel': { count: 0, hz: 100.0, latency_ms: 12.0 },
  '/yolo_detector': { count: 0, hz: 30.0, latency_ms: 18.0 },
  '/odom_filtered': { count: 0, hz: 50.0, latency_ms: 8.0 },
  '/sensors/battery_state': { count: 0, hz: 10.0, latency_ms: 4.0 },
  '/sonar/bathymetry_raw': { count: 0, hz: 20.0, latency_ms: 15.0 },
};

export function TelemetryProvider({ children }) {
  const [telemetry, setTelemetry] = useState(DEFAULT_TELEMETRY);
  const [topics, setTopics] = useState(DEFAULT_TOPICS);
  const [isConnected, setIsConnected] = useState(false);
  const [emergency, setEmergency] = useState(false);
  const wsRef = useRef(null);
  const reconnectTimeoutRef = useRef(null);
  const heartbeatIntervalRef = useRef(null);

  // Dynamic host determination: supports both localhost & LAN IP access
  const getBackendBaseUrl = useCallback(() => {
    const host = window.location.hostname || 'localhost';
    return `http://${host}:8000`;
  }, []);

  const getWebSocketUrl = useCallback(() => {
    const host = window.location.hostname || 'localhost';
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    return `${protocol}//${host}:8000/ws/telemetry`;
  }, []);

  // Connect WebSocket
  const connectWebSocket = useCallback(() => {
    if (wsRef.current && (wsRef.current.readyState === WebSocket.OPEN || wsRef.current.readyState === WebSocket.CONNECTING)) {
      return;
    }

    const wsUrl = getWebSocketUrl();
    try {
      const ws = new WebSocket(wsUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        setIsConnected(true);
        // Start keepalive heartbeat (1 Hz)
        if (heartbeatIntervalRef.current) clearInterval(heartbeatIntervalRef.current);
        heartbeatIntervalRef.current = setInterval(() => {
          if (ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'heartbeat', time: Date.now() }));
          }
        }, 1000);
      };

      ws.onmessage = (event) => {
        try {
          const payload = JSON.parse(event.data);
          if (payload.type === 'telemetry_update') {
            if (payload.data) {
              setTelemetry((prev) => ({
                ...prev,
                ...payload.data,
                navigation: { ...prev.navigation, ...(payload.data.navigation || {}) },
                thrusters: { ...prev.thrusters, ...(payload.data.thrusters || {}) },
                battery: { ...prev.battery, ...(payload.data.battery || {}) },
                environment: { ...prev.environment, ...(payload.data.environment || {}) },
                sonar: { ...prev.sonar, ...(payload.data.sonar || {}) },
                detections: { ...prev.detections, ...(payload.data.detections || {}) },
                mission: { ...prev.mission, ...(payload.data.mission || {}) },
                subsystems: { ...prev.subsystems, ...(payload.data.subsystems || {}) },
              }));
            }
            if (payload.topics) {
              setTopics((prev) => ({ ...prev, ...payload.topics }));
            }
            if (payload.emergency !== undefined) {
              setEmergency(payload.emergency);
            }
          }
        } catch {
          // ignore malformed frame
        }
      };

      ws.onerror = () => {
        ws.close();
      };

      ws.onclose = () => {
        setIsConnected(false);
        if (heartbeatIntervalRef.current) clearInterval(heartbeatIntervalRef.current);
        // Auto-reconnect after 2 seconds
        reconnectTimeoutRef.current = setTimeout(() => {
          connectWebSocket();
        }, 2000);
      };
    } catch {
      setIsConnected(false);
      reconnectTimeoutRef.current = setTimeout(() => {
        connectWebSocket();
      }, 3000);
    }
  }, [getWebSocketUrl]);

  useEffect(() => {
    connectWebSocket();
    return () => {
      if (reconnectTimeoutRef.current) clearTimeout(reconnectTimeoutRef.current);
      if (heartbeatIntervalRef.current) clearInterval(heartbeatIntervalRef.current);
      if (wsRef.current) wsRef.current.close();
    };
  }, [connectWebSocket]);

  // REST Control Actions
  const toggleEmergency = useCallback(async (disarm) => {
    try {
      const res = await fetch(`${getBackendBaseUrl()}/api/control/emergency`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ disarm }),
      });
      const data = await res.json();
      setEmergency(disarm);
      return data;
    } catch (e) {
      console.error('Failed to trigger emergency:', e);
      setEmergency(disarm);
    }
  }, [getBackendBaseUrl]);

  const sendCmdVel = useCallback(async (linear_x, angular_z) => {
    try {
      const res = await fetch(`${getBackendBaseUrl()}/api/control/cmd_vel`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ linear_x, angular_z }),
      });
      return await res.json();
    } catch (e) {
      console.error('Failed to send cmd_vel:', e);
    }
  }, [getBackendBaseUrl]);

  const setMode = useCallback(async (mode) => {
    try {
      const res = await fetch(`${getBackendBaseUrl()}/api/control/mode`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode }),
      });
      return await res.json();
    } catch (e) {
      console.error('Failed to set mode:', e);
    }
  }, [getBackendBaseUrl]);

  const toggleSubsystem = useCallback(async (subsystem, enabled) => {
    try {
      const res = await fetch(`${getBackendBaseUrl()}/api/control/subsystems`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ subsystem, enabled }),
      });
      return await res.json();
    } catch (e) {
      console.error(`Failed to toggle subsystem ${subsystem}:`, e);
    }
  }, [getBackendBaseUrl]);

  const setSurgeLimit = useCallback(async (limit) => {
    try {
      const res = await fetch(`${getBackendBaseUrl()}/api/control/surge_limit`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ limit }),
      });
      return await res.json();
    } catch (e) {
      console.error('Failed to set surge limit:', e);
    }
  }, [getBackendBaseUrl]);

  const startMission = useCallback(async () => {
    try {
      const res = await fetch(`${getBackendBaseUrl()}/api/mission/start`, { method: 'POST' });
      return await res.json();
    } catch (e) {
      console.error('Failed to start mission:', e);
    }
  }, [getBackendBaseUrl]);

  const pauseMission = useCallback(async () => {
    try {
      const res = await fetch(`${getBackendBaseUrl()}/api/mission/pause`, { method: 'POST' });
      return await res.json();
    } catch (e) {
      console.error('Failed to pause mission:', e);
    }
  }, [getBackendBaseUrl]);

  const resetMission = useCallback(async () => {
    try {
      const res = await fetch(`${getBackendBaseUrl()}/api/mission/reset`, { method: 'POST' });
      return await res.json();
    } catch (e) {
      console.error('Failed to reset mission:', e);
    }
  }, [getBackendBaseUrl]);

  const value = {
    telemetry,
    topics,
    isConnected,
    emergency,
    toggleEmergency,
    sendCmdVel,
    setMode,
    toggleSubsystem,
    setSurgeLimit,
    startMission,
    pauseMission,
    resetMission,
    backendUrl: getBackendBaseUrl(),
  };

  return (
    <TelemetryContext.Provider value={value}>
      {children}
    </TelemetryContext.Provider>
  );
}

export function useTelemetry() {
  const context = useContext(TelemetryContext);
  if (!context) {
    throw new Error('useTelemetry must be used within a TelemetryProvider');
  }
  return context;
}
