import React, { useState, useEffect, useRef, useCallback } from 'react';
import * as maptilersdk from '@maptiler/sdk';
import '@maptiler/sdk/dist/maptiler-sdk.css';
import { useTelemetry } from '../context/TelemetryContext';

// Coordinates for the active mission area.
// MapTiler expects coordinates in [longitude, latitude] order.
const USV_LAT = -7.286365011018518;
const USV_LON = 112.7959597208925;
const USV_COORDS = [USV_LON, USV_LAT];
const WP1_COORDS = [USV_LON + 0.0126, USV_LAT - 0.0078];
const WP2_COORDS = [USV_LON + 0.0266, USV_LAT - 0.0138];
const BUOY_COORDS = [USV_LON + 0.0036, USV_LAT - 0.0023];
const STATION_COORDS = [USV_LON - 0.0134, USV_LAT + 0.0122];

const ROUTE_GEOJSON = {
  type: 'FeatureCollection',
  features: [
    {
      type: 'Feature',
      properties: { type: 'history' },
      geometry: {
        type: 'LineString',
        coordinates: [
          [USV_LON - 0.0174, USV_LAT + 0.0102],
          [USV_LON - 0.0104, USV_LAT + 0.0052],
          USV_COORDS,
        ],
      },
    },
    {
      type: 'Feature',
      properties: { type: 'planned' },
      geometry: {
        type: 'LineString',
        coordinates: [
          USV_COORDS,
          WP1_COORDS,
          WP2_COORDS,
        ],
      },
    },
  ],
};

const MAPTILER_API_KEY = import.meta.env.VITE_MAPTILER_API_KEY || 'el2TG0a2d25RjhVq8HYL';

export default function LocationLiveCard() {
  const { telemetry } = useTelemetry();
  const mapContainerRef = useRef(null);
  const mapRef = useRef(null);
  const markersRef = useRef([]);

  // Live telemetry dynamic values
  const lat = (telemetry?.navigation?.latitude ?? USV_LAT).toFixed(6);
  const lon = (telemetry?.navigation?.longitude ?? USV_LON).toFixed(6);
  const spdKnots = (telemetry?.navigation?.knots ?? 2.84).toFixed(2);
  const hdg = (telemetry?.navigation?.heading_deg ?? 214.0).toFixed(1);
  const batteryPct = Math.round(telemetry?.battery?.percentage ?? 96);
  const targetName = telemetry?.detections?.active_target ?? 'BUOY-04';
  const targetConf = Math.round((telemetry?.detections?.confidence ?? 0.94) * 100);
  const targetDist = (telemetry?.detections?.target_distance_m ?? 18.4).toFixed(1);
  const missionProg = telemetry?.mission?.progress_percent ?? 68;
  const mode = telemetry?.mode ?? 'MANUAL';

  // 'nav' = Tactical Navigation (Dark), 'map' = Bathymetry Ocean, 'video' = Satellite Hybrid
  const [viewMode, setViewMode] = useState('map');
  const [hudVisible, setHudVisible] = useState(true);
  const [mapLoaded, setMapLoaded] = useState(false);

  // Helper to add GeoJSON route layers
  const addRouteLayers = useCallback((map) => {
    if (!map) return;

    if (map.getSource('mission-route')) {
      return;
    }

    try {
      map.addSource('mission-route', {
        type: 'geojson',
        data: ROUTE_GEOJSON,
      });

      // Breadcrumb history line
      map.addLayer({
        id: 'route-history',
        type: 'line',
        source: 'mission-route',
        filter: ['==', ['get', 'type'], 'history'],
        layout: {
          'line-join': 'round',
          'line-cap': 'round',
        },
        paint: {
          'line-color': '#0284c7',
          'line-width': 3,
          'line-opacity': 0.7,
        },
      });

      // Planned route line (dashed glow)
      map.addLayer({
        id: 'route-planned',
        type: 'line',
        source: 'mission-route',
        filter: ['==', ['get', 'type'], 'planned'],
        layout: {
          'line-join': 'round',
          'line-cap': 'round',
        },
        paint: {
          'line-color': '#38bdf8',
          'line-width': 3.5,
          'line-dasharray': [3, 2],
          'line-opacity': 0.9,
        },
      });
    } catch (e) {
      console.warn('Could not add route layers:', e);
    }
  }, []);

  // Helper to attach custom DOM markers
  const attachMarkers = useCallback((map) => {
    if (!map) return;

    // Clear previous markers
    markersRef.current.forEach((m) => m.remove());
    markersRef.current = [];

    // 1. Vessel (USV LSS-02) Marker with Animated Pulse
    const usvEl = document.createElement('div');
    usvEl.className = 'relative flex items-center justify-center cursor-pointer';
    usvEl.style.width = '48px';
    usvEl.style.height = '48px';
    usvEl.innerHTML = `
      <div class="absolute inset-0 rounded-full bg-sky-500/20 animate-ping" style="animation-duration: 2.2s;"></div>
      <div class="absolute inset-1 rounded-full bg-sky-400/30 animate-pulse"></div>
      <div class="relative w-8 h-8 rounded-full bg-slate-900 border-2 border-sky-400 shadow-[0_0_12px_#38bdf8] flex items-center justify-center transform" style="transform: rotate(218deg);">
        <svg class="w-4 h-4 text-sky-300" viewBox="0 0 24 24" fill="currentColor">
          <path d="M12 2L4.5 20.29l.71.71L12 18l6.79 3 .71-.71z"/>
        </svg>
      </div>
      <div class="absolute -bottom-5 bg-slate-900/90 text-[9px] font-mono font-bold text-sky-400 px-1.5 py-0.5 rounded shadow border border-sky-400/30 whitespace-nowrap pointer-events-none">
        LSS-02
      </div>
    `;

    const usvPopup = new maptilersdk.Popup({ offset: 25, closeButton: false }).setHTML(`
      <div style="font-family: monospace; font-size: 11px; padding: 4px; color: #0f172a;">
        <strong style="color: #0284c7;">LSS-02 USV Cockpit</strong><br/>
        Speed: <b>2.84 kts</b><br/>
        Heading: <b>218.2° (SSW)</b><br/>
        Depth: <b>32.4 m</b><br/>
        Fix: <span style="color: #10b981; font-weight: bold;">RTK Fixed</span>
      </div>
    `);

    const usvMarker = new maptilersdk.Marker({ element: usvEl })
      .setLngLat(USV_COORDS)
      .setPopup(usvPopup)
      .addTo(map);
    markersRef.current.push(usvMarker);

    // 2. Waypoint WP-02 Marker
    const wp2El = document.createElement('div');
    wp2El.className = 'relative flex items-center justify-center cursor-pointer';
    wp2El.innerHTML = `
      <div class="w-5 h-5 rounded-full bg-amber-500/90 border-2 border-white shadow-md flex items-center justify-center text-white text-[9px] font-bold">
        W2
      </div>
      <div class="absolute -top-4 text-[9px] font-mono font-bold text-amber-300 bg-slate-900/80 px-1 rounded shadow whitespace-nowrap">
        WP-02
      </div>
    `;
    const wp2Marker = new maptilersdk.Marker({ element: wp2El })
      .setLngLat(WP2_COORDS)
      .addTo(map);
    markersRef.current.push(wp2Marker);

    // 3. Sensor Buoy BUOY-04 Marker
    const buoyEl = document.createElement('div');
    buoyEl.className = 'relative flex items-center justify-center cursor-pointer';
    buoyEl.innerHTML = `
      <div class="w-4 h-4 rounded-full bg-emerald-500 border border-white shadow-sm flex items-center justify-center text-[8px] text-white font-bold animate-bounce" style="animation-duration: 2.8s;">
        ●
      </div>
      <div class="absolute -top-4 text-[8px] font-mono text-emerald-400 bg-slate-900/80 px-1 rounded shadow whitespace-nowrap">
        BUOY-04
      </div>
    `;
    const buoyMarker = new maptilersdk.Marker({ element: buoyEl })
      .setLngLat(BUOY_COORDS)
      .addTo(map);
    markersRef.current.push(buoyMarker);

    // 4. Harbor Base Station Marker
    const stationEl = document.createElement('div');
    stationEl.className = 'relative flex items-center justify-center cursor-pointer';
    stationEl.innerHTML = `
      <div class="w-6 h-6 rounded-lg bg-sky-600 border border-white/80 shadow flex items-center justify-center text-white">
        <svg class="w-3.5 h-3.5" viewBox="0 0 24 24" fill="currentColor">
          <path d="M12 3L2 12h3v8h6v-6h2v6h6v-8h3L12 3z"/>
        </svg>
      </div>
      <div class="absolute -top-4 text-[8px] font-mono text-sky-300 bg-slate-900/80 px-1 rounded shadow whitespace-nowrap">
        STATION NORTH
      </div>
    `;
    const stationMarker = new maptilersdk.Marker({ element: stationEl })
      .setLngLat(STATION_COORDS)
      .addTo(map);
    markersRef.current.push(stationMarker);
  }, []);

  // Initialize MapTiler Map
  useEffect(() => {
    if (!mapContainerRef.current) return;

    maptilersdk.config.apiKey = MAPTILER_API_KEY;

    let initialStyle = maptilersdk.MapStyle.OCEAN;
    if (viewMode === 'video') initialStyle = maptilersdk.MapStyle.HYBRID;
    if (viewMode === 'nav') initialStyle = maptilersdk.MapStyle.DATAVIZ.DARK;

    const map = new maptilersdk.Map({
      container: mapContainerRef.current,
      style: initialStyle,
      center: USV_COORDS,
      zoom: 13.2,
      pitch: 38,
      bearing: -20,
      navigationControl: false,
      geolocateControl: false,
      attributionControl: false,
    });

    mapRef.current = map;

    map.on('load', () => {
      setMapLoaded(true);
      addRouteLayers(map);
      attachMarkers(map);
    });

    map.on('style.load', () => {
      addRouteLayers(map);
      attachMarkers(map);
    });

    return () => {
      markersRef.current.forEach((m) => m.remove());
      markersRef.current = [];
      map.remove();
      mapRef.current = null;
    };
  }, [addRouteLayers, attachMarkers]);

  // Handle View Mode switch
  const handleModeChange = (mode) => {
    setViewMode(mode);
    if (!mapRef.current) return;

    if (mode === 'map') {
      mapRef.current.setStyle(maptilersdk.MapStyle.OCEAN);
    } else if (mode === 'video') {
      mapRef.current.setStyle(maptilersdk.MapStyle.HYBRID);
    } else if (mode === 'nav') {
      mapRef.current.setStyle(maptilersdk.MapStyle.DATAVIZ.DARK);
    }
  };

  // Zoom and navigation handlers
  const handleZoomIn = () => {
    if (mapRef.current) mapRef.current.zoomIn({ duration: 300 });
  };

  const handleZoomOut = () => {
    if (mapRef.current) mapRef.current.zoomOut({ duration: 300 });
  };

  const handleRecenter = () => {
    if (mapRef.current) {
      mapRef.current.flyTo({
        center: USV_COORDS,
        zoom: 13.5,
        pitch: 38,
        bearing: -20,
        essential: true,
        duration: 1000,
      });
    }
  };

  const handleFlyToWp2 = () => {
    if (mapRef.current) {
      mapRef.current.flyTo({
        center: WP2_COORDS,
        zoom: 14.5,
        duration: 1200,
      });
    }
  };

  return (
    <div className="bg-white dark:bg-[#151921] rounded-3xl p-5 shadow-cockpit border border-slate-200/50 dark:border-slate-800 flex flex-col justify-between h-full transition-colors">
      {/* Header bar of card */}
      <div className="flex items-center justify-between pb-3">
        <div>
          <div className="flex items-center gap-2">
            <h2 className="text-base font-bold text-slate-900 dark:text-white tracking-tight">Location Live</h2>
            <span className="inline-flex items-center gap-1 px-1.5 py-0.5 rounded-full text-[9px] font-mono font-medium bg-emerald-100 text-emerald-800 dark:bg-emerald-950/60 dark:text-emerald-300 border border-emerald-300 dark:border-emerald-800">
              <span className="w-1.5 h-1.5 rounded-full bg-emerald-500 animate-pulse"></span>
              MapTiler Live
            </span>
          </div>
          <p className="text-[11px] text-slate-400 dark:text-slate-500 font-mono">
             Active Mission Area • RTK Fixed (0.02m)
          </p>
        </div>

        {/* View Mode Switcher Pill */}
        <div className="flex items-center gap-1 bg-slate-100/80 dark:bg-slate-800/80 p-1 rounded-full">
          <button
            onClick={() => handleModeChange('video')}
            className={`w-7 h-7 rounded-full flex items-center justify-center transition-all ${
              viewMode === 'video'
                ? 'bg-white text-slate-900 dark:bg-white dark:text-slate-900 shadow-xs'
                : 'text-slate-500 hover:text-black dark:text-slate-400 dark:hover:text-white'
            }`}
            title="Satellite Earth Feed (MapTiler Hybrid)"
            type="button"
          >
            <span className="material-symbols-outlined text-sm">satellite_alt</span>
          </button>
          <button
            onClick={() => handleModeChange('map')}
            className={`w-7 h-7 rounded-full flex items-center justify-center transition-all ${
              viewMode === 'map'
                ? 'bg-white text-slate-900 dark:bg-white dark:text-slate-900 shadow-xs'
                : 'text-slate-500 hover:text-black dark:text-slate-400 dark:hover:text-white'
            }`}
            title="Bathymetric Ocean Map (MapTiler Ocean)"
            type="button"
          >
            <span className="material-symbols-outlined text-sm">map</span>
          </button>
          <button
            onClick={() => handleModeChange('nav')}
            className={`w-7 h-7 rounded-full flex items-center justify-center transition-all ${
              viewMode === 'nav'
                ? 'bg-slate-900 text-white dark:bg-sky-500 dark:text-slate-950 shadow-xs'
                : 'text-slate-500 hover:text-black dark:text-slate-400 dark:hover:text-white'
            }`}
            title="Tactical HUD Navigation (MapTiler Dark)"
            type="button"
          >
            <span className="material-symbols-outlined text-sm">near_me</span>
          </button>
        </div>
      </div>

      {/* Visual Map Viewport with MapTiler & Tactical HUD */}
      <div className="relative w-full aspect-square rounded-2xl overflow-hidden bg-slate-950 flex items-center justify-center select-none group border border-slate-800/40 shadow-inner">
        {/* MapTiler Container */}
        <div ref={mapContainerRef} className="absolute inset-0 w-full h-full" />

        {/* MapTiler Loading Skeleton Fallback */}
        {!mapLoaded && (
          <div className="absolute inset-0 flex items-center justify-center bg-slate-950 text-sky-400 font-mono text-xs z-10">
            <div className="flex items-center gap-2">
              <span className="w-3 h-3 rounded-full border-2 border-sky-400 border-t-transparent animate-spin"></span>
              <span>SYNCHRONIZING MAPTILER TILES...</span>
            </div>
          </div>
        )}

        {/* Sleek Cockpit Vignette Gradient Overlay */}
        <div className="absolute inset-0 pointer-events-none bg-gradient-to-t from-slate-950/70 via-transparent to-slate-950/50"></div>

        {/* Telemetry Overlays matching original cockpit design */}
        {hudVisible && (
          <>
            <div className="absolute top-2 left-12 text-[7.5px] font-mono text-white/80 leading-tight pointer-events-none hidden sm:block drop-shadow-[0_1px_2px_rgba(0,0,0,0.8)]">
              <div className="font-bold text-sky-300">LSS-02 ASV | OP-ALPHA | {mode}</div>
              <div>LAT: {lat}° | LON: {lon}° | SPD: {spdKnots} KTS</div>
              <div className="text-white/60 pt-0.5">PROPULSION: {batteryPct}%</div>
              <div className="text-white/60">COMM: SECURE (SATCOM)</div>
              <div className="text-white/60">DEPTH CONTOUR: 15m - 45m</div>
              <div className="text-white/60">ENGINEERING: ONLINE</div>
            </div>

            <div className="absolute top-2 right-3 text-[7.5px] font-mono text-white/80 leading-tight text-right pointer-events-none hidden sm:block drop-shadow-[0_1px_2px_rgba(0,0,0,0.8)]">
              <div>HDG: {hdg}° | MODE: {mode}</div>
              <div>TIDE: EBB | SEA STATE: 4 | WX: CLEAR</div>
              <div className="text-white/60 pt-0.5">DISTANCE TO WP: {targetDist}m</div>
              <div className="text-white/60">BEARING: {hdg}°</div>
              <div className="text-rose-300 font-bold">TARGET: {targetName} ({targetConf}%)</div>
              <div className="text-white/60">NAV: {mode}</div>
              <div className="text-white/60">NEXT TURN: 0.8 nm</div>
              <div className="text-white/60">ROUTE: ALPHA-4</div>
            </div>

            {/* Tactical Reticle Horizon Indicator */}
            <div className="absolute inset-0 pointer-events-none flex items-center justify-center opacity-60">
              <div className="relative w-40 h-16 flex items-center justify-center">
                <div className="absolute w-full h-[1px] bg-sky-300/30"></div>
                <div className="absolute w-10 h-[2px] bg-sky-400 shadow-[0_0_8px_#38bdf8]"></div>
                <div className="absolute h-4 w-[1px] bg-sky-300/30"></div>
                <div className="w-3 h-3 rounded-full border border-sky-400/60 flex items-center justify-center">
                  <div className="w-1 h-1 rounded-full bg-sky-400"></div>
                </div>
                <div className="absolute -top-2 w-4 h-[1px] bg-sky-300/30"></div>
                <div className="absolute -bottom-2 w-4 h-[1px] bg-sky-300/30"></div>
              </div>
            </div>
          </>
        )}

        {/* Top-Left Map Controls Pill: Zoom In, Zoom Out, Recenter USV */}
        <div className="absolute top-3 left-3 bg-slate-900/90 backdrop-blur-md rounded-xl p-1 flex flex-col shadow-lg text-[11px] font-bold text-white border border-white/10 z-20">
          <button
            onClick={handleZoomIn}
            className="w-6 h-6 flex items-center justify-center hover:bg-slate-800 rounded-lg transition-colors text-white hover:text-sky-400"
            title="Zoom In"
            type="button"
          >
            +
          </button>
          <div className="w-4 h-[1px] bg-slate-800 mx-auto my-0.5"></div>
          <button
            onClick={handleZoomOut}
            className="w-6 h-6 flex items-center justify-center hover:bg-slate-800 rounded-lg transition-colors text-white hover:text-sky-400"
            title="Zoom Out"
            type="button"
          >
            -
          </button>
          <div className="w-4 h-[1px] bg-slate-800 mx-auto my-0.5"></div>
          <button
            onClick={handleRecenter}
            className="w-6 h-6 flex items-center justify-center hover:bg-slate-800 rounded-lg transition-colors text-sky-400 hover:text-sky-300"
            title="Center on USV LSS-02"
            type="button"
          >
            <span className="material-symbols-outlined text-[13px]">my_location</span>
          </button>
        </div>

        {/* Detection Badge Overlay in top right on mobile / floating */}
        <div className="absolute top-3 right-3 sm:right-32 bg-slate-900/90 backdrop-blur-md px-2.5 py-1 rounded-full text-white text-[9.5px] font-mono flex items-center gap-1.5 border border-white/10 shadow-lg z-20">
          <span className="w-2 h-2 rounded-full bg-emerald-400 animate-pulse"></span>
          <span>{targetName} [{targetConf}%] • {targetDist}m</span>
        </div>

        {/* Floating Waypoint Marker Tag (Clickable to focus WP-02) */}
        <button
          onClick={handleFlyToWp2}
          type="button"
          className="absolute bottom-3 left-3 bg-slate-900/90 hover:bg-slate-800 backdrop-blur-md px-2.5 py-1 rounded-full text-[10px] font-bold text-white shadow-md flex items-center gap-1 border border-white/10 transition-colors z-20 group"
          title="Fly to Waypoint 02"
        >
          <span className="material-symbols-outlined text-xs text-amber-400 group-hover:scale-110 transition-transform">flag</span>
          <span className="font-mono">WP-02 (3.1 nm)</span>
        </button>

        {/* HUD Toggle / Map Layer Style Badge */}
        <div className="absolute bottom-3 right-3 flex items-center gap-1.5 z-20">
          <button
            onClick={() => setHudVisible(!hudVisible)}
            type="button"
            className="bg-slate-900/80 hover:bg-slate-800 backdrop-blur-md px-2 py-0.5 rounded-full text-[9px] font-mono text-slate-300 border border-white/10 transition-colors"
            title="Toggle Cockpit HUD Telemetry Overlay"
          >
            HUD: {hudVisible ? 'ON' : 'OFF'}
          </button>
          <span className="bg-slate-900/80 backdrop-blur-md px-2 py-0.5 rounded-full text-[9px] font-mono text-sky-400 border border-white/10">
            {viewMode === 'map' ? 'MapTiler Ocean' : viewMode === 'video' ? 'MapTiler Sat' : 'MapTiler Nav'}
          </span>
        </div>
      </div>

      {/* Path Waypoint Progress Bar (Direct match to original design) */}
      <div className="pt-3 flex flex-col gap-1.5">
        <div className="flex items-center justify-between text-[11px] font-medium text-slate-500 dark:text-slate-400">
          <span className="font-bold text-slate-800 dark:text-slate-200">
            Harbor Station North <span className="font-normal text-slate-400 dark:text-slate-500 ml-1">5.1 km</span>
          </span>
          <span className="font-mono text-slate-700 dark:text-slate-300">
             Lat {lat}°, Lon {lon}° <span className="font-bold text-sky-600 dark:text-sky-400 ml-1">09 min</span>
          </span>
        </div>
        <div className="relative w-full bg-slate-100 dark:bg-slate-800 h-2 rounded-full overflow-hidden">
          <div 
            className="bg-slate-900 dark:bg-slate-200 h-full rounded-full relative transition-all duration-500"
            style={{ width: `${Math.max(10, Math.min(100, missionProg))}%` }}
          >
            <div className="absolute right-0 top-0 bottom-0 w-2.5 bg-sky-400 rounded-full shadow-[0_0_8px_#38bdf8]"></div>
          </div>
        </div>
      </div>
    </div>
  );
}
