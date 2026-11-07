import React, { useState, useEffect, useRef, useCallback } from 'react';
import * as maptilersdk from '@maptiler/sdk';
import '@maptiler/sdk/dist/maptiler-sdk.css';
import { useTelemetry } from '../context/TelemetryContext';

// Coordinates for the active mission area.
// MapTiler expects coordinates in [longitude, latitude] order.
const USV_LAT = -7.286340;
const USV_LON = 112.795933;
const USV_COORDS = [USV_LON, USV_LAT];
const WP1_COORDS = [USV_LON + 0.0126, USV_LAT - 0.0078];
const WP2_COORDS = [USV_LON + 0.0266, USV_LAT - 0.0138];
const BUOY_COORDS = [USV_LON + 0.0036, USV_LAT - 0.0023];

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

// Shared ids so the route line and the waypoint icons always use the same sources
const MISSION_ROUTE_SOURCE = 'mission-route';
const MISSION_WP_SOURCE = 'mission-waypoints';
const MISSION_SEGMENT_SOURCE = 'mission-segments';
const ROUTE_HISTORY_LAYER = 'route-history';
const ROUTE_PLANNED_LAYER = 'route-planned';
const SEGMENT_HIGHLIGHT_LAYER = 'waypoint-segment-highlight';
const WP_CIRCLE_LAYER = 'waypoint-circle';
const WP_CENTER_LAYER = 'waypoint-center';
const WP_LABEL_LAYER = 'waypoint-label';

export default function LocationLiveCard() {
  const {
    telemetry,
    plannedWaypoints,
    addPlannedWaypoint,
    clearPlannedWaypoints,
    isMapPlanningMode,
    setIsMapPlanningMode,
    deployPlannedWaypoints,
    rosSyncStatus,
  } = useTelemetry();

  const mapContainerRef = useRef(null);
  const mapRef = useRef(null);
  const markersRef = useRef([]);
  const usvMarkerRef = useRef(null);
  const wpPopupRef = useRef(null);

  // Storing refs for map event closures
  const isMapPlanningModeRef = useRef(isMapPlanningMode);
  isMapPlanningModeRef.current = isMapPlanningMode;
  const plannedWaypointsRef = useRef(plannedWaypoints);
  plannedWaypointsRef.current = plannedWaypoints;
  const telemetryRef = useRef(telemetry);
  telemetryRef.current = telemetry;

  // Single source of truth for the vessel position.
  // The marker, the route origin and the click projection must all use this.
  const getUsvCoord = useCallback(() => {
    const nav = telemetryRef.current?.navigation || {};
    const lng = Number(nav.longitude);
    const lat = Number(nav.latitude);
    return [
      Number.isFinite(lng) ? lng : USV_LON,
      Number.isFinite(lat) ? lat : USV_LAT,
    ];
  }, []);

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

  // Build route GeoJSON always anchored to the *live* vessel position
  const buildRouteGeoJSON = useCallback(() => {
    const currentUsvCoord = getUsvCoord();
    const plannedCoords = (plannedWaypointsRef.current || []).map((wp) => [
      Number(wp.lon),
      Number(wp.lat),
    ]);
    const plannedLine = plannedCoords.length > 0 ? [currentUsvCoord, ...plannedCoords] : [];

    return {
      type: 'FeatureCollection',
      features: [
        {
          type: 'Feature',
          properties: { type: 'history' },
          geometry: {
            type: 'LineString',
            coordinates: [
              [currentUsvCoord[0] - 0.0174, currentUsvCoord[1] + 0.0102],
              [currentUsvCoord[0] - 0.0104, currentUsvCoord[1] + 0.0052],
              currentUsvCoord,
            ],
          },
        },
        {
          type: 'Feature',
          properties: { type: 'planned' },
          geometry: {
            type: 'LineString',
            coordinates: plannedLine,
          },
        },
      ],
    };
  }, [getUsvCoord]);

  // Waypoints are rendered by MapLibre itself (canvas layers), so they share
  // the exact same geographic -> screen projection as the route line.
  const buildWaypointsGeoJSON = useCallback(() => {
    return {
      type: 'FeatureCollection',
      features: (plannedWaypointsRef.current || []).map((wp, idx) => ({
        type: 'Feature',
        properties: {
          index: idx,
          label: String(idx + 1),
          id: wp.id || `WP-${String(idx + 1).padStart(2, '0')}`,
          name: wp.name || `Waypoint ${idx + 1}`,
          action: wp.action || 'PASS',
          lat: Number(wp.lat),
          lon: Number(wp.lon),
        },
        geometry: {
          type: 'Point',
          coordinates: [Number(wp.lon), Number(wp.lat)],
        },
      })),
    };
  }, []);

  // One segment per waypoint: previous point (vessel or previous WP) -> waypoint
  const buildSegmentsGeoJSON = useCallback(() => {
    const waypoints = plannedWaypointsRef.current || [];
    const currentUsvCoord = getUsvCoord();

    return {
      type: 'FeatureCollection',
      features: waypoints.map((wp, idx) => {
        const from = idx === 0
          ? currentUsvCoord
          : [Number(waypoints[idx - 1].lon), Number(waypoints[idx - 1].lat)];

        return {
          type: 'Feature',
          properties: { index: idx },
          geometry: {
            type: 'LineString',
            coordinates: [from, [Number(wp.lon), Number(wp.lat)]],
          },
        };
      }),
    };
  }, [getUsvCoord]);

  // Helper to add GeoJSON route + waypoint layers
  const addRouteLayers = useCallback((map) => {
    if (!map) return;

    if (map.getSource(MISSION_ROUTE_SOURCE)) {
      return;
    }

    const emptyCollection = { type: 'FeatureCollection', features: [] };

    try {
      map.addSource(MISSION_ROUTE_SOURCE, {
        type: 'geojson',
        data: ROUTE_GEOJSON,
      });

      // Breadcrumb history line
      map.addLayer({
        id: ROUTE_HISTORY_LAYER,
        type: 'line',
        source: MISSION_ROUTE_SOURCE,
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
        id: ROUTE_PLANNED_LAYER,
        type: 'line',
        source: MISSION_ROUTE_SOURCE,
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

      map.addSource(MISSION_SEGMENT_SOURCE, {
        type: 'geojson',
        data: emptyCollection,
      });

      // Highlighted segment for the hovered waypoint (debug aid)
      map.addLayer({
        id: SEGMENT_HIGHLIGHT_LAYER,
        type: 'line',
        source: MISSION_SEGMENT_SOURCE,
        filter: ['==', ['get', 'index'], -1],
        layout: {
          'line-join': 'round',
          'line-cap': 'round',
        },
        paint: {
          'line-color': '#fbbf24',
          'line-width': 7,
          'line-opacity': 0.85,
        },
      });

      map.addSource(MISSION_WP_SOURCE, {
        type: 'geojson',
        data: emptyCollection,
      });

      // Waypoint icon (amber circle), drawn above the route line
      map.addLayer({
        id: WP_CIRCLE_LAYER,
        type: 'circle',
        source: MISSION_WP_SOURCE,
        paint: {
          'circle-radius': 11,
          'circle-color': '#f59e0b',
          'circle-stroke-color': '#ffffff',
          'circle-stroke-width': 2,
          'circle-opacity': 1,
          'circle-stroke-opacity': 1,
        },
      });

      // Tiny dot showing the exact coordinate (visual debugging indicator)
      map.addLayer({
        id: WP_CENTER_LAYER,
        type: 'circle',
        source: MISSION_WP_SOURCE,
        paint: {
          'circle-radius': 2,
          'circle-color': '#0f172a',
          'circle-stroke-color': '#ffffff',
          'circle-stroke-width': 1,
          'circle-opacity': 1,
        },
      });

      // Waypoint id label above the marker
      map.addLayer({
        id: WP_LABEL_LAYER,
        type: 'symbol',
        source: MISSION_WP_SOURCE,
        layout: {
          'text-field': ['get', 'id'],
          'text-font': ['Roboto Medium'],
          'text-size': 9,
          'text-offset': [0, -1.9],
          'text-anchor': 'bottom',
          'text-allow-overlap': true,
          'text-ignore-placement': true,
        },
        paint: {
          'text-color': '#fcd34d',
          'text-halo-color': '#0f172a',
          'text-halo-width': 1.2,
        },
      });
    } catch (e) {
      console.warn('Could not add route layers:', e);
    }
  }, []);

  // Push the freshest route + waypoint data into the existing map sources
  const syncRouteToMap = useCallback((map) => {
    if (!map) return;

    const routeSource = map.getSource(MISSION_ROUTE_SOURCE);
    if (routeSource) routeSource.setData(buildRouteGeoJSON());

    const wpSource = map.getSource(MISSION_WP_SOURCE);
    if (wpSource) wpSource.setData(buildWaypointsGeoJSON());

    const segmentSource = map.getSource(MISSION_SEGMENT_SOURCE);
    if (segmentSource) segmentSource.setData(buildSegmentsGeoJSON());
  }, [buildRouteGeoJSON, buildWaypointsGeoJSON, buildSegmentsGeoJSON]);

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

    const usvMarker = new maptilersdk.Marker({ element: usvEl, anchor: 'center' })
      .setLngLat(getUsvCoord())
      .setPopup(usvPopup)
      .addTo(map);
    usvMarkerRef.current = usvMarker;
    markersRef.current.push(usvMarker);

    // 2. Sensor Buoy BUOY-04 Marker
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
  }, [getUsvCoord]);

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
      center: getUsvCoord(),
      zoom: 13.2,
      pitch: 38,
      bearing: -20,
      navigationControl: false,
      geolocateControl: false,
      attributionControl: false,
    });

    mapRef.current = map;

    // Keep the canvas buffer in sync with the actual card size.
    // Without this, markers/lines and clicked coordinates drift after layout/font reflow.
    const resizeObserver = new ResizeObserver(() => {
      requestAnimationFrame(() => {
        if (mapRef.current === map) map.resize();
      });
    });
    resizeObserver.observe(mapContainerRef.current);

    const wpPopup = new maptilersdk.Popup({ offset: 14, closeButton: false, closeOnClick: false });
    wpPopupRef.current = wpPopup;

    const queryWaypointHit = (point) => {
      if (!map.getLayer(WP_CIRCLE_LAYER)) return null;
      const hits = map.queryRenderedFeatures(point, { layers: [WP_CIRCLE_LAYER] });
      return hits.length > 0 ? hits[0] : null;
    };

    const highlightSegment = (index) => {
      if (!map.getLayer(SEGMENT_HIGHLIGHT_LAYER)) return;
      map.setFilter(SEGMENT_HIGHLIGHT_LAYER, ['==', ['get', 'index'], index]);
    };

    // Handle map click: inspect an existing waypoint, otherwise add a new one
    map.on('click', (e) => {
      const hit = queryWaypointHit(e.point);

      if (hit) {
        const props = hit.properties || {};
        const hitIndex = Number(props.index);
        highlightSegment(Number.isFinite(hitIndex) ? hitIndex : -1);
        wpPopup
          .setLngLat(hit.geometry.coordinates.slice())
          .setHTML(`
            <div style="font-family: monospace; font-size: 11px; padding: 4px; color: #0f172a;">
              <strong style="color: #d97706;">${props.id || 'Waypoint'} (${props.name || 'Waypoint'})</strong><br/>
              Lat: <b>${Number(props.lat).toFixed(6)}</b><br/>
              Lon: <b>${Number(props.lon).toFixed(6)}</b><br/>
              Action: <b>${props.action || 'PASS'}</b>
            </div>
          `)
          .addTo(map);
        return;
      }

      wpPopup.remove();

      if (!isMapPlanningModeRef.current) return;
      const clickLng = Number(e.lngLat.lng.toFixed(6));
      const clickLat = Number(e.lngLat.lat.toFixed(6));
      const nextCount = (plannedWaypointsRef.current?.length || 0) + 1;

      addPlannedWaypoint({
        id: `WP-${String(nextCount).padStart(2, '0')}`,
        name: `WP-${String(nextCount).padStart(2, '0')}`,
        lat: clickLat,
        lon: clickLng,
        alt: 0.0,
        action: 'PASS',
        target_speed: 1.5,
      });
    });

    // Hover: highlight the segment that ends at the waypoint
    map.on('mousemove', (e) => {
      const hit = queryWaypointHit(e.point);

      if (!hit) {
        highlightSegment(-1);
        map.getCanvas().style.cursor = isMapPlanningModeRef.current ? 'crosshair' : '';
        return;
      }

      const idx = Number(hit.properties?.index);
      highlightSegment(Number.isFinite(idx) ? idx : -1);
      map.getCanvas().style.cursor = 'pointer';
    });

    map.on('mouseout', () => {
      highlightSegment(-1);
    });

    map.on('load', () => {
      setMapLoaded(true);
      addRouteLayers(map);
      syncRouteToMap(map);
      attachMarkers(map);
    });

    map.on('style.load', () => {
      addRouteLayers(map);
      syncRouteToMap(map);
      attachMarkers(map);
    });

    return () => {
      resizeObserver.disconnect();
      markersRef.current.forEach((m) => m.remove());
      markersRef.current = [];
      usvMarkerRef.current = null;
      wpPopup.remove();
      wpPopupRef.current = null;
      map.remove();
      mapRef.current = null;
    };
  }, [addRouteLayers, syncRouteToMap, attachMarkers, addPlannedWaypoint, getUsvCoord]);

  // Adjust cursor based on planning mode
  useEffect(() => {
    if (!mapRef.current) return;
    try {
      const canvas = mapRef.current.getCanvas();
      if (canvas) {
        canvas.style.cursor = isMapPlanningMode ? 'crosshair' : '';
      }
    } catch (e) {
      console.warn('Canvas cursor update err:', e);
    }
  }, [isMapPlanningMode]);

  // Keep the vessel icon, route line and canvas waypoint layers in sync
  useEffect(() => {
    const map = mapRef.current;
    if (!map || !mapLoaded) return;

    if (usvMarkerRef.current) {
      usvMarkerRef.current.setLngLat(getUsvCoord());
    }
    syncRouteToMap(map);
  }, [plannedWaypoints, mapLoaded, telemetry, syncRouteToMap, getUsvCoord]);

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
        center: getUsvCoord(),
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

        {/* Header Right Controls: Mode Toggle & View Mode Switcher Pill */}
        <div className="flex items-center gap-2">
          {/* Toggle Add Waypoint Planning Mode */}
          <button
            onClick={() => setIsMapPlanningMode(!isMapPlanningMode)}
            type="button"
            className={`px-2.5 py-1 rounded-full text-[11px] font-bold flex items-center gap-1.5 transition-all shadow-xs ${
              isMapPlanningMode
                ? 'bg-amber-500 text-slate-950 ring-2 ring-amber-300 ring-offset-1 dark:ring-offset-slate-900 animate-pulse'
                : 'bg-slate-100 hover:bg-slate-200 text-slate-700 dark:bg-slate-800 dark:hover:bg-slate-700 dark:text-slate-200'
            }`}
            title={isMapPlanningMode ? "Matikan mode klik waypoint" : "Aktifkan klik peta untuk membuat jalur"}
          >
            <span className="material-symbols-outlined text-sm">
              {isMapPlanningMode ? 'edit_location' : 'add_location_alt'}
            </span>
            <span className="hidden sm:inline">{isMapPlanningMode ? 'Mode WP Aktif' : '+ Klik Waypoint'}</span>
            <span className="sm:hidden">{isMapPlanningMode ? 'WP ON' : '+ WP'}</span>
            {plannedWaypoints?.length > 0 && (
              <span className="px-1.5 py-0.2 rounded-full text-[9.5px] bg-slate-900/20 text-current font-mono font-bold">
                {plannedWaypoints.length}
              </span>
            )}
          </button>

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

        {/* Active Planning Mode Top Instruction Banner */}
        {isMapPlanningMode && (
          <div className="absolute top-3 left-1/2 -translate-x-1/2 bg-amber-500/95 backdrop-blur-md text-slate-950 text-[10px] font-mono font-bold px-3 py-1 rounded-full shadow-xl border border-amber-300 flex items-center gap-2 z-30 animate-pulse">
            <span className="w-2 h-2 rounded-full bg-slate-950 animate-ping"></span>
            <span>KLIK PETA: Tambah Waypoint ({plannedWaypoints?.length || 0} WP)</span>
            <button
              onClick={() => setIsMapPlanningMode(false)}
              type="button"
              className="ml-1 bg-slate-950 hover:bg-slate-800 text-white px-1.5 py-0.5 rounded-full text-[9px] font-sans font-semibold transition-colors"
            >
              Selesai
            </button>
          </div>
        )}

        {/* Detection Badge Overlay in top right on mobile / floating */}
        <div className="absolute top-3 right-3 sm:right-32 bg-slate-900/90 backdrop-blur-md px-2.5 py-1 rounded-full text-white text-[9.5px] font-mono flex items-center gap-1.5 border border-white/10 shadow-lg z-20">
          <span className="w-2 h-2 rounded-full bg-emerald-400 animate-pulse"></span>
          <span>{targetName} [{targetConf}%] • {targetDist}m</span>
        </div>

        {/* Floating Waypoint Status & Navigation Pill */}
        <div className="absolute bottom-3 left-3 flex items-center gap-1.5 z-20">
          {plannedWaypoints && plannedWaypoints.length > 0 ? (
            <div className="bg-slate-900/90 backdrop-blur-md px-2.5 py-1 rounded-full text-[10px] font-bold text-white shadow-md flex items-center gap-2 border border-white/10">
              <button
                onClick={() => {
                  if (mapRef.current && plannedWaypoints.length > 0) {
                    const lastWp = plannedWaypoints[plannedWaypoints.length - 1];
                    mapRef.current.flyTo({
                      center: [Number(lastWp.lon), Number(lastWp.lat)],
                      zoom: 14.5,
                      duration: 1000,
                    });
                  }
                }}
                type="button"
                className="flex items-center gap-1 text-amber-300 hover:text-amber-200 transition-colors"
                title="Fokus ke Waypoint Terakhir"
              >
                <span className="material-symbols-outlined text-xs text-amber-400">flag</span>
                <span className="font-mono">
                  {plannedWaypoints.length} WPs ({plannedWaypoints[plannedWaypoints.length - 1].lat.toFixed(4)}, {plannedWaypoints[plannedWaypoints.length - 1].lon.toFixed(4)})
                </span>
              </button>
              <div className="w-px h-3 bg-white/20"></div>
              <button
                onClick={clearPlannedWaypoints}
                type="button"
                className="text-[9px] text-rose-400 hover:text-rose-300 font-mono transition-colors"
                title="Hapus semua rute waypoint"
              >
                Reset
              </button>
            </div>
          ) : (
            <button
              onClick={() => setIsMapPlanningMode(true)}
              type="button"
              className="bg-slate-900/90 hover:bg-slate-800 backdrop-blur-md px-2.5 py-1 rounded-full text-[10px] font-bold text-white shadow-md flex items-center gap-1 border border-white/10 transition-colors"
            >
              <span className="material-symbols-outlined text-xs text-sky-400">add_location</span>
              <span className="font-mono">Klik Peta untuk Rute Baru</span>
            </button>
          )}
        </div>

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
