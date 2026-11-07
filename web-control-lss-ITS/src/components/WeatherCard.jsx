import React, { useState } from 'react';
import { useTelemetry } from '../context/TelemetryContext';

const waypointPlans = [
  {
    id: 'harbor-survey-alpha',
    name: 'Harbor Survey Alpha',
    description: 'Survey loop around the active mission area',
    waypoints: [
      { id: 'WP-01', name: 'Launch Point', lat: -7.286365, lon: 112.79596 },
      { id: 'WP-02', name: 'North Transect', lat: -7.2792, lon: 112.8075 },
      { id: 'WP-03', name: 'Sonar Transect', lat: -7.2928, lon: 112.8141 },
      { id: 'WP-04', name: 'Return Dock', lat: -7.2868, lon: 112.7981 },
    ],
    params: {
      planner: 'waypoint_follower',
      cruiseSpeed: '1.20',
      acceptanceRadius: '3.0',
      frameId: 'map',
      parameterNamespace: '/path_planner',
      rosbridgeUrl: 'ws://localhost:9090',
      loop: false,
    },
  },
  {
    id: 'coastal-perimeter',
    name: 'Coastal Perimeter',
    description: 'Perimeter route for shoreline inspection',
    waypoints: [
      { id: 'WP-01', name: 'Dock Exit', lat: -7.286365, lon: 112.79596 },
      { id: 'WP-02', name: 'East Gate', lat: -7.2795, lon: 112.812 },
      { id: 'WP-03', name: 'South Boundary', lat: -7.3001, lon: 112.806 },
      { id: 'WP-04', name: 'Recovery Point', lat: -7.2868, lon: 112.7981 },
    ],
    params: {
      planner: 'nav2_smac_hybrid',
      cruiseSpeed: '0.85',
      acceptanceRadius: '5.0',
      frameId: 'map',
      parameterNamespace: '/path_planner',
      rosbridgeUrl: 'ws://localhost:9090',
      loop: true,
    },
  },
  {
    id: 'bathymetry-grid',
    name: 'Bathymetry Grid',
    description: 'Parallel transects for seabed mapping',
    waypoints: [
      { id: 'WP-01', name: 'Grid Start', lat: -7.286365, lon: 112.79596 },
      { id: 'WP-02', name: 'Grid Line A', lat: -7.281, lon: 112.805 },
      { id: 'WP-03', name: 'Grid Line B', lat: -7.289, lon: 112.805 },
      { id: 'WP-04', name: 'Grid Line C', lat: -7.297, lon: 112.805 },
      { id: 'WP-05', name: 'Grid Exit', lat: -7.2868, lon: 112.7981 },
    ],
    params: {
      planner: 'coverage_planner',
      cruiseSpeed: '0.65',
      acceptanceRadius: '2.0',
      frameId: 'map',
      parameterNamespace: '/path_planner',
      rosbridgeUrl: 'ws://localhost:9090',
      loop: false,
    },
  },
];

const rosParameter = (name, type, valueKey, value) => ({
  name,
  value: { type, [valueKey]: value },
});

const buildRosbridgePayload = (plan, params) => ({
  op: 'call_service',
  service: `${params.parameterNamespace}/set_parameters_atomically`,
  type: 'rcl_interfaces/srv/SetParametersAtomically',
  args: {
    parameters: [
      rosParameter('path_id', 4, 'string_value', plan.id),
      rosParameter('planner_type', 4, 'string_value', params.planner),
      rosParameter('cruise_speed_mps', 3, 'double_value', Number(params.cruiseSpeed)),
      rosParameter('acceptance_radius_m', 3, 'double_value', Number(params.acceptanceRadius)),
      rosParameter('frame_id', 4, 'string_value', params.frameId),
      rosParameter('loop_path', 1, 'bool_value', params.loop),
      rosParameter('waypoints_json', 4, 'string_value', JSON.stringify(plan.waypoints)),
    ],
  },
});

export default function WeatherCard() {
  const {
    startMission,
    pauseMission,
    resetMission,
    loadWaypoints,
    telemetry,
    plannedWaypoints,
    setPlannedWaypoints,
    removePlannedWaypoint,
    clearPlannedWaypoints,
    isMapPlanningMode,
    setIsMapPlanningMode,
    rosSyncStatus,
    setRosSyncStatus,
  } = useTelemetry();

  const [selectedPlanId, setSelectedPlanId] = useState('custom-map');
  const [selectedParams, setSelectedParams] = useState({
    planner: 'waypoint_follower',
    cruiseSpeed: '1.20',
    acceptanceRadius: '2.5',
    frameId: 'map',
    parameterNamespace: '/gps_navigation',
    trajectory: 'A',
    rosbridgeUrl: 'ws://localhost:9090',
    loop: false,
  });
  const [draftPlanId, setDraftPlanId] = useState('custom-map');
  const [draftParams, setDraftParams] = useState({
    planner: 'waypoint_follower',
    cruiseSpeed: '1.20',
    acceptanceRadius: '2.5',
    frameId: 'map',
    parameterNamespace: '/gps_navigation',
    trajectory: 'A',
    rosbridgeUrl: 'ws://localhost:9090',
    loop: false,
  });
  const [isConfigOpen, setIsConfigOpen] = useState(false);
  const [showPayload, setShowPayload] = useState(false);
  const [configStatus, setConfigStatus] = useState('SYNCED');

  const missionStatus = telemetry?.mission?.status || 'IDLE';

  const openConfiguration = () => {
    setDraftPlanId(selectedPlanId);
    setDraftParams({ ...selectedParams });
    setShowPayload(false);
    setIsConfigOpen(true);
  };

  const handlePlanChange = (event) => {
    const planId = event.target.value;
    setDraftPlanId(planId);
    if (planId === 'custom-map') {
      // Keep current planned waypoints
    } else {
      const nextPlan = waypointPlans.find((plan) => plan.id === planId);
      if (nextPlan) {
        setDraftParams((prev) => ({
          ...prev,
          ...nextPlan.params,
        }));
        // Populate custom plannedWaypoints with this preset
        setPlannedWaypoints(
          nextPlan.waypoints.map((wp, i) => ({
            id: wp.id || `WP-${String(i + 1).padStart(2, '0')}`,
            name: wp.name,
            lat: wp.lat,
            lon: wp.lon,
            target_speed: Number(nextPlan.params.cruiseSpeed) || 1.5,
            action: 'PASS',
          }))
        );
      }
    }
  };

  const handleParamChange = (field, value) => {
    setDraftParams((current) => ({ ...current, [field]: value }));
  };

  const handleSaveConfiguration = async (event) => {
    event.preventDefault();
    setSelectedPlanId(draftPlanId);
    setSelectedParams({ ...draftParams });
    setConfigStatus('SYNCING...');
    try {
      if (loadWaypoints && plannedWaypoints.length > 0) {
        await loadWaypoints(
          plannedWaypoints,
          draftParams.trajectory || 'A',
          Number(draftParams.acceptanceRadius) || 2.5
        );
        setConfigStatus('SYNCED TO ROS2');
      } else {
        setConfigStatus('CONFIG STAGED');
      }
    } catch {
      setConfigStatus('SYNC ERROR');
    }
    setIsConfigOpen(false);
  };

  const handleSyncToRos = async () => {
    if (!plannedWaypoints || plannedWaypoints.length === 0) return;
    setConfigStatus('SYNCING...');
    try {
      await loadWaypoints(
        plannedWaypoints,
        selectedParams.trajectory || 'A',
        Number(selectedParams.acceptanceRadius) || 2.5
      );
      setConfigStatus('SYNCED TO ROS2');
    } catch {
      setConfigStatus('SYNC ERROR');
    }
  };

  return (
    <>
      <div className="bg-white dark:bg-[#151921] rounded-3xl p-4 shadow-cockpit border border-slate-200/50 dark:border-slate-800 flex flex-col gap-2 transition-colors">
        <div className="flex items-center justify-between gap-2">
          <div className="flex items-center gap-2.5 min-w-0">
            <div className="w-8 h-8 shrink-0 rounded-xl bg-amber-50 text-amber-600 dark:bg-amber-950/50 dark:text-amber-400 flex items-center justify-center">
              <span className="material-symbols-outlined text-base">route</span>
            </div>
            <div className="min-w-0">
              <h3 className="text-xs font-bold text-slate-900 dark:text-white truncate">Waypoint Plan</h3>
              <p className="text-[10px] text-slate-400 dark:text-slate-500 truncate">
                {selectedPlanId === 'custom-map' ? 'Custom Path (Map Planning)' : (waypointPlans.find(p => p.id === selectedPlanId)?.name || 'Custom Plan')}
              </p>
            </div>
          </div>
          <button
            onClick={openConfiguration}
            className="w-7 h-7 shrink-0 rounded-full flex items-center justify-center text-slate-400 hover:text-slate-700 hover:bg-slate-100 dark:hover:text-white dark:hover:bg-slate-800 transition-colors"
            type="button"
            title="Configure path planning"
          >
            <span className="material-symbols-outlined text-sm">tune</span>
          </button>
        </div>

        <div className="flex items-center justify-between text-[9px] font-mono text-slate-400 dark:text-slate-500">
          <span>{plannedWaypoints?.length || 0} WPs · {missionStatus}</span>
          <span className="flex items-center gap-1 text-emerald-600 dark:text-emerald-400 font-semibold">
            <span className={`w-1.5 h-1.5 rounded-full ${missionStatus === 'RUNNING' ? 'bg-emerald-500 animate-ping' : 'bg-emerald-500'}`}></span>
            {configStatus}
          </span>
        </div>

        {/* Dynamic Waypoints List displaying real Lat/Lon */}
        <div className="space-y-1 pt-1 max-h-44 overflow-y-auto pr-0.5">
          {plannedWaypoints && plannedWaypoints.length > 0 ? (
            plannedWaypoints.map((waypoint, index) => (
              <div
                key={waypoint.id || index}
                className="group flex items-center gap-2 rounded-xl px-1.5 py-1 hover:bg-slate-50 dark:hover:bg-slate-800/60 transition-colors"
              >
                <div className="flex flex-col items-center self-stretch">
                  <span className="w-5 h-5 rounded-full bg-slate-900 text-white dark:bg-amber-500 dark:text-slate-950 flex items-center justify-center text-[9px] font-mono font-bold">
                    {index + 1}
                  </span>
                  {index < plannedWaypoints.length - 1 && (
                    <span className="w-px flex-1 bg-slate-200 dark:bg-slate-700 mt-0.5"></span>
                  )}
                </div>
                <div className="flex-1 min-w-0">
                  <div className="flex items-center justify-between gap-1">
                    <span className="text-[10px] font-bold text-slate-800 dark:text-slate-200 truncate">
                      {waypoint.name || `Waypoint ${index + 1}`}
                    </span>
                    <span className="text-[8.5px] font-mono text-amber-600 dark:text-amber-400 font-semibold">
                      {waypoint.action || 'PASS'}
                    </span>
                  </div>
                  <p className="text-[9px] font-mono text-slate-500 dark:text-slate-400 truncate">
                    Lat: {Number(waypoint.lat).toFixed(6)}, Lon: {Number(waypoint.lon).toFixed(6)}
                  </p>
                </div>
                {/* Delete button per waypoint */}
                <button
                  onClick={() => removePlannedWaypoint(waypoint.id)}
                  className="opacity-0 group-hover:opacity-100 p-0.5 text-slate-400 hover:text-rose-500 transition-all rounded"
                  title="Hapus Waypoint"
                  type="button"
                >
                  <span className="material-symbols-outlined text-[13px]">delete</span>
                </button>
              </div>
            ))
          ) : (
            <div className="py-3 text-center rounded-xl bg-slate-50 dark:bg-slate-800/40 border border-dashed border-slate-200 dark:border-slate-800">
              <p className="text-[9.5px] text-slate-500 dark:text-slate-400">Belum ada titik waypoint</p>
              <button
                onClick={() => setIsMapPlanningMode(true)}
                type="button"
                className="mt-1 text-[9px] font-bold text-sky-600 dark:text-sky-400 hover:underline inline-flex items-center gap-1"
              >
                <span className="material-symbols-outlined text-xs">add_location_alt</span>
                Klik Peta untuk Menambah
              </button>
            </div>
          )}
        </div>

        {/* Quick Route Actions */}
        <div className="flex items-center gap-1.5 pt-1.5 border-t border-slate-100 dark:border-slate-800/60">
          <button
            onClick={() => setIsMapPlanningMode(!isMapPlanningMode)}
            type="button"
            className={`flex-1 rounded-xl px-2 py-1.5 text-[9.5px] font-bold flex items-center justify-center gap-1 transition-all ${
              isMapPlanningMode
                ? 'bg-amber-500 text-slate-950 animate-pulse'
                : 'bg-slate-100 dark:bg-slate-800/80 text-slate-700 dark:text-slate-200 hover:bg-slate-200 dark:hover:bg-slate-700'
            }`}
            title="Klik peta untuk menambah titik waypoint"
          >
            <span className="material-symbols-outlined text-xs">
              {isMapPlanningMode ? 'check_circle' : 'add_location_alt'}
            </span>
            {isMapPlanningMode ? 'Selesai Klik' : '+ Klik Map'}
          </button>
          <button
            onClick={handleSyncToRos}
            disabled={!plannedWaypoints || plannedWaypoints.length === 0}
            type="button"
            className="flex-1 rounded-xl bg-sky-600 hover:bg-sky-700 disabled:opacity-40 text-white px-2 py-1.5 text-[9.5px] font-bold flex items-center justify-center gap-1 shadow-xs transition-colors"
            title="Upload waypoints as ROS 2 parameters to /gps_navigation"
          >
            <span className="material-symbols-outlined text-xs">cloud_upload</span>
            Upload ROS 2
          </button>
          {plannedWaypoints && plannedWaypoints.length > 0 && (
            <button
              onClick={clearPlannedWaypoints}
              type="button"
              className="rounded-xl bg-slate-100 dark:bg-slate-800 hover:bg-rose-100 hover:text-rose-600 dark:hover:bg-rose-950/40 text-slate-400 px-2 py-1.5 text-[9.5px] font-bold transition-colors"
              title="Reset semua waypoint"
            >
              <span className="material-symbols-outlined text-xs">delete_sweep</span>
            </button>
          )}
        </div>

        {/* Mission Run Controls */}
        <div className="flex items-center gap-1.5 pt-1">
          <button
            onClick={openConfiguration}
            type="button"
            className="flex-1 rounded-xl bg-slate-100 dark:bg-slate-800/80 px-2.5 py-1.5 text-[10px] font-bold text-slate-700 dark:text-slate-200 hover:bg-slate-200 dark:hover:bg-slate-700 transition-colors flex items-center justify-center gap-1"
          >
            <span className="material-symbols-outlined text-xs">tune</span>
            Config
          </button>
          <button
            onClick={startMission}
            type="button"
            className="rounded-xl bg-emerald-600 hover:bg-emerald-700 text-white px-2.5 py-1.5 text-[10px] font-bold flex items-center justify-center gap-1 shadow-xs transition-colors"
            title="Start autonomous mission"
          >
            <span className="material-symbols-outlined text-xs">play_arrow</span>
            Start
          </button>
          <button
            onClick={pauseMission}
            type="button"
            className="rounded-xl bg-slate-800 dark:bg-slate-700 hover:bg-slate-700 text-white px-2 py-1.5 text-[10px] font-bold flex items-center justify-center transition-colors"
            title="Pause mission"
          >
            <span className="material-symbols-outlined text-xs">pause</span>
          </button>
          <button
            onClick={resetMission}
            type="button"
            className="rounded-xl bg-slate-200 dark:bg-slate-800 hover:bg-slate-300 dark:hover:bg-slate-700 text-slate-700 dark:text-slate-300 px-2 py-1.5 text-[10px] font-bold flex items-center justify-center transition-colors"
            title="Reset mission"
          >
            <span className="material-symbols-outlined text-xs">restart_alt</span>
          </button>
        </div>
      </div>

      {isConfigOpen && (
        <div
          className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-slate-950/60 backdrop-blur-sm"
          onClick={() => setIsConfigOpen(false)}
          role="presentation"
        >
          <div
            className="w-full max-w-lg max-h-[90vh] overflow-y-auto bg-white dark:bg-[#151921] rounded-3xl p-5 shadow-2xl border border-slate-200 dark:border-slate-800"
            onClick={(event) => event.stopPropagation()}
            role="dialog"
            aria-modal="true"
            aria-labelledby="waypoint-config-title"
          >
            <div className="flex items-center justify-between gap-3 pb-4 border-b border-slate-100 dark:border-slate-800">
              <div className="flex items-center gap-2.5">
                <div className="w-8 h-8 rounded-xl bg-amber-50 text-amber-600 dark:bg-amber-950/50 dark:text-amber-400 flex items-center justify-center">
                  <span className="material-symbols-outlined text-base">alt_route</span>
                </div>
                <div>
                  <h2 id="waypoint-config-title" className="text-sm font-bold text-slate-900 dark:text-white">Path Planning Configuration</h2>
                  <p className="text-[10px] text-slate-400 dark:text-slate-500">Select route and ROS 2 parameters</p>
                </div>
              </div>
              <button
                onClick={() => setIsConfigOpen(false)}
                type="button"
                className="text-slate-400 hover:text-slate-700 dark:hover:text-white"
                title="Close configuration"
              >
                <span className="material-symbols-outlined text-base">close</span>
              </button>
            </div>

            <form onSubmit={handleSaveConfiguration} className="space-y-4 pt-4 text-xs">
              <div>
                <label htmlFor="waypoint-plan" className="block mb-1 font-semibold text-slate-700 dark:text-slate-300">Preset / Active Path Selection</label>
                <select
                  id="waypoint-plan"
                  value={draftPlanId}
                  onChange={handlePlanChange}
                  className="w-full px-3 py-2.5 rounded-xl border border-slate-200 dark:border-slate-700 bg-slate-50 dark:bg-slate-800/80 text-slate-800 dark:text-white focus:outline-none focus:ring-2 focus:ring-sky-500"
                >
                  <option value="custom-map">📍 Custom Map Plan ({plannedWaypoints?.length || 0} Points)</option>
                  {waypointPlans.map((plan) => (
                    <option key={plan.id} value={plan.id}>{plan.name} · {plan.waypoints.length} WPs</option>
                  ))}
                </select>
                <p className="mt-1 text-[10px] text-slate-400 dark:text-slate-500">
                  {draftPlanId === 'custom-map' ? 'Titik-titik rute aktif yang ditambahkan langsung melalui klik pada peta.' : (waypointPlans.find(p => p.id === draftPlanId)?.description || '')}
                </p>
              </div>

              <div className="rounded-2xl bg-slate-50 dark:bg-slate-800/50 p-3 border border-slate-100 dark:border-slate-800">
                <div className="flex items-center justify-between mb-2">
                  <span className="font-semibold text-slate-700 dark:text-slate-300">Route Waypoints (Lat / Lon)</span>
                  <span className="font-mono text-[10px] text-slate-400">{plannedWaypoints?.length || 0} points</span>
                </div>
                <div className="space-y-1.5 max-h-36 overflow-y-auto pr-1">
                  {plannedWaypoints && plannedWaypoints.length > 0 ? (
                    plannedWaypoints.map((waypoint, index) => (
                      <div key={waypoint.id || index} className="flex items-center gap-2">
                        <span className="w-5 h-5 rounded-full bg-slate-900 text-white dark:bg-amber-500 dark:text-slate-950 flex items-center justify-center text-[9px] font-mono font-bold">{index + 1}</span>
                        <span className="flex-1 text-[10px] font-medium text-slate-700 dark:text-slate-300">{waypoint.name || `WP-${index + 1}`}</span>
                        <span className="text-[9px] font-mono text-slate-500 dark:text-slate-400">Lat: {Number(waypoint.lat).toFixed(6)}, Lon: {Number(waypoint.lon).toFixed(6)}</span>
                      </div>
                    ))
                  ) : (
                    <p className="text-[10px] text-slate-400 text-center py-2">Belum ada titik waypoint.</p>
                  )}
                </div>
              </div>

              <div className="grid grid-cols-2 gap-3">
                <div>
                  <label htmlFor="trajectory-select" className="block mb-1 font-semibold text-slate-700 dark:text-slate-300">ROS 2 Trajectory Slot</label>
                  <select
                    id="trajectory-select"
                    value={draftParams.trajectory || 'A'}
                    onChange={(event) => handleParamChange('trajectory', event.target.value)}
                    className="w-full px-3 py-2.5 rounded-xl border border-slate-200 dark:border-slate-700 bg-slate-50 dark:bg-slate-800/80 text-slate-800 dark:text-white font-mono text-[10px] focus:outline-none focus:ring-2 focus:ring-sky-500"
                  >
                    <option value="A">Trajectory A (navigation.waypoints_A)</option>
                    <option value="B">Trajectory B (navigation.waypoints_B)</option>
                  </select>
                </div>
                <div>
                  <label htmlFor="planner-type" className="block mb-1 font-semibold text-slate-700 dark:text-slate-300">Planner Type</label>
                  <select
                    id="planner-type"
                    value={draftParams.planner}
                    onChange={(event) => handleParamChange('planner', event.target.value)}
                    className="w-full px-3 py-2.5 rounded-xl border border-slate-200 dark:border-slate-700 bg-slate-50 dark:bg-slate-800/80 text-slate-800 dark:text-white font-mono text-[10px] focus:outline-none focus:ring-2 focus:ring-sky-500"
                  >
                    <option value="waypoint_follower">Waypoint Follower (/gps_navigation)</option>
                    <option value="nav2_smac_hybrid">Nav2 Smac Hybrid</option>
                    <option value="coverage_planner">Coverage Planner</option>
                  </select>
                </div>
                <div>
                  <label htmlFor="cruise-speed" className="block mb-1 font-semibold text-slate-700 dark:text-slate-300">Cruise speed (m/s)</label>
                  <input
                    id="cruise-speed"
                    type="number"
                    min="0.1"
                    max="3"
                    step="0.05"
                    value={draftParams.cruiseSpeed}
                    onChange={(event) => handleParamChange('cruiseSpeed', event.target.value)}
                    className="w-full px-3 py-2.5 rounded-xl border border-slate-200 dark:border-slate-700 bg-slate-50 dark:bg-slate-800/80 text-slate-800 dark:text-white font-mono text-[10px] focus:outline-none focus:ring-2 focus:ring-sky-500"
                  />
                </div>
                <div>
                  <label htmlFor="acceptance-radius" className="block mb-1 font-semibold text-slate-700 dark:text-slate-300">Acceptance radius (m)</label>
                  <input
                    id="acceptance-radius"
                    type="number"
                    min="0.5"
                    max="25"
                    step="0.5"
                    value={draftParams.acceptanceRadius}
                    onChange={(event) => handleParamChange('acceptanceRadius', event.target.value)}
                    className="w-full px-3 py-2.5 rounded-xl border border-slate-200 dark:border-slate-700 bg-slate-50 dark:bg-slate-800/80 text-slate-800 dark:text-white font-mono text-[10px] focus:outline-none focus:ring-2 focus:ring-sky-500"
                  />
                </div>
              </div>

              <div className="rounded-2xl border border-sky-100 dark:border-sky-900/60 bg-sky-50/70 dark:bg-sky-950/20 p-3">
                <div className="flex items-center justify-between gap-2">
                  <div>
                    <p className="text-[10px] font-bold text-sky-800 dark:text-sky-300">Target ROS 2 Node Parameter</p>
                    <p className="text-[9px] font-mono text-sky-700/70 dark:text-sky-400/70">Node: /gps_navigation · Slot: {draftParams.trajectory || 'A'} · {plannedWaypoints?.length || 0} WPs</p>
                  </div>
                  <span className="text-[9px] font-semibold text-emerald-600 dark:text-emerald-400">READY</span>
                </div>
              </div>

              <div className="flex items-center justify-end gap-2 pt-1">
                <button
                  type="button"
                  onClick={() => setIsConfigOpen(false)}
                  className="px-4 py-2 rounded-full font-semibold text-slate-600 dark:text-slate-400 hover:bg-slate-100 dark:hover:bg-slate-800 transition-colors"
                >
                  Cancel
                </button>
                <button
                  type="submit"
                  className="px-4 py-2 rounded-full font-bold bg-sky-600 text-white hover:bg-sky-700 shadow-md transition-colors"
                >
                  Apply & Upload to ROS 2
                </button>
              </div>
            </form>
          </div>
        </div>
      )}
    </>
  );
}
