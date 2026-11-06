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
  const { startMission, pauseMission, resetMission, loadWaypoints, telemetry } = useTelemetry();
  const [selectedPlanId, setSelectedPlanId] = useState(waypointPlans[0].id);
  const [selectedParams, setSelectedParams] = useState(waypointPlans[0].params);
  const [draftPlanId, setDraftPlanId] = useState(waypointPlans[0].id);
  const [draftParams, setDraftParams] = useState(waypointPlans[0].params);
  const [isConfigOpen, setIsConfigOpen] = useState(false);
  const [showPayload, setShowPayload] = useState(false);
  const [configStatus, setConfigStatus] = useState('SYNCED');

  const missionStatus = telemetry?.mission?.status || 'IDLE';

  const selectedPlan = waypointPlans.find((plan) => plan.id === selectedPlanId) || waypointPlans[0];
  const draftPlan = waypointPlans.find((plan) => plan.id === draftPlanId) || waypointPlans[0];
  const rosbridgePayload = buildRosbridgePayload(draftPlan, draftParams);

  const openConfiguration = () => {
    setDraftPlanId(selectedPlanId);
    setDraftParams({ ...selectedParams });
    setShowPayload(false);
    setIsConfigOpen(true);
  };

  const handlePlanChange = (event) => {
    const nextPlan = waypointPlans.find((plan) => plan.id === event.target.value) || waypointPlans[0];
    setDraftPlanId(nextPlan.id);
    setDraftParams({ ...nextPlan.params });
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
      if (loadWaypoints) {
        const formattedWaypoints = draftPlan.waypoints.map((w, idx) => ({
          id: idx + 1,
          lat: w.lat,
          lon: w.lon,
          target_speed: Number(draftParams.cruiseSpeed) || 1.2,
          acceptance_radius: Number(draftParams.acceptanceRadius) || 3.0,
          action: 'PASS',
        }));
        await loadWaypoints(formattedWaypoints);
      }
      setConfigStatus('SYNCED TO ROS2');
    } catch {
      setConfigStatus('CONFIG STAGED');
    }
    setIsConfigOpen(false);
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
              <p className="text-[10px] text-slate-400 dark:text-slate-500 truncate">{selectedPlan.name}</p>
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
          <span>{selectedPlan.waypoints.length} WPs · {missionStatus}</span>
          <span className="flex items-center gap-1 text-emerald-600 dark:text-emerald-400 font-semibold">
            <span className={`w-1.5 h-1.5 rounded-full ${missionStatus === 'RUNNING' ? 'bg-emerald-500 animate-ping' : 'bg-emerald-500'}`}></span>
            {configStatus}
          </span>
        </div>

        <div className="space-y-1 pt-1">
          {selectedPlan.waypoints.slice(0, 4).map((waypoint, index) => (
            <div key={waypoint.id} className="flex items-center gap-2 rounded-xl px-1.5 py-1 hover:bg-slate-50 dark:hover:bg-slate-800/60 transition-colors">
              <div className="flex flex-col items-center self-stretch">
                <span className="w-5 h-5 rounded-full bg-slate-900 text-white dark:bg-sky-500 dark:text-slate-950 flex items-center justify-center text-[9px] font-bold">
                  {index + 1}
                </span>
                {index < Math.min(selectedPlan.waypoints.length, 4) - 1 && (
                  <span className="w-px flex-1 bg-slate-200 dark:bg-slate-700 mt-0.5"></span>
                )}
              </div>
              <div className="flex-1 min-w-0">
                <div className="flex items-center justify-between gap-2">
                  <span className="text-[10px] font-bold text-slate-800 dark:text-slate-200 truncate">{waypoint.name}</span>
                  <span className="text-[9px] font-mono text-slate-400 dark:text-slate-500">{waypoint.id}</span>
                </div>
                <p className="text-[9px] font-mono text-slate-400 dark:text-slate-500 truncate">
                  {waypoint.lat.toFixed(5)}, {waypoint.lon.toFixed(5)}
                </p>
              </div>
            </div>
          ))}
        </div>

        <div className="flex items-center gap-1.5 mt-1">
          <button
            onClick={openConfiguration}
            type="button"
            className="flex-1 rounded-xl bg-slate-100 dark:bg-slate-800/80 px-2.5 py-1.5 text-[10px] font-bold text-slate-700 dark:text-slate-200 hover:bg-slate-200 dark:hover:bg-slate-700 transition-colors flex items-center justify-center gap-1"
          >
            <span className="material-symbols-outlined text-xs">edit_location_alt</span>
            Route
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
                <label htmlFor="waypoint-plan" className="block mb-1 font-semibold text-slate-700 dark:text-slate-300">Waypoint configuration</label>
                <select
                  id="waypoint-plan"
                  value={draftPlanId}
                  onChange={handlePlanChange}
                  className="w-full px-3 py-2.5 rounded-xl border border-slate-200 dark:border-slate-700 bg-slate-50 dark:bg-slate-800/80 text-slate-800 dark:text-white focus:outline-none focus:ring-2 focus:ring-sky-500"
                >
                  {waypointPlans.map((plan) => (
                    <option key={plan.id} value={plan.id}>{plan.name} · {plan.waypoints.length} WPs</option>
                  ))}
                </select>
                <p className="mt-1 text-[10px] text-slate-400 dark:text-slate-500">{draftPlan.description}</p>
              </div>

              <div className="rounded-2xl bg-slate-50 dark:bg-slate-800/50 p-3 border border-slate-100 dark:border-slate-800">
                <div className="flex items-center justify-between mb-2">
                  <span className="font-semibold text-slate-700 dark:text-slate-300">Route waypoints</span>
                  <span className="font-mono text-[10px] text-slate-400">{draftPlan.waypoints.length} points</span>
                </div>
                <div className="space-y-1.5">
                  {draftPlan.waypoints.map((waypoint, index) => (
                    <div key={waypoint.id} className="flex items-center gap-2">
                      <span className="w-5 h-5 rounded-full bg-slate-900 text-white dark:bg-sky-500 dark:text-slate-950 flex items-center justify-center text-[9px] font-bold">{index + 1}</span>
                      <span className="flex-1 text-[10px] font-medium text-slate-700 dark:text-slate-300">{waypoint.name}</span>
                      <span className="text-[9px] font-mono text-slate-400">{waypoint.lat.toFixed(4)}, {waypoint.lon.toFixed(4)}</span>
                    </div>
                  ))}
                </div>
              </div>

              <div className="grid grid-cols-2 gap-3">
                <div>
                  <label htmlFor="planner-type" className="block mb-1 font-semibold text-slate-700 dark:text-slate-300">Planner</label>
                  <select
                    id="planner-type"
                    value={draftParams.planner}
                    onChange={(event) => handleParamChange('planner', event.target.value)}
                    className="w-full px-3 py-2.5 rounded-xl border border-slate-200 dark:border-slate-700 bg-slate-50 dark:bg-slate-800/80 text-slate-800 dark:text-white font-mono text-[10px] focus:outline-none focus:ring-2 focus:ring-sky-500"
                  >
                    <option value="waypoint_follower">Waypoint Follower</option>
                    <option value="nav2_smac_hybrid">Nav2 Smac Hybrid</option>
                    <option value="coverage_planner">Coverage Planner</option>
                  </select>
                </div>
                <div>
                  <label htmlFor="frame-id" className="block mb-1 font-semibold text-slate-700 dark:text-slate-300">Frame ID</label>
                  <input
                    id="frame-id"
                    value={draftParams.frameId}
                    onChange={(event) => handleParamChange('frameId', event.target.value)}
                    className="w-full px-3 py-2.5 rounded-xl border border-slate-200 dark:border-slate-700 bg-slate-50 dark:bg-slate-800/80 text-slate-800 dark:text-white font-mono text-[10px] focus:outline-none focus:ring-2 focus:ring-sky-500"
                  />
                </div>
                <div>
                  <label htmlFor="rosbridge-url" className="block mb-1 font-semibold text-slate-700 dark:text-slate-300">ROSBridge URL</label>
                  <input
                    id="rosbridge-url"
                    value={draftParams.rosbridgeUrl}
                    onChange={(event) => handleParamChange('rosbridgeUrl', event.target.value)}
                    className="w-full px-3 py-2.5 rounded-xl border border-slate-200 dark:border-slate-700 bg-slate-50 dark:bg-slate-800/80 text-slate-800 dark:text-white font-mono text-[10px] focus:outline-none focus:ring-2 focus:ring-sky-500"
                  />
                </div>
                <div>
                  <label htmlFor="parameter-namespace" className="block mb-1 font-semibold text-slate-700 dark:text-slate-300">Parameter namespace</label>
                  <input
                    id="parameter-namespace"
                    value={draftParams.parameterNamespace}
                    onChange={(event) => handleParamChange('parameterNamespace', event.target.value)}
                    className="w-full px-3 py-2.5 rounded-xl border border-slate-200 dark:border-slate-700 bg-slate-50 dark:bg-slate-800/80 text-slate-800 dark:text-white font-mono text-[10px] focus:outline-none focus:ring-2 focus:ring-sky-500"
                  />
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

              <label className="flex items-center gap-2 text-[10px] font-medium text-slate-600 dark:text-slate-400 cursor-pointer">
                <input
                  type="checkbox"
                  checked={draftParams.loop}
                  onChange={(event) => handleParamChange('loop', event.target.checked)}
                  className="rounded border-slate-300 text-sky-500 focus:ring-sky-500"
                />
                Loop route after reaching the final waypoint
              </label>

              <div className="rounded-2xl border border-sky-100 dark:border-sky-900/60 bg-sky-50/70 dark:bg-sky-950/20 p-3">
                <div className="flex items-center justify-between gap-2">
                  <div>
                    <p className="text-[10px] font-bold text-sky-800 dark:text-sky-300">ROSBridge parameter target</p>
                    <p className="text-[9px] font-mono text-sky-700/70 dark:text-sky-400/70">{draftParams.rosbridgeUrl} · {draftParams.parameterNamespace}/set_parameters_atomically</p>
                  </div>
                  <span className="text-[9px] font-semibold text-amber-600 dark:text-amber-400">READY FOR INTEGRATION</span>
                </div>
                <button
                  type="button"
                  onClick={() => setShowPayload((current) => !current)}
                  className="mt-2 text-[10px] font-semibold text-sky-700 dark:text-sky-300 hover:underline"
                >
                  {showPayload ? 'Hide ROSBridge payload' : 'Preview ROSBridge payload'}
                </button>
                {showPayload && (
                  <pre className="mt-2 max-h-40 overflow-auto rounded-xl bg-slate-950 p-2 text-[8px] leading-relaxed text-sky-200 font-mono">
                    {JSON.stringify(rosbridgePayload, null, 2)}
                  </pre>
                )}
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
                  className="px-4 py-2 rounded-full font-bold bg-slate-900 text-white dark:bg-sky-500 dark:text-slate-950 shadow-md hover:bg-slate-800 transition-colors"
                >
                  Stage parameters
                </button>
              </div>
            </form>
          </div>
        </div>
      )}
    </>
  );
}
