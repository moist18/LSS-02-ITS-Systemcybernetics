import React from 'react';

export default function SonarPlayerCard() {
  const depth = 24.8;
  const depthRange = 50;
  const depthProgress = `${(depth / depthRange) * 100}%`;

  return (
    <div className="bg-[#14171d] text-white rounded-3xl p-4 shadow-cockpit flex flex-col gap-2 border border-white/5 transition-all w-full">
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2.5">
          <div className="w-8 h-8 rounded-xl bg-sky-500/15 text-sky-400 flex items-center justify-center">
            <span className="material-symbols-outlined text-base">waves</span>
          </div>
          <div>
            <h3 className="text-xs font-bold tracking-tight">Sonar Depth</h3>
            <p className="text-[10px] text-slate-400">Bathymetric bottom reading</p>
          </div>
        </div>
        <span className="flex items-center gap-1 text-[9px] font-semibold text-emerald-400">
          <span className="w-1.5 h-1.5 rounded-full bg-emerald-400 animate-pulse"></span>
          LIVE
        </span>
      </div>

      <div className="flex items-end justify-between pt-1">
        <div>
          <p className="text-[10px] text-slate-500 uppercase tracking-wider">Current depth</p>
          <div className="flex items-baseline gap-1">
            <span className="text-2xl font-bold font-mono text-white">{depth.toFixed(1)}</span>
            <span className="text-xs font-semibold text-sky-400">m</span>
          </div>
        </div>
        <div className="text-right text-[10px] font-mono text-slate-400">
          <p>Ping: 200kHz</p>
          <p>Range: 0–{depthRange}m</p>
        </div>
      </div>

      <div className="pt-1">
        <div className="h-1.5 rounded-full bg-slate-800 overflow-hidden">
          <div
            className="h-full rounded-full bg-sky-400 shadow-[0_0_8px_#38bdf8] transition-all duration-500"
            style={{ width: depthProgress }}
          />
        </div>
      </div>
    </div>
  );
}
