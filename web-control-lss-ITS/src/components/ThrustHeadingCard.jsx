import React, { useState } from 'react';

export default function ThrustHeadingCard() {
  const [surge, setSurge] = useState(1.45);

  // Knots calculation: 1 m/s ~= 1.94384 knots
  const knots = (surge * 1.94384).toFixed(2);

  // Dynamic RPM estimation based on surge limit (max ~2000 RPM at 2.0 m/s)
  const portRpm = Math.round((surge / 2.0) * 2000);
  const stbdRpm = Math.round(portRpm * 1.02); // slight differential for trim

  const portPercent = Math.min(Math.round((portRpm / 2000) * 100), 100);
  const stbdPercent = Math.min(Math.round((stbdRpm / 2000) * 100), 100);

  return (
    <div className="bg-white dark:bg-[#151921] rounded-3xl p-5 shadow-cockpit border border-slate-200/50 dark:border-slate-800 flex flex-col gap-4 transition-colors">
      <div className="flex items-center justify-between">
        <div>
          <h3 className="text-xs font-bold text-slate-900 dark:text-white uppercase tracking-wider">
            Thrust &amp; Heading Alignment
          </h3>
          <p className="text-[10px] text-slate-400 dark:text-slate-500">
            Dual T1/T2 Brushless Thrusters
          </p>
        </div>
        <div className="flex items-center gap-1 text-[11px] font-bold text-sky-700 dark:text-sky-300 bg-sky-50 dark:bg-sky-950/60 border border-sky-100 dark:border-sky-800/50 px-2.5 py-1 rounded-full shadow-xs">
          <span>HDG 214° SW</span>
        </div>
      </div>

      {/* Slider Control */}
      <div className="flex flex-col gap-2">
        <div className="flex items-center justify-between text-xs">
          <span className="text-slate-500 dark:text-slate-400 font-medium">Target Surge Limit</span>
          <span className="font-mono font-bold text-slate-900 dark:text-white">
            <span>{surge.toFixed(2)}</span> m/s ({knots} kn)
          </span>
        </div>
        
        <input
          type="range"
          min="0"
          max="2.0"
          step="0.05"
          value={surge}
          onChange={(e) => setSurge(parseFloat(e.target.value))}
          className="w-full accent-slate-900 dark:accent-sky-400 h-2 bg-slate-100 dark:bg-slate-800 rounded-lg cursor-pointer"
        />

        <div className="flex justify-between text-[10px] font-mono text-slate-400 dark:text-slate-500">
          <span>0.0 m/s</span>
          <span>Cruise: 1.20</span>
          <span>Max: 2.0 m/s</span>
        </div>
      </div>

      {/* Dual Thruster RPM Gauges */}
      <div className="grid grid-cols-2 gap-3 pt-1">
        {/* Port (T1) */}
        <div className="p-2.5 rounded-2xl bg-slate-50 dark:bg-slate-800/60 border border-slate-100 dark:border-slate-800 flex flex-col gap-1 transition-colors">
          <div className="flex justify-between items-center text-[10px] text-slate-500 dark:text-slate-400 font-medium">
            <span>Port (T1)</span>
            <span className="font-mono text-slate-900 dark:text-white font-bold">
              {portRpm.toLocaleString()} RPM
            </span>
          </div>
          <div className="w-full bg-slate-200 dark:bg-slate-700 h-1 rounded-full overflow-hidden">
            <div
              className="bg-slate-800 dark:bg-sky-400 h-full rounded-full transition-all duration-300"
              style={{ width: `${portPercent}%` }}
            ></div>
          </div>
        </div>

        {/* Starboard (T2) */}
        <div className="p-2.5 rounded-2xl bg-slate-50 dark:bg-slate-800/60 border border-slate-100 dark:border-slate-800 flex flex-col gap-1 transition-colors">
          <div className="flex justify-between items-center text-[10px] text-slate-500 dark:text-slate-400 font-medium">
            <span>Starboard (T2)</span>
            <span className="font-mono text-slate-900 dark:text-white font-bold">
              {stbdRpm.toLocaleString()} RPM
            </span>
          </div>
          <div className="w-full bg-slate-200 dark:bg-slate-700 h-1 rounded-full overflow-hidden">
            <div
              className="bg-slate-800 dark:bg-sky-400 h-full rounded-full transition-all duration-300"
              style={{ width: `${stbdPercent}%` }}
            ></div>
          </div>
        </div>
      </div>
    </div>
  );
}
