import React from 'react';

export default function BaseGatewayCard() {
  return (
    <div className="bg-white dark:bg-[#151921] rounded-3xl p-5 shadow-cockpit border border-slate-200/50 dark:border-slate-800 flex flex-col justify-between transition-colors">
      <div className="flex items-center justify-between">
        <div>
          <h4 className="text-xs font-bold text-slate-900 dark:text-white">Base Gateway</h4>
          <button 
            onClick={() => alert("Shore Gateway Info:\nSSID: LSS-SHORE-MESH-5G\nIP: 192.168.10.1\nUptime: 48h 12m\nPacket Loss: 0.01%")}
            className="text-[10px] text-slate-400 dark:text-slate-500 hover:text-slate-700 dark:hover:text-slate-300 flex items-center gap-0.5 transition-colors"
          >
            See link info <span className="material-symbols-outlined text-[11px]">arrow_forward</span>
          </button>
        </div>
        <span className="relative flex h-2 w-2">
          <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-emerald-400 opacity-75"></span>
          <span className="relative inline-flex rounded-full h-2 w-2 bg-emerald-500"></span>
        </span>
      </div>

      {/* Base Station Mast Graphic matching screen.png */}
      <div className="h-20 flex items-center justify-center my-1">
        <div className="flex items-end gap-3">
          {/* Secondary repeater antenna */}
          <div className="w-7 h-16 bg-slate-800 dark:bg-slate-700 rounded-lg p-1 flex flex-col justify-between items-center shadow-sm">
            <div className="w-3 h-1 bg-emerald-400 rounded-full"></div>
            <div className="w-1.5 h-6 bg-slate-700 dark:bg-slate-600 rounded-full"></div>
            <div className="w-4 h-1.5 bg-slate-600 dark:bg-slate-500 rounded"></div>
          </div>
          {/* Primary base antenna mast */}
          <div className="w-8 h-18 bg-slate-900 dark:bg-slate-800 rounded-lg p-1 flex flex-col justify-between items-center shadow-md">
            <span className="material-symbols-outlined text-xs text-sky-400">cell_tower</span>
            <div className="w-4 h-1 bg-emerald-400 rounded-full shadow-[0_0_6px_#34d399]"></div>
            <div className="w-5 h-2 bg-slate-800 dark:bg-slate-700 rounded"></div>
          </div>
        </div>
      </div>

      {/* Metrics */}
      <div className="flex items-center justify-between text-[10px] text-slate-500 dark:text-slate-400 font-mono border-t border-slate-100 dark:border-slate-800/60 pt-2">
        <span>PING: <strong className="text-slate-800 dark:text-slate-200">12ms</strong></span>
        <span>BW: <strong className="text-slate-800 dark:text-slate-200">48.2M</strong></span>
      </div>
    </div>
  );
}
