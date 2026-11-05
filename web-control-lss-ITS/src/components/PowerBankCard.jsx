import React from 'react';

export default function PowerBankCard() {
  return (
    <div className="bg-white dark:bg-[#151921] rounded-3xl p-5 shadow-cockpit border border-slate-200/50 dark:border-slate-800 flex flex-col justify-between transition-colors">
      <div className="flex items-center justify-between">
        <h4 className="text-xs font-bold text-slate-900 dark:text-white">Power Bank</h4>
        <button 
          onClick={() => alert("Battery System Diagnostic:\nChemistry: Lithium Polymer (LiPo 6S8P)\nCapacity: 120 Ah (3,024 Wh)\nCell Delta: 0.012V (Balanced)\nCycle Count: 142 cycles")}
          className="text-slate-400 hover:text-slate-600 dark:hover:text-slate-200"
        >
          <span className="material-symbols-outlined text-sm">more_horiz</span>
        </button>
      </div>

      <div className="flex items-center gap-4 my-2">
        {/* Automotive Vertical Battery Icon matching screen.png */}
        <div className="relative w-12 h-20 bg-slate-900 dark:bg-slate-800 rounded-xl p-1 flex flex-col items-center justify-between shadow-md border border-white/5">
          {/* Battery terminal top tip */}
          <div className="w-4 h-1.5 bg-slate-400 rounded-t-sm -mt-2"></div>
          <span className="material-symbols-outlined text-amber-400 text-sm mt-0.5">bolt</span>
          
          {/* Charge fill level indicator */}
          <div className="w-full bg-emerald-500 h-10 rounded-lg flex items-center justify-center text-[10px] font-bold text-white font-mono shadow-[0_0_8px_rgba(16,185,129,0.3)]">
            96%
          </div>
        </div>

        {/* Battery Telemetry details */}
        <div className="flex flex-col gap-1.5">
          <div>
            <div className="text-base font-extrabold text-slate-900 dark:text-white font-mono leading-none">
              48.2 nm
            </div>
            <span className="text-[10px] text-slate-400 dark:text-slate-500">Range Left</span>
          </div>
          <div>
            <div className="text-xs font-bold text-slate-800 dark:text-slate-200 font-mono leading-none">
              25.2 V
            </div>
            <span className="text-[10px] text-slate-400 dark:text-slate-500">Pack Voltage</span>
          </div>
          <div>
            <div className="text-[11px] font-bold text-slate-700 dark:text-slate-300 font-mono leading-none">
              631.2 nm
            </div>
            <span className="text-[9px] text-slate-400 dark:text-slate-500">Total Traveled</span>
          </div>
        </div>
      </div>

      {/* Health Status */}
      <div className="flex items-center justify-between text-[10px] text-emerald-600 dark:text-emerald-400 font-semibold border-t border-slate-100 dark:border-slate-800/60 pt-2">
        <span className="flex items-center gap-1">
          <span className="w-1.5 h-1.5 rounded-full bg-emerald-500"></span>
          Nominal (LiPo)
        </span>
        <span className="font-mono text-slate-500 dark:text-slate-400">Temp: 38°C</span>
      </div>
    </div>
  );
}
