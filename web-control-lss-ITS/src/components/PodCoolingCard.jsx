import React, { useState } from 'react';

export default function PodCoolingCard() {
  const [coolingActive, setCoolingActive] = useState(true);

  return (
    <div className="bg-white dark:bg-[#151921] rounded-3xl p-4 shadow-cockpit border border-slate-200/50 dark:border-slate-800 flex flex-col gap-2 transition-colors">
      <div className="flex items-center justify-between">
        <div>
          <h3 className="text-xs font-bold text-slate-900 dark:text-white">Pod Cooling</h3>
          <p className="text-[10px] text-slate-400 dark:text-slate-500">
            Jetson Orin: {coolingActive ? '46°C' : '58°C'}
          </p>
        </div>
        {/* Mini toggle */}
        <button
          type="button"
          onClick={() => setCoolingActive(!coolingActive)}
          className={`w-8 h-4 rounded-full flex items-center p-0.5 transition-colors ${
            coolingActive ? 'bg-slate-900 dark:bg-sky-500 justify-end' : 'bg-slate-200 dark:bg-slate-700 justify-start'
          }`}
          title="Toggle Auto Cooling"
        >
          <div className="w-3 h-3 rounded-full bg-white shadow-xs"></div>
        </button>
      </div>

      <div className="space-y-2 pt-1">
        {/* Fan speed */}
        <div className="flex items-center gap-2 text-[10px] text-slate-500 dark:text-slate-400 font-medium">
          <span className="w-8">Fan</span>
          <div className="flex-1 bg-slate-100 dark:bg-slate-800 h-1.5 rounded-full overflow-hidden">
            <div 
              className={`h-full rounded-full transition-all duration-500 ${
                coolingActive ? 'w-2/3 bg-slate-800 dark:bg-slate-200' : 'w-1/4 bg-slate-400'
              }`}
            ></div>
          </div>
          <span className="font-mono text-slate-800 dark:text-slate-200 font-bold w-10 text-right">
            {coolingActive ? '2,400' : '800'}
          </span>
        </div>

        {/* Pump speed */}
        <div className="flex items-center gap-2 text-[10px] text-slate-500 dark:text-slate-400 font-medium">
          <span className="w-8">Pump</span>
          <div className="flex-1 bg-slate-100 dark:bg-slate-800 h-1.5 rounded-full overflow-hidden">
            <div 
              className={`h-full rounded-full transition-all duration-500 ${
                coolingActive ? 'w-4/5 bg-sky-500 shadow-[0_0_6px_#0ea5e9]' : 'w-1/5 bg-slate-400'
              }`}
            ></div>
          </div>
          <span className="font-mono text-slate-800 dark:text-slate-200 font-bold w-10 text-right">
            {coolingActive ? '85%' : '20%'}
          </span>
        </div>
      </div>
    </div>
  );
}
