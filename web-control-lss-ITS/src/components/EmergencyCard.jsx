import React, { useState } from 'react';

export default function EmergencyCard() {
  const [isArmed, setIsArmed] = useState(false);

  const handleToggle = () => {
    if (!isArmed) {
      if (window.confirm("TRIGGER SIL-3 EMERGENCY DISARM?\nThis will cut thruster power immediately and deploy sea anchors.")) {
        setIsArmed(true);
      }
    } else {
      setIsArmed(false);
    }
  };

  return (
    <div className={`rounded-3xl p-4 shadow-cockpit border transition-all duration-300 flex items-center justify-between ${
      isArmed 
        ? 'bg-rose-50 dark:bg-rose-950/30 border-rose-200 dark:border-rose-900' 
        : 'bg-white dark:bg-[#151921] border-slate-200/50 dark:border-slate-800'
    }`}>
      <div className="flex items-center gap-3">
        <div className={`w-9 h-9 rounded-full flex items-center justify-center transition-colors ${
          isArmed 
            ? 'bg-rose-600 text-white animate-pulse' 
            : 'bg-slate-900 text-white dark:bg-slate-800'
        }`}>
          <span className="material-symbols-outlined text-base">emergency_home</span>
        </div>
        <div>
          <h3 className={`text-xs font-bold ${isArmed ? 'text-rose-700 dark:text-rose-400' : 'text-slate-900 dark:text-white'}`}>
            Emergency
          </h3>
          <p className="text-[10px] text-slate-400 dark:text-slate-500 leading-tight">
            {isArmed ? 'EMERGENCY DISARM ACTIVE' : 'Instant SIL-3 Disarm'}
          </p>
        </div>
      </div>

      {/* Toggle switch styled like screen.png */}
      <label className="relative inline-flex items-center cursor-pointer">
        <input 
          type="checkbox" 
          checked={isArmed} 
          onChange={handleToggle}
          className="sr-only peer" 
        />
        <div className="w-11 h-6 bg-slate-200 dark:bg-slate-700 peer-focus:outline-none rounded-full peer peer-checked:after:translate-x-full peer-checked:after:border-white after:content-[''] after:absolute after:top-[2px] after:left-[2px] after:bg-white after:border-slate-300 dark:after:border-slate-600 after:border after:rounded-full after:h-5 after:w-5 after:transition-all peer-checked:bg-rose-600"></div>
      </label>
    </div>
  );
}
