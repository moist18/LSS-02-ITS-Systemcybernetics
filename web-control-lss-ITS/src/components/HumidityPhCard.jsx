import React from 'react';

const sensors = [
  {
    label: 'Humidity',
    value: '68%',
    level: '68%',
    icon: 'humidity_high',
    iconClass: 'bg-sky-50 text-sky-600 dark:bg-sky-950/50 dark:text-sky-400',
    barClass: 'bg-sky-500',
  },
  {
    label: 'pH Sensor',
    value: '7.82',
    level: '56%',
    icon: 'science',
    iconClass: 'bg-emerald-50 text-emerald-600 dark:bg-emerald-950/50 dark:text-emerald-400',
    barClass: 'bg-emerald-500',
  },
];

export default function HumidityPhCard() {
  return (
    <div className="bg-white dark:bg-[#151921] rounded-3xl p-4 shadow-cockpit border border-slate-200/50 dark:border-slate-800 flex flex-col gap-2 transition-colors">
      <div className="flex items-center justify-between">
        <div>
          <h3 className="text-xs font-bold text-slate-900 dark:text-white">Humidity &amp; pH Sensors</h3>
          <p className="text-[10px] text-slate-400 dark:text-slate-500">Environmental monitoring</p>
        </div>
        <span className="flex items-center gap-1 text-[9px] font-semibold text-emerald-600 dark:text-emerald-400">
          <span className="w-1.5 h-1.5 rounded-full bg-emerald-500 animate-pulse"></span>
          ONLINE
        </span>
      </div>

      <div className="space-y-2 pt-1">
        {sensors.map((sensor) => (
          <div key={sensor.label} className="flex items-center gap-2">
            <div className={`w-7 h-7 rounded-xl flex items-center justify-center ${sensor.iconClass}`}>
              <span className="material-symbols-outlined text-sm">{sensor.icon}</span>
            </div>

            <div className="flex-1 min-w-0">
              <div className="flex items-center justify-between gap-2 text-[10px]">
                <span className="font-medium text-slate-500 dark:text-slate-400">{sensor.label}</span>
                <span className="font-mono font-bold text-slate-900 dark:text-white">{sensor.value}</span>
              </div>
              <div className="mt-1 h-1.5 rounded-full bg-slate-100 dark:bg-slate-800 overflow-hidden">
                <div
                  className={`h-full rounded-full transition-all duration-500 ${sensor.barClass}`}
                  style={{ width: sensor.level }}
                />
              </div>
            </div>

            <span className="text-[9px] font-semibold text-emerald-600 dark:text-emerald-400">STABLE</span>
          </div>
        ))}
      </div>
    </div>
  );
}
