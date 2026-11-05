import React from 'react';

export default function Sidebar({ activeNav, setActiveNav }) {
  const navItems = [
    { id: 'layers', icon: 'layers', label: 'Tactical Cockpit' },
    // { id: 'actuators', icon: 'tune', label: 'Actuator Dynamics' },
    // { id: 'calendar', icon: 'calendar_today', label: 'Mission Schedule' },
    // { id: 'settings', icon: 'settings', label: 'Vessel Config' },
    // { id: 'bathymetry', icon: 'water', label: 'Bathymetry & Sonar' },
    // { id: 'sensors', icon: 'sensors', label: 'Acoustic Sonar Link' },
  ];

  return (
    <aside className="w-[78px] bg-[#14171d] flex-shrink-0 flex flex-col items-center justify-between py-6 z-50 shadow-2xl transition-colors duration-300">
      {/* Top Logo & Core Navigation */}
      <div className="flex flex-col items-center gap-7 w-full">
        {/* Minimalist Tech Constellation Logo */}
        <div
          onClick={() => setActiveNav('layers')}
          className="w-11 h-11 rounded-2xl bg-gradient-to-tr from-slate-900 via-slate-800 to-slate-700 flex items-center justify-center border border-white/10 shadow-lg text-white cursor-pointer hover:border-sky-500/50 transition-all"
          title="LSS Marine Systems"
        >
          <svg className="w-6 h-6 text-white" fill="none" stroke="currentColor" strokeWidth="2" viewBox="0 0 24 24">
            <circle cx="8" cy="8" r="4" strokeDasharray="2 3" />
            <circle cx="16" cy="8" r="4" />
            <circle cx="8" cy="16" r="4" />
            <circle cx="16" cy="16" r="4" strokeDasharray="2 3" />
          </svg>
        </div>

        {/* Icon Bar Navigation Stack */}
        <nav className="flex flex-col items-center gap-4 w-full">
          {navItems.map((item) => {
            const isActive = activeNav === item.id;
            return (
              <button
                key={item.id}
                onClick={() => setActiveNav(item.id)}
                className={`w-11 h-11 rounded-full flex items-center justify-center transition-all duration-200 ${isActive
                    ? 'bg-white text-slate-900 shadow-lg shadow-white/10 scale-105'
                    : 'text-slate-400 hover:text-white hover:bg-white/10'
                  }`}
                title={item.label}
                type="button"
              >
                <span className="material-symbols-outlined text-[21px]">{item.icon}</span>
              </button>
            );
          })}
        </nav>
      </div>

      {/* Bottom Action / Exit Switch */}
      <div className="flex flex-col items-center gap-3">
        <button
          onClick={() => {
            if (window.confirm("Disconnect telemetry from vessel LSS-02?")) {
              alert("Telemetry disconnected. Vessel entering autonomous hold station.");
            }
          }}
          className="w-10 h-10 rounded-full text-slate-400 hover:text-rose-400 hover:bg-white/10 flex items-center justify-center transition-colors"
          title="Disconnect / Exit"
          type="button"
        >
          <span className="material-symbols-outlined text-lg">logout</span>
        </button>
      </div>
    </aside>
  );
}
