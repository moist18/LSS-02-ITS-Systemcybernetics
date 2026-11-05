import React from 'react';

export default function Header({
  darkMode,
  setDarkMode,
  searchQuery,
  setSearchQuery,
  onAddTelemetry,
  notificationsCount = 3
}) {
  return (
    <header className="px-8 pt-6 pb-4 flex flex-wrap items-center justify-between gap-4">
      {/* Left: Voice / ROS Command Pill & Sub-tabs */}
      <div className="flex items-center gap-5 flex-1 max-w-xl">
        <div className="flex-1 flex items-center gap-3 bg-white dark:bg-[#1a1f29] px-4 py-2.5 rounded-full shadow-cockpit border border-slate-200/60 dark:border-slate-800 transition-colors">
          <span className="material-symbols-outlined text-slate-400 dark:text-slate-500 text-lg">graphic_eq</span>
          <input
            type="text"
            value={searchQuery}
            onChange={(e) => setSearchQuery(e.target.value)}
            className="bg-transparent border-0 outline-none text-xs text-slate-700 dark:text-slate-200 placeholder-slate-400 dark:placeholder-slate-500 w-full focus:ring-0 p-0 font-medium"
            placeholder="Give a voice command or filter ROS topic..."
          />
          {searchQuery && (
            <button
              onClick={() => setSearchQuery('')}
              className="text-slate-400 hover:text-slate-600 dark:hover:text-slate-200 text-xs"
            >
              <span className="material-symbols-outlined text-base">close</span>
            </button>
          )}
        </div>

        <div className="hidden lg:flex items-center gap-4 text-xs font-semibold">
          <button
            onClick={() => alert("Control panel overview opened.")}
            className="text-slate-800 dark:text-slate-200 hover:text-black dark:hover:text-white transition-colors"
          >
            Control panel
          </button>
          <button
            onClick={onAddTelemetry}
            className="text-slate-400 hover:text-slate-600 dark:hover:text-slate-300 font-medium transition-colors"
          >
            + Add telemetry
          </button>
        </div>
      </div>

      {/* Right: Light/Dark Pill, Notification Bell, User Pill, Date Badge */}
      <div className="flex items-center gap-3">
        {/* Light / Dark Switch Pill */}
        <div className="flex items-center bg-white dark:bg-[#1a1f29] p-1 rounded-full shadow-cockpit border border-slate-200/60 dark:border-slate-800 text-xs font-medium transition-colors">
          <button
            type="button"
            onClick={() => setDarkMode(false)}
            className={`px-3.5 py-1 rounded-full transition-all ${!darkMode
              ? 'bg-slate-100 text-slate-900 font-semibold shadow-xs'
              : 'text-slate-400 hover:text-slate-200'
              }`}
          >
            Light
          </button>
          <button
            type="button"
            onClick={() => setDarkMode(true)}
            className={`px-3.5 py-1 rounded-full transition-all ${darkMode
              ? 'bg-slate-800 text-white font-semibold shadow-xs'
              : 'text-slate-400 hover:text-slate-700'
              }`}
          >
            Dark
          </button>
        </div>

        {/* Notification Bell with Ping Dot */}
        <button
          type="button"
          onClick={() => alert(`You have ${notificationsCount} notifications:\n1. Benoa Harbor RTK Lock Acquired\n2. Buoy-04 Visual Target Classified (94%)\n3. Sonar Bathymetry Bottom Locked at 24.8m`)}
          className="w-10 h-10 rounded-full bg-white dark:bg-[#1a1f29] shadow-cockpit border border-slate-200/60 dark:border-slate-800 flex items-center justify-center text-slate-600 dark:text-slate-300 relative hover:text-black dark:hover:text-white transition-colors"
          title="Notifications"
        >
          <span className="material-symbols-outlined text-[19px]">notifications</span>
          <span className="absolute top-2.5 right-2.5 w-2 h-2 rounded-full bg-sky-500 ring-2 ring-white dark:ring-[#1a1f29]"></span>
        </button>

        {/* User Operator Capsule */}
        <div className="flex items-center gap-2.5 bg-white dark:bg-[#1a1f29] pl-1.5 pr-4 py-1.5 rounded-full shadow-cockpit border border-slate-200/60 dark:border-slate-800 transition-colors cursor-pointer hover:border-slate-300 dark:hover:border-slate-700">
          <img
            alt="LSS 02 ITS ELEKTRO"
            className="w-7 h-7 rounded-full object-cover ring-1 ring-slate-200 dark:ring-slate-700"
            src="https://images.unsplash.com/photo-1534528741775-53994a69daeb?w=100&auto=format&fit=crop&q=80"
            onError={(e) => {
              // fallback if external image fails
              e.currentTarget.src = "https://lh3.googleusercontent.com/aida-public/AB6AXuBS7EM9DnFRUhWiZmGiCKmxLDwFoJaR_A2R-rCFTtdCBcMRPI8Hqiz7ylvR2Zjn8iwIPKkg6JJMvz5h8Y2pafQ9BUVr9TfE8qwb_hfjssUKn6Ao-Vfjlk_-Y5FVavmgXgHZmqsqo9Juv9--U2CfwTzagMyCofHhVRGrTNz6SvSWQxJ9RiOZFCJNRXxL8Y87LgenIjkChIbH3cWO7xsEVLO8ickXRJpFFWdTEuLRXrSxwQ-nVl3pFCJH";
            }}
          />
          <span className="text-xs font-semibold text-slate-800 dark:text-slate-200">LSS 02 ITS ELEKTRO</span>
        </div>

        {/* Calendar / Mission Date Pill */}
        <div className="px-4 py-2 bg-white dark:bg-[#1a1f29] rounded-full shadow-cockpit border border-slate-200/60 dark:border-slate-800 text-xs font-semibold text-slate-700 dark:text-slate-300 transition-colors">
          22 May
        </div>
      </div>
    </header>
  );
}
