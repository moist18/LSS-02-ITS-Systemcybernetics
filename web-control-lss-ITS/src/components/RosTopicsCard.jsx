import React, { useState } from 'react';

export default function RosTopicsCard({ searchQuery = '' }) {
  const [activeTab, setActiveTab] = useState('Live'); // 'Live' | 'Filtered' | 'Alerts'
  const [favorites, setFavorites] = useState(['/yolo_detector']);

  const initialTopics = [
    {
      id: '/cmd_vel',
      name: '/cmd_vel',
      detail: 'Linear: 1.45 m/s • Yaw: -0.04',
      hz: '100 Hz',
      latency: '12ms',
      icon: 'navigation',
      iconBg: 'bg-sky-50 dark:bg-sky-950/50 text-sky-600 dark:text-sky-400',
    },
    {
      id: '/yolo_detector',
      name: '/yolo_detector',
      detail: 'Buoy_04 target conf=0.94',
      hz: '30 Hz',
      latency: '18ms',
      icon: 'radar',
      iconBg: 'bg-slate-900 dark:bg-sky-500 text-white',
      isActiveHighlight: true,
    },
    {
      id: '/odom_filtered',
      name: '/odom_filtered',
      detail: 'Pose x=14.28 y=-84.10',
      hz: '50 Hz',
      latency: '08ms',
      icon: 'explore',
      iconBg: 'bg-slate-100 dark:bg-slate-800 text-slate-600 dark:text-slate-300',
    },
    {
      id: '/sonar_bathymetry',
      name: '/sonar/bathymetry_raw',
      detail: 'Chirp 200kHz bottom echo',
      hz: '20 Hz',
      latency: '15ms',
      icon: 'water',
      iconBg: 'bg-teal-50 dark:bg-teal-950/50 text-teal-600 dark:text-teal-400',
      alert: true,
    },
    {
      id: '/battery_state',
      name: '/sensors/battery_state',
      detail: '25.2V • 96% • LiPo 6S',
      hz: '10 Hz',
      latency: '04ms',
      icon: 'bolt',
      iconBg: 'bg-amber-50 dark:bg-amber-950/50 text-amber-600 dark:text-amber-400',
    }
  ];

  const toggleFavorite = (id, e) => {
    e.stopPropagation();
    setFavorites(prev => 
      prev.includes(id) ? prev.filter(x => x !== id) : [...prev, id]
    );
  };

  // Filter based on tab and search query
  const displayedTopics = initialTopics.filter(t => {
    const matchesSearch = searchQuery === '' || 
      t.name.toLowerCase().includes(searchQuery.toLowerCase()) || 
      t.detail.toLowerCase().includes(searchQuery.toLowerCase());
      
    if (!matchesSearch) return false;

    if (activeTab === 'Filtered') return favorites.includes(t.id);
    if (activeTab === 'Alerts') return t.alert === true;
    return ['/cmd_vel', '/yolo_detector', '/odom_filtered'].includes(t.id) || searchQuery !== '';
  });

  return (
    <div className="bg-white dark:bg-[#151921] rounded-3xl p-5 shadow-cockpit border border-slate-200/50 dark:border-slate-800 flex flex-col justify-between transition-colors">
      <div>
        <div className="flex items-center justify-between pb-3">
          <div className="flex items-center gap-4">
            <h3 className="text-sm font-bold text-slate-900 dark:text-white">ROS 2 Topics</h3>
            <div className="flex items-center gap-2 text-xs font-semibold">
              <button
                onClick={() => setActiveTab('Live')}
                className={`pb-0.5 transition-colors ${
                  activeTab === 'Live'
                    ? 'text-slate-900 dark:text-white border-b-2 border-slate-900 dark:border-white font-bold'
                    : 'text-slate-400 hover:text-slate-600 dark:text-slate-500 dark:hover:text-slate-300'
                }`}
              >
                Live
              </button>
              <button
                onClick={() => setActiveTab('Filtered')}
                className={`pb-0.5 transition-colors ${
                  activeTab === 'Filtered'
                    ? 'text-slate-900 dark:text-white border-b-2 border-slate-900 dark:border-white font-bold'
                    : 'text-slate-400 hover:text-slate-600 dark:text-slate-500 dark:hover:text-slate-300'
                }`}
              >
                Filtered
              </button>
              <button
                onClick={() => setActiveTab('Alerts')}
                className={`pb-0.5 transition-colors ${
                  activeTab === 'Alerts'
                    ? 'text-slate-900 dark:text-white border-b-2 border-slate-900 dark:border-white font-bold'
                    : 'text-slate-400 hover:text-slate-600 dark:text-slate-500 dark:hover:text-slate-300'
                }`}
              >
                Alerts
              </button>
            </div>
          </div>
          <button 
            onClick={() => alert("ROS 2 Master: CycloneDDS\nDomain ID: 42\nTotal Publishers: 34\nTotal Subscribers: 28")}
            className="text-slate-400 hover:text-slate-600 dark:hover:text-slate-200"
          >
            <span className="material-symbols-outlined text-sm">more_horiz</span>
          </button>
        </div>

        {/* Topic Row Items styled like Songs in screen.png */}
        <div className="space-y-1 text-xs">
          {displayedTopics.map((topic) => {
            const isFav = favorites.includes(topic.id);
            const isHighlighted = topic.isActiveHighlight && activeTab === 'Live';

            return (
              <div
                key={topic.id}
                className={`py-2 px-2.5 rounded-xl flex items-center justify-between transition-colors ${
                  isHighlighted
                    ? 'bg-slate-50 dark:bg-slate-800/80 shadow-xs'
                    : 'hover:bg-slate-50/70 dark:hover:bg-slate-800/40'
                }`}
              >
                <div className="flex items-center gap-3">
                  <div className={`w-8 h-8 rounded-full flex items-center justify-center font-bold text-[10px] ${topic.iconBg}`}>
                    <span className="material-symbols-outlined text-sm">{topic.icon}</span>
                  </div>
                  <div>
                    <p className={`font-bold text-[11px] ${isHighlighted ? 'text-slate-900 dark:text-white' : 'text-slate-800 dark:text-slate-200'}`}>
                      {topic.name}
                    </p>
                    <p className="text-[10px] text-slate-400 dark:text-slate-500">
                      {topic.detail}
                    </p>
                  </div>
                </div>

                <div className="flex items-center gap-4">
                  <span className="text-[10px] text-slate-400 dark:text-slate-500 font-mono hidden sm:inline">
                    {topic.hz}
                  </span>
                  <span className="font-mono text-[10px] text-emerald-600 dark:text-emerald-400 font-semibold">
                    {topic.latency}
                  </span>
                  <button 
                    onClick={(e) => toggleFavorite(topic.id, e)}
                    className="flex items-center justify-center p-1 hover:scale-110 transition-transform"
                    title={isFav ? "Remove from favorite" : "Add to favorite"}
                  >
                    <span className={`material-symbols-outlined text-sm transition-colors ${
                      isFav 
                        ? 'text-slate-800 dark:text-white' 
                        : 'text-slate-300 dark:text-slate-600 hover:text-slate-500'
                    }`}>
                      favorite
                    </span>
                  </button>
                </div>
              </div>
            );
          })}
        </div>
      </div>

      <div className="pt-2 mt-2 flex items-center justify-between text-[11px] text-slate-400 dark:text-slate-500 border-t border-slate-100 dark:border-slate-800/60">
        <span className="font-medium text-slate-600 dark:text-slate-400">18 Nodes Active</span>
        <span className="font-mono text-emerald-600 dark:text-emerald-400 flex items-center gap-1.5">
          <span className="w-1.5 h-1.5 rounded-full bg-emerald-500 animate-pulse"></span>
          ws://localhost:9090 connected
        </span>
      </div>
    </div>
  );
}
