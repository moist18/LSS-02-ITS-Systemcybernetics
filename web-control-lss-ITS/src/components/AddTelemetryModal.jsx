import React, { useState } from 'react';

export default function AddTelemetryModal({ isOpen, onClose, onAdd }) {
  const [topicName, setTopicName] = useState('');
  const [topicType, setTopicType] = useState('sensor_msgs/NavSatFix');
  const [rate, setRate] = useState('50');

  if (!isOpen) return null;

  const handleSubmit = (e) => {
    e.preventDefault();
    if (!topicName.trim()) return;
    if (onAdd) {
      onAdd({
        name: topicName.startsWith('/') ? topicName : `/${topicName}`,
        type: topicType,
        rate: `${rate} Hz`,
      });
    }
    setTopicName('');
    onClose();
  };

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-slate-900/60 backdrop-blur-sm animate-fadeIn">
      <div className="bg-white dark:bg-[#151921] rounded-3xl p-6 shadow-2xl border border-slate-200 dark:border-slate-800 w-full max-w-md">
        <div className="flex items-center justify-between pb-4 border-b border-slate-100 dark:border-slate-800">
          <div className="flex items-center gap-2.5">
            <div className="w-8 h-8 rounded-full bg-sky-50 dark:bg-sky-950/60 text-sky-600 dark:text-sky-400 flex items-center justify-center">
              <span className="material-symbols-outlined text-base">sensors</span>
            </div>
            <h3 className="text-sm font-bold text-slate-900 dark:text-white">Add ROS Telemetry Stream</h3>
          </div>
          <button
            onClick={onClose}
            className="text-slate-400 hover:text-slate-600 dark:hover:text-slate-200 p-1 rounded-lg"
          >
            <span className="material-symbols-outlined text-base">close</span>
          </button>
        </div>

        <form onSubmit={handleSubmit} className="space-y-4 pt-4 text-xs">
          <div>
            <label className="block font-semibold text-slate-700 dark:text-slate-300 mb-1">
              ROS 2 Topic Name
            </label>
            <input
              type="text"
              placeholder="/imu/data_raw"
              value={topicName}
              onChange={(e) => setTopicName(e.target.value)}
              className="w-full px-3.5 py-2.5 rounded-xl border border-slate-200 dark:border-slate-700 bg-slate-50 dark:bg-slate-800/80 text-slate-800 dark:text-white font-mono focus:outline-none focus:ring-2 focus:ring-sky-500"
              required
            />
          </div>

          <div>
            <label className="block font-semibold text-slate-700 dark:text-slate-300 mb-1">
              Message Interface
            </label>
            <select
              value={topicType}
              onChange={(e) => setTopicType(e.target.value)}
              className="w-full px-3.5 py-2.5 rounded-xl border border-slate-200 dark:border-slate-700 bg-slate-50 dark:bg-slate-800/80 text-slate-800 dark:text-white font-mono focus:outline-none focus:ring-2 focus:ring-sky-500"
            >
              <option value="sensor_msgs/NavSatFix">sensor_msgs/NavSatFix (GPS/RTK)</option>
              <option value="sensor_msgs/Imu">sensor_msgs/Imu (9-DOF AHRS)</option>
              <option value="geometry_msgs/Twist">geometry_msgs/Twist (Command Velocity)</option>
              <option value="marine_sensor_msgs/Bathymetry">marine_sensor_msgs/Bathymetry (Sonar)</option>
              <option value="sensor_msgs/BatteryState">sensor_msgs/BatteryState (BMS)</option>
            </select>
          </div>

          <div>
            <label className="block font-semibold text-slate-700 dark:text-slate-300 mb-1">
              Update Frequency (Hz)
            </label>
            <input
              type="number"
              min="1"
              max="500"
              value={rate}
              onChange={(e) => setRate(e.target.value)}
              className="w-full px-3.5 py-2.5 rounded-xl border border-slate-200 dark:border-slate-700 bg-slate-50 dark:bg-slate-800/80 text-slate-800 dark:text-white font-mono focus:outline-none focus:ring-2 focus:ring-sky-500"
            />
          </div>

          <div className="flex items-center justify-end gap-3 pt-3">
            <button
              type="button"
              onClick={onClose}
              className="px-4 py-2 rounded-full font-semibold text-slate-600 dark:text-slate-400 hover:bg-slate-100 dark:hover:bg-slate-800 transition-colors"
            >
              Cancel
            </button>
            <button
              type="submit"
              className="px-5 py-2 rounded-full font-bold bg-slate-900 text-white dark:bg-sky-500 dark:text-slate-950 shadow-md hover:bg-slate-800 transition-colors"
            >
              Subscribe Stream
            </button>
          </div>
        </form>
      </div>
    </div>
  );
}
