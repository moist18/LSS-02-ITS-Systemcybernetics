import React, { useState, useEffect } from 'react';
import { TelemetryProvider } from './context/TelemetryContext';
import Sidebar from './components/Sidebar';
import Header from './components/Header';
import LocationLiveCard from './components/LocationLiveCard';
import EmergencyCard from './components/EmergencyCard';
import WeatherCard from './components/WeatherCard';
import PodCoolingCard from './components/PodCoolingCard';
import RosTopicsCard from './components/RosTopicsCard';
import SonarPlayerCard from './components/SonarPlayerCard';
import HumidityPhCard from './components/HumidityPhCard';

import PerceptionCard from './components/PerceptionCard';
import AddTelemetryModal from './components/AddTelemetryModal';

export default function App() {
  const [darkMode, setDarkMode] = useState(false);
  const [activeNav, setActiveNav] = useState('layers');
  const [searchQuery, setSearchQuery] = useState('');
  const [isModalOpen, setIsModalOpen] = useState(false);

  useEffect(() => {
    if (darkMode) {
      document.documentElement.classList.add('dark');
    } else {
      document.documentElement.classList.remove('dark');
    }
  }, [darkMode]);

  return (
    <TelemetryProvider>
      <div className="min-h-screen bg-[#f1f3f6] dark:bg-[#0c0e14] text-slate-800 dark:text-slate-100 font-sans antialiased flex selection:bg-slate-900 selection:text-white transition-colors duration-300">
        {/* 1. LEFT ICONIC COCKPIT SIDEBAR */}
        <Sidebar activeNav={activeNav} setActiveNav={setActiveNav} />

        {/* 2. MAIN COCKPIT VIEWPORT */}
        <div className="flex-1 flex flex-col min-h-screen overflow-x-hidden">
          {/* Top Header Bar */}
          <Header
            darkMode={darkMode}
            setDarkMode={setDarkMode}
            searchQuery={searchQuery}
            setSearchQuery={setSearchQuery}
            onAddTelemetry={() => setIsModalOpen(true)}
          />

          {/* Main Body Grid Layout */}
          <main className="px-8 pb-8 flex-1 grid grid-cols-12 gap-6">
            {/* ================= LEFT / CENTER 7 COLUMNS ================= */}
            <div className="col-span-12 xl:col-span-7 flex flex-col gap-6">
              {/* ROW 1: LIVE FEED & TACTICAL MAP CONTAINER + MINI SWITCHES */}
              <div className="grid grid-cols-12 gap-6">
                {/* 1A: Primary Tactical Surface Feed Card (8 cols) */}
                <div className="col-span-12 md:col-span-8 flex">
                  <div className="w-full">
                    <LocationLiveCard />
                  </div>
                </div>

                {/* 1B: Vertical Mini-Switches Stack (4 cols) */}
                <div className="col-span-12 md:col-span-4 flex flex-col gap-4 justify-between">
                  <EmergencyCard />
                  <SonarPlayerCard />
                  <HumidityPhCard />
                  <WeatherCard />
                  <PodCoolingCard />
                </div>
              </div>
            </div>

            {/* ================= RIGHT 5 COLUMNS (ROS TOPICS & TELEMETRY) ================= */}
            <div className="col-span-12 xl:col-span-5 flex flex-col gap-6 h-full justify-between">
              {/* 3A: ROS Topics Feed Table */}
              <RosTopicsCard searchQuery={searchQuery} />

              {/* 3B: Perception Computer Vision Live Feed & Object Tracking Card */}
              <PerceptionCard className="flex-1" />
            </div>
          </main>
        </div>

        {/* Add Telemetry Stream Modal */}
        <AddTelemetryModal
          isOpen={isModalOpen}
          onClose={() => setIsModalOpen(false)}
          onAdd={(data) => {
            alert(`Successfully subscribed to topic: ${data.name} (${data.type}) at ${data.rate}`);
          }}
        />
      </div>
    </TelemetryProvider>
  );
}
