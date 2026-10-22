import React, { useEffect, useState } from 'react';
import { BrowserRouter, Routes, Route } from 'react-router-dom';
import HomePage from './pages/HomePage';
import DashboardPage from './pages/DashboardPage';
import SettingsPage from './pages/SettingsPage';
import RemoteControlPage from './pages/RemoteControlPage';

export default function App() {
  const [battery, setBattery] = useState(null);
  const [isCharging, setIsCharging] = useState(false);


  useEffect(() => {
    const fetchBattery = () => {
      fetch('http://192.168.3.5/percent')
        .then(response => response.text())
        .then(text => {
          const match = text.match(/\d+(\.\d+)?/);
          if (match) {
            const parsed = parseFloat(match[0]);
            setBattery(parsed);
            setIsCharging(false); // Ajusta si tienes info de carga
          } else {
            console.warn("⚠️ No se pudo parsear el valor de batería:", text);
          }
        })
        .catch(err => console.error("Error consultando batería:", err));
    };

    fetchBattery();
    const interval = setInterval(fetchBattery, 1000);
    return () => clearInterval(interval);
  }, []);

  useEffect(() => {
    const ws = new WebSocket(`ws://${window.location.hostname}:8000/ws_frames`);
    ws.onmessage = (e) => {
      const data = JSON.parse(e.data);
      if (typeof data.battery === 'number') {
        setBattery(data.battery);
      }
      if (typeof data.charging === 'boolean') {
        setIsCharging(data.charging);
      }
    };
    return () => ws.close();
  }, []);


  return (
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<HomePage battery={battery} isCharging={isCharging} />} />
        <Route path="/dashboard" element={<DashboardPage battery={battery} isCharging={isCharging} />} />
        <Route path="/settings" element={<SettingsPage battery={battery} isCharging={isCharging} />} />
        <Route path="/control" element={<RemoteControlPage battery={battery} isCharging={isCharging} />} />
      </Routes>
    </BrowserRouter>
  );
}