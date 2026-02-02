import { Capacitor } from "@capacitor/core";
import { StatusBar } from "@capacitor/status-bar";

import React, { useRef, useState, useEffect } from "react";
import { BrowserRouter, Routes, Route, Outlet } from "react-router-dom";

import TopBar from "./components/TopBar";
import BottomNav from "./components/BottomNav";

import Tank from "./pages/Tank";
import Pump from "./pages/Pump";
import Taps from "./pages/Taps";
import Settings from "./pages/Settings";
import Account from "./pages/Account";

/* App shell layout — measures top & bottom heights and gives pages the exact area */
function AppLayout() {
  const topRef = useRef(null);
  const bottomRef = useRef(null);
  const [topH, setTopH] = useState(0);
  const [bottomH, setBottomH] = useState(0);

  useEffect(() => {
    const measure = () => {
      const topRect = topRef.current?.getBoundingClientRect();
      const bottomRect = bottomRef.current?.getBoundingClientRect();
      setTopH(topRect?.height ?? 0);
      setBottomH(bottomRect?.height ?? 0);
    };

    // initial measure
    measure();

    // react to normal resize/orientation
    window.addEventListener("resize", measure);
    window.addEventListener("orientationchange", measure);

    // react to visualViewport changes on mobile (URL bar)
    const vv = window.visualViewport;
    if (vv) {
      vv.addEventListener("resize", measure);
      vv.addEventListener("scroll", measure);
    }

    return () => {
      window.removeEventListener("resize", measure);
      window.removeEventListener("orientationchange", measure);
      if (vv) {
        vv.removeEventListener("resize", measure);
        vv.removeEventListener("scroll", measure);
      }
    };
  }, []);

  
  // page area height — exact available space (reacts to measured values)
  const pageAreaStyle = {
    height: `calc(100vh - ${Math.round(topH)}px - ${Math.round(bottomH)}px)`,
  };

  return (
    <div className="h-screen flex flex-col overflow-hidden bg-background">
      {/* TopBar measured by ref */}
      <div ref={topRef}>
        <TopBar />
      </div>

      {/* Page area: pages render here (height exactly excludes top & bottom) */}
      <div style={pageAreaStyle} className="relative">
        <Outlet />
      </div>

      {/* Bottom nav measured by ref */}
      <div ref={bottomRef}>
        <BottomNav />
      </div>
    </div>
  );
}

export default function App() {
  useEffect(() => {
    if (Capacitor.getPlatform() === "android") {
      StatusBar.setOverlaysWebView({ overlay: false });
    }
  }, []);

  return (
    <BrowserRouter>
      <Routes>
        {/* Layout route that wraps all pages */}
        <Route element={<AppLayout />}>
          <Route path="/" element={<Tank />} />
          <Route path="/pump" element={<Pump />} />
          <Route path="/taps" element={<Taps />} />
          <Route path="/settings" element={<Settings />} />
          <Route path="/account" element={<Account />} />
        </Route>
      </Routes>
    </BrowserRouter>
  );
}