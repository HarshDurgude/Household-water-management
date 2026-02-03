import React from "react";
import { useLocation, Link } from "react-router-dom";
import { Settings, User } from "lucide-react";
import { useStatusBarPadding } from "../utils/useStatusBarPadding";

export default function TopBar() {
  const topInset = useStatusBarPadding();
  const loc = useLocation();

  const isHome = loc.pathname === "/";

  const map = {
    "/pump": "Pump Control",
    "/taps": "Taps",
    "/settings": "Settings",
    "/account": "Account",
  };

  return (
    <header
      style={{ paddingTop: topInset }}   // ✅ CRITICAL: DO NOT CHANGE
      className="w-full px-4 py-2 bg-background border-b border-white/10"
    >
      <div className="max-w-xl mx-auto flex items-center justify-between">
        
        {/* LEFT TITLE */}
        {isHome ? (
          <h1 className="text-xl font-bold tracking-wide text-sky-400">
            TankOS
          </h1>
        ) : (
          <h1 className="text-lg font-semibold text-white">
            {map[loc.pathname] ?? "TankOS"}
          </h1>
        )}

        {/* RIGHT ICONS */}
        <div className="flex items-center gap-3">
          <Link to="/settings" className="p-2">
            <Settings className="w-5 h-5 text-white/80" />
          </Link>
          <Link to="/account" className="p-2">
            <User className="w-5 h-5 text-white/80" />
          </Link>
        </div>

      </div>
    </header>
  );
}