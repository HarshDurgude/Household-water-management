import React from "react";
import { useLocation, Link } from "react-router-dom";
import { Settings, User } from "lucide-react";
import { useStatusBarPadding } from "../utils/useStatusBarPadding";

export default function TopBar() {
  const topInset = useStatusBarPadding();
  const loc = useLocation();

  const map = {
    "/": "Tank Monitor",
    "/pump": "Pump Control",
    "/taps": "Taps",
    "/settings": "Settings",
    "/account": "Account",
  };

  const title = map[loc.pathname] ?? "Water Manager";

  return (
    <header
      style={{ paddingTop: topInset }}
      className="w-full px-4 py-3 bg-background border-b border-white/10"
    >
      <div className="max-w-xl mx-auto flex items-center justify-between">
        <h1 className="text-lg font-semibold text-white">{title}</h1>

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