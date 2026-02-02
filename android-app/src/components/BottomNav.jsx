import { Link, useLocation } from "react-router-dom";
import { Zap, Droplet, GitBranch } from "lucide-react";

export default function BottomNav() {
  const { pathname } = useLocation();

  return (
    <nav className="h-[72px] bg-gradient-to-t from-[#0b1220] to-[#0f172a] border-t border-white/10">
      <div className="relative h-full flex items-center justify-between px-6">

        {/* Pump */}
        <Link
          to="/pump"
          className={`flex flex-col items-center gap-1 text-sm ${
            pathname === "/pump" ? "text-white" : "text-white/70"
          }`}
        >
          <Zap className="w-6 h-6" />
          <span>Pump</span>
        </Link>

        {/* Center Tank button */}
        <Link to="/" className="absolute left-1/2 -translate-x-1/2 -top-6">
          <div className="w-16 h-16 rounded-full bg-primary flex items-center justify-center shadow-xl">
            <Droplet className="w-7 h-7 text-white" />
          </div>
        </Link>

        {/* Taps */}
        <Link
          to="/taps"
          className={`flex flex-col items-center gap-1 text-sm ${
            pathname === "/taps" ? "text-white" : "text-white/70"
          }`}
        >
          <GitBranch className="w-6 h-6" />
          <span>Taps</span>
        </Link>

      </div>
    </nav>
  );
}