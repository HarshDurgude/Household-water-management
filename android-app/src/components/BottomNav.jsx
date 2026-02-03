import { Link, useLocation } from "react-router-dom";
import { Zap, Droplet, GitBranch } from "lucide-react";

export default function BottomNav() {
  const { pathname } = useLocation();

  const items = [
    {
      to: "/pump",
      label: "Pump",
      icon: Zap,
    },
    {
      to: "/",
      label: "Tank",
      icon: Droplet,
    },
    {
      to: "/taps",
      label: "Taps",
      icon: GitBranch,
    },
  ];

  return (
    <nav
      className="bg-gradient-to-t from-[#0b1220] to-[#0f172a] border-t border-white/10"
      style={{
        paddingBottom: "calc(env(safe-area-inset-bottom) + 8px)",
        height: "72px",
      }}
    >
      <div className="h-full grid grid-cols-3 gap-3 px-4 items-center">
        {items.map(({ to, label, icon: Icon }) => {
          const active = pathname === to;

          return (
            <Link
              key={to}
              to={to}
              className={`
                flex flex-col items-center justify-center gap-1
                h-12 rounded-xl text-sm transition
                ${active
                  ? "bg-primary/20 text-white"
                  : "text-white/70 hover:bg-white/5 active:bg-white/10"}
              `}
            >
              <Icon className="w-5 h-5" />
              <span>{label}</span>
            </Link>
          );
        })}
      </div>
    </nav>
  );
}