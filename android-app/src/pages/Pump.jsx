import { Power } from "lucide-react";

export default function Pump() {
  return (
    <div className="h-full flex items-center justify-center px-4">
      <div className="flex flex-col items-center gap-6">

        {/* Power button */}
        <div className="w-36 h-36 rounded-full bg-card flex items-center justify-center shadow-inner">
          <Power className="w-16 h-16 text-red-500" />
        </div>

        {/* Action button */}
        <button className="px-12 py-4 bg-primary rounded-full text-white font-semibold text-lg">
          Toggle Pump
        </button>

        {/* Helper text */}
        <p className="text-sm text-white/60">
          Manual override
        </p>

      </div>
    </div>
  );
}