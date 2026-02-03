import { useTankStatus } from "../hooks/useTankStatus";
import React from "react";

export default function Tank() {
  const { data } = useTankStatus();
  return (
    <div className="h-full flex items-center justify-center px-4">
      <div className="flex flex-col items-center gap-6">

        <div className="w-64 h-72 rounded-3xl border-4 border-white/20 relative overflow-hidden">
          <div
            className="absolute bottom-0 left-0 w-full bg-sky-400 transition-all duration-700"
            style={{ height: data ? `${data.tankPercent}%` : "0%" }}
          />
        </div>

        <div className="text-center">
          <div className="text-5xl font-bold">
            {data ? Math.round(data.tankPercent) + "%" : "--"}
          </div>
          <div className="text-sm text-white/60">100 L Tank</div>
        </div>

        <div className="flex gap-4">
          <div className="bg-card px-6 py-4 rounded-xl text-center w-32">
            <div className="text-sm text-white/60">Water</div>
            <div className="text-lg font-semibold text-green-400">
              {data?.sessionActive ? "Active" : "Idle"}
            </div>
          </div>

          <div className="bg-card px-6 py-4 rounded-xl text-center w-32">
            <div className="text-sm text-white/60">Status</div>
            <div className="text-lg font-semibold text-green-400">Normal</div>
          </div>
        </div>

      </div>
    </div>
  );
}