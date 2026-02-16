import { useState } from "react";

export default function Settings() {
  const [commLossTime, setCommLossTime] = useState(5); // minutes
  const [powerSaving, setPowerSaving] = useState(true);

  const [scheduleEnabled, setScheduleEnabled] = useState(false);
  const [scheduleTime, setScheduleTime] = useState("06:00");

  const [outageHours, setOutageHours] = useState(5);

  return (
    <div className="p-3 space-y-4 text-foreground">

      {/* AUTO PUMP SAFETY */}
      <div className="bg-card rounded-xl p-4 shadow-sm">
        <h3 className="font-medium mb-2">Auto Pump Safety</h3>
        <p className="text-sm text-muted-foreground mb-3">
          Turn off pump if indicator communication is lost.
        </p>

        <div className="flex items-center justify-between">
          <span className="text-sm">Cut-off time (minutes)</span>
          <input
            type="number"
            min={1}
            max={60}
            value={commLossTime}
            onChange={(e) => setCommLossTime(e.target.value)}
            className="w-20 rounded-lg bg-[#0f1b34] border border-white/10 
px-3 py-2 text-center text-white 
focus:outline-none focus:ring-2 focus:ring-blue-500"

          />
        </div>
      </div>

            {/* POWER OUTAGE RECOVERY */}
      <div className="bg-card rounded-xl p-3 shadow-sm">
        <h3 className="font-medium mb-2">Power Outage Recovery</h3>
        <p className="text-sm text-muted-foreground mb-3">
          Fully fill tank if power outage exceeds set duration.
        </p>

        <div className="flex items-center justify-between">
          <span className="text-sm">Outage duration (hours)</span>
          <input
            type="number"
            min={1}
            max={24}
            value={outageHours}
            onChange={(e) => setOutageHours(e.target.value)}
            className="w-20 rounded-lg bg-[#0f1b34] border border-white/10 
px-3 py-2 text-center text-white 
focus:outline-none focus:ring-2 focus:ring-blue-500"

          />
        </div>
      </div>

      {/* INDICATOR POWER SAVING */}
      <div className="bg-card rounded-xl p-3 shadow-sm">
        <h3 className="font-medium mb-2">Indicator Power Saving</h3>
        <p className="text-sm text-muted-foreground mb-3">
          Reduce indicator power usage when idle.
        </p>

        <div className="flex items-center justify-between">
          <span className="text-sm">Power saving mode</span>

          <button
            onClick={() => setPowerSaving(!powerSaving)}
            className={`w-12 h-6 rounded-full transition-colors ${powerSaving ? "bg-green-500" : "bg-gray-400"
              }`}
          >
            <div
              className={`h-5 w-5 bg-white rounded-full transition-transform ${powerSaving ? "translate-x-6" : "translate-x-1"
                }`}
            />
          </button>
        </div>
      </div>

      {/* SCHEDULED MOTOR CONTROL */}
      <div className="bg-card rounded-xl p-3 shadow-sm">
        <h3 className="font-medium mb-2">Scheduled Motor Control</h3>
        <p className="text-sm text-muted-foreground mb-3">
          Run motor at a specific time if tank is full.
        </p>

        <div className="flex items-center justify-between mb-2">
          <span className="text-sm">Enable schedule</span>

          <button
            onClick={() => setScheduleEnabled(!scheduleEnabled)}
            className={`w-12 h-6 rounded-full transition-colors ${scheduleEnabled ? "bg-green-500" : "bg-gray-400"
              }`}
          >
            <div
              className={`h-5 w-5 bg-white rounded-full transition-transform ${scheduleEnabled ? "translate-x-6" : "translate-x-1"
                }`}
            />
          </button>
        </div>

        <div className="flex items-center justify-between opacity-90">
          <span className="text-sm">Time</span>
          <input
            type="time"
            disabled={!scheduleEnabled}
            value={scheduleTime}
            onChange={(e) => setScheduleTime(e.target.value)}
            className="rounded-lg bg-[#0f1b34] border border-white/10 
px-3 py-2 text-white 
focus:outline-none focus:ring-2 focus:ring-blue-500 
disabled:opacity-40"

          />
        </div>
      </div>



    </div>
  );
}
