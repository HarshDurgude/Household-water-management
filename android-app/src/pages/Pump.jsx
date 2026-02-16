import { Power } from "lucide-react";
import { useEffect, useState } from "react";
import { fetchStatus, pumpOn, pumpOff } from "../api/status";

export default function Pump() {
  const [status, setStatus] = useState(null);
  const [busy, setBusy] = useState(false);

  // 🔴 Fetch server truth ONCE when page opens
  useEffect(() => {
    fetchStatus()
      .then(setStatus)
      .catch(err => console.warn("Initial status fetch failed", err));
  }, []);

  async function togglePump() {
    if (!status) return;

    setBusy(true);

    try {
      if (status.pumpOn) {
        await pumpOff();
      } else {
        await pumpOn();
      }

      // 🔴 WAIT A BIT FOR SERVO TO FINISH
      await new Promise(r => setTimeout(r, 700));

      // 🔴 FETCH REAL TRUTH FROM SERVER
      const fresh = await fetchStatus();
      setStatus(fresh);

    } catch (e) {
      alert(e.message);
    } finally {
      setBusy(false);
    }
  }

  const pumpIsOn = status?.pumpOn ?? false;

  return (
    <div className="h-full flex items-center justify-center px-4">
      <div className="flex flex-col items-center gap-6">

        <div
          className={`w-36 h-36 rounded-full flex items-center justify-center shadow-inner
          ${pumpIsOn ? "bg-green-500/20" : "bg-red-500/20"}`}
        >
          <Power
            className={`w-16 h-16 ${pumpIsOn ? "text-green-500" : "text-red-500"}`}
          />
        </div>

        <button
          disabled={busy || !status}
          onClick={togglePump}
          className={`px-12 py-4 rounded-full text-white font-semibold text-lg
          ${busy
            ? "bg-gray-500"
            : pumpIsOn
              ? "bg-red-500"
              : "bg-green-500"}`}
        >
          {busy
            ? "Please wait..."
            : pumpIsOn
              ? "Turn OFF Pump"
              : "Turn ON Pump"}
        </button>

        <p className="text-sm text-white/60">
          Manual override
        </p>
      </div>
    </div>
  );
}
