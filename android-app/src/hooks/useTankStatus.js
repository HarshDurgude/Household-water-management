import { useEffect, useState } from "react";
import { fetchStatus } from "../api/status";

export function useTankStatus(enabled = true, intervalMs = 1500) {
  const [data, setData] = useState(null);

  useEffect(() => {
    if (!enabled) return;

    let alive = true;

    async function poll() {
      try {
        const json = await fetchStatus();
        if (alive) setData(json);
      } catch (e) {
        // ❌ DO NOT show popup errors for background polling
        console.warn("Tank status fetch failed:", e.message);
      }
    }

    poll();
    const id = setInterval(poll, intervalMs);

    return () => {
      alive = false;
      clearInterval(id);
    };
  }, [enabled, intervalMs]);

  return { data };
}
