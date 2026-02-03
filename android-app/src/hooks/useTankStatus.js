import { useEffect, useState } from "react";
import { fetchStatus } from "../api/status";

export function useTankStatus(intervalMs = 1500) {
  const [data, setData] = useState(null);
  const [error, setError] = useState(null);

  useEffect(() => {
    let alive = true;

    async function poll() {
      try {
        const json = await fetchStatus();
        if (alive) setData(json);
      } catch (e) {
        if (alive) setError(e.message);
      }
    }

    poll();
    const id = setInterval(poll, intervalMs);

    return () => {
      alive = false;
      clearInterval(id);
    };
  }, [intervalMs]);

  return { data, error };
}