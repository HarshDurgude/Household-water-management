const BASE = "http://192.168.4.10";

export async function fetchStatus() {
  const res = await fetch(`${BASE}/status`, { cache: "no-store" });
  if (!res.ok) throw new Error("Failed to fetch status");
  return res.json();
}

export async function pumpOn() {
  const res = await fetch(`${BASE}/pump/on`, { method: "POST" });
  if (!res.ok) throw new Error("Pump ON failed");
}

export async function pumpOff() {
  const res = await fetch(`${BASE}/pump/off`, { method: "POST" });
  if (!res.ok) throw new Error("Pump OFF failed");
}
