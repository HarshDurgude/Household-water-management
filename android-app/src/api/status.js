export async function fetchStatus() {
  const res = await fetch("http://192.168.4.1/status", {
    cache: "no-store",
  });

  if (!res.ok) {
    throw new Error("Failed to fetch status");
  }

  return res.json();
}