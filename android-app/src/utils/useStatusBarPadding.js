import { useEffect, useState } from "react";
import { Capacitor } from "@capacitor/core";
import { StatusBar } from "@capacitor/status-bar";

export function useStatusBarPadding() {
  const [top, setTop] = useState(0);

  useEffect(() => {
    if (Capacitor.getPlatform() !== "android") return;

    StatusBar.getInfo().then(info => {
      if (info?.height) {
        setTop(info.height);
      }
    });
  }, []);

  return top;
}