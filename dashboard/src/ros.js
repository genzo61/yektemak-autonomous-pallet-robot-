// rosbridge baglantisi. Panel, robotlarla filo yoneticisi uzerinden konusur:
//   /fleet/state   (abone)  -> butun filo durumu, saniyede 2 kez
//   /fleet/command (yayin)  -> gorev olustur, iptal, acil durdurma, otomatik mod
//   /<robot>/plan  (abone)  -> Nav2 rotasi (haritada cizilir)
import ROSLIB from "roslib";
import { useEffect, useRef, useState } from "react";

const defaultUrl = () => {
  const q = new URLSearchParams(window.location.search).get("ros");
  return q || `ws://${window.location.hostname || "localhost"}:9090`;
};

export function useFleet() {
  const [status, setStatus] = useState("baglaniyor");
  const [state, setState] = useState(null);
  const [plans, setPlans] = useState({});
  const rosRef = useRef(null);
  const cmdRef = useRef(null);
  const planSubs = useRef({});

  useEffect(() => {
    let closed = false;
    let retry;
    const connect = () => {
      const ros = new ROSLIB.Ros({ url: defaultUrl() });
      rosRef.current = ros;
      ros.on("connection", () => setStatus("bagli"));
      ros.on("error", () => setStatus("hata"));
      ros.on("close", () => {
        setStatus("kopuk");
        planSubs.current = {};
        if (!closed) retry = setTimeout(connect, 2000);
      });
      cmdRef.current = new ROSLIB.Topic({ ros, name: "/fleet/command", messageType: "std_msgs/String" });
      const st = new ROSLIB.Topic({ ros, name: "/fleet/state", messageType: "std_msgs/String", throttle_rate: 400 });
      st.subscribe((m) => {
        try {
          const s = JSON.parse(m.data);
          setState(s);
          for (const r of s.robots) {
            if (!planSubs.current[r.name]) {
              const t = new ROSLIB.Topic({ ros, name: `/${r.name}/plan`, messageType: "nav_msgs/Path", throttle_rate: 1000 });
              t.subscribe((p) => {
                const pts = p.poses.filter((_, i) => i % 4 === 0).map((ps) => [ps.pose.position.x, ps.pose.position.y]);
                setPlans((old) => ({ ...old, [r.name]: { pts, t: Date.now() } }));
              });
              planSubs.current[r.name] = t;
            }
          }
        } catch (e) { /* bozuk mesaj */ }
      });
    };
    connect();
    return () => { closed = true; clearTimeout(retry); rosRef.current && rosRef.current.close(); };
  }, []);

  const send = (cmd) => {
    if (cmdRef.current) cmdRef.current.publish({ data: JSON.stringify(cmd) });
  };
  return { status, state, plans, send };
}
