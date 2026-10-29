import React, { useEffect, useState } from "react";
import { ROBOT_LABEL, ZONE_NAME, robotColor } from "./labels.js";

// Bolge cerceveleri (dunyadaki renkli zeminlerle ayni, metre cinsinden: cx, cy, sx, sy)
const ZONES = [
  { key: "inbound", cx: 7.5, cy: -8.6, sx: 11, sy: 6.4 },
  { key: "storage", cx: 21.5, cy: -8.6, sx: 13, sy: 6.4 },
  { key: "production", cx: 20.5, cy: 6.0, sx: 15, sy: 3.8 },
  { key: "outbound", cx: 31.3, cy: 0, sx: 5, sy: 9 },
  { key: "charge", cx: 0.6, cy: 0, sx: 2.6, sy: 5.4 },
];

export default function FactoryMap({ state, plans, selection, onStation }) {
  const [meta, setMeta] = useState(null);
  useEffect(() => { fetch("map.json").then((r) => r.json()).then(setMeta).catch(() => {}); }, []);
  if (!meta) return <div className="map-empty">Harita yükleniyor</div>;

  const [ox, oy] = meta.origin;
  const w = meta.width * meta.resolution;
  const h = meta.height * meta.resolution;
  const top = oy + h;
  const Y = (y) => top - y;               // SVG ekseni asagi bakar, harita yukari
  const deg = (yaw) => (-yaw * 180) / Math.PI;
  const locks = state?.locks || {};
  const robots = state?.robots || [];

  return (
    <svg className="map" viewBox={`${ox} 0 ${w} ${h}`} role="img" aria-label="Fabrika haritası">
      <rect x={ox} y={0} width={w} height={h} className="map-floor" />
      {ZONES.map((z) => {
        const owner = locks[z.key];
        return (
          <g key={z.key} className={`zone zone-${z.key}${owner ? " locked" : ""}`}>
            <rect x={z.cx - z.sx / 2} y={Y(z.cy + z.sy / 2)} width={z.sx} height={z.sy} rx={0.25} />
            {z.key !== "charge" && <text x={z.cx - z.sx / 2 + 0.3} y={Y(z.cy + z.sy / 2) + 0.75} className="zone-label">
              {ZONE_NAME[z.key]}{owner ? `  ·  ${ROBOT_LABEL(owner)} çalışıyor` : ""}
            </text>}
          </g>
        );
      })}
      <image href="map.png" x={ox} y={0} width={w} height={h} preserveAspectRatio="none" />

      {(state?.stations || []).map((s) => {
        const sel = selection.pick === s.name ? "pick" : selection.drop === s.name ? "drop" : "";
        const clickable = s.type !== "charge";
        return (
          <g key={s.name} transform={`translate(${s.x} ${Y(s.y)}) rotate(${deg(s.yaw)})`}
             className={`station st-${s.type} ${s.pallet ? "full" : "empty"} ${s.reserved_by ? "reserved" : ""} ${sel}`}
             onClick={() => clickable && onStation(s)} style={{ cursor: clickable ? "pointer" : "default" }}>
            <title>{`${s.name}${s.pallet ? ` · ${s.pallet}` : " · boş"}${s.reserved_by ? ` · ${s.reserved_by}` : ""}`}</title>
            <rect x={-0.72} y={-0.52} width={1.44} height={1.04} className="st-frame" />
            {s.pallet && <rect x={-0.6} y={-0.4} width={1.2} height={0.8} className="st-pallet" />}
          </g>
        );
      })}
      {(state?.stations || []).map((s) => (
        <text key={s.name + "l"} x={s.x} y={Y(s.y) + (s.type === "charge" ? 1.05 : 1.25)} className="st-label">{s.name}</text>
      ))}

      {robots.map((r, i) => {
        const p = plans[r.name];
        if (!p || r.state === "idle" || r.state === "charging" || Date.now() - p.t > 30000) return null;
        return <polyline key={r.name + "p"} className="plan" stroke={robotColor(r.name, i)}
                         points={p.pts.map(([x, y]) => `${x},${Y(y)}`).join(" ")} />;
      })}

      {robots.map((r, i) => (
        <g key={r.name} className={`robot ${r.state}`} transform={`translate(${r.x} ${Y(r.y)}) rotate(${deg(r.yaw)})`}>
          {state?.holds?.[r.name] && <circle r={0.95} className="robot-hold" />}
          {r.loaded && <rect x={-0.62} y={-0.42} width={1.24} height={0.84} className="robot-load" />}
          <rect x={-0.42} y={-0.25} width={0.94} height={0.5} rx={0.08} fill={robotColor(r.name, i)} className="robot-body" />
          <path d="M 0.62 0 L 0.3 -0.2 L 0.3 0.2 Z" className="robot-nose" />
        </g>
      ))}
      {robots.map((r) => (
        <text key={r.name + "t"} x={r.x} y={Y(r.y) - 0.85} className="robot-tag">{ROBOT_LABEL(r.name)}</text>
      ))}
    </svg>
  );
}
