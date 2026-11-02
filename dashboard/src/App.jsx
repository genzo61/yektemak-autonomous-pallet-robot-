import React, { useMemo, useState } from "react";
import FactoryMap from "./FactoryMap.jsx";
import { useFleet } from "./ros.js";
import { ROBOT_LABEL, STATE_TEXT, TASK_STATE, ZONE_NAME, robotColor } from "./labels.js";

function Battery({ value }) {
  const v = Math.max(0, Math.min(100, value ?? 0));
  const cls = v < 30 ? "low" : v < 60 ? "mid" : "ok";
  return (
    <div className={`battery ${cls}`} aria-label={`Batarya yüzde ${v.toFixed(0)}`}>
      <div className="battery-fill" style={{ width: `${v}%` }} />
      <span>%{v.toFixed(0)}</span>
    </div>
  );
}

function RobotCard({ r, i, task, send, hold }) {
  const color = robotColor(r.name, i);
  return (
    <section className={`robot-card state-${r.state}`} style={{ "--rc": color }}>
      <header>
        <h3>{ROBOT_LABEL(r.name)}</h3>
        <span className={`badge${hold ? " hold" : ""}`}>
          {hold ? `Yol veriyor (${ROBOT_LABEL(hold.by)})` : STATE_TEXT[r.state] || r.state}
        </span>
      </header>
      <p className="step">{r.state === "offline" ? "Robottan haber alınamıyor" : r.step || "—"}</p>
      <Battery value={r.battery} />
      <dl>
        <div><dt>Görev</dt><dd>{task ? `${task.id} · ${task.kind === "transport" ? `${task.pick} → ${task.drop}` : task.target}` : "yok"}</dd></div>
        <div><dt>Yük</dt><dd>{r.loaded ? r.pallet || "palet" : "boş"}</dd></div>
        <div><dt>Tamamlanan</dt><dd>{r.tasks_done}</dd></div>
      </dl>
      <div className="row">
        <button className={r.estop ? "btn release" : "btn stop"}
                onClick={() => send({ cmd: "estop", robot: r.name, on: !r.estop })}>
          {r.estop ? "Durdurmayı kaldır" : "Durdur"}
        </button>
        <button className="btn ghost" disabled={r.state !== "idle"}
                onClick={() => send({ cmd: "goto", robot: r.name, target: r.home })}>
          Şarja gönder
        </button>
      </div>
    </section>
  );
}

function TaskComposer({ state, selection, setSelection, send }) {
  const stations = state?.stations || [];
  const pickable = stations.filter((s) => s.pallet && !s.reserved_by && s.type !== "charge");
  const droppable = stations.filter((s) => !s.pallet && !s.reserved_by && s.type !== "charge");
  const robots = state?.robots || [];
  const opt = (s) => <option key={s.name} value={s.name}>{s.name} · {ZONE_NAME[s.type]}</option>;
  const ready = selection.pick && selection.drop;
  return (
    <section className="composer">
      <h2>Yeni taşıma görevi</h2>
      <p className="hint">Haritada önce paletli bir istasyona, sonra boş bir istasyona tıklayabilirsin.</p>
      <label>Alınacak palet
        <select value={selection.pick || ""} onChange={(e) => setSelection({ ...selection, pick: e.target.value })}>
          <option value="">Seç</option>{pickable.map(opt)}
        </select>
      </label>
      <label>Bırakılacak yer
        <select value={selection.drop || ""} onChange={(e) => setSelection({ ...selection, drop: e.target.value })}>
          <option value="">Seç</option>{droppable.map(opt)}
        </select>
      </label>
      <label>Robot
        <select value={selection.robot || "auto"} onChange={(e) => setSelection({ ...selection, robot: e.target.value })}>
          <option value="auto">Otomatik seç (en yakın boş robot)</option>
          {robots.map((r) => <option key={r.name} value={r.name}>{ROBOT_LABEL(r.name)}</option>)}
        </select>
      </label>
      <button className="btn primary" disabled={!ready}
              onClick={() => { send({ cmd: "create_task", ...selection }); setSelection({ robot: selection.robot }); }}>
        Görevi oluştur
      </button>
    </section>
  );
}

function TaskList({ tasks, send }) {
  const list = [...tasks].reverse();
  if (!list.length) return <p className="empty">Henüz görev yok. Yukarıdan bir taşıma görevi oluştur ya da otomatik üretimi aç.</p>;
  return (
    <ol className="tasks">
      {list.map((t) => (
        <li key={t.id} className={`task t-${t.state}`}>
          <div className="task-main">
            <strong>{t.id}</strong>
            <span>{t.kind === "transport" ? `${t.pick} → ${t.drop}` : `${t.target}`}</span>
            <span className="who">{t.robot ? ROBOT_LABEL(t.robot) : "—"}</span>
          </div>
          <div className="task-sub">
            <span className="tstate">{TASK_STATE[t.state] || t.state}</span>
            {t.source === "auto" && <span className="src">otomatik</span>}
            {t.source === "system" && <span className="src">sistem</span>}
            {t.message && t.state === "failed" && <span className="err">{t.message}</span>}
            {t.state === "queued" && (
              <button className="link" onClick={() => send({ cmd: "cancel_task", id: t.id })}>İptal et</button>
            )}
          </div>
        </li>
      ))}
    </ol>
  );
}

export default function App() {
  const { status, state, plans, send } = useFleet();
  const [selection, setSelection] = useState({ robot: "auto" });
  const tasks = state?.tasks || [];
  const byId = useMemo(() => Object.fromEntries(tasks.map((t) => [t.id, t])), [tasks]);
  const anyStop = (state?.robots || []).some((r) => r.estop);

  const onStation = (s) => {
    if (s.reserved_by) return;
    if (s.pallet) setSelection({ ...selection, pick: s.name });
    else setSelection({ ...selection, drop: s.name });
  };

  return (
    <div className="app">
      <header className="topbar">
        <div className="brand">
          <span className="mark" aria-hidden="true" />
          <h1>Yektemak filo kontrol</h1>
        </div>
        <div className={`conn conn-${status}`}>
          {status === "bagli" ? (state ? "Filo bağlı" : "Filo yöneticisi bekleniyor")
            : status === "baglaniyor" ? "Bağlanıyor" : "Bağlantı yok, yeniden deneniyor"}
        </div>
        <div className="stats">
          <div><b>{state?.stats.done ?? 0}</b><span>taşıma</span></div>
          <div><b>{state?.stats.shipped ?? 0}</b><span>sevk</span></div>
          <div><b>{tasks.filter((t) => ["queued", "assigned", "running"].includes(t.state)).length}</b><span>açık görev</span></div>
        </div>
        <label className="auto">
          <input type="checkbox" checked={!!state?.auto} disabled={!state}
                 onChange={(e) => send({ cmd: "auto", on: e.target.checked })} />
          <span>Otomatik üretim</span>
        </label>
        <button className={`estop ${anyStop ? "active" : ""}`} disabled={!state}
                onClick={() => send({ cmd: "estop", robot: "all", on: !anyStop })}>
          {anyStop ? "Tüm robotları serbest bırak" : "ACİL DURDUR"}
        </button>
      </header>

      <main className="layout">
        <div className="map-col">
          <div className="map-wrap">
            <FactoryMap state={state} plans={plans} selection={selection} onStation={onStation} />
            <ul className="legend">
              <li><i className="lg pallet" />Palet</li>
              <li><i className="lg reserved" />Göreve ayrılmış</li>
              <li><i className="lg pick" />Seçili alma</li>
              <li><i className="lg drop" />Seçili bırakma</li>
            </ul>
          </div>
          <section className="events">
            <h2>Olay akışı</h2>
            <ul>
              {[...(state?.events || [])].reverse().slice(0, 14).map((e, i) => (
                <li key={i} className={`ev ev-${e.level}`}>
                  {e.robot && <i style={{ background: robotColor(e.robot) }} />}{e.text}
                </li>
              ))}
            </ul>
          </section>
        </div>

        <aside className="rail">
          <div className="robots">
            {(state?.robots || []).map((r, i) => <RobotCard key={r.name} r={r} i={i} task={byId[r.task_id]} send={send} hold={state.holds?.[r.name]} />)}
            {!state && <p className="empty">Filo yöneticisine bağlanılamadı. Simülasyon ve <code>fleet.launch.py</code> çalışıyor mu?</p>}
          </div>
          <TaskComposer state={state} selection={selection} setSelection={setSelection} send={send} />
          <section className="task-list">
            <h2>Görevler</h2>
            <TaskList tasks={tasks} send={send} />
          </section>
        </aside>
      </main>
    </div>
  );
}
