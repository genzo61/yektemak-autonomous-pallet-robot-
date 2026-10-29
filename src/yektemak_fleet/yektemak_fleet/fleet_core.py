"""Filo yoneticisinin beyni (ROS bagimsiz, birim testli).

Sorumluluklar
-------------
* Gorev kuyrugu: olustur, ata, iptal et, bitir.
* Robot secimi: bostaki, bataryasi yeterli, palete en yakin robot.
* Palet takibi: hangi istasyonda hangi palet var, hangi istasyon bir goreve ayrildi.
* Bolge kilitleri: ayni anda bir bolgede (ornek depo sirasi) tek robot calisir.
  Robot bolgeye girmeden kilidi ister; kilit doluysa bekleme noktasinda bekler.
  Robot bolgeden uzaklasinca kilidi birakir, siradaki robota verilir (FIFO).
* Otomatik uretim modu: mal kabul -> depo -> uretim hatti -> sevkiyat akisini
  kendiliginden gorevlere cevirir; sevk edilen paletler yeni "tir" ile mal kabule doner.
* Batarya: dusuk bataryali robota gorev verilmez, sarja gonderilir.
"""

import itertools
import math
import time
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional

ZONE_OF_TYPE = {"inbound": "inbound", "storage": "storage", "production": "production",
                "outbound": "outbound"}

STEP_TEXT = {
    "queued": "Sirada", "assigned": "Atandi", "running": "Calisiyor",
    "done": "Tamamlandi", "failed": "Basarisiz", "cancelled": "Iptal edildi",
}


@dataclass
class Station:
    name: str
    x: float
    y: float
    yaw: float
    type: str
    pallet: Optional[str] = None      # istasyondaki palet
    reserved_by: Optional[str] = None  # bu istasyonu kullanacak gorev id
    since: float = 0.0                 # palet bu istasyona ne zaman geldi

    @property
    def zone(self):
        return ZONE_OF_TYPE.get(self.type)


@dataclass
class Task:
    id: str
    kind: str                 # "transport" | "goto"
    pick: Optional[str] = None
    drop: Optional[str] = None
    target: Optional[str] = None     # goto icin istasyon adi
    robot: Optional[str] = None      # atanan robot
    requested_robot: Optional[str] = None
    state: str = "queued"
    step: str = ""
    created: float = 0.0
    started: float = 0.0
    finished: float = 0.0
    message: str = ""
    source: str = "operator"  # operator | auto | system


@dataclass
class Robot:
    name: str
    home: str
    state: str = "offline"    # offline | idle | busy | charging | estop | error
    step: str = ""
    task_id: Optional[str] = None
    battery: float = 100.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    loaded: bool = False
    pallet: Optional[str] = None
    last_seen: float = 0.0
    estop: bool = False
    tasks_done: int = 0


@dataclass
class FleetConfig:
    battery_low: float = 30.0
    battery_ok: float = 80.0
    auto_process_time: float = 45.0   # uretim hattinda palet isleme suresi (s)
    auto_ship_time: float = 30.0      # sevkiyatta tirin gelip almasi (s)
    robot_timeout: float = 6.0


class FleetCore:

    def __init__(self, stations: Dict[str, dict], robots: Dict[str, str],
                 initial_pallets: Dict[str, str], wait_points: Dict[str, list],
                 config: FleetConfig = FleetConfig(), clock=time.time):
        self.clock = clock
        self.cfg = config
        self.stations = {n: Station(n, s["x"], s["y"], s["yaw"], s["type"]) for n, s in stations.items()}
        for st, pallet in initial_pallets.items():
            self.stations[st].pallet = pallet
            self.stations[st].since = clock()
        self.robots = {n: Robot(n, home) for n, home in robots.items()}
        self.wait_points = wait_points
        self.tasks: Dict[str, Task] = {}
        self.order: List[str] = []
        self._ids = itertools.count(1)
        self.locks: Dict[str, Optional[str]] = {z: None for z in set(ZONE_OF_TYPE.values())}
        self.lock_queue: Dict[str, List[str]] = {z: [] for z in self.locks}
        self.auto = False
        self.stats = {"done": 0, "failed": 0, "shipped": 0, "delivered": 0}
        self.events: List[dict] = []
        self.outbox: List[tuple] = []   # (robot, message dict) agent'a gidecekler
        self.sim_actions: List[dict] = []  # Gazebo'ya yapilacaklar (palet tasima vb.)

    # ------------------------------------------------------------------ olaylar
    def log(self, text, level="info", robot=None):
        self.events.append({"t": round(self.clock(), 1), "text": text, "level": level, "robot": robot})
        self.events = self.events[-80:]

    # ------------------------------------------------------------------ gorevler
    def create_transport(self, pick, drop, robot=None, source="operator"):
        err = self.validate_transport(pick, drop, robot)
        if err:
            self.log(f"Gorev reddedildi ({pick} -> {drop}): {err}", "warn")
            return None, err
        t = Task(f"T{next(self._ids)}", "transport", pick=pick, drop=drop,
                 requested_robot=robot or None, created=self.clock(), source=source)
        self.stations[pick].reserved_by = t.id
        self.stations[drop].reserved_by = t.id
        self.tasks[t.id] = t
        self.order.append(t.id)
        self.log(f"{t.id} olusturuldu: {pick} -> {drop}" + (f" ({robot})" if robot else ""))
        return t, None

    def create_goto(self, robot, target, source="operator"):
        if robot not in self.robots or target not in self.stations:
            return None, "bilinmeyen robot/istasyon"
        t = Task(f"T{next(self._ids)}", "goto", target=target, requested_robot=robot,
                 created=self.clock(), source=source)
        self.tasks[t.id] = t
        self.order.append(t.id)
        self.log(f"{t.id}: {robot} -> {target}")
        return t, None

    def validate_transport(self, pick, drop, robot):
        if pick not in self.stations or drop not in self.stations:
            return "bilinmeyen istasyon"
        if pick == drop:
            return "alma ve birakma ayni"
        sp, sd = self.stations[pick], self.stations[drop]
        if sp.type == "charge" or sd.type == "charge":
            return "sarj istasyonu palet istasyonu degil"
        if not sp.pallet:
            return f"{pick} istasyonunda palet yok"
        if sd.pallet:
            return f"{drop} istasyonu dolu"
        if sp.reserved_by or sd.reserved_by:
            return "istasyon baska bir goreve ayrilmis"
        if robot and robot not in self.robots:
            return "bilinmeyen robot"
        return None

    def cancel(self, task_id):
        t = self.tasks.get(task_id)
        if not t or t.state not in ("queued",):
            return "sadece siradaki (baslamamis) gorevler iptal edilebilir"
        self._close(t, "cancelled", "Operator iptal etti")
        return None

    def _close(self, t, state, msg=""):
        t.state, t.message, t.finished = state, msg, self.clock()
        for name in (t.pick, t.drop):
            if name and self.stations[name].reserved_by == t.id:
                self.stations[name].reserved_by = None
        if t.robot and self.robots[t.robot].task_id == t.id:
            self.robots[t.robot].task_id = None

    # ------------------------------------------------------------------ atama
    def robot_available(self, r: Robot):
        return (r.state in ("idle", "charging") and not r.task_id and not r.estop
                and (r.battery >= self.cfg.battery_ok if r.state == "charging"
                     else r.battery >= self.cfg.battery_low))

    def dispatch(self):
        """Siradaki gorevleri uygun robotlara ata. Atanan (robot, gorev) listesi doner."""
        out = []
        for tid in list(self.order):
            t = self.tasks[tid]
            if t.state != "queued":
                continue
            if t.kind == "goto":
                r = self.robots[t.requested_robot]
                if r.state in ("idle", "charging") and not r.task_id and not r.estop:
                    out.append(self._assign(t, r))
                continue
            cands = [r for r in self.robots.values() if self.robot_available(r)
                     and (not t.requested_robot or r.name == t.requested_robot)]
            if not cands:
                continue
            sp = self.stations[t.pick]
            r = min(cands, key=lambda r: math.hypot(r.x - sp.x, r.y - sp.y))
            out.append(self._assign(t, r))
        return out

    def _assign(self, t, r):
        t.robot, t.state, t.started = r.name, "assigned", self.clock()
        r.task_id = t.id
        msg = {"id": t.id, "kind": t.kind}
        if t.kind == "transport":
            for key in ("pick", "drop"):
                s = self.stations[getattr(t, key)]
                msg[key] = s.name
                msg[f"{key}_pose"] = [s.x, s.y, s.yaw]
                msg[f"{key}_zone"] = s.zone
                msg[f"{key}_wait"] = self.wait_points.get(s.zone)
        else:
            s = self.stations[t.target]
            msg.update(target=s.name, pose=[s.x, s.y, s.yaw])
        self.outbox.append((r.name, msg))
        self.log(f"{t.id} -> {r.name} atandi", robot=r.name)
        return r.name, t.id

    # ------------------------------------------------------------------ ajan mesajlari
    def on_robot_status(self, s: dict):
        r = self.robots.get(s.get("robot"))
        if not r:
            return
        was_offline = r.state == "offline"
        for k in ("state", "step", "battery", "x", "y", "yaw", "loaded", "pallet", "estop"):
            if k in s:
                setattr(r, k, s[k])
        r.last_seen = self.clock()
        if was_offline:
            self.log(f"{r.name} baglandi", robot=r.name)
        t = self.tasks.get(r.task_id or "")
        if t and t.state in ("assigned", "running"):
            t.state, t.step = "running", s.get("step", t.step)

    def on_task_result(self, res: dict):
        t = self.tasks.get(res.get("id"))
        if not t or t.state in ("done", "failed", "cancelled"):
            return
        r = self.robots.get(t.robot)
        ok = bool(res.get("ok"))
        if t.kind == "transport":
            sp, sd = self.stations[t.pick], self.stations[t.drop]
            if ok:
                sd.pallet = res.get("pallet") or sp.pallet
                sd.since = self.clock()
                sp.pallet = None
                self.stats["done"] += 1
                if r:
                    r.tasks_done += 1
            elif res.get("picked"):
                # Palet alindi ama birakilamadi: palet robotta, istasyon bilgisini temizle
                sp.pallet = None
        if not ok and t.kind == "transport":
            self.stats["failed"] += 1
        self._close(t, "done" if ok else "failed", res.get("message", ""))
        self.log(f"{t.id} {'tamamlandi' if ok else 'BASARISIZ: ' + res.get('message', '')}",
                 "info" if ok else "error", robot=t.robot)

    # ------------------------------------------------------------------ bolge kilitleri
    def zone_request(self, robot, zone, op):
        if zone not in self.locks:
            return
        q = self.lock_queue[zone]
        if op == "acquire":
            if self.locks[zone] in (None, robot):
                self.locks[zone] = robot
                if robot in q:
                    q.remove(robot)
            elif robot not in q:
                q.append(robot)
                self.log(f"{robot}: {zone} bolgesi dolu ({self.locks[zone]}), bekliyor", robot=robot)
        elif op == "release":
            if robot in q:
                q.remove(robot)
            if self.locks[zone] == robot:
                self.locks[zone] = q.pop(0) if q else None
                if self.locks[zone]:
                    self.log(f"{zone} bolgesi {self.locks[zone]} robotuna verildi", robot=self.locks[zone])

    # ------------------------------------------------------------------ periyodik
    def tick(self):
        now = self.clock()
        for r in self.robots.values():
            if r.state != "offline" and now - r.last_seen > self.cfg.robot_timeout:
                r.state = "offline"
                self.log(f"{r.name} ile baglanti koptu", "error", robot=r.name)
                for z, owner in self.locks.items():
                    if owner == r.name:
                        self.zone_request(r.name, z, "release")
        self._battery_policy()
        if self.auto:
            self._auto_production()
        return self.dispatch()

    def _battery_policy(self):
        for r in self.robots.values():
            if (r.state == "idle" and not r.task_id and r.battery < self.cfg.battery_low
                    and not any(t.state in ("queued", "assigned", "running") and t.kind == "goto"
                                and t.requested_robot == r.name for t in self.tasks.values())):
                self.log(f"{r.name} bataryasi dusuk (%{r.battery:.0f}), sarja gidiyor", "warn", robot=r.name)
                self.create_goto(r.name, r.home, source="system")

    def _open_tasks(self):
        return [t for t in self.tasks.values() if t.state in ("queued", "assigned", "running")]

    def _free(self, kind):
        return [s for s in self.stations.values()
                if s.type == kind and not s.pallet and not s.reserved_by]

    def _full(self, kind, min_age=0.0):
        now = self.clock()
        return [s for s in self.stations.values()
                if s.type == kind and s.pallet and not s.reserved_by and now - s.since >= min_age]

    def _auto_production(self):
        now = self.clock()
        # Sevkiyat: tir paleti alip goturur, yeni mal ile mal kabule geri getirir
        for s in self._full("outbound", self.cfg.auto_ship_time):
            free_in = self._free("inbound")
            if free_in:
                dst = free_in[0]
                self.sim_actions.append({"op": "move_pallet", "pallet": s.pallet,
                                         "x": dst.x, "y": dst.y, "yaw": dst.yaw})
                dst.pallet, dst.since = s.pallet, now
                self.log(f"Tir {s.name} istasyonundan {s.pallet} paletini sevk etti, "
                         f"{dst.name} istasyonuna yeni mal geldi")
                s.pallet = None
                self.stats["shipped"] += 1
        busy_robots = len(self._open_tasks())
        usable = sum(1 for r in self.robots.values() if r.state != "offline" and not r.estop)
        if busy_robots >= usable:
            return
        # Oncelik: uretimden sevkiyata > depodan uretime > mal kabulden depoya
        flows = [("production", "outbound", self.cfg.auto_process_time),
                 ("storage", "production", 0.0),
                 ("inbound", "storage", 0.0)]
        for src, dst, age in flows:
            full, free = self._full(src, age), self._free(dst)
            if full and free:
                self.create_transport(full[0].name, free[0].name, source="auto")
                return

    # ------------------------------------------------------------------ durum
    def snapshot(self):
        now = self.clock()
        recent = [self.tasks[i] for i in self.order[-40:]]
        return {
            "t": round(now, 1),
            "auto": self.auto,
            "stats": self.stats,
            "robots": [asdict(r) for r in self.robots.values()],
            "tasks": [dict(asdict(t), state_text=STEP_TEXT.get(t.state, t.state)) for t in recent],
            "stations": [dict(asdict(s), zone=s.zone,
                              age=round(now - s.since, 0) if s.pallet else None)
                         for s in self.stations.values()],
            "locks": self.locks,
            "lock_queue": self.lock_queue,
            "events": self.events[-40:],
        }
