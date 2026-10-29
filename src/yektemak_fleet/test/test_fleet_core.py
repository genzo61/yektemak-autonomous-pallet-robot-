"""Filo cekirdegi birim testleri (ROS gerektirmez)."""

import os

import yaml

from yektemak_fleet.fleet_core import FleetConfig, FleetCore

HERE = os.path.dirname(os.path.abspath(__file__))
ST = yaml.safe_load(open(os.path.join(HERE, "..", "config", "stations.yaml")))


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def make(clock=None):
    clock = clock or Clock()
    core = FleetCore(ST["stations"], {"robot1": "CH_1", "robot2": "CH_2"},
                     ST["initial_pallets"], {"storage": [19, -2.5, 0]}, FleetConfig(), clock)
    for name, (x, y) in {"robot1": (0.5, 1.5), "robot2": (0.5, -1.5)}.items():
        core.on_robot_status({"robot": name, "state": "idle", "x": x, "y": y, "battery": 100})
    return core, clock


def test_create_and_dispatch_nearest_robot():
    core, _ = make()
    t, err = core.create_transport("IN_1", "ST_3")
    assert err is None
    assigned = core.dispatch()
    assert assigned == [("robot2", t.id)]          # robot2 IN_1'e daha yakin
    robot, msg = core.outbox[-1]
    assert msg["pick"] == "IN_1" and msg["drop_zone"] == "storage"


def test_reject_invalid_tasks():
    core, _ = make()
    assert core.create_transport("ST_3", "ST_4")[1]      # ST_3 bos
    assert core.create_transport("IN_1", "ST_1")[1]      # ST_1 dolu
    core.create_transport("IN_1", "ST_3")
    assert core.create_transport("IN_1", "ST_4")[1]      # IN_1 zaten ayrildi


def test_requested_robot_is_respected():
    core, _ = make()
    t, _ = core.create_transport("IN_1", "ST_3", robot="robot1")
    assert core.dispatch() == [("robot1", t.id)]


def test_task_result_moves_pallet():
    core, _ = make()
    t, _ = core.create_transport("IN_1", "ST_3")
    core.dispatch()
    core.on_task_result({"id": t.id, "ok": True, "pallet": "pallet_1"})
    st = {s.name: s for s in core.stations.values()}
    assert st["IN_1"].pallet is None and st["ST_3"].pallet == "pallet_1"
    assert st["ST_3"].reserved_by is None and core.stats["done"] == 1


def test_zone_lock_fifo():
    core, _ = make()
    core.zone_request("robot1", "storage", "acquire")
    core.zone_request("robot2", "storage", "acquire")
    assert core.locks["storage"] == "robot1" and core.lock_queue["storage"] == ["robot2"]
    core.zone_request("robot1", "storage", "release")
    assert core.locks["storage"] == "robot2"


def test_low_battery_goes_home_and_gets_no_tasks():
    core, _ = make()
    core.on_robot_status({"robot": "robot2", "state": "idle", "battery": 20})
    core.tick()
    goto = [t for t in core.tasks.values() if t.kind == "goto"]
    assert goto and goto[0].requested_robot == "robot2" and goto[0].target == "CH_2"
    t, _ = core.create_transport("IN_1", "ST_3")
    core.dispatch()
    assert t.robot == "robot1"


def test_offline_robot_releases_locks():
    core, clock = make()
    core.zone_request("robot1", "storage", "acquire")
    clock.t += 30
    core.on_robot_status({"robot": "robot2", "state": "idle"})
    core.tick()
    assert core.robots["robot1"].state == "offline" and core.locks["storage"] is None


def test_auto_production_cycle():
    core, clock = make()
    core.auto = True
    core.tick()
    first = [t for t in core.tasks.values() if t.source == "auto"]
    assert first and first[0].pick.startswith("ST") and first[0].drop.startswith("PL")
    # Uretimdeki paletler islem suresi dolunca sevkiyata gider
    pl = core.stations["PL_2"]
    pl.pallet, pl.since = "pallet_9", clock.t
    for t in list(core.tasks.values()):
        core._close(t, "done")
    clock.t += core.cfg.auto_process_time + 1
    for r in core.robots.values():
        r.last_seen = clock.t          # robotlar hala bagli
    core.tick()
    assert any(t.pick == "PL_2" and t.drop.startswith("OUT") for t in core.tasks.values())


def test_shipping_returns_pallet_to_inbound():
    core, clock = make()
    core.auto = True
    for n in ("IN_1", "IN_2", "IN_3"):
        core.stations[n].pallet = None
    out = core.stations["OUT_1"]
    out.pallet, out.since = "pallet_7", clock.t
    clock.t += core.cfg.auto_ship_time + 1
    for r in core.robots.values():
        r.last_seen = clock.t
    core.tick()
    assert core.stats["shipped"] == 1 and out.pallet is None
    assert core.sim_actions and core.sim_actions[0]["pallet"] == "pallet_7"


def _busy(core, name, x, y, loaded=False, task=None):
    core.on_robot_status({"robot": name, "state": "busy", "x": x, "y": y, "loaded": loaded})
    core.robots[name].task_id = task


def test_traffic_loaded_robot_has_priority():
    core, _ = make()
    _busy(core, "robot1", 10.0, 0.0, loaded=False)
    _busy(core, "robot2", 11.5, 0.0, loaded=True)
    core.tick()
    assert "robot1" in core.holds and core.holds["robot1"]["by"] == "robot2"
    assert ("robot1", True) in core.hold_changes


def test_traffic_older_task_wins_and_release_on_distance():
    core, clock = make()
    t1, _ = core.create_transport("IN_1", "ST_3")
    clock.t += 5
    t2, _ = core.create_transport("IN_2", "ST_4")
    _busy(core, "robot1", 10.0, 0.0, task=t2.id)
    _busy(core, "robot2", 11.0, 0.0, task=t1.id)
    core.dispatch = lambda: []          # atamayi bu testte devre disi birak
    core.tick()
    assert core.holds.get("robot1", {}).get("by") == "robot2"
    _busy(core, "robot2", 15.0, 0.0, task=t1.id)
    core.tick()
    assert "robot1" not in core.holds and ("robot1", False) in core.hold_changes


def test_traffic_hold_timeout_and_cooldown():
    core, clock = make()
    _busy(core, "robot1", 10.0, 0.0)
    _busy(core, "robot2", 11.0, 0.0, loaded=True)
    core.tick()
    assert "robot1" in core.holds
    clock.t += core.cfg.traffic_max_hold + 1
    for r in core.robots.values():
        r.last_seen = clock.t
    core.tick()
    assert "robot1" not in core.holds     # zaman asimi, serbest
    core.tick()
    assert "robot1" not in core.holds     # bekleme suresi: tekrar bekletilmez
