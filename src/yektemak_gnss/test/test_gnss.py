import math

from yektemak_gnss.rtk_model import RtkConfig, RtkModel
from yektemak_gnss.utm import LocalFrame, forward, inverse


def test_utm_round_trip_mm():
    for lat, lon in [(40.965, 27.0), (41.01, 28.97), (38.4, 26.1)]:
        e, n, z = forward(lat, lon)
        la, lo = inverse(e, n, z)
        assert abs(la - lat) < 1e-8 and abs(lo - lon) < 1e-8


def test_local_frame_distances():
    f = LocalFrame(40.965, 27.0)
    lat, lon = f.to_geo(60.0, -20.0)
    x, y = f.to_local(lat, lon)
    assert abs(x - 60.0) < 1e-3 and abs(y + 20.0) < 1e-3
    # 1 derece enlem ~ 111 km: yaklasik kontrol
    lat2, _ = f.to_geo(0.0, 1000.0)
    assert abs((lat2 - 40.965) * 111_000 - 1000) < 15


ZONES = [{"name": "sokak", "rect": [10, -5, 30, 5], "cap": "FLOAT"}]
QUIET = RtkConfig(link_drop_interval=1e12, outage_interval=1e12)


def run(m, t0, t1, x, y, dt=0.2):
    t, st = t0, None
    while t <= t1:
        st = m.step(t, x, y)
        t += dt
    return st


def test_fix_in_open_sky_accuracy():
    m = RtkModel(ZONES, QUIET, seed=1)
    errs = []
    for k in range(200):
        st = m.step(k * 0.2, 0.0, 0.0)
        errs.append(math.hypot(st.x, st.y))
    assert st.mode == "FIX" and sum(errs) / len(errs) < 0.04


def test_zone_caps_to_float():
    m = RtkModel(ZONES, QUIET, seed=2)
    assert run(m, 0, 5, 20.0, 0.0).mode == "FLOAT"
    assert run(m, 5, 10, 0.0, 0.0).mode == "FIX"


def test_link_loss_float_then_single_then_reconverge():
    m = RtkModel(ZONES, QUIET, seed=3)
    run(m, 0, 2, 0, 0)
    m.drop_link(40.0)
    assert run(m, 2, 12, 0, 0).mode == "FLOAT"     # ~10 s yas
    assert run(m, 12, 40, 0, 0).mode == "SINGLE"   # >30 s yas
    assert run(m, 40, 43, 0, 0).mode == "FLOAT"    # baglanti dondu, yakinsama
    assert run(m, 43, 47, 0, 0).mode == "FIX"
