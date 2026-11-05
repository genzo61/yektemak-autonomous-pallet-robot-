"""RTK-GNSS alici modeli (ROS bagimsiz, birim testli).

Gercek bir RTK alicisi gibi davranir:
  FIX     baz istasyonundan taze duzeltme + iyi uydu gorusu      sigma ~ 1.5 cm
  FLOAT   duzeltme yaslandi, kismen kapali gok (konteyner sokagi,
          kren alti) ya da yeniden yakinsama sirasinda           sigma ~ 25 cm + yavas kayma
  SINGLE  duzeltme yok (>30 s) ya da cok kapali alan             sigma ~ 1.8 m + kayma
  NONE    alici sinyal kaybetti (nadir, kisa)                    konum yok

Olaylar (Poisson): duzeltme baglantisi kopmasi (sik, 8-25 s) ve alici kesintisi
(nadir, 2-6 s). Baglanti donunce alici once birkac saniye FLOAT kalir (yakinsama).
"""

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional

ORDER = {"NONE": 0, "SINGLE": 1, "FLOAT": 2, "FIX": 3}
SIGMA = {"FIX": 0.015, "FLOAT": 0.25, "SINGLE": 1.8}
BIAS = {"FIX": 0.0, "FLOAT": 0.20, "SINGLE": 1.5}
SATS = {"FIX": (20, 26), "FLOAT": (11, 16), "SINGLE": (6, 9), "NONE": (0, 3)}


@dataclass
class RtkConfig:
    link_drop_interval: float = 200.0   # ortalama duzeltme kopmasi araligi (s)
    link_drop_min: float = 8.0
    link_drop_max: float = 25.0
    outage_interval: float = 900.0      # ortalama alici kesintisi araligi (s)
    outage_min: float = 2.0
    outage_max: float = 6.0
    fix_age: float = 5.0                # bu yasa kadar duzeltme FIX'e yeter (s)
    float_age: float = 30.0             # bunun ustu SINGLE (s)
    convergence: float = 4.0            # baglanti donunce FLOAT'ta kalma suresi (s)


@dataclass
class RtkState:
    mode: str = "FIX"
    sigma: float = 0.015
    age: float = 0.0
    sats: int = 22
    zone: Optional[str] = None
    x: float = 0.0
    y: float = 0.0


class RtkModel:

    def __init__(self, zones: List[dict], cfg: RtkConfig = RtkConfig(), seed: Optional[int] = None):
        self.zones = zones
        self.cfg = cfg
        self.rng = random.Random(seed)
        self.t = None
        self.last_corr = 0.0
        self.link_down_until = -1.0
        self.outage_until = -1.0
        self.converge_until = -1.0
        self.bias = [0.0, 0.0]
        self.forced = None        # test / panelden zorlama: "link" ya da "outage"

    def zone_cap(self, x, y):
        cap, name = "FIX", None
        for z in self.zones:
            x0, y0, x1, y1 = z["rect"]
            if x0 <= x <= x1 and y0 <= y <= y1 and ORDER[z["cap"]] < ORDER[cap]:
                cap, name = z["cap"], z.get("name")
        return cap, name

    def _events(self, t, dt):
        c = self.cfg
        if t >= self.link_down_until and self.rng.random() < dt / c.link_drop_interval:
            self.link_down_until = t + self.rng.uniform(c.link_drop_min, c.link_drop_max)
        if t >= self.outage_until and self.rng.random() < dt / c.outage_interval:
            self.outage_until = t + self.rng.uniform(c.outage_min, c.outage_max)

    def drop_link(self, seconds):
        self.link_down_until = (self.t or 0.0) + seconds

    def step(self, t, true_x, true_y):
        dt = 0.0 if self.t is None else max(0.0, t - self.t)
        if self.t is None:
            self.last_corr = t
        self.t = t
        self._events(t, dt)
        link_ok = t >= self.link_down_until
        if link_ok:
            if t - self.last_corr > self.cfg.fix_age:       # baglanti yeni dondu: yakinsama
                self.converge_until = t + self.cfg.convergence
            self.last_corr = t
        age = t - self.last_corr
        mode = "FIX" if age <= self.cfg.fix_age else ("FLOAT" if age <= self.cfg.float_age else "SINGLE")
        if mode == "FIX" and t < self.converge_until:
            mode = "FLOAT"
        cap, zname = self.zone_cap(true_x, true_y)
        if ORDER[cap] < ORDER[mode]:
            mode = cap
        if t < self.outage_until:
            mode = "NONE"
        # Kayma (bias): kotu modlarda yavas rastgele yuruyus, FIX'te sifira doner
        b = BIAS.get(mode, 0.0)
        for i in range(2):
            if b == 0.0:
                self.bias[i] *= math.exp(-dt / 2.0) if dt else 1.0
            else:
                self.bias[i] += self.rng.gauss(0.0, b * 0.15 * math.sqrt(max(dt, 1e-3)))
                self.bias[i] = max(-b, min(b, self.bias[i]))
        st = RtkState(mode=mode, age=round(age, 1), zone=zname,
                      sats=self.rng.randint(*SATS[mode]))
        if mode != "NONE":
            st.sigma = SIGMA[mode]
            st.x = true_x + self.bias[0] + self.rng.gauss(0.0, st.sigma)
            st.y = true_y + self.bias[1] + self.rng.gauss(0.0, st.sigma)
        else:
            st.sigma = float("inf")
        return st
