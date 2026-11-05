"""WGS84 <-> UTM donusumu (Kruger serisi, mm hassasiyet).

robot_localization'in navsat_transform_node'u GNSS konumunu UTM'e cevirip datuma
gore fark alir. Simulatorde konumu bu donusumun tersiyle uretirsek, navsat'in
hesapladigi harita konumu Gazebo dunyasindaki konumla birebir ortusur.
"""

import math

A = 6378137.0
F = 1 / 298.257223563
K0 = 0.9996
E2 = F * (2 - F)
N = F / (2 - F)
A_ = A / (1 + N) * (1 + N ** 2 / 4 + N ** 4 / 64)
ALPHA = (N / 2 - 2 * N ** 2 / 3 + 5 * N ** 3 / 16, 13 * N ** 2 / 48 - 3 * N ** 3 / 5, 61 * N ** 3 / 240)
BETA = (N / 2 - 2 * N ** 2 / 3 + 37 * N ** 3 / 96, N ** 2 / 48 + N ** 3 / 15, 17 * N ** 3 / 480)
DELTA = (2 * N - 2 * N ** 2 / 3 - 2 * N ** 3, 7 * N ** 2 / 3 - 8 * N ** 3 / 5, 56 * N ** 3 / 15)


def zone_of(lon):
    return int((lon + 180) // 6) + 1


def central_meridian(zone):
    return math.radians(zone * 6 - 183)


def forward(lat, lon, zone=None):
    """(lat, lon) derece -> (easting, northing, zone). Kuzey yarikure."""
    zone = zone or zone_of(lon)
    phi, lam = math.radians(lat), math.radians(lon) - central_meridian(zone)
    e = math.sqrt(E2)
    t = math.sinh(math.atanh(math.sin(phi)) - e * math.atanh(e * math.sin(phi)))
    xi_, eta_ = math.atan2(t, math.cos(lam)), math.atanh(math.sin(lam) / math.sqrt(1 + t * t))
    xi, eta = xi_, eta_
    for j, a in enumerate(ALPHA, 1):
        xi += a * math.sin(2 * j * xi_) * math.cosh(2 * j * eta_)
        eta += a * math.cos(2 * j * xi_) * math.sinh(2 * j * eta_)
    return 500000.0 + K0 * A_ * eta, K0 * A_ * xi, zone


def inverse(easting, northing, zone):
    """(easting, northing, zone) -> (lat, lon) derece. Kuzey yarikure."""
    xi, eta = northing / (K0 * A_), (easting - 500000.0) / (K0 * A_)
    xi_, eta_ = xi, eta
    for j, b in enumerate(BETA, 1):
        xi_ -= b * math.sin(2 * j * xi) * math.cosh(2 * j * eta)
        eta_ -= b * math.cos(2 * j * xi) * math.sinh(2 * j * eta)
    chi = math.asin(math.sin(xi_) / math.cosh(eta_))
    phi = chi
    for j, d in enumerate(DELTA, 1):
        phi += d * math.sin(2 * j * chi)
    lam = math.atan2(math.sinh(eta_), math.cos(xi_))
    return math.degrees(phi), math.degrees(lam + central_meridian(zone))


class LocalFrame:
    """Dunya (x dogu, y kuzey) <-> cografi konum, datum UTM'inden farkla."""

    def __init__(self, lat0, lon0):
        self.e0, self.n0, self.zone = forward(lat0, lon0)

    def to_geo(self, x, y):
        return inverse(self.e0 + x, self.n0 + y, self.zone)

    def to_local(self, lat, lon):
        e, n, _ = forward(lat, lon, self.zone)
        return e - self.e0, n - self.n0
