"""Small geodesy helpers (WGS84 spherical, local equirectangular projection for short distances). Pure Python."""
import math

R_EARTH = 6371008.8


def hav(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R_EARTH * math.asin(min(1.0, math.sqrt(a)))


def bearing(lat1, lon1, lat2, lon2):
    """compass bearing 0=N 90=E from point 1 to point 2"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(y, x)) % 360.0


def ang_diff(a, b):
    """signed a - b in (-180, 180]; positive = a is clockwise (to the right) of b"""
    d = (a - b + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def project(lat, lon, alat, alon, blat, blon):
    """Project point onto segment a-b. Returns (dist_m, along_m, proj_lat, proj_lon, seg_len_m)."""
    k = math.cos(math.radians(lat)) * (math.pi / 180.0) * R_EARTH
    m = (math.pi / 180.0) * R_EARTH
    ax, ay = (alon - lon) * k, (alat - lat) * m
    bx, by = (blon - lon) * k, (blat - lat) * m
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    t = 0.0 if seg2 == 0 else max(0.0, min(1.0, -(ax * dx + ay * dy) / seg2))
    px, py = ax + t * dx, ay + t * dy
    return (math.hypot(px, py), t * math.sqrt(seg2), lat + py / m, lon + px / k, math.sqrt(seg2))


def polyline_length(pts):
    return sum(hav(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1]) for i in range(len(pts) - 1))


def is_finite_number(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def valid_latlon(lat, lon):
    return is_finite_number(lat) and is_finite_number(lon) and -90.0 < lat < 90.0 and -180.0 <= lon <= 180.0
