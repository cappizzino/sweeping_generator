#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple, Dict, Any
from pyproj import Transformer

# -------------------- Waypoint generator --------------------

EARTH_RADIUS_M = 6378137.0  # spherical approx

def _deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0

def _rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi

def haversine_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, lam1 = _deg2rad(lat1), _deg2rad(lon1)
    phi2, lam2 = _deg2rad(lat2), _deg2rad(lon2)
    dphi = phi2 - phi1
    dlam = lam2 - lam1
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * (math.sin(dlam / 2.0) ** 2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_RADIUS_M * c

def initial_bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, phi2 = _deg2rad(lat1), _deg2rad(lat2)
    lam1, lam2 = _deg2rad(lon1), _deg2rad(lon2)
    dlam = lam2 - lam1
    y = math.sin(dlam) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    brng = math.atan2(y, x)
    return (_rad2deg(brng) + 360.0) % 360.0

def destination_point(lat: float, lon: float, bearing_deg: float, distance_m: float) -> Tuple[float, float]:
    phi1 = _deg2rad(lat)
    lam1 = _deg2rad(lon)
    theta = _deg2rad(bearing_deg)
    delta = distance_m / EARTH_RADIUS_M

    sin_phi2 = math.sin(phi1) * math.cos(delta) + math.cos(phi1) * math.sin(delta) * math.cos(theta)
    phi2 = math.asin(max(-1.0, min(1.0, sin_phi2)))

    y = math.sin(theta) * math.sin(delta) * math.cos(phi1)
    x = math.cos(delta) - math.sin(phi1) * math.sin(phi2)
    lam2 = lam1 + math.atan2(y, x)

    lon2 = (_rad2deg(lam2) + 540.0) % 360.0 - 180.0
    lat2 = _rad2deg(phi2)
    return lat2, lon2

def _lateral_offset_index(i: int) -> Tuple[int, int]:
    if i == 0:
        return 0, 0
    k = (i + 1) // 2
    side = +1 if (i % 2 == 1) else -1
    return side, k

@dataclass(frozen=True)
class SwarmInputs:
    lat_start: float
    lon_start: float
    lat_end: float
    lon_end: float
    drones: Sequence[str]
    distance_between_drones: float
    distance_line: float
    altitude: float

# def compute_swarm_waypoint_lines(inp: SwarmInputs) -> List[List[Tuple[float, float]]]:
#     if inp.distance_between_drones <= 0.0:
#         raise ValueError("distance_between_drones must be > 0")
#     if inp.distance_line <= 0.0:
#         raise ValueError("distance_line must be > 0")
#     if not inp.drones:
#         raise ValueError("drones must contain at least the leader")

#     track_bearing = initial_bearing_deg(inp.lat_start, inp.lon_start, inp.lat_end, inp.lon_end)
#     total_dist = haversine_distance_m(inp.lat_start, inp.lon_start, inp.lat_end, inp.lon_end)

#     segments = max(1, int(math.ceil(total_dist / inp.distance_line)))
#     step_dist = total_dist / segments

#     lines: List[List[Tuple[float, float]]] = []
#     for s in range(segments + 1):
#         leader_lat, leader_lon = destination_point(inp.lat_start, inp.lon_start, track_bearing, step_dist * s)

#         line_pts: List[Tuple[float, float]] = []
#         for i in range(len(inp.drones)):
#             side, k = _lateral_offset_index(i)
#             if side == 0:
#                 line_pts.append((leader_lat, leader_lon))
#             else:
#                 lateral_bearing = (track_bearing + (90.0 * side)) % 360.0
#                 offset_m = k * inp.distance_between_drones
#                 lat_i, lon_i = destination_point(leader_lat, leader_lon, lateral_bearing, offset_m)
#                 line_pts.append((lat_i, lon_i))
#         lines.append(line_pts)

#     return lines

def compute_swarm_waypoint_lines(
    inp: SwarmInputs,
) -> List[List[Tuple[float, float, float]]]:

    if inp.distance_between_drones <= 0.0:
        raise ValueError("distance_between_drones must be > 0")
    if inp.distance_line <= 0.0:
        raise ValueError("distance_line must be > 0")
    if inp.altitude < 0.0:
        raise ValueError("altitude must be >= 0")
    if not inp.drones:
        raise ValueError("drones must contain at least the leader")

    track_bearing = initial_bearing_deg(
        inp.lat_start, inp.lon_start,
        inp.lat_end, inp.lon_end
    )

    total_dist = haversine_distance_m(
        inp.lat_start, inp.lon_start,
        inp.lat_end, inp.lon_end
    )

    segments = max(1, int(math.ceil(total_dist / inp.distance_line)))
    step_dist = total_dist / segments

    lines: List[List[Tuple[float, float, float]]] = []

    for s in range(segments + 1):
        leader_lat, leader_lon = destination_point(
            inp.lat_start,
            inp.lon_start,
            track_bearing,
            step_dist * s
        )

        line_pts: List[Tuple[float, float, float]] = []

        for i in range(len(inp.drones)):
            side, k = _lateral_offset_index(i)

            if side == 0:
                lat, lon = leader_lat, leader_lon
            else:
                lateral_bearing = (track_bearing + 90.0 * side) % 360.0
                offset_m = k * inp.distance_between_drones
                lat, lon = destination_point(
                    leader_lat,
                    leader_lon,
                    lateral_bearing,
                    offset_m
                )

            line_pts.append((lat, lon, inp.altitude))

        lines.append(line_pts)

    return lines