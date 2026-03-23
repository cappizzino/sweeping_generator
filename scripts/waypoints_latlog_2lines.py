#!/usr/bin/env python3
import argparse
import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple


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


def _split_front_back(total_drones: int) -> Tuple[int, int]:
    # Matches requested examples:
    # 4 drones -> 2 + 2
    # 5 drones -> 3 + 2
    front_count = (total_drones + 1) // 2
    back_count = total_drones - front_count
    return front_count, back_count


@dataclass(frozen=True)
class TwoLineSwarmInputs:
    lat_start: float
    lon_start: float
    lat_end: float
    lon_end: float
    drones: Sequence[str]
    distance_between_drones: float
    distance_line: float
    distance_between_lines: float
    altitude: float
    second_line_lateral_shift_fraction: float = 0.0


def _compute_row_from_anchor(
    anchor_lat: float,
    anchor_lon: float,
    row_size: int,
    track_bearing: float,
    distance_between_drones: float,
    altitude: float,
) -> List[Tuple[float, float, float]]:
    row_pts: List[Tuple[float, float, float]] = []
    for i in range(row_size):
        side, k = _lateral_offset_index(i)
        if side == 0:
            row_pts.append((anchor_lat, anchor_lon, altitude))
            continue
        lateral_bearing = (track_bearing + (90.0 * side)) % 360.0
        offset_m = k * distance_between_drones
        lat, lon = destination_point(anchor_lat, anchor_lon, lateral_bearing, offset_m)
        row_pts.append((lat, lon, altitude))
    return row_pts


def _row_center_lateral_offset_m(row_size: int, distance_between_drones: float) -> float:
    if row_size <= 0:
        return 0.0
    offsets: List[float] = []
    for i in range(row_size):
        side, k = _lateral_offset_index(i)
        offsets.append(side * k * distance_between_drones)
    return sum(offsets) / row_size


def compute_two_line_swarm_waypoint_lines(inp: TwoLineSwarmInputs) -> List[List[Tuple[float, float, float]]]:
    if inp.distance_between_drones <= 0.0:
        raise ValueError("distance_between_drones must be > 0")
    if inp.distance_line <= 0.0:
        raise ValueError("distance_line must be > 0")
    if inp.distance_between_lines <= 0.0:
        raise ValueError("distance_between_lines must be > 0")
    if inp.altitude < 0.0:
        raise ValueError("altitude must be >= 0")
    if not inp.drones:
        raise ValueError("drones must contain at least one drone")

    track_bearing = initial_bearing_deg(inp.lat_start, inp.lon_start, inp.lat_end, inp.lon_end)
    total_dist = haversine_distance_m(inp.lat_start, inp.lon_start, inp.lat_end, inp.lon_end)

    segments = max(1, int(math.ceil(total_dist / inp.distance_line)))
    step_dist = total_dist / segments

    n_drones = len(inp.drones)
    front_count, back_count = _split_front_back(n_drones)

    lines: List[List[Tuple[float, float, float]]] = []
    for s in range(segments + 1):
        # Front row anchor follows the same path as the original leader.
        front_anchor_lat, front_anchor_lon = destination_point(
            inp.lat_start,
            inp.lon_start,
            track_bearing,
            step_dist * s,
        )

        front_row = _compute_row_from_anchor(
            anchor_lat=front_anchor_lat,
            anchor_lon=front_anchor_lon,
            row_size=front_count,
            track_bearing=track_bearing,
            distance_between_drones=inp.distance_between_drones,
            altitude=inp.altitude,
        )

        # Back row sits behind the front row and is laterally staggered.
        back_anchor_lat, back_anchor_lon = destination_point(
            front_anchor_lat,
            front_anchor_lon,
            (track_bearing + 180.0) % 360.0,
            inp.distance_between_lines,
        )
        front_center_m = _row_center_lateral_offset_m(front_count, inp.distance_between_drones)
        back_center_m = _row_center_lateral_offset_m(back_count, inp.distance_between_drones)
        centering_shift_m = front_center_m - back_center_m
        lateral_shift_m = centering_shift_m + (inp.second_line_lateral_shift_fraction * inp.distance_between_drones)
        if back_count > 0 and abs(lateral_shift_m) > 0.0:
            back_anchor_lat, back_anchor_lon = destination_point(
                back_anchor_lat,
                back_anchor_lon,
                (track_bearing + 90.0) % 360.0,
                lateral_shift_m,
            )

        back_row = _compute_row_from_anchor(
            anchor_lat=back_anchor_lat,
            anchor_lon=back_anchor_lon,
            row_size=back_count,
            track_bearing=track_bearing,
            distance_between_drones=inp.distance_between_drones,
            altitude=inp.altitude,
        )

        # Keep drone order aligned with inp.drones:
        # first front-row drones, then back-row drones.
        lines.append(front_row + back_row)

    return lines
