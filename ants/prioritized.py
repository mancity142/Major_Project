"""Prioritized planning: plan rovers one by one; each later rover treats earlier paths as reserved.
Fast, but depends on the priority order (can fail or give worse solutions than CBS)."""
from __future__ import annotations

from ants.spacetime import space_time_astar


def prioritized_plan(graph, starts: dict, wps: dict, dwell: int = 1, order=None, horizon: int = 200):
    """Return {rover: TimedPath} or None if some rover cannot be planned."""
    order = list(order) if order is not None else sorted(starts)
    paths = {}
    for a in order:
        p = space_time_astar(graph, starts[a], wps[a], dwell, reserved=list(paths.values()), horizon=horizon)
        if p is None:
            return None
        paths[a] = p
    return paths
