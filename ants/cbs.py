"""Conflict-Based Search (Sharon et al. 2015) on top of Space-Time A*.

High level : best-first search over a constraint tree; node cost = sum of path end ticks.
             Take the earliest conflict, branch on each of its two rovers adding ONE constraint:
               vertex   -> rover may not be at node n at tick t
               swap/corr-> rover may not depart u->v at tick t   (exact move that collided)
             and replan only that rover.
Low level  : space_time_astar under the rover's constraints.
Terminates with conflict-free paths, or reports failure (node limit / time limit / no path).

Claims we CAN defend: output is conflict-free (verified by conflicts.find_conflicts); optimal in sum of
end ticks among solutions within the time horizon, since the low level is optimal and the constraints
are exact-move constraints.  Not claimed: new algorithm, optimality beyond the horizon / node limit.
"""
from __future__ import annotations

import heapq
import itertools
import time
from dataclasses import dataclass, field

from ants.conflicts import find_conflicts
from ants.spacetime import space_time_astar


@dataclass
class CBSResult:
    success: bool
    paths: dict = field(default_factory=dict)
    nodes_expanded: int = 0
    reason: str = ""
    root_conflicts: int = 0
    elapsed: float = 0.0


def cbs(graph, starts: dict, wps: dict, dwell: int = 1, node_limit: int = 3000, horizon: int = 200,
        time_limit: float = 60.0) -> CBSResult:
    t0 = time.perf_counter()
    agents = sorted(starts)

    def low(a, vc, ec):
        return space_time_astar(graph, starts[a], wps[a], dwell, vc, ec, (), horizon)

    paths = {}
    for a in agents:
        p = low(a, frozenset(), frozenset())
        if p is None:
            return CBSResult(False, {}, 0, f"no individual path for rover {a}", 0, time.perf_counter() - t0)
        paths[a] = p
    confs = find_conflicts(graph, paths)
    root = len(confs)
    cnt = itertools.count()
    node = {"vc": {a: frozenset() for a in agents}, "ec": {a: frozenset() for a in agents},
            "paths": paths, "conf": confs}
    cost = lambda ps: sum(p[-1][0] for p in ps.values())
    heap = [(cost(paths), len(confs), next(cnt), node)]
    expanded = 0
    while heap:
        if time.perf_counter() - t0 > time_limit:
            return CBSResult(False, {}, expanded, "time limit", root, time.perf_counter() - t0)
        _, _, _, node = heapq.heappop(heap)
        if not node["conf"]:
            return CBSResult(True, node["paths"], expanded, "ok", root, time.perf_counter() - t0)
        expanded += 1
        if expanded > node_limit:
            return CBSResult(False, {}, expanded, "node limit", root, time.perf_counter() - t0)
        c = node["conf"][0]
        for idx, a in enumerate(c.agents):
            vc, ec = dict(node["vc"]), dict(node["ec"])
            if c.kind == "vertex":
                vc[a] = vc[a] | {(c.where[0], c.time)}
            else:
                ec[a] = ec[a] | {c.detail[idx]}
            p = low(a, vc[a], ec[a])
            if p is None:
                continue
            new_paths = {**node["paths"], a: p}
            new_conf = find_conflicts(graph, new_paths)
            child = {"vc": vc, "ec": ec, "paths": new_paths, "conf": new_conf}
            heapq.heappush(heap, (cost(new_paths), len(new_conf), next(cnt), child))
    return CBSResult(False, {}, expanded, "search exhausted", root, time.perf_counter() - t0)
