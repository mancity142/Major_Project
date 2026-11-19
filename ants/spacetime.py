"""Space-Time A*: single-agent planner whose state is (node, tick, stage).

* Moves take graph.travel_time ticks; waiting (1 tick) is allowed AT NODES only.
* `stage` = number of waypoints already serviced (e.g. pickup done, drop done). Servicing a
  waypoint keeps the rover on the node for `dwell` ticks (loading/unloading).
* Constraints from CBS:  vertex (node, tick)  and  edge (u, v, depart_tick).
* `reserved` = timed paths of higher-priority rovers (prioritized planning): forbidden vertices,
  opposite-direction edge overlap, and nodes where those rovers end up parked.
* The rover parks on its final waypoint forever (goal = be ON that node after all waypoints are serviced),
  so the goal is only accepted after the last tick at which that node is constrained/reserved.
* Heuristic = exact travel time through the remaining waypoints (admissible and consistent), so the
  search is optimal in arrival time for the given constraints (within `horizon`).
"""
from __future__ import annotations

import heapq
import itertools

from ants.astar import dijkstra_times
from ants.contracts import TimedPath, segments


class _Reservations:
    def __init__(self, paths):
        self.vertex: set[tuple[int, int]] = set()
        self.parked: dict[int, int] = {}
        self.edges: dict[tuple[int, int], list[tuple[int, int]]] = {}
        self.last: dict[int, int] = {}
        for p in paths:
            for t1, u, t2, v in segments(p):
                if u == v:
                    for t in range(t1, t2 + 1):
                        self._occ(u, t)
                else:
                    self._occ(u, t1)
                    self._occ(v, t2)
                    self.edges.setdefault((u, v), []).append((t1, t2))
            t_last, n_last = p[-1]
            self._occ(n_last, t_last)
            self.parked[n_last] = min(self.parked.get(n_last, t_last), t_last)

    def _occ(self, n: int, t: int) -> None:
        self.vertex.add((n, t))
        self.last[n] = max(self.last.get(n, -1), t)


def space_time_astar(graph, start: int, waypoints, dwell: int = 1, vertex_constraints=frozenset(),
                     edge_constraints=frozenset(), reserved=(), horizon: int = 200,
                     max_expansions: int = 400_000, stats: dict | None = None) -> TimedPath | None:
    wps = list(waypoints)
    k = len(wps)
    final = wps[-1] if k else start
    res = _Reservations(reserved)
    vc, ec = vertex_constraints, edge_constraints

    if final in res.parked:
        return None  # goal node is permanently occupied by a higher-priority rover
    block = max([res.last.get(final, -1)] + [t for (n, t) in vc if n == final])

    def ok_node(n: int, t: int) -> bool:
        if (n, t) in vc or (n, t) in res.vertex:
            return False
        return not (n in res.parked and t >= res.parked[n])

    def edge_ok(u: int, v: int, t: int, d: int) -> bool:
        if (u, v, t) in ec:
            return False
        return all(not (max(t, s1) <= min(t + d, s2)) for s1, s2 in res.edges.get((v, u), ()))

    dist = [dijkstra_times(graph, w) for w in wps]
    tail = [0] * (k + 1)
    for j in range(k - 2, -1, -1):
        tail[j] = tail[j + 1] + dist[j][wps[j + 1]]

    dist_final = dist[k - 1] if k else dijkstra_times(graph, start)  # travel time to the parking node

    def h(n: int, s: int) -> int:
        return dist_final[n] if s >= k else dist[s][n] + tail[s] + dwell * (k - s)

    if not ok_node(start, 0):
        return None
    cnt = itertools.count()
    s0 = (start, 0, 0)
    heap = [(h(start, 0), 0, next(cnt), s0)]
    parent = {s0: None}
    closed: set = set()
    expansions = 0
    while heap:
        _, _, _, s = heapq.heappop(heap)
        if s in closed:
            continue
        closed.add(s)
        n, t, st = s
        if st >= k and n == final and t > block:
            if stats is not None:
                stats["expansions"] = expansions
            return _build(parent, s)
        expansions += 1
        if expansions > max_expansions:
            return None
        if t >= horizon:
            continue

        def push(ns):
            if ns not in parent:
                parent[ns] = s
                heapq.heappush(heap, (ns[1] + h(ns[0], ns[2]), ns[1], next(cnt), ns))

        if ok_node(n, t + 1):  # wait
            push((n, t + 1, st))
        if st < k and n == wps[st] and all(ok_node(n, t + i) for i in range(1, dwell + 1)):
            push((n, t + dwell, st + 1))  # service (load/unload)
        for v in graph.neighbors(n):  # move
            d = graph.travel_time(n, v)
            if t + d <= horizon and ok_node(v, t + d) and edge_ok(n, v, t, d):
                push((v, t + d, st))
    return None


def _build(parent, s) -> TimedPath:
    seq = []
    while s is not None:
        seq.append(s)
        s = parent[s]
    seq.reverse()
    raw = [(seq[0][1], seq[0][0])]
    for n, t, _ in seq[1:]:
        if (t, n) != raw[-1]:
            raw.append((t, n))
    out = [raw[0]]
    for t, n in raw[1:]:
        if n == out[-1][1]:
            if len(out) >= 2 and out[-2][1] == n:
                out[-1] = (t, n)  # extend an existing wait
            else:
                out.append((t, n))
        else:
            out.append((t, n))
    return out
