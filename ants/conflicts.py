"""Conflict checker: the referee for every planner.  Rules are defined in ants/contracts.py."""
from __future__ import annotations

from ants.contracts import Conflict, TimedPath, segments, state_at


def _node_at(path: TimedPath, t: int):
    s = state_at(path, t)
    return s[1] if s[0] == "node" else None


def _vertex_conflicts(a: int, b: int, pa: TimedPath, pb: TimedPath, horizon: int) -> list[Conflict]:
    out: list[Conflict] = []
    run_node = None
    for t in range(horizon + 1):
        na, nb = _node_at(pa, t), _node_at(pb, t)
        if na is not None and na == nb:
            if run_node != na:  # one event per continuous overlap
                out.append(Conflict("vertex", (a, b), t, (na,)))
            run_node = na
        else:
            run_node = None
    return out


def _edge_conflicts(graph, a: int, b: int, pa: TimedPath, pb: TimedPath) -> list[Conflict]:
    out: list[Conflict] = []
    for t1, u, t2, v in segments(pa):
        if u == v:
            continue
        for s1, x, s2, y in segments(pb):
            if x == y:
                continue
            if x == v and y == u and max(t1, s1) <= min(t2, s2):
                kind = "swap" if graph.travel_time(u, v) == 1 else "corridor"
                out.append(Conflict(kind, (a, b), max(t1, s1), (u, v),
                                    detail=((u, v, t1), (v, u, s1))))
    return out


def find_conflicts(graph, paths: dict[int, TimedPath]) -> list[Conflict]:
    """All conflicts between all pairs, sorted by time (earliest first)."""
    ids = sorted(paths)
    if len(ids) < 2:
        return []
    horizon = max(p[-1][0] for p in paths.values())
    out: list[Conflict] = []
    for i, a in enumerate(ids):
        for b in ids[i + 1:]:
            out += _vertex_conflicts(a, b, paths[a], paths[b], horizon)
            out += _edge_conflicts(graph, a, b, paths[a], paths[b])
    return sorted(out, key=lambda c: (c.time, c.kind, c.agents))
