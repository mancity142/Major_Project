"""Shared contract between the planners and the conflict checker / simulation.

TimedPath = [(tick, node_id), ...]
  - first entry is the start (tick 0); ticks strictly increase.
  - consecutive entries with the SAME node = the rover waits (or loads/unloads) there.
  - consecutive entries with DIFFERENT nodes = a move along one edge:
        depart at t1 from u, arrive at t2 at v, and t2 - t1 == graph.travel_time(u, v).
  - after the last entry the rover stays parked on the last node forever.

Conflict rules (checked by conflicts.find_conflicts):
  - vertex   : two rovers at the same node at the same tick (waiting, parked, departing and
               arriving ticks all count as "at the node").
  - swap     : opposite-direction use of a length-1 edge with overlapping closed tick intervals.
  - corridor : opposite-direction use of a longer edge with overlapping closed tick intervals.
  Same-direction following on a lane is allowed (equal speeds, so followers cannot collide).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Iterator

if TYPE_CHECKING:
    from ants.world_model import Graph

TimedPath = list[tuple[int, int]]


@dataclass(frozen=True)
class Task:
    task_id: int
    pickup: str  # station label, e.g. "A"
    drop: str  # station label, e.g. "X"
    priority: int = 0


@dataclass(frozen=True)
class Conflict:
    kind: str  # "vertex" | "swap" | "corridor"
    agents: tuple[int, int]
    time: int  # first tick at which the conflict occurs
    where: tuple[int, ...]  # (node,) for vertex, (u, v) for swap/corridor
    detail: tuple = ()  # swap/corridor: ((u, v, depart_a), (v, u, depart_b))


def segments(path: TimedPath) -> Iterator[tuple[int, int, int, int]]:
    """Yield (t1, u, t2, v) for each consecutive pair of entries."""
    for (t1, u), (t2, v) in zip(path, path[1:]):
        yield t1, u, t2, v


def state_at(path: TimedPath, t: int):
    """('node', n) if the rover is at a node at tick t,
    else ('edge', u, v, fraction) with 0 < fraction < 1."""
    if t <= path[0][0]:
        return ("node", path[0][1])
    for t1, u, t2, v in segments(path):
        if t1 <= t <= t2:
            if u == v or t == t1:
                return ("node", u)
            if t == t2:
                return ("node", v)
            return ("edge", u, v, (t - t1) / (t2 - t1))
    return ("node", path[-1][1])


def validate_timed_path(graph: "Graph", path: TimedPath) -> list[str]:
    """Return a list of problems; an empty list means the path obeys the contract."""
    if not path:
        return ["empty path"]
    errors: list[str] = []
    for t1, u, t2, v in segments(path):
        if t2 <= t1:
            errors.append(f"ticks must increase: {t1} -> {t2}")
        elif u != v:
            if v not in graph.adj[u]:
                errors.append(f"nodes {u} and {v} are not adjacent")
            elif t2 - t1 != graph.travel_time(u, v):
                errors.append(f"move {u}->{v} takes {graph.travel_time(u, v)} ticks, not {t2 - t1}")
    return errors
