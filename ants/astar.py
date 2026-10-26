"""Single-agent A* on a Graph (edge cost = travel time in ticks).

Heuristic: Manhattan distance between node cells. It is admissible and consistent because every
edge is at least as long (in cells) as the Manhattan distance between its end nodes.
"""
from __future__ import annotations

import heapq

from ants.contracts import TimedPath

INF = float("inf")


def astar(graph, start: int, goal: int) -> list[int] | None:
    """Shortest node path from start to goal, or None if the goal is unreachable."""
    if start == goal:
        return [start]
    gr, gc = graph.nodes[goal].cell

    def h(n: int) -> int:
        r, c = graph.nodes[n].cell
        return abs(r - gr) + abs(c - gc)

    open_: list[tuple[int, int, int]] = [(h(start), 0, start)]
    best = {start: 0}
    parent: dict[int, int] = {}
    closed: set[int] = set()
    while open_:
        _, g, u = heapq.heappop(open_)
        if u in closed:
            continue
        if u == goal:
            path = [u]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return path[::-1]
        closed.add(u)
        for v in graph.neighbors(u):
            ng = g + graph.travel_time(u, v)
            if ng < best.get(v, INF):
                best[v] = ng
                parent[v] = u
                heapq.heappush(open_, (ng + h(v), ng, v))
    return None


def dijkstra_times(graph, source: int) -> dict[int, int]:
    """Travel time from source to every node (used for heuristics and the cost matrix)."""
    dist = {source: 0}
    heap = [(0, source)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist.get(u, INF):
            continue
        for v in graph.neighbors(u):
            nd = d + graph.travel_time(u, v)
            if nd < dist.get(v, INF):
                dist[v] = nd
                heapq.heappush(heap, (nd, v))
    return dist


def timed_path_from_waypoints(graph, start: int, waypoints: list[int], dwell: int = 1) -> TimedPath | None:
    """Independent (single-agent) plan: A* between consecutive waypoints, `dwell` ticks of
    loading/unloading at each waypoint. Ignores all other rovers."""
    path: TimedPath = [(0, start)]
    t, cur = 0, start
    for wp in waypoints:
        nodes = astar(graph, cur, wp)
        if nodes is None:
            return None
        for u, v in zip(nodes, nodes[1:]):
            t += graph.travel_time(u, v)
            path.append((t, v))
        cur = wp
        if dwell > 0:
            t += dwell
            path.append((t, wp))
    return path
