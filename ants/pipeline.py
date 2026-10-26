"""End-to-end software pipeline:  tasks + rover state -> allocation -> routing -> conflict check -> metrics.

planner:  "independent"  A* per rover, other rovers ignored (baseline)
          "prioritized"  Space-Time A* with reservations, fixed priority order (comparison)
          "cbs"          Conflict-Based Search + Space-Time A* (ANTS planner)
All metrics are computed from the produced schedules; nothing is hard-coded.
"""
from __future__ import annotations

import statistics
import time
from dataclasses import dataclass, field

from ants.allocation import assign, build_cost_matrix
from ants.astar import timed_path_from_waypoints
from ants.cbs import cbs
from ants.conflicts import find_conflicts
from ants.prioritized import prioritized_plan

PLANNERS = ("independent", "prioritized", "cbs")
DWELL = 1  # ticks spent loading / unloading at a station


@dataclass
class RunResult:
    scenario: object
    graph_name: str
    planner: str
    rover_ids: list
    cost_matrix: list
    assignment: dict  # rover -> Task
    unassigned: list
    paths: dict  # rover -> TimedPath ({} if planning failed)
    conflicts_initial: list
    conflicts_final: list
    success: bool
    reason: str
    metrics: dict = field(default_factory=dict)


def _path_length(graph, path):
    return sum(graph.edge(u, v).length for (_, u), (_, v) in zip(path, path[1:]) if u != v)


def _timed(fn, repeat):
    times, out = [], None
    for _ in range(repeat):
        t0 = time.perf_counter()
        out = fn()
        times.append((time.perf_counter() - t0) * 1000)
    return out, statistics.median(times)


def run_pipeline(graph, scenario, planner: str, dwell: int = DWELL, repeat: int = 1,
                 cbs_node_limit: int = 3000, cbs_time_limit: float = 30.0, horizon: int = 200) -> RunResult:
    rover_ids = sorted(scenario.rovers)
    starts = {r: graph.node_by_label(f"R{r}") for r in rover_ids}

    # 1) allocation (Hungarian)
    (matrix, assignment, unassigned), alloc_ms = _timed(
        lambda: (lambda m: (m, *assign(m, rover_ids, list(scenario.tasks))[:2]))(
            build_cost_matrix(graph, starts, scenario.tasks, dwell)), repeat)
    wps = {r: ([graph.node_by_label(assignment[r].pickup), graph.node_by_label(assignment[r].drop)]
               if r in assignment else []) for r in rover_ids}

    # 2) independent routes (always computed: baseline + "initial conflicts" reference)
    indep = {r: timed_path_from_waypoints(graph, starts[r], wps[r], dwell) for r in rover_ids}
    conflicts_initial = find_conflicts(graph, indep)

    # 3) planning
    failure = ""
    if planner == "independent":
        paths, plan_ms = _timed(lambda: {r: timed_path_from_waypoints(graph, starts[r], wps[r], dwell)
                                         for r in rover_ids}, repeat)
        nodes = 0
    elif planner == "prioritized":
        order = sorted(rover_ids, key=lambda r: (r in assignment, -(assignment[r].priority if r in assignment else 0), r))
        paths, plan_ms = _timed(lambda: prioritized_plan(graph, starts, wps, dwell, order, horizon), repeat)
        nodes, failure = 0, ("" if paths else "no conflict-free prioritized solution")
    elif planner == "cbs":
        res, plan_ms = _timed(lambda: cbs(graph, starts, wps, dwell, cbs_node_limit, horizon, cbs_time_limit), repeat)
        paths, nodes = (res.paths if res.success else None), res.nodes_expanded
        failure = "" if res.success else f"CBS failed: {res.reason}"
    else:
        raise ValueError(planner)

    paths = paths or {}
    final = find_conflicts(graph, paths) if paths else []
    active = [r for r in rover_ids if r in assignment]
    ok = bool(paths) and not final and not failure
    m = {"alloc_ms": round(alloc_ms, 3), "plan_ms": round(plan_ms, 3), "cbs_nodes": nodes,
         "conflicts_initial": len(conflicts_initial), "conflicts_final": len(final) if paths else None,
         "success": int(ok), "tasks_assigned": len(assignment), "tasks_pending": len(unassigned)}
    if paths:
        ends = [paths[r][-1][0] for r in active]
        iso = [indep[r][-1][0] for r in active]
        m.update(makespan=max(ends) if ends else 0, sum_of_costs=sum(ends),
                 total_path_length=sum(_path_length(graph, paths[r]) for r in rover_ids),
                 path_efficiency=round(sum(iso) / sum(ends), 4) if sum(ends) else 1.0)
    return RunResult(scenario, graph.name, planner, rover_ids, matrix, assignment, unassigned, paths,
                     conflicts_initial, final, ok, failure or ("ok" if ok else "conflicts remain"), m)
