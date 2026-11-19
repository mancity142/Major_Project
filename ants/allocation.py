"""Task allocation: cost matrix + Hungarian algorithm (scipy.optimize.linear_sum_assignment).

Cost C[i][j] (UNITS: ticks; 1 tick = 1 cell of travel) = travel time rover j -> pickup of task i
                                                         + dwell + travel pickup -> drop + dwell.
Computed with Dijkstra on the same graph the planner uses, ignoring other rovers.
Unreachable pairs get UNREACHABLE and are never assigned.  Unequal counts: the Hungarian method on a
rectangular matrix assigns min(#tasks, #rovers) pairs; leftover tasks stay PENDING, leftover rovers idle.
"""
from __future__ import annotations

from scipy.optimize import linear_sum_assignment

from ants.astar import dijkstra_times

UNREACHABLE = 10 ** 6


def build_cost_matrix(graph, rover_starts: dict, tasks, dwell: int = 1):
    """Matrix as list rows=tasks, cols=rovers (sorted by rover id)."""
    rovers = sorted(rover_starts)
    matrix = []
    for task in tasks:
        pick, drop = graph.node_by_label(task.pickup), graph.node_by_label(task.drop)
        d_pick_drop = dijkstra_times(graph, pick).get(drop)
        row = []
        for r in rovers:
            d_start_pick = dijkstra_times(graph, rover_starts[r]).get(pick)
            if d_start_pick is None or d_pick_drop is None:
                row.append(UNREACHABLE)
            else:
                row.append(d_start_pick + dwell + d_pick_drop + dwell)
        matrix.append(row)
    return matrix


def assign(matrix, rover_ids, tasks):
    """Minimum total-cost assignment. Returns ({rover_id: Task}, [unassigned tasks], total_cost)."""
    if not matrix or not rover_ids:
        return {}, list(tasks), 0
    rows, cols = linear_sum_assignment(matrix)
    assignment, used, total = {}, set(), 0
    for i, j in zip(rows, cols):
        if matrix[i][j] >= UNREACHABLE:
            continue
        assignment[rover_ids[j]] = tasks[i]
        used.add(i)
        total += matrix[i][j]
    return assignment, [t for i, t in enumerate(tasks) if i not in used], total
