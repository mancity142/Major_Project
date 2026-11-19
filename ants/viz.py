"""Pygame rendering.  The rovers are drawn ONLY from the planner's timed paths (no separate motion logic)."""
from __future__ import annotations

import math

import pygame

CELL = 40
PANEL = 330
ROVER_COL = {1: (215, 65, 65), 2: (60, 110, 215), 3: (60, 165, 95)}
WALL_C, FREE_C = (62, 65, 72), (236, 238, 241)
PICK_C, DROP_C = (90, 175, 125), (225, 155, 65)
TOPO_C = (160, 175, 215)
BAD = (230, 30, 30)


def window_size(wh):
    return wh.width * CELL + PANEL, max(wh.height * CELL + 70, 520)


def make_fonts():
    names = "dejavusansmono,consolas,couriernew,monospace"
    return (pygame.font.SysFont(names, 18, bold=True), pygame.font.SysFont(names, 14), pygame.font.SysFont(names, 12))


def center(cell):
    return (cell[1] * CELL + CELL / 2, cell[0] * CELL + CELL / 2)


def position_at(graph, path, tau):
    """(row, col) float position of a rover at time tau (ticks), interpolated along the corridor cells."""
    if tau <= path[0][0]:
        return graph.nodes[path[0][1]].cell
    for (t1, u), (t2, v) in zip(path, path[1:]):
        if t1 <= tau <= t2:
            if u == v:
                return graph.nodes[u].cell
            cells = graph.edge(u, v).cells
            x = (tau - t1) / (t2 - t1) * (len(cells) - 1)
            i = min(int(x), len(cells) - 2)
            f = x - i
            return (cells[i][0] + (cells[i + 1][0] - cells[i][0]) * f, cells[i][1] + (cells[i + 1][1] - cells[i][1]) * f)
    return graph.nodes[path[-1][1]].cell


def task_status(graph, path, task, tau):
    pick = graph.node_by_label(task.pickup)
    t_pick = next((t for t, n in path if n == pick), None)
    t_end = path[-1][0]
    if tau >= t_end:
        return "COMPLETED"
    return "IN_PROGRESS" if t_pick is not None and tau >= t_pick else "ASSIGNED"


def live_collisions(graph, paths, tau, radius=0.75):
    """Rover pairs whose drawn positions overlap right now (independent geometric check)."""
    pos = {r: position_at(graph, p, tau) for r, p in paths.items()}
    ids = sorted(pos)
    return [(a, b) for i, a in enumerate(ids) for b in ids[i + 1:]
            if math.dist(pos[a], pos[b]) < radius]


def draw_scene(surface, wh, graph, result, tau, fonts, show_graph=True, live_event_count=0, hint=""):
    big, mid, small = fonts
    surface.fill((250, 250, 250))
    for r in range(wh.height):
        for c in range(wh.width):
            pygame.draw.rect(surface, WALL_C if wh.rows[r][c] == "#" else FREE_C, (c * CELL, r * CELL, CELL - 1, CELL - 1))
    if show_graph and graph.name == "topometric":
        for e in graph.edges:
            pygame.draw.line(surface, TOPO_C, center(e.cells[0]), center(e.cells[-1]), 3)
        for n in graph.nodes.values():
            pygame.draw.circle(surface, TOPO_C, center(n.cell), 4)
    for label, cell in {**wh.pickups, **wh.drops}.items():
        col = PICK_C if label in wh.pickups else DROP_C
        pygame.draw.rect(surface, col, (cell[1] * CELL + 3, cell[0] * CELL + 3, CELL - 7, CELL - 7), border_radius=6)
        t = big.render(label, True, (255, 255, 255))
        surface.blit(t, t.get_rect(center=center(cell)))

    paths = result.paths
    # planned route lines (thin), then rovers
    for r, p in paths.items():
        pts = [center(c) for (_, u), (_, v) in zip(p, p[1:]) if u != v for c in graph.edge(u, v).cells]
        if len(pts) > 1:
            pygame.draw.lines(surface, ROVER_COL[r], False, pts, 1)
    bad = live_collisions(graph, paths, tau) if paths else []
    for r, p in paths.items():
        x, y = center(position_at(graph, p, tau))
        pygame.draw.circle(surface, ROVER_COL[r], (x, y), CELL * 0.32)
        pygame.draw.circle(surface, (255, 255, 255), (x, y), CELL * 0.32, 2)
        t = mid.render(str(r), True, (255, 255, 255))
        surface.blit(t, t.get_rect(center=(x, y)))
    for a, b in bad:
        mx = sum(center(position_at(graph, paths[k], tau))[0] for k in (a, b)) / 2
        my = sum(center(position_at(graph, paths[k], tau))[1] for k in (a, b)) / 2
        pygame.draw.circle(surface, BAD, (mx, my), CELL * 0.62, 4)

    # ---- side panel ----
    x0, y = wh.width * CELL + 12, 8
    def line(text, font=mid, color=(30, 30, 30), dy=19):
        nonlocal y
        surface.blit(font.render(text, True, color), (x0, y))
        y += dy
    m = result.metrics
    line("ANTS  (SIMULATION)", big, (20, 40, 90), 26)
    line(f"scenario : {result.scenario.name}")
    line(f"map      : {graph.name} ({graph.num_nodes} nodes)")
    line(f"planner  : {result.planner}", color=(20, 90, 20) if result.success else BAD)
    line(f"tick     : {tau:5.1f}")
    ok = result.success
    line(f"conflicts in schedule: {m.get('conflicts_final') if paths else 'n/a'}", color=(20, 120, 20) if ok else BAD)
    line(f"live overlaps now / events: {len(bad)} / {live_event_count}", color=BAD if bad else (30, 30, 30))
    if not paths:
        line(f"PLANNING FAILED: {result.reason}", small, BAD)
    y += 4
    line("Hungarian allocation (cost, ticks)", mid, (20, 40, 90))
    header = "task   " + " ".join(f"R{r:<3}" for r in result.rover_ids)
    line(header, small, dy=15)
    for i, t in enumerate(result.scenario.tasks):
        cells = []
        for j, r in enumerate(result.rover_ids):
            mark = "*" if result.assignment.get(r) == t else " "
            cells.append(f"{result.cost_matrix[i][j]:>3}{mark}")
        line(f"{t.pickup}->{t.drop}   " + " ".join(cells), small, dy=15)
    y += 4
    line("Task status", mid, (20, 40, 90))
    for r, t in sorted(result.assignment.items()):
        st = task_status(graph, paths[r], t, tau) if paths else "FAILED"
        line(f"R{r}: {t.pickup}->{t.drop}  {st}", small, ROVER_COL[r], 15)
    for t in result.unassigned:
        line(f"--: {t.pickup}->{t.drop}  PENDING", small, (110, 110, 110), 15)
    y += 4
    if paths:
        line(f"makespan {m.get('makespan')}  sum-of-costs {m.get('sum_of_costs')}", small, dy=15)
        line(f"plan {m['plan_ms']} ms  CBS nodes {m['cbs_nodes']}", small, dy=15)
    y = wh.height * CELL + 6
    surface.blit(small.render(hint or "SPACE pause | <- -> step | +/- speed | R restart | 1/2/3 planner | N scenario | M map | G graph | S shot",
                              True, (70, 70, 70)), (6, y))
    surface.blit(small.render("Simulated rovers only: no camera / ESP32 / PID in this demo.", True, (150, 60, 60)), (6, y + 16))
    return bad
