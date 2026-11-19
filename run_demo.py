"""Interactive ANTS simulation.  python run_demo.py
Keys: SPACE pause | LEFT/RIGHT step 1 tick | +/- speed | R restart | 1 independent | 2 prioritized | 3 CBS
      N next scenario | M switch topometric/grid map | G toggle graph overlay | S screenshot | ESC quit
"""
import pygame

from ants.environment import PROJECT_ROOT, load_layout
from ants.pipeline import PLANNERS, run_pipeline
from ants.scenarios import all_scenarios
from ants.viz import draw_scene, live_collisions, make_fonts, window_size
from ants.world_model import build_grid_graph, build_topometric_graph


def main():
    wh = load_layout()
    graphs = {"topometric": build_topometric_graph(wh), "grid": build_grid_graph(wh)}
    scenarios = all_scenarios()
    si, planner, gname = 0, "cbs", "topometric"
    cache = {}

    def result():
        key = (si, planner, gname)
        if key not in cache:
            cache[key] = run_pipeline(graphs[gname], scenarios[si], planner)
        return cache[key]

    pygame.init()
    screen = pygame.display.set_mode(window_size(wh))
    pygame.display.set_caption("ANTS - simulation")
    fonts = make_fonts()
    clock = pygame.time.Clock()
    tau, speed, paused, show_graph, events, was_bad = 0.0, 4.0, False, True, 0, set()
    running = True
    while running:
        dt = clock.tick(30) / 1000
        res = result()
        end = max((p[-1][0] for p in res.paths.values()), default=0) + 3
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                k = e.key
                if k == pygame.K_ESCAPE: running = False
                elif k == pygame.K_SPACE: paused = not paused
                elif k == pygame.K_RIGHT: tau, paused = min(end, tau + 1), True
                elif k == pygame.K_LEFT: tau, paused = max(0, tau - 1), True
                elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS): speed = min(32, speed * 1.5)
                elif k in (pygame.K_MINUS, pygame.K_KP_MINUS): speed = max(0.5, speed / 1.5)
                elif k == pygame.K_g: show_graph = not show_graph
                elif k in (pygame.K_1, pygame.K_2, pygame.K_3): planner = PLANNERS[k - pygame.K_1]; tau, events, was_bad = 0.0, 0, set()
                elif k == pygame.K_n: si = (si + 1) % len(scenarios); tau, events, was_bad = 0.0, 0, set()
                elif k == pygame.K_m: gname = "grid" if gname == "topometric" else "topometric"; tau, events, was_bad = 0.0, 0, set()
                elif k == pygame.K_r: tau, events, was_bad = 0.0, 0, set()
                elif k == pygame.K_s:
                    out = PROJECT_ROOT / "outputs"; out.mkdir(exist_ok=True)
                    pygame.image.save(screen, str(out / f"screenshot_{scenarios[si].name}_{planner}_{gname}.png"))
        if not paused:
            tau = min(end, tau + dt * speed)
        res = result()
        now = set(live_collisions(graphs[gname], res.paths, tau)) if res.paths else set()
        events += len(now - was_bad)
        was_bad = now
        draw_scene(screen, wh, graphs[gname], res, tau, fonts, show_graph, events)
        pygame.display.flip()
    pygame.quit()


if __name__ == "__main__":
    main()
