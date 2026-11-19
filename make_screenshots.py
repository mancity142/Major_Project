"""Headless screenshots for the report/slides:  python make_screenshots.py"""
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import pygame

from ants.environment import PROJECT_ROOT, load_layout
from ants.pipeline import run_pipeline
from ants.scenarios import demo_scenario
from ants.viz import draw_scene, make_fonts, window_size
from ants.world_model import build_topometric_graph


def main():
    wh = load_layout()
    g = build_topometric_graph(wh)
    pygame.init()
    screen = pygame.display.set_mode(window_size(wh))
    fonts = make_fonts()
    out = PROJECT_ROOT / "outputs"
    out.mkdir(exist_ok=True)
    sc = demo_scenario()
    ind = run_pipeline(g, sc, "independent")
    cbs = run_pipeline(g, sc, "cbs")
    t_conf = ind.conflicts_final[0].time  # first conflict tick of the independent plan
    for name, res, tau in [("1_independent_conflict", ind, t_conf), ("2_cbs_same_moment", cbs, t_conf),
                           ("3_cbs_start", cbs, 0), ("4_cbs_midway", cbs, 15), ("5_cbs_done", cbs, cbs.metrics["makespan"] + 1)]:
        draw_scene(screen, wh, g, res, tau, fonts)
        pygame.image.save(screen, str(out / f"demo_{name}.png"))
        print("saved", out / f"demo_{name}.png", f"(tick {tau})")
    # Stage-1 style environment view
    pygame.quit()


if __name__ == "__main__":
    main()
