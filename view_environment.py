"""Static view of the environment + both graphs.  Keys: T topometric, G grid, S screenshot, Esc quit."""
import pygame

from ants.environment import PROJECT_ROOT, load_layout
from ants.viz import CELL, ROVER_COL, WALL_C, FREE_C, PICK_C, DROP_C, center, make_fonts
from ants.world_model import build_grid_graph, build_topometric_graph


def draw(screen, wh, topo, grid, show_topo, show_grid, fonts):
    big, mid, small = fonts
    screen.fill((250, 250, 250))
    for r in range(wh.height):
        for c in range(wh.width):
            pygame.draw.rect(screen, WALL_C if wh.rows[r][c] == "#" else FREE_C, (c * CELL, r * CELL, CELL - 1, CELL - 1))
    if show_grid:
        for e in grid.edges:
            pygame.draw.line(screen, (170, 170, 180), center(e.cells[0]), center(e.cells[-1]), 1)
        for n in grid.nodes.values():
            pygame.draw.circle(screen, (170, 170, 180), center(n.cell), 3)
    if show_topo:
        for e in topo.edges:
            pygame.draw.line(screen, (30, 90, 200), center(e.cells[0]), center(e.cells[-1]), 4)
        for n in topo.nodes.values():
            pygame.draw.circle(screen, (30, 90, 200), center(n.cell), 7)
    for label, cell in {**wh.pickups, **wh.drops}.items():
        pygame.draw.rect(screen, PICK_C if label in wh.pickups else DROP_C, (cell[1] * CELL + 3, cell[0] * CELL + 3, CELL - 7, CELL - 7), border_radius=6)
        t = big.render(label, True, (255, 255, 255)); screen.blit(t, t.get_rect(center=center(cell)))
    for rid, cell in wh.rover_starts.items():
        pygame.draw.circle(screen, ROVER_COL[rid], center(cell), CELL // 3)
        t = mid.render(str(rid), True, (255, 255, 255)); screen.blit(t, t.get_rect(center=center(cell)))
    y = wh.height * CELL + 4
    screen.blit(small.render(f"topometric: {topo.num_nodes} nodes / {topo.num_edges} edges    grid: {grid.num_nodes} nodes / {grid.num_edges} edges", True, (40, 40, 40)), (6, y))
    screen.blit(small.render("[T] topometric  [G] grid  [S] screenshot  [Esc] quit   (SIMULATED environment)", True, (90, 90, 90)), (6, y + 18))


def main():
    wh = load_layout()
    topo, grid = build_topometric_graph(wh), build_grid_graph(wh)
    pygame.init()
    screen = pygame.display.set_mode((wh.width * CELL, wh.height * CELL + 44))
    pygame.display.set_caption("ANTS - environment")
    fonts = make_fonts()
    show_topo, show_grid, running = True, False, True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE):
                running = False
            elif e.type == pygame.KEYDOWN and e.key == pygame.K_t: show_topo = not show_topo
            elif e.type == pygame.KEYDOWN and e.key == pygame.K_g: show_grid = not show_grid
            elif e.type == pygame.KEYDOWN and e.key == pygame.K_s:
                out = PROJECT_ROOT / "outputs"; out.mkdir(exist_ok=True)
                pygame.image.save(screen, str(out / "environment.png")); print("saved", out / "environment.png")
        draw(screen, wh, topo, grid, show_topo, show_grid, fonts)
        pygame.display.flip()
        pygame.time.Clock().tick(30)
    pygame.quit()


if __name__ == "__main__":
    main()
