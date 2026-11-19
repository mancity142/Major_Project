from ants.environment import load_layout
from ants.world_model import build_grid_graph, build_topometric_graph


def overlay(wh, graph):
    canvas = [list(row) for row in wh.rows]
    for n in graph.nodes.values():
        if n.kind in ("junction", "corner", "dead_end"):
            r, c = n.cell
            canvas[r][c] = "o"
    return ["".join(row) for row in canvas]


def main():
    wh = load_layout()
    topo, grid = build_topometric_graph(wh), build_grid_graph(wh)
    print(f"Layout '{wh.name}': {wh.height} rows x {wh.width} cols, {len(wh.free_cells())} free cells")
    print("Pickups:", wh.pickups, "\nDrops:  ", wh.drops, "\nRovers: ", wh.rover_starts)
    print("\nTopometric nodes shown as 'o' (stations/rovers keep their letters):")
    print("\n".join(overlay(wh, topo)))
    print("\nNODES (id, cell, kind, label)")
    for n in topo.nodes.values():
        print(f"  {n.id:2d} {n.cell} {n.kind:11s} {n.label}")
    print("\nEDGES (u -> v, length in cells = ticks)")
    for e in topo.edges:
        print(f"  {e.u:2d} -> {e.v:2d}  length {e.length}")
    print("\nSUMMARY")
    print(f"  topometric: {topo.num_nodes} nodes, {topo.num_edges} edges, total length {topo.total_edge_length}, connected={topo.is_connected()}")
    print(f"  grid:       {grid.num_nodes} nodes, {grid.num_edges} edges, total length {grid.total_edge_length}, connected={grid.is_connected()}")
    print(f"  sparsity:   topometric uses {topo.num_nodes / grid.num_nodes:.0%} of the grid's nodes")


if __name__ == "__main__":
    main()
