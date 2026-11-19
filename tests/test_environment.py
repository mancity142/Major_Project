import pytest

from ants.environment import load_layout, parse_layout
from ants.world_model import build_grid_graph, build_topometric_graph


@pytest.fixture(scope="module")
def wh():
    return load_layout()


@pytest.fixture(scope="module")
def topo(wh):
    return build_topometric_graph(wh)


@pytest.fixture(scope="module")
def grid(wh):
    return build_grid_graph(wh)


def test_layout_dimensions_and_stations(wh):
    assert (wh.height, wh.width) == (11, 17)
    assert sorted(wh.pickups) == ["A", "B", "C"]
    assert sorted(wh.drops) == ["X", "Y", "Z"]
    assert sorted(wh.rover_starts) == [1, 2, 3]
    for cell in [*wh.pickups.values(), *wh.drops.values(), *wh.rover_starts.values()]:
        assert wh.is_free(cell)


def test_bad_layouts_rejected():
    with pytest.raises(ValueError):
        parse_layout("###\n##\n")
    with pytest.raises(ValueError):
        parse_layout("#Q#\n")
    with pytest.raises(ValueError):
        parse_layout("#AA#\n")


def test_counts_for_warehouse_v1(topo, grid):
    assert (topo.num_nodes, topo.num_edges) == (27, 32)
    assert (grid.num_nodes, grid.num_edges) == (75, 80)


def test_both_graphs_connected(topo, grid):
    assert topo.is_connected() and grid.is_connected()


def test_topometric_is_a_faithful_abstraction_of_grid(topo, grid):
    assert topo.total_edge_length == grid.num_edges


def test_edges_are_symmetric_and_consistent(topo):
    for u, nbrs in topo.adj.items():
        for v, e in nbrs.items():
            back = topo.adj[v][u]
            assert back.length == e.length and back.cells == tuple(reversed(e.cells))
            assert e.cells[0] == topo.nodes[u].cell and e.cells[-1] == topo.nodes[v].cell
            for a, b in zip(e.cells, e.cells[1:]):
                assert abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


@pytest.mark.parametrize("graph_name", ["topo", "grid"])
def test_labels_resolve_to_correct_cells(request, wh, graph_name):
    g = request.getfixturevalue(graph_name)
    for label, cell in {**wh.pickups, **wh.drops, **{f"R{k}": v for k, v in wh.rover_starts.items()}}.items():
        assert g.nodes[g.node_by_label(label)].cell == cell


def test_known_corridor_length(topo):
    assert topo.travel_time(topo.node_by_label("R1"), topo.cell_to_node[(5, 5)]) == 4
    assert topo.travel_time(topo.node_by_label("R3"), topo.cell_to_node[(5, 13)]) == 2
