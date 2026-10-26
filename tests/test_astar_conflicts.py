import itertools

import pytest

from ants.astar import astar, dijkstra_times, timed_path_from_waypoints
from ants.conflicts import find_conflicts
from ants.contracts import validate_timed_path
from ants.environment import load_layout
from ants.world_model import build_grid_graph, build_topometric_graph


@pytest.fixture(scope="module")
def topo():
    return build_topometric_graph(load_layout())


@pytest.fixture(scope="module")
def grid():
    return build_grid_graph(load_layout())


@pytest.mark.parametrize("graph_name", ["topo", "grid"])
def test_astar_is_optimal_vs_dijkstra(request, graph_name):
    g = request.getfixturevalue(graph_name)
    for a, b in itertools.product(["R1", "R2", "R3", "A", "Z"], ["X", "B", "R3", "C"]):
        u, v = g.node_by_label(a), g.node_by_label(b)
        path = astar(g, u, v)
        cost = sum(g.travel_time(x, y) for x, y in zip(path, path[1:]))
        assert cost == dijkstra_times(g, u)[v]


def test_astar_start_equals_goal_and_unreachable(topo):
    r1 = topo.node_by_label("R1")
    assert astar(topo, r1, r1) == [r1]
    from ants.world_model import Graph, Node
    g = Graph("two")
    g.add_node(Node(0, (0, 0), "cell"))
    g.add_node(Node(1, (5, 5), "cell"))
    assert astar(g, 0, 1) is None


def test_topometric_and_grid_give_same_travel_time(topo, grid):
    for a, b in [("R1", "X"), ("R3", "A"), ("C", "Z")]:
        assert dijkstra_times(topo, topo.node_by_label(a))[topo.node_by_label(b)] == \
               dijkstra_times(grid, grid.node_by_label(a))[grid.node_by_label(b)]


def test_independent_timed_path_is_valid(topo):
    p = timed_path_from_waypoints(topo, topo.node_by_label("R1"), [topo.node_by_label("A"), topo.node_by_label("Z")], 1)
    assert validate_timed_path(topo, p) == []


# ---- conflict checker: the referee ----
def test_vertex_conflict_same_node_same_tick(topo):
    j = topo.cell_to_node[(5, 5)]
    r1, r2 = topo.node_by_label("R1"), topo.node_by_label("R2")
    a = [(0, r1), (4, j)]
    b = [(0, r2), (4, j)]
    cs = find_conflicts(topo, {1: a, 2: b})
    assert [c.kind for c in cs] == ["vertex"] and cs[0].time == 4 and cs[0].where == (j,)


def test_parked_rover_blocks_a_node(topo):
    j = topo.cell_to_node[(5, 5)]
    r1, r2 = topo.node_by_label("R1"), topo.node_by_label("R2")
    a = [(0, j)]                      # parked at the junction
    b = [(0, r2), (4, j), (6, j)]     # arrives there later
    assert any(c.kind == "vertex" and c.time == 4 for c in find_conflicts(topo, {1: a, 2: b}))


def test_head_on_corridor_conflict(topo):
    r1, j = topo.node_by_label("R1"), topo.cell_to_node[(5, 5)]
    a = [(0, r1), (4, j)]
    b = [(0, j), (2, j), (6, r1)]     # leaves the junction at tick 2, overlapping [2,4]
    cs = find_conflicts(topo, {1: a, 2: b})
    assert [c.kind for c in cs] == ["corridor"] and cs[0].time == 2
    assert cs[0].detail == ((r1, j, 0), (j, r1, 2))


def test_swap_on_unit_edge_in_grid(grid):
    u, v = grid.cell_to_node[(5, 2)], grid.cell_to_node[(5, 3)]
    cs = find_conflicts(grid, {1: [(0, u), (1, v)], 2: [(0, v), (1, u)]})
    assert [c.kind for c in cs] == ["swap"]


def test_following_in_same_direction_is_allowed(topo):
    n11, r1, j, n59 = (topo.cell_to_node[(1, 1)], topo.node_by_label("R1"),
                       topo.cell_to_node[(5, 5)], topo.cell_to_node[(5, 9)])
    a = [(0, r1), (4, j), (8, n59)]
    b = [(0, n11), (4, r1), (8, j)]
    assert find_conflicts(topo, {1: a, 2: b}) == []


def test_waiting_on_a_node_the_other_rover_arrives_at_is_a_vertex_conflict(topo):
    r1, j = topo.node_by_label("R1"), topo.cell_to_node[(5, 5)]
    a = [(0, r1), (4, j)]
    b = [(0, j), (4, j), (8, r1)]     # b waits at j until a arrives... but a sits on j -> vertex conflict
    assert any(c.kind == "vertex" for c in find_conflicts(topo, {1: a, 2: b}))
