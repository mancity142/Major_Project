import pytest

from ants.astar import timed_path_from_waypoints
from ants.cbs import cbs
from ants.conflicts import find_conflicts
from ants.contracts import validate_timed_path
from ants.environment import load_layout
from ants.pipeline import run_pipeline
from ants.scenarios import all_scenarios, demo_scenario
from ants.spacetime import space_time_astar
from ants.world_model import build_grid_graph, build_topometric_graph


@pytest.fixture(scope="module")
def topo():
    return build_topometric_graph(load_layout())


@pytest.fixture(scope="module")
def grid():
    return build_grid_graph(load_layout())


def test_unconstrained_spacetime_equals_astar_cost(topo):
    r1, a, z = (topo.node_by_label(x) for x in ("R1", "A", "Z"))
    p1 = timed_path_from_waypoints(topo, r1, [a, z], 1)
    p2 = space_time_astar(topo, r1, [a, z], 1)
    assert p1[-1][0] == p2[-1][0] and validate_timed_path(topo, p2) == []
    assert p2[-1][1] == z                      # rover really ends ON its final waypoint


def test_vertex_constraint_is_respected(topo):
    r1, a = topo.node_by_label("R1"), topo.node_by_label("A")
    base = space_time_astar(topo, r1, [a], 1)
    t_arr = next(t for t, n in base if n == a)
    p = space_time_astar(topo, r1, [a], 1, vertex_constraints=frozenset({(a, t_arr)}))
    assert p[-1][0] > base[-1][0] and validate_timed_path(topo, p) == []
    assert all(not (n == a and t == t_arr) for t, n in p)


def test_edge_constraint_forces_wait_or_detour(topo):
    r1, a = topo.node_by_label("R1"), topo.node_by_label("A")
    base = space_time_astar(topo, r1, [a], 1)
    (t1, u), (_, v) = base[0], base[1]
    p = space_time_astar(topo, r1, [a], 1, edge_constraints=frozenset({(u, v, t1)}))
    assert not (p[0] == (0, u) and p[1][1] == v and p[1][0] == t1 + topo.travel_time(u, v)) or p[-1][0] > base[-1][0]


def test_reserved_path_is_avoided(topo):
    r1, a = topo.node_by_label("R1"), topo.node_by_label("A")
    other = space_time_astar(topo, r1, [a], 1)
    r2 = topo.node_by_label("R2")
    mine = space_time_astar(topo, r2, [topo.node_by_label("B")], 1, reserved=[other])
    assert find_conflicts(topo, {1: other, 2: mine}) == []


def test_parked_goal_unreachable_returns_none(topo):
    r1, a = topo.node_by_label("R1"), topo.node_by_label("A")
    parked = [(0, a)]
    assert space_time_astar(topo, r1, [a], 1, reserved=[parked]) is None


@pytest.mark.parametrize("sc", all_scenarios(), ids=lambda s: s.name)
def test_cbs_and_prioritized_are_conflict_free_on_topometric(topo, sc):
    c = run_pipeline(topo, sc, "cbs")
    p = run_pipeline(topo, sc, "prioritized")
    assert c.success and p.success
    for r in (c, p):
        assert find_conflicts(topo, r.paths) == []
        assert all(validate_timed_path(topo, path) == [] for path in r.paths.values())
        for rover, task in r.assignment.items():
            assert r.paths[rover][-1][1] == topo.node_by_label(task.drop)
    assert c.metrics["sum_of_costs"] <= p.metrics["sum_of_costs"]   # CBS is optimal in sum of costs


def test_demo_scenario_independent_has_conflicts_but_cbs_none(topo):
    ind, cb = run_pipeline(topo, demo_scenario(), "independent"), run_pipeline(topo, demo_scenario(), "cbs")
    assert ind.metrics["conflicts_final"] > 0 and not ind.success
    assert cb.metrics["conflicts_final"] == 0 and cb.success


def test_cbs_reports_failure_when_impossible(topo):
    # two rovers must park on the same drop station: impossible, CBS must report failure (not hang / lie)
    from ants.contracts import Task
    from ants.scenarios import Scenario
    sc = Scenario("IMPOSSIBLE", (1, 2, 3), (Task(1, "A", "X"), Task(2, "C", "X"), Task(3, "B", "Y")))
    r = run_pipeline(topo, sc, "cbs", cbs_node_limit=200, cbs_time_limit=10)
    assert not r.success and r.paths == {}


def test_grid_cbs_matches_topometric_cost_on_demo(topo, grid):
    a, b = run_pipeline(topo, demo_scenario(), "cbs"), run_pipeline(grid, demo_scenario(), "cbs", cbs_time_limit=60)
    assert b.success and find_conflicts(grid, b.paths) == []
    assert a.metrics["sum_of_costs"] == b.metrics["sum_of_costs"]
