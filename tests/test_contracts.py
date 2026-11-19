from ants.contracts import state_at, validate_timed_path
from ants.environment import load_layout
from ants.world_model import build_topometric_graph


def _setup():
    g = build_topometric_graph(load_layout())
    return g, g.node_by_label("R1"), g.cell_to_node[(5, 5)]


def test_valid_move_and_wait():
    g, r1, j = _setup()
    assert validate_timed_path(g, [(0, r1), (4, j), (6, j)]) == []


def test_wrong_duration_and_non_adjacent_rejected():
    g, r1, j = _setup()
    assert validate_timed_path(g, [(0, r1), (3, j)])
    assert validate_timed_path(g, [(0, r1), (4, g.node_by_label("R3"))])


def test_state_at():
    g, r1, j = _setup()
    path = [(0, r1), (4, j), (6, j)]
    assert state_at(path, 0) == ("node", r1)
    assert state_at(path, 2) == ("edge", r1, j, 0.5)
    assert state_at(path, 4) == ("node", j)
    assert state_at(path, 5) == ("node", j)
    assert state_at(path, 99) == ("node", j)
