import itertools
import random

from ants.allocation import UNREACHABLE, assign, build_cost_matrix
from ants.contracts import Task
from ants.environment import load_layout
from ants.scenarios import demo_scenario
from ants.world_model import build_topometric_graph


def _brute_force(matrix):
    n_t, n_r = len(matrix), len(matrix[0])
    k = min(n_t, n_r)
    best = None
    for rows in itertools.combinations(range(n_t), k):
        for cols in itertools.permutations(range(n_r), k):
            c = sum(matrix[i][j] for i, j in zip(rows, cols))
            best = c if best is None or c < best else best
    return best


def test_hungarian_matches_brute_force_random():
    rng = random.Random(7)
    for n_t, n_r in [(3, 3), (4, 3), (2, 3), (1, 3), (3, 4)]:
        for _ in range(20):
            m = [[rng.randint(1, 50) for _ in range(n_r)] for _ in range(n_t)]
            tasks = [Task(i, "A", "X") for i in range(n_t)]
            assignment, unassigned, total = assign(m, list(range(n_r)), tasks)
            assert total == _brute_force(m)
            assert len(assignment) == min(n_t, n_r) and len(unassigned) == n_t - len(assignment)


def test_cost_matrix_on_demo_scenario():
    g = build_topometric_graph(load_layout())
    sc = demo_scenario()
    starts = {r: g.node_by_label(f"R{r}") for r in sc.rovers}
    m = build_cost_matrix(g, starts, sc.tasks, 1)
    assert m == [[35, 27, 29], [27, 31, 37], [27, 23, 29]]
    assignment, unassigned, total = assign(m, [1, 2, 3], list(sc.tasks))
    assert unassigned == [] and total == 29 + 27 + 23
    assert {r: (t.pickup, t.drop) for r, t in assignment.items()} == {3: ("C", "X"), 1: ("A", "Z"), 2: ("B", "Y")}


def test_unreachable_pairs_never_assigned():
    m = [[UNREACHABLE, 5], [UNREACHABLE, UNREACHABLE]]
    assignment, unassigned, _ = assign(m, [1, 2], [Task(1, "A", "X"), Task(2, "B", "Y")])
    assert list(assignment) == [2] and len(unassigned) == 1
