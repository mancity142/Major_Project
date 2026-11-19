"""Repeatable evaluation scenarios.  Same layout, same rover starts, fixed task sets (no randomness)."""
from __future__ import annotations

import itertools
from dataclasses import dataclass

from ants.contracts import Task


@dataclass(frozen=True)
class Scenario:
    name: str
    rovers: tuple[int, ...]
    tasks: tuple[Task, ...]
    description: str = ""


ROVERS = (1, 2, 3)


def _s(name, pairs, desc=""):
    return Scenario(name, ROVERS, tuple(Task(i + 1, p, d) for i, (p, d) in enumerate(pairs)), desc)


def demo_scenario() -> Scenario:
    """Main demo: the west and east rovers' tasks cross the whole warehouse."""
    return _s("DEMO", [("C", "X"), ("A", "Z"), ("B", "Y")],
              "3 tasks, 3 rovers; C->X and A->Z cross each other in the shared lanes")


def all_scenarios() -> list[Scenario]:
    """DEMO + the 6 permutations of pickups A,B,C onto drops X,Y,Z + one overload case (4 tasks, 3 rovers)."""
    out = [demo_scenario()]
    for perm in itertools.permutations("XYZ"):
        pairs = list(zip("ABC", perm))
        out.append(_s("P-" + "".join(f"{p}{d}" for p, d in pairs), pairs))
    out.append(_s("OVERLOAD", [("A", "Y"), ("A", "Z"), ("B", "X"), ("B", "Z")],
                  "4 tasks, 3 rovers: one task stays PENDING"))
    return out
