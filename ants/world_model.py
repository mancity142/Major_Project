"""World model: one Graph class, two builders (topometric = sparse, grid = dense baseline).

Time convention: robot speed is 1 cell per tick, so travel_time(u, v) == edge length in cells == ticks.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from ants.environment import Cell, Warehouse


@dataclass(frozen=True)
class Node:
    id: int
    cell: Cell
    kind: str  # pickup, drop, rover_start, junction, corner, dead_end, cell
    label: str = ""  # "A", "X", "R1", or ""


@dataclass(frozen=True)
class Edge:
    u: int
    v: int
    cells: tuple[Cell, ...]  # grid cells from u to v, endpoints included

    @property
    def length(self) -> int:
        return len(self.cells) - 1


class Graph:
    def __init__(self, name: str):
        self.name = name
        self.nodes: dict[int, Node] = {}
        self.adj: dict[int, dict[int, Edge]] = {}
        self.cell_to_node: dict[Cell, int] = {}
        self.label_to_node: dict[str, int] = {}

    def add_node(self, node: Node) -> None:
        self.nodes[node.id] = node
        self.adj.setdefault(node.id, {})
        self.cell_to_node[node.cell] = node.id
        if node.label:
            self.label_to_node[node.label] = node.id

    def add_edge(self, u: int, v: int, cells) -> None:
        """Undirected edge. If a parallel edge exists, the shorter one is kept."""
        if u == v:
            return
        cells = list(cells)
        existing = self.adj[u].get(v)
        if existing is not None and existing.length <= len(cells) - 1:
            return
        self.adj[u][v] = Edge(u, v, tuple(cells))
        self.adj[v][u] = Edge(v, u, tuple(reversed(cells)))

    def neighbors(self, u: int) -> list[int]:
        return sorted(self.adj[u])

    def edge(self, u: int, v: int) -> Edge:
        return self.adj[u][v]

    def travel_time(self, u: int, v: int) -> int:
        return self.adj[u][v].length

    def node_by_label(self, label: str) -> int:
        return self.label_to_node[label]

    @property
    def edges(self) -> list[Edge]:
        """Each undirected edge once (u < v)."""
        return [e for u in sorted(self.adj) for v, e in sorted(self.adj[u].items()) if u < v]

    @property
    def num_nodes(self) -> int:
        return len(self.nodes)

    @property
    def num_edges(self) -> int:
        return len(self.edges)

    @property
    def total_edge_length(self) -> int:
        return sum(e.length for e in self.edges)

    def is_connected(self) -> bool:
        if not self.nodes:
            return True
        start = next(iter(self.nodes))
        seen = {start}
        queue = deque([start])
        while queue:
            u = queue.popleft()
            for v in self.adj[u]:
                if v not in seen:
                    seen.add(v)
                    queue.append(v)
        return len(seen) == len(self.nodes)


def _special_cells(wh: Warehouse) -> dict[Cell, tuple[str, str]]:
    special: dict[Cell, tuple[str, str]] = {}
    for label, cell in wh.pickups.items():
        special[cell] = ("pickup", label)
    for label, cell in wh.drops.items():
        special[cell] = ("drop", label)
    for rid, cell in wh.rover_starts.items():
        special[cell] = ("rover_start", f"R{rid}")
    return special


def _is_straight(cell: Cell, nbrs: list[Cell]) -> bool:
    a, b = nbrs
    return a[0] + b[0] == 2 * cell[0] and a[1] + b[1] == 2 * cell[1]


def build_topometric_graph(wh: Warehouse) -> Graph:
    """Sparse map: nodes = stations, rover starts, junctions (degree>=3), corners, dead ends.
    Edges = corridors between nodes, weighted by length. Straight lane cells are NOT nodes."""
    g = Graph("topometric")
    special = _special_cells(wh)
    for cell in wh.free_cells():
        nbrs = wh.free_neighbors(cell)
        if cell in special:
            kind, label = special[cell]
        elif len(nbrs) == 1:
            kind, label = "dead_end", ""
        elif len(nbrs) >= 3:
            kind, label = "junction", ""
        elif len(nbrs) == 2 and not _is_straight(cell, nbrs):
            kind, label = "corner", ""
        else:
            continue
        g.add_node(Node(len(g.nodes), cell, kind, label))

    max_steps = len(wh.free_cells()) + 1
    for u, node in list(g.nodes.items()):
        for first in wh.free_neighbors(node.cell):
            path = [node.cell, first]
            prev, cur = node.cell, first
            while cur not in g.cell_to_node:
                nxt = [c for c in wh.free_neighbors(cur) if c != prev]
                prev, cur = cur, nxt[0]
                path.append(cur)
                if len(path) > max_steps:
                    raise RuntimeError("corridor walk did not terminate")
            g.add_edge(u, g.cell_to_node[cur], path)
    return g


def build_grid_graph(wh: Warehouse) -> Graph:
    """Dense baseline map: every free cell is a node, every edge has length 1."""
    g = Graph("grid")
    special = _special_cells(wh)
    for cell in wh.free_cells():
        kind, label = special.get(cell, ("cell", ""))
        g.add_node(Node(len(g.nodes), cell, kind, label))
    for cell in wh.free_cells():
        u = g.cell_to_node[cell]
        for nb in wh.free_neighbors(cell):
            v = g.cell_to_node[nb]
            if u < v:
                g.add_edge(u, v, [cell, nb])
    return g
