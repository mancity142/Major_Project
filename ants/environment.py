"""Warehouse environment: loads an ASCII layout into a 2D cell grid."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

Cell = tuple[int, int]  # (row, col); row 0 is the top

WALL = "#"
FREE = "."
PICKUP_CHARS = "ABC"
DROP_CHARS = "XYZ"
ROVER_CHARS = "123"

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_LAYOUT = PROJECT_ROOT / "layouts" / "warehouse_v1.txt"


@dataclass
class Warehouse:
    name: str
    rows: tuple[str, ...]
    pickups: dict[str, Cell]
    drops: dict[str, Cell]
    rover_starts: dict[int, Cell]

    @property
    def height(self) -> int:
        return len(self.rows)

    @property
    def width(self) -> int:
        return len(self.rows[0])

    def in_bounds(self, cell: Cell) -> bool:
        r, c = cell
        return 0 <= r < self.height and 0 <= c < self.width

    def is_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and self.rows[cell[0]][cell[1]] != WALL

    def free_cells(self) -> list[Cell]:
        """All traversable cells in row-major order (deterministic)."""
        return [(r, c) for r in range(self.height) for c in range(self.width)
                if self.rows[r][c] != WALL]

    def free_neighbors(self, cell: Cell) -> list[Cell]:
        """4-connected free neighbours in fixed order: up, right, down, left."""
        r, c = cell
        candidates = [(r - 1, c), (r, c + 1), (r + 1, c), (r, c - 1)]
        return [n for n in candidates if self.is_free(n)]


def parse_layout(text: str, name: str = "layout") -> Warehouse:
    lines = [ln.rstrip() for ln in text.splitlines()
             if ln.strip() and not ln.lstrip().startswith(";")]
    if not lines:
        raise ValueError("layout is empty")
    width = len(lines[0])
    pickups: dict[str, Cell] = {}
    drops: dict[str, Cell] = {}
    rovers: dict[int, Cell] = {}
    allowed = set(WALL + FREE + PICKUP_CHARS + DROP_CHARS + ROVER_CHARS)

    for r, line in enumerate(lines):
        if len(line) != width:
            raise ValueError(f"row {r} has length {len(line)}, expected {width}")
        for c, ch in enumerate(line):
            if ch not in allowed:
                raise ValueError(f"unknown character {ch!r} at row {r}, col {c}")
            if ch in PICKUP_CHARS:
                if ch in pickups:
                    raise ValueError(f"duplicate pickup {ch}")
                pickups[ch] = (r, c)
            elif ch in DROP_CHARS:
                if ch in drops:
                    raise ValueError(f"duplicate drop {ch}")
                drops[ch] = (r, c)
            elif ch in ROVER_CHARS:
                rid = int(ch)
                if rid in rovers:
                    raise ValueError(f"duplicate rover start {ch}")
                rovers[rid] = (r, c)
    return Warehouse(name, tuple(lines), pickups, drops, rovers)


def load_layout(path: Path | str = DEFAULT_LAYOUT) -> Warehouse:
    path = Path(path)
    return parse_layout(path.read_text(encoding="utf-8"), name=path.stem)
