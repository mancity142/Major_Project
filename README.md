# ANTS - software simulation (Phase I)

Simulated warehouse with 3 rovers: topometric map -> Hungarian allocation -> A* / prioritized Space-Time A* / CBS
-> conflict checker -> animated schedule -> metrics.  **Everything here is simulated.** No camera, ESP32, PID or
Wi-Fi code exists in this repository.

## Setup (Windows PowerShell)
```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python -m pytest -q          # 42 tests
python show_environment.py   # text view of the map + graphs
python run_demo.py           # interactive animated demo
python evaluation.py         # writes outputs/results.csv + summary.csv
python make_charts.py        # outputs/chart_summary.png
python make_screenshots.py   # outputs/demo_*.png
```
If script activation is blocked: `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass`.
Use `pygame-ce` (do not install plain `pygame` next to it).

## Demo keys
SPACE pause | LEFT/RIGHT step one tick | +/- speed | R restart | 1 independent A* | 2 prioritized | 3 CBS |
N next scenario | M topometric/grid map | G graph overlay | S screenshot | ESC quit

## Modules
| File | Role |
|---|---|
| `ants/environment.py`, `layouts/warehouse_v1.txt` | ASCII warehouse -> cells |
| `ants/world_model.py` | topometric (sparse) graph + grid (dense baseline) graph |
| `ants/contracts.py` | timed-path format, conflict rules (the shared definition) |
| `ants/astar.py` | single-agent A*, Dijkstra, independent plans |
| `ants/conflicts.py` | conflict checker (vertex / swap / corridor) |
| `ants/allocation.py` | cost matrix + Hungarian (scipy) |
| `ants/spacetime.py` | Space-Time A* (waiting, constraints, reservations, load/unload dwell) |
| `ants/prioritized.py`, `ants/cbs.py` | prioritized planning, Conflict-Based Search |
| `ants/pipeline.py`, `ants/scenarios.py`, `evaluation.py` | pipeline, 8 fixed scenarios, metrics |
| `ants/viz.py`, `run_demo.py` | Pygame animation drawn only from the planned schedules |

## Model (state this in the viva)
* Discrete ticks; speed 1 cell/tick; an edge takes `length` ticks; waiting only at nodes; load/unload = 1 tick.
* Conflicts: same node at same tick (incl. parked rovers); opposite-direction use of a lane with overlapping ticks.
* A rover parks on its drop node after finishing, so two tasks cannot drop at the same station in one round.
* Time-indexed discrete model; NOT the continuous-time interval CBS of the base paper.
