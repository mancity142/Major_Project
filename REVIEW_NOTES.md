# Demo script, results summary, viva Q&A

## 2-minute demo script
1. `python run_demo.py` (scenario DEMO, CBS on the topometric map). Say: "This is a simulation; rovers are not physical."
2. Point at the Hungarian table: rows = tasks, columns = rovers, cost in ticks, `*` = chosen assignment.
3. Press **1** (independent A*): rovers 1 and 2 meet head-on on the top lane, a red ring appears, "conflicts in schedule: 3".
4. Press **3** (CBS): same tasks, no overlap, "conflicts in schedule: 0", CBS nodes 2.
5. Press **M** (grid map): same result on the dense map; compare planning time / node count in `summary.csv`.
6. Press **N** to show the other scenarios, incl. OVERLOAD (4 tasks, 3 rovers: one stays PENDING).
7. Show `outputs/chart_summary.png` and say what is NOT done: vision, WebSockets, ESP32/PID, physical tests.

## Real results (outputs/summary.csv; 8 scenarios, 3 rovers)
| map | planner | success | unresolved conflicts | mean makespan | mean sum of costs | mean plan ms | mean CBS nodes |
|---|---|---|---|---|---|---|---|
| topometric | independent A* | 3/8 | 1.75 | 27.25 | 74.5 | 0.1 | - |
| topometric | prioritized | 8/8 | 0 | 27.25 | 74.88 | 0.64 | - |
| topometric | CBS | 8/8 | 0 | 27.38 | 74.88 | 1.8 | 0.88 |
| grid | independent A* | 3/8 | 3.75 | 27.25 | 74.5 | 0.21 | - |
| grid | prioritized | 8/8 | 0 | 27.25 | 75.25 | 1.31 | - |
| grid | CBS | 8/8 | 0 | 27.38 | 74.88 | 4.18 | 1.0 |

Read honestly: independent A* leaves conflicts in 5 of 8 scenarios; prioritized and CBS fix all of them. CBS is cheapest in
sum of costs on the grid (74.88 vs 75.25); on the topometric map it ties prioritized on this small set. Topometric CBS
planned about 2.3x faster than grid CBS here. Timings are machine-dependent (median of 5 runs, milliseconds), the
conflict counts on the two maps are not directly comparable (the grid splits a lane into many cells), and 3 rovers /
8 scenarios is a small study, so these numbers show feasibility, not a general performance claim.

## Likely viva questions
* **Why a topometric graph?** It keeps only decision points (27 nodes vs 75 grid cells) and turns lanes into weighted
  edges; edge lengths sum to the same 80 cells, so no corridor is lost (tested).
* **Cost matrix?** C[task][rover] = ticks rover->pickup + 1 + pickup->drop + 1, via Dijkstra, other rovers ignored.
  Hungarian minimises the total; rectangular matrices leave tasks PENDING or rovers idle.
* **Space-Time A* state?** (node, tick, stage). Moves take edge-length ticks, waiting is allowed at nodes, constraints are
  (node, tick) and (edge, departure tick). Heuristic = exact remaining travel time, so it is optimal under the constraints.
* **CBS?** Plan each rover alone, find the earliest conflict, branch: forbid it for rover A or rover B, replan only that
  rover, best-first on sum of costs. Stops at a conflict-free set or reports failure (node/time limit).
* **Is it optimal/complete?** Optimal in sum of end ticks within the time horizon (200 ticks) because the low level is optimal
  and constraints are exact; we do not claim more, and we do not claim a new MAPF algorithm (scope slide).
* **Difference from the base paper?** The paper uses continuous-time movement intervals; ours is discrete ticks.
* **How do you know paths are collision-free?** `find_conflicts` re-checks every output; tests cover head-on corridor,
  swap, same-node, parked rover and following; the animation additionally flags overlapping drawn positions.
* **What is physically validated?** Nothing yet. Vision, WebSockets, ESP32/PID are planned for the next phase.

## Known limitations
* Rovers park on the drop node, so two tasks cannot share a drop station in one round (CBS then reports failure; tested).
* Fixed 1 cell/tick speed, no acceleration, turning time or rover size model; edge occupancy is exclusive per lane.
* One planning round (no continuous task arrival / replanning loop yet).
* Small scenario set; timings are machine-dependent.
