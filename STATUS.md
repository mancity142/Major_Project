# ANTS software status (generated from real runs on 26 Oct 2026)

Labels: **Implemented-tested** = code + passing automated tests | **Simulated** = exists only inside the simulation | **Planned** = no code

| Module | Status | Evidence |
|---|---|---|
| Warehouse layout + topometric graph (27 nodes / 32 edges) + grid graph (75 / 80) | Implemented-tested | tests/test_environment.py |
| Shared path/conflict contract | Implemented-tested | tests/test_contracts.py |
| A* single-agent planner, Dijkstra | Implemented-tested | tests/test_astar_conflicts.py |
| Conflict checker (vertex, swap, corridor, parked) | Implemented-tested | tests/test_astar_conflicts.py |
| Task cost matrix + Hungarian allocation (matches brute force) | Implemented-tested | tests/test_allocation.py |
| Space-Time A* (waiting, vertex/edge constraints, reservations) | Implemented-tested | tests/test_planners.py |
| Prioritized planning | Implemented-tested | tests/test_planners.py |
| CBS (vertex + exact-move edge constraints, node/time limit, failure report) | Implemented-tested | tests/test_planners.py |
| Pygame animation, task status, live overlap check, controls | Simulated | outputs/demo_*.png, run_demo.py |
| Evaluation harness (8 scenarios x 2 maps x 3 planners) | Implemented-tested | outputs/results.csv, summary.csv |
| Overhead camera + ArUco localization | **Planned** (not implemented) | - |
| WebSocket waypoint dispatch / telemetry | **Planned** (not implemented) | - |
| ESP32 firmware, encoders, PID, ultrasonic safety | **Planned** (not implemented) | - |
| Physical rovers / physical validation | **Planned** (not implemented) | - |
| Monitoring dashboard (Next.js/Supabase) | **Planned** (optional) | - |

Automated tests: 42 passed.
