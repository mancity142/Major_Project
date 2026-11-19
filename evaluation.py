"""Repeatable evaluation:  python evaluation.py
Runs every scenario on both maps with every planner and writes outputs/results.csv and outputs/summary.csv.
Planning/allocation times are the MEDIAN of REPEAT runs on this machine (so they vary by computer).
"""
import csv
import statistics

from ants.environment import PROJECT_ROOT, load_layout
from ants.pipeline import PLANNERS, run_pipeline
from ants.scenarios import all_scenarios
from ants.world_model import build_grid_graph, build_topometric_graph

REPEAT = 5
FIELDS = ["scenario", "map", "planner", "success", "tasks_assigned", "tasks_pending", "alloc_ms", "plan_ms", "makespan",
          "sum_of_costs", "total_path_length", "path_efficiency", "conflicts_initial", "conflicts_final", "cbs_nodes", "note"]


def main():
    wh = load_layout()
    graphs = {"topometric": build_topometric_graph(wh), "grid": build_grid_graph(wh)}
    rows = []
    for sc in all_scenarios():
        for gname, g in graphs.items():
            for pl in PLANNERS:
                r = run_pipeline(g, sc, pl, repeat=REPEAT, cbs_node_limit=3000, cbs_time_limit=60)
                row = {"scenario": sc.name, "map": gname, "planner": pl, "note": "" if r.success else r.reason}
                row.update({k: r.metrics.get(k, "") for k in FIELDS if k in r.metrics})
                rows.append(row)
    out = PROJECT_ROOT / "outputs"
    out.mkdir(exist_ok=True)
    with open(out / "results.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)

    def mean(vals):
        vals = [float(v) for v in vals if v not in ("", None)]
        return round(statistics.mean(vals), 2) if vals else ""

    summary = []
    for gname in graphs:
        for pl in PLANNERS:
            rs = [r for r in rows if r["map"] == gname and r["planner"] == pl]
            summary.append({"map": gname, "planner": pl, "runs": len(rs),
                            "mission_success_rate": f"{sum(int(r['success']) for r in rs)}/{len(rs)}",
                            "mean_conflicts_initial": mean(r["conflicts_initial"] for r in rs),
                            "mean_conflicts_unresolved": mean(r["conflicts_final"] for r in rs),
                            "mean_makespan": mean(r["makespan"] for r in rs),
                            "mean_sum_of_costs": mean(r["sum_of_costs"] for r in rs),
                            "mean_path_length": mean(r["total_path_length"] for r in rs),
                            "mean_path_efficiency": mean(r["path_efficiency"] for r in rs),
                            "mean_alloc_ms": mean(r["alloc_ms"] for r in rs),
                            "mean_plan_ms": mean(r["plan_ms"] for r in rs),
                            "mean_cbs_nodes": mean(r["cbs_nodes"] for r in rs)})
    with open(out / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0]))
        w.writeheader()
        w.writerows(summary)

    print(f"{len(rows)} runs written to {out / 'results.csv'}\n")
    cols = ["map", "planner", "mission_success_rate", "mean_conflicts_initial", "mean_conflicts_unresolved",
            "mean_makespan", "mean_sum_of_costs", "mean_plan_ms", "mean_cbs_nodes"]
    print(" | ".join(f"{c[:14]:>14}" for c in cols))
    for s in summary:
        print(" | ".join(f"{str(s[c]):>14}" for c in cols))
    bad = [r for r in rows if r["note"]]
    if bad:
        print("\nRuns that did not succeed:")
        for r in bad:
            print(f"  {r['scenario']:12s} {r['map']:10s} {r['planner']:12s} -> {r['note']}")


if __name__ == "__main__":
    main()
