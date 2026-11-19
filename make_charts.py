"""Charts from outputs/summary.csv (needs matplotlib):  python make_charts.py"""
import csv

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from ants.environment import PROJECT_ROOT

rows = list(csv.DictReader(open(PROJECT_ROOT / "outputs" / "summary.csv")))
labels = [f"{r['planner']}\n({r['map']})" for r in rows]
colors = ["#c0504d" if r["planner"] == "independent" else "#9bbb59" if r["planner"] == "prioritized" else "#4f81bd" for r in rows]
succ = [int(r["mission_success_rate"].split("/")[0]) / int(r["mission_success_rate"].split("/")[1]) * 100 for r in rows]
fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
for a, vals, title in [(ax[0], succ, "Mission success rate (%, conflict-free schedules)"),
                       (ax[1], [float(r["mean_sum_of_costs"]) for r in rows], "Mean sum of costs (ticks)"),
                       (ax[2], [float(r["mean_plan_ms"]) for r in rows], "Mean planning time (ms, median of 5 runs)")]:
    a.bar(range(len(rows)), vals, color=colors)
    a.set_xticks(range(len(rows)))
    a.set_xticklabels(labels, fontsize=7)
    a.set_title(title, fontsize=10)
    for i, v in enumerate(vals):
        a.text(i, v, f"{v:.1f}" if v % 1 else f"{int(v)}", ha="center", va="bottom", fontsize=8)
fig.suptitle("ANTS simulation: 8 repeatable scenarios, 3 rovers (all values computed by evaluation.py)", fontsize=11)
fig.tight_layout()
fig.savefig(PROJECT_ROOT / "outputs" / "chart_summary.png", dpi=140)
print("saved outputs/chart_summary.png")
