# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib>=3.8", "pandas>=2.0"]
# ///
"""Turn results/*.csv from `cargo run --release --bin bench` into figures.

    uv run scripts/plot.py [results_dir] [out_dir]
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

RESULTS = Path(sys.argv[1] if len(sys.argv) > 1 else "results")
OUT = Path(sys.argv[2] if len(sys.argv) > 2 else "docs/img")

# Validated categorical slots 1-2 (light surface), plus text and grid inks.
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
COLORS = {"speculative": "#2a78d6", "centralized": "#eb6834"}
LABELS = {"speculative": "Speculative (trees on robots)", "centralized": "Centralized (edge streams commands)"}
MODES = ["speculative", "centralized"]

plt.rcParams.update(
    {
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
        "axes.edgecolor": GRID,
        "axes.labelcolor": INK_2,
        "axes.titlecolor": INK,
        "axes.titleweight": "bold",
        "axes.titlesize": 12,
        "axes.titlelocation": "left",
        "xtick.color": INK_2,
        "ytick.color": INK_2,
        "axes.grid": True,
        "axes.axisbelow": True,
        "grid.color": GRID,
        "grid.linewidth": 0.8,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.spines.left": False,
        "font.size": 10,
        "legend.frameon": False,
        "legend.labelcolor": INK,
    }
)


def summarize(df: pd.DataFrame, by: list[str]) -> pd.DataFrame:
    """Success rate, and mean time with a 95% CI over successful runs."""
    def agg(g: pd.DataFrame) -> pd.Series:
        ok = g[g["reached"]]["time_s"]
        n = len(ok)
        ci = 1.96 * ok.std(ddof=1) / n**0.5 if n > 1 else 0.0
        return pd.Series(
            {"success": g["reached"].mean() * 100, "time": ok.mean() if n else float("nan"), "ci": ci}
        )

    return df.groupby(by).apply(agg, include_groups=False).reset_index()


def bottom_legend(fig, ax):
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside lower center", ncols=2)


def line_panel(ax, s: pd.DataFrame, x: str, y: str, ci: str | None, label_last: bool = True):
    for mode in MODES:
        m = s[s["control"] == mode].sort_values(x)
        c = COLORS[mode]
        if ci:
            ax.fill_between(m[x], m[y] - m[ci], m[y] + m[ci], color=c, alpha=0.15, linewidth=0)
        ax.plot(m[x], m[y], color=c, linewidth=2, marker="o", markersize=7,
                markeredgecolor=SURFACE, markeredgewidth=2, label=LABELS[mode])
        if label_last:
            last = m.dropna(subset=[y]).iloc[-1]
            # Nudge apart so the two end labels never sit on top of each other.
            dy = 7 if mode == "centralized" else -7
            ax.annotate(mode, (last[x], last[y]), xytext=(8, dy), textcoords="offset points",
                        va="center", color=INK, fontsize=9)


def latency_figure(sweeps: pd.DataFrame):
    s = summarize(sweeps[sweeps["sweep"] == "latency_ms"], ["value", "control"])
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4.4), layout="constrained")
    line_panel(a, s, "value", "time", "ci")
    a.set_title("Time to deliver vs. network latency")
    a.set_xlabel("One-way latency (ms), 10% packet loss")
    a.set_ylabel("Time to goal (s), successful runs")
    a.set_ylim(bottom=0)
    line_panel(b, s, "value", "success", None)
    b.set_title("Delivery success rate")
    b.set_xlabel("One-way latency (ms), 10% packet loss")
    b.set_ylabel("Runs reaching the goal (%)")
    b.set_ylim(-5, 110)
    a.margins(x=0.12)
    b.margins(x=0.12)
    bottom_legend(fig, a)
    fig.savefig(OUT / "latency.png", dpi=160)


def outage_figure(sweeps: pd.DataFrame):
    s = summarize(sweeps[sweeps["sweep"] == "outage_s"], ["value", "control"])
    fig, ax = plt.subplots(figsize=(6.5, 4.4), layout="constrained")
    line_panel(ax, s, "value", "time", "ci")
    ax.set_title("Time to deliver vs. edge outage length")
    ax.set_xlabel("Edge unreachable from t = 20 s for (s)")
    ax.set_ylabel("Time to goal (s)")
    ax.set_ylim(bottom=0)
    ax.margins(x=0.15)
    bottom_legend(fig, ax)
    fig.savefig(OUT / "outage.png", dpi=160)


def bandwidth_figure(bench: pd.DataFrame):
    s = bench.groupby(["condition", "control"])["bytes_per_robot_s"].mean().unstack()
    order = list(dict.fromkeys(bench["condition"]))
    s = s.loc[order[::-1]]
    fig, ax = plt.subplots(figsize=(7.5, 4.6), layout="constrained")
    h = 0.36
    ys = range(len(s))
    for k, mode in enumerate(MODES):
        offs = [y + (h / 2 + 0.02) * (1 if k == 0 else -1) for y in ys]
        bars = ax.barh(offs, s[mode], height=h, color=COLORS[mode], label=LABELS[mode])
        for bar, v in zip(bars, s[mode]):
            ax.text(bar.get_width() + 6, bar.get_y() + h / 2, f"{v:.0f}", va="center",
                    color=INK_2, fontsize=8.5)
    ax.set_yticks(list(ys), s.index)
    ax.grid(axis="y", visible=False)
    ax.set_xlabel("Robot radio traffic (bytes per robot per second, both directions)")
    ax.set_title("Radio bandwidth by condition")
    ax.margins(x=0.08)
    bottom_legend(fig, ax)
    fig.savefig(OUT / "bandwidth.png", dpi=160)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    bench = pd.read_csv(RESULTS / "bench.csv")
    sweeps = pd.read_csv(RESULTS / "sweeps.csv")
    latency_figure(sweeps)
    outage_figure(sweeps)
    bandwidth_figure(bench)
    print(f"wrote {OUT}/latency.png, outage.png, bandwidth.png")


if __name__ == "__main__":
    main()
