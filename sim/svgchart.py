"""Writes the results chart as a hand-built SVG (no plotting dependency)."""
from __future__ import annotations

from xml.sax.saxutils import escape

CRIMSON = "#E5383B"
STEEL = "#3D5A80"
STEEL_LIGHT = "#98AFCB"
GREY = "#B8BEC4"
INK = "#14171A"
MUTED = "#4A5157"
GRID = "#ECEEF0"
FONT = "'Inter','Segoe UI','Helvetica Neue',Arial,sans-serif"
MONO = "'JetBrains Mono','Cascadia Mono',Consolas,'Courier New',monospace"


def _text(x, y, s, size=12, fill=INK, anchor="start", weight="400", family=FONT):
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}" '
            f'text-anchor="{anchor}" font-weight="{weight}" font-family="{family}">{escape(s)}</text>')


def _panel(x0, y0, w, h, title, subtitle, groups, series, value_fmt):
    """Grouped vertical bars. groups: list of (label, [values per series], [notes])."""
    out = [f'<rect x="{x0}" y="{y0}" width="{w}" height="{h}" rx="8" fill="#FDFDFE" stroke="#D3D7DB"/>',
           _text(x0 + 18, y0 + 28, title, 15, INK, weight="700"),
           _text(x0 + 18, y0 + 47, subtitle, 11, MUTED, family=MONO)]
    px0, py0, pw, ph = x0 + 48, y0 + 70, w - 66, h - 130
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = py0 + ph * (1 - frac)
        out.append(f'<line x1="{px0}" y1="{y:.1f}" x2="{px0 + pw}" y2="{y:.1f}" stroke="{GRID}"/>')
        out.append(_text(px0 - 8, y + 4, f"{int(frac * 100)}%", 10, MUTED, "end", family=MONO))
    n_g, n_s = len(groups), len(series)
    gw = pw / n_g
    bw = min(26.0, gw * 0.8 / n_s)
    for gi, (label, values, notes) in enumerate(groups):
        gx = px0 + gi * gw + (gw - bw * n_s) / 2
        for si, v in enumerate(values):
            bh = ph * max(0.0, min(1.0, v))
            x = gx + si * bw
            out.append(f'<rect x="{x:.1f}" y="{py0 + ph - bh:.1f}" width="{bw - 3:.1f}" '
                       f'height="{max(bh, 1.0):.1f}" rx="2" fill="{series[si][1]}"/>')
            out.append(_text(x + (bw - 3) / 2, py0 + ph - bh - 4, value_fmt(v), 9, MUTED, "middle", family=MONO))
        out.append(_text(px0 + gi * gw + gw / 2, py0 + ph + 18, label, 11, INK, "middle", "600"))
        for ni, note in enumerate(notes):
            out.append(_text(px0 + gi * gw + gw / 2, py0 + ph + 33 + 13 * ni, note, 9, MUTED, "middle", family=MONO))
    lx = x0 + 18
    for name, color in series:
        out.append(f'<rect x="{lx}" y="{y0 + h - 22}" width="10" height="10" rx="2" fill="{color}"/>')
        out.append(_text(lx + 15, y0 + h - 13, name, 9.5, MUTED))
        lx += 22 + 5.3 * len(name)
    return out


def results_chart(path, act, idle, fa=None):
    """act: per_class results; idle: prying summary keyed by force;
    fa: optional legend notes with the measured false alarms."""
    fa = fa or {}
    pct = lambda v: f"{v * 100:.0f}"   # noqa: E731
    classes = ("stall", "linkage", "brownout", "friction")
    labels = {"stall": "Jam / blocked", "linkage": "Linkage loss", "brownout": "Brown-out",
              "friction": "Friction"}
    g1 = [(labels[c], [act[c]["telltale"]["detected"]["rate"],
                       act[c]["overcurrent"]["detected"]["rate"],
                       act[c]["fixed_peak"]["detected"]["rate"], 0.0],
           [f"AUC {act[c]['telltale']['auc']:.3f}"]) for c in classes]
    s1 = [("Telltale" + fa.get("telltale", ""), CRIMSON),
          ("Per-lock peak" + fa.get("overcurrent", ""), STEEL_LIGHT),
          ("Fixed peak" + fa.get("fixed_peak", ""), STEEL), ("Original lock", GREY)]
    g2 = []
    for key, entry in idle.items():
        med = entry["cusum"]["latency_ms"]["p50"]
        g2.append((key.replace("N", " N"), [entry["cusum"]["detected"]["rate"],
                                            entry["debounced"]["detected"]["rate"],
                                            entry["instant"]["detected"]["rate"]],
                   [f"{med:.0f} ms" if med is not None else "-"]))
    s2 = [("Telltale CUSUM" + fa.get("cusum", ""), CRIMSON),
          ("Debounced" + fa.get("debounced", ""), STEEL),
          ("Instant threshold" + fa.get("instant", ""), GREY)]
    w, h = 1200, 470
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" height="{h}" '
             f'role="img" aria-labelledby="t d">',
             '<title id="t">TELLTALE simulated results</title>',
             '<desc id="d">Detection rates for actuation faults and for prying while locked, '
             'from the simulated evaluation (results/eval.json).</desc>',
             f'<rect width="{w}" height="{h}" rx="12" fill="#F7F8F9"/>']
    parts += _panel(16, 16, 576, h - 32, "Faults caught during actuation",
                    "SIMULATED · threshold set for <= 1 false alarm / 1000 actuations", g1, s1, pct)
    parts += _panel(608, 16, 576, h - 32, "Prying caught while locked",
                    "SIMULATED · peak force on the bolt · median Telltale latency", g2, s2, pct)
    parts.append("</svg>")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(parts) + "\n")


def chart_from_results(results: dict, path) -> None:
    """Draw the chart from a results/eval.json dictionary."""
    act, idle = results["actuation"], results["idle"]
    h = act["healthy"]
    per_k = lambda r: f" ({r['rate'] * 1000:.1f} FA/1k)"   # noqa: E731
    ih = idle["healthy"]
    fa = {"telltale": per_k(h["telltale_false_alarms"]),
          "overcurrent": per_k(h["overcurrent_false_alarms"]),
          "fixed_peak": per_k(h["fixed_peak_false_alarms"]),
          "cusum": f" ({ih['cusum_alarms']} FA/{ih['hours']:.0f} h)",
          "debounced": f" ({ih['debounced_units_with_alarm']} locks w/ FA)",
          "instant": f" ({ih['instant_alarms']} FA/{ih['hours']:.0f} h)"}
    results_chart(path, act["per_class"], idle["prying"], fa)


if __name__ == "__main__":
    import json
    import sys
    from pathlib import Path
    root = Path(__file__).resolve().parent.parent
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "results" / "eval.json"
    dst = Path(sys.argv[2]) if len(sys.argv) > 2 else root / "docs" / "assets" / "results.svg"
    chart_from_results(json.loads(src.read_text(encoding="utf-8")), dst)
    print(f"wrote {dst}")
