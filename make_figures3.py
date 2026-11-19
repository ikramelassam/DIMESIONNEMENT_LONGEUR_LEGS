"""Figures du rapport v3 (coherence simulateur/optimiseur, lecture inverse). Usage : python make_figures3.py <dossier>"""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
A = os.path.join(HERE, "audit")
RED, GREEN, ORANGE, BLUE, GREY = "#e0191b", "#0a9a1e", "#e8890c", "#1a1aa6", "#999999"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "mathtext.fontset": "cm", "axes.spines.top": False, "axes.spines.right": False})
refs = json.load(open(os.path.join(HERE, "sim", "ref_fronts.json")))
def load_exp():
    E = {}
    for part in ("A", "B", "C", "E"):
        f = os.path.join(A, f"experiences_{part}.json")
        if os.path.exists(f):
            d = json.load(open(f)); E.update({k: v for k, v in d.items() if k != "reference" or "reference" not in E})
    return E
nz = lambda v: np.nan if v is None else v

def box(ax, x, y, w, h, t, fc="#f4f4f2", ec="#555", fs=8.3, bold=False):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.01,rounding_size=0.012", fc=fc, ec=ec, lw=1))
    ax.text(x + w / 2, y + h / 2, t, ha="center", va="center", fontsize=fs, weight="bold" if bold else None, linespacing=1.25)
def arr(ax, p, q, c="#444", lw=1.3, style="-|>"):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle=style, mutation_scale=10, color=c, lw=lw))

def fig_archi():
    fig, ax = plt.subplots(figsize=(7.4, 5.0)); ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
    box(ax, 0.33, 0.88, 0.34, 0.09, "config.yaml\n(entrées : géométrie, masses, servos, marges)", "#fff3d6", "#b8860b", bold=True)
    ax.text(0.02, 0.80, "OPTIMISATION (direct)", ha="left", fontsize=8.8, weight="bold", color=BLUE)
    ax.text(0.98, 0.80, "SIMULATEUR (inverse)", ha="right", fontsize=8.8, weight="bold", color=RED)
    box(ax, 0.02, 0.60, 0.30, 0.15, "gr2/ (Python, numpy)\nparams · kinematics · gait\nstatics · evaluate", "#e6eefa", BLUE)
    box(ax, 0.68, 0.60, 0.30, 0.15, "sim/model.js (JavaScript)\nmême modèle, même ordre\nde calcul, mode « exact »", "#fbe6e6", RED)
    box(ax, 0.02, 0.38, 0.30, 0.15, "gr2/pipeline.py\nSobol → filtre → raffinement\n→ Pareto 3D → Monte-Carlo\n→ coude · ε-contrainte", "#e6eefa", BLUE)
    box(ax, 0.68, 0.38, 0.30, 0.15, "sim/opt.js + compare.js\nport du pipeline ; contraintes,\nobjectifs, écart à l'optimum", "#fbe6e6", RED)
    box(ax, 0.02, 0.14, 0.30, 0.13, "results_exact/\nCSV : admissibles, Pareto,\nrobuste, résumé", "#eaf5ea", GREEN)
    box(ax, 0.68, 0.14, 0.30, 0.13, "sim/gr2_simulateur.html\ncurseurs → contraintes,\nf₁ f₂ f₃, comparaison", "#fbe6e6", RED)
    box(ax, 0.38, 0.38, 0.24, 0.15, "serve.py\n(API locale :\njobs Python)", "#eee", "#555")
    box(ax, 0.36, 0.14, 0.28, 0.13, "sim/ref_fronts.json\nfronts de référence\n(extraits des CSV)", "#eaf5ea", GREEN)
    arr(ax, (0.42, 0.88), (0.27, 0.75)); arr(ax, (0.58, 0.88), (0.73, 0.75))
    arr(ax, (0.17, 0.60), (0.17, 0.53)); arr(ax, (0.83, 0.60), (0.83, 0.53)); arr(ax, (0.17, 0.38), (0.17, 0.27)); arr(ax, (0.83, 0.38), (0.83, 0.27), style="<|-|>")
    arr(ax, (0.32, 0.455), (0.38, 0.455), style="<|-|>"); arr(ax, (0.62, 0.455), (0.68, 0.455), style="<|-|>")
    arr(ax, (0.32, 0.20), (0.36, 0.20)); arr(ax, (0.64, 0.20), (0.68, 0.20))
    ax.annotate("", xy=(0.68, 0.675), xytext=(0.32, 0.675), arrowprops=dict(arrowstyle="<|-|>", color=ORANGE, lw=1.6, ls="--"))
    ax.text(0.5, 0.70, "audit : mêmes valeurs ?", ha="center", fontsize=8.5, color=ORANGE, weight="bold")
    ax.text(0.5, 0.045, "Le simulateur calcule f(x) et g(x) pour des variables x données ; l'optimiseur cherche les x qui minimisent f sous g(x) ≤ 0.\nLe simulateur se place ensuite par rapport au front trouvé par l'optimiseur.", ha="center", fontsize=8.3, color="#333")
    fig.savefig(os.path.join(OUT, "archi.png"), dpi=200, bbox_inches="tight", facecolor="white"); plt.close(fig)

def fig_audit():
    P = json.load(open(os.path.join(A, "audit_pairs.json")))
    f1p, f1j, f2p, f2j, f3p, f3j = [np.array([q[i] for q in P], float) for i in (2, 3, 4, 5, 6, 7)]
    fig, axs = plt.subplots(1, 3, figsize=(7.6, 2.8))
    for ax, (a, b, lab, c) in zip(axs[:2], [(f2p, f2j, "$f_2$ NESM (mm)", BLUE), (f3p, f3j, "$f_3$ couple $C_2$ (N·m)", RED)]):
        ax.scatter(a, b, s=7, c=c, alpha=.5, lw=0); lo, hi = min(a.min(), b.min()), max(a.max(), b.max()); ax.plot([lo, hi], [lo, hi], c="#444", lw=.8)
        ax.set_xlabel("Python"); ax.set_ylabel("simulateur"); ax.set_title(lab, fontsize=9)
    err = np.concatenate([np.abs(f1p - f1j) / np.maximum(1, np.abs(f1p)), np.abs(f2p - f2j) / np.maximum(1, np.abs(f2p)), np.abs(f3p - f3j) / np.maximum(1, np.abs(f3p))])
    e = np.log10(np.maximum(err, 1e-17)); axs[2].hist(e, bins=np.arange(-17.5, -5.4, 1), color=GREEN, ec="white"); axs[2].set_xlabel("log$_{10}$ de l'écart relatif"); axs[2].set_ylabel("nombre de valeurs"); axs[2].set_title("écart $f_1$, $f_2$, $f_3$", fontsize=9)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "audit.png"), dpi=200, facecolor="white"); plt.close(fig)

def fig_oat():
    E = load_exp(); ref = E["reference"]; names = {"L2": "$L_2$ (mm)", "L3": "$L_3$ (mm)", "alpha": r"$\alpha$ (°)", "beta": r"$\beta$ (°)", "xi": r"$\xi$ (°)"}
    fig, axs = plt.subplots(2, 5, figsize=(9.6, 4.6), sharex="col")
    for j, v in enumerate(names):
        rows = sorted([r for r in E["A"] if r["var"] == v] + [dict(ref, value=ref["x"][v], bad=[], adm=True)], key=lambda r: r["value"])
        xs = [r["value"] for r in rows]
        for i, (key, c, lab) in enumerate([("f2", BLUE, "$f_2$ NESM (mm)"), ("f3", RED, "$f_3$ couple $C_2$ (N·m)")]):
            ax = axs[i, j]; y = [nz(r[key]) for r in rows]
            ax.plot(xs, y, c=c, lw=1, alpha=.6)
            for r, yy in zip(rows, y):
                if yy == yy:
                    ax.scatter([r["value"]], [yy], s=26, facecolor=c if r["adm"] else "white", edgecolor=c, lw=1.2, zorder=3)
                else:
                    ax.scatter([r["value"]], [ax.get_ylim()[0]], marker="x", c="k", s=22, zorder=3)
            ax.axvline(ref["x"][v], c="#888", ls=":", lw=1)
            if i == 1: ax.set_xlabel(names[v])
            if j == 0: ax.set_ylabel(lab)
            if i == 0:
                for r in rows:
                    ids = sorted({b["id"] for b in r["bad"] if b["id"] in ("G5", "G3", "G6", "G7", "PH")}, key=str)
                    if ids and not r["adm"]: ax.annotate("\n".join(ids), (r["value"], 1.0), xycoords=("data", "axes fraction"), ha="center", va="top", fontsize=6.3, color="#a00")
    fig.text(0.5, 0.995, "disque plein : design admissible ; disque vide : une contrainte dure est violée (valeur indicative) ; × : objectif indéfini ; texte rouge : contrainte cassée", ha="center", va="top", fontsize=7.6, color="#333")
    fig.tight_layout(rect=(0, 0, 1, .97)); fig.savefig(os.path.join(OUT, "oat.png"), dpi=200, facecolor="white"); plt.close(fig)

def fig_plan():
    E = load_exp(); R = refs["dedans_S2"]
    fig, ax = plt.subplots(figsize=(6.6, 4.4))
    fr = R["front"]; ax.scatter([q["f2_nesm"] for q in fr], [q["f3_couple_C2"] for q in fr], s=[12 + 60 * (q["f1_masse_pattes"] - 1.0) / .5 for q in fr], c="#bbb", ec="#777", lw=.5, label="front de Pareto 3D (taille = masse $f_1$)")
    col = {"L2": "#e0191b", "L3": "#0a9a1e", "alpha": "#e8890c", "beta": "#1a1aa6", "xi": "#8b3fa8"}
    for r in E["A"]:
        if r["f2"] is None or r["f3"] is None: continue
        ax.scatter([r["f2"]], [r["f3"]], marker="o" if r["adm"] else "x", s=24, c=col[r["var"]] if r["adm"] else col[r["var"]], lw=1.1, alpha=.9)
    for v, c in col.items(): ax.scatter([], [], c=c, s=22, label={"L2": "variation de $L_2$", "L3": "$L_3$", "alpha": r"$\alpha$", "beta": r"$\beta$", "xi": r"$\xi$"}[v])
    ax.scatter([], [], marker="x", c="k", s=24, label="× : hors domaine admissible")
    ax.scatter([E["reference"]["f2"]], [E["reference"]["f3"]], marker="*", s=200, c="k", zorder=5, label="design recommandé (coude)")
    ax.set_xlabel("$f_2$ NESM garanti (mm) — à maximiser →"); ax.set_ylabel("$f_3$ couple $C_2$ requis (N·m) — à minimiser"); ax.legend(fontsize=7, loc="upper left", frameon=False)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "plan.png"), dpi=200, facecolor="white"); plt.close(fig)

def fig_whatif():
    E = load_exp(); C = E["C"]; b = C[0]
    rows = [c for c in C[1:] if not c["label"].startswith("marge NESM")]
    fig, axs = plt.subplots(1, 3, figsize=(8.8, 3.4), sharey=True)
    for ax, (k, t, c) in zip(axs, [("adm", "designs admissibles", GREEN), ("min_f3", "meilleur $f_3$ (couple $C_2$)", RED), ("max_f2", "meilleur $f_2$ (NESM)", BLUE)]):
        v = [100 * (r[k] / b[k] - 1) for r in rows]; ax.barh(range(len(v)), v, color=c); ax.axvline(0, c="#444", lw=.8); ax.set_title(t, fontsize=8.6); ax.set_xlabel("variation (%)", fontsize=8)
        lo, hi = min(v + [0]), max(v + [0]); ax.set_xlim(lo - 0.25 * (hi - lo + 1), hi + 0.25 * (hi - lo + 1))
        for i, x in enumerate(v): ax.text(x, i, f" {x:+.0f} %", va="center", ha="left" if x >= 0 else "right", fontsize=7)
    axs[0].set_yticks(range(len(rows))); axs[0].set_yticklabels([r["label"] for r in rows], fontsize=7); axs[0].invert_yaxis()
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "whatif.png"), dpi=200, facecolor="white"); plt.close(fig)

for f in sys.argv[2:] or ["archi", "audit", "oat", "plan", "whatif"]:
    {"archi": fig_archi, "audit": fig_audit, "oat": fig_oat, "plan": fig_plan, "whatif": fig_whatif}[f]()
print("ok")
