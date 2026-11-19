"""Figures du rapport v2 (mode exact, catalogue reel, 3 objectifs).
Style : fond blanc, couleurs du document de reference (L2 rouge, L3 vert, poids orange).
Usage : python make_figures2.py <dossier_sortie>"""
import json, os, sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Arc, FancyArrowPatch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from gr2.params import Params
from gr2.evaluate import geometry

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "results_exact", "figures")
os.makedirs(OUT, exist_ok=True)
RES = os.path.join(HERE, "results_exact")
CASES = ["dedans_S1", "dedans_S2", "dehors_S1", "dehors_S2"]
RED, GREEN, ORANGE, BLUE, GREY = "#e0191b", "#0a9a1e", "#e8890c", "#1a1aa6", "#999999"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "mathtext.fontset": "cm",
                     "axes.spines.top": False, "axes.spines.right": False})


def arrow(ax, p, q, c, lw=1.4):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=9, color=c, lw=lw))


def fig_leg_schema(path):
    """Schema generique de la patte, dans le plan vertical (convention du rapport)."""
    L2, L3, al, be = 100.0, 140.0, np.radians(28), np.radians(62)
    e, h = 22.0, 0.0
    C2 = np.array([0.0, 0.0])
    K = C2 + L2 * np.array([np.cos(al), np.sin(al)])
    F = K + L3 * np.array([np.cos(be), -np.sin(be)])
    H2 = C2[1] - F[1]
    fig, ax = plt.subplots(figsize=(6.4, 4.7))
    ax.add_patch(Rectangle((-110, -e), 110, 2 * e + 6, fc="#e8e8e8", ec="none"))
    ax.text(-100, 4, "Châssis", fontsize=9, weight="bold")
    ax.plot([-60, 0], [0, 0], c="k", lw=3)
    ax.plot([C2[0], K[0]], [C2[1], K[1]], c=RED, lw=3.2)
    ax.plot([K[0], F[0]], [K[1], F[1]], c=GREEN, lw=3.2)
    for p in (C2, K):
        ax.plot(*p, "ko", ms=7)
    ax.plot(*F, "ko", ms=5)
    gy = F[1]
    ax.add_patch(Rectangle((-125, gy - 12), 330, 12, fc="none", hatch="////", ec="#444", lw=0.6))
    ax.plot([-125, 205], [gy, gy], c="k", lw=1)
    ax.plot([C2[0], F[0]], [C2[1], C2[1]], ls="--", c=GREY, lw=.9)
    ax.plot([F[0], F[0]], [C2[1], gy], ls="--", c=GREY, lw=.9)
    arrow(ax, (C2[0], -170), (F[0], -170), BLUE, 1.0); arrow(ax, (F[0], -170), (C2[0], -170), BLUE, 1.0)
    ax.text(F[0] / 2, -162, r"$D=L_2\cos\alpha+L_3\cos\beta$", color=BLUE, ha="center", fontsize=9)
    ax.plot([C2[0], C2[0]], [-170, 0], ls=":", c=GREY, lw=.7)
    arrow(ax, (F[0] + 28, 0), (F[0] + 28, gy), BLUE, 1.0); arrow(ax, (F[0] + 28, gy), (F[0] + 28, 0), BLUE, 1.0)
    ax.text(F[0] + 34, gy / 2, r"$H_2=L_3\sin\beta-L_2\sin\alpha$", color=BLUE, fontsize=9, va="center")
    arrow(ax, (-85, -e), (-85, gy), ORANGE, 0.9); arrow(ax, (-85, gy), (-85, -e), ORANGE, 0.9)
    ax.text(-80, (gy - e) / 2, r"$h=H_2-e$ (garde au sol)", color=ORANGE, fontsize=8.5, va="center")
    mid2, mid3 = (C2 + K) / 2, (K + F) / 2
    arrow(ax, mid2, mid2 + [0, -48], ORANGE); ax.text(mid2[0] - 22, mid2[1] - 62, r"$m_2 g$", color=ORANGE, fontsize=10)
    arrow(ax, mid3, mid3 + [0, -48], ORANGE); ax.text(mid3[0] + 8, mid3[1] - 58, r"$m_3 g$", color=ORANGE, fontsize=10)
    arrow(ax, (F[0] + 6, gy - 55), (F[0] + 6, gy + 1), RED, 1.8)
    ax.text(F[0] + 12, gy - 38, r"$R_i$", color=RED, fontsize=11)
    ax.text(mid2[0] - 6, mid2[1] + 12, r"$L_2$", color=RED, fontsize=12)
    ax.text(mid3[0] + 10, mid3[1] + 4, r"$L_3$", color=GREEN, fontsize=12)
    ax.text(-52, 8, r"$L_1$", fontsize=11); ax.text(C2[0] - 20, 12, r"$C_2$", fontsize=11)
    ax.text(K[0] + 4, K[1] + 8, r"$C_3$", fontsize=11); ax.text(-72, -17, r"$C_1$", fontsize=11)
    ax.add_patch(Arc(C2, 70, 70, theta1=0, theta2=28, color=RED, lw=1.2))
    ax.text(38, 6, r"$\alpha$", color=RED, fontsize=11)
    ax.add_patch(Arc(K, 70, 70, theta1=-62, theta2=0, color=GREEN, lw=1.2))
    ax.text(K[0] + 36, K[1] - 22, r"$\beta$", color=GREEN, fontsize=11)
    ax.plot([K[0], K[0] + 60], [K[1], K[1]], ls="--", c=GREY, lw=.9)
    ax.set_aspect("equal"); ax.axis("off"); ax.set_xlim(-130, 215); ax.set_ylim(-185, 85)
    fig.tight_layout(); fig.savefig(path, dpi=200); plt.close(fig)


def load_case(case):
    d = os.path.join(RES, case)
    r = json.load(open(os.path.join(d, "resume.json")))
    rd = lambda f: pd.read_csv(os.path.join(d, f)) if os.path.getsize(os.path.join(d, f)) > 5 else pd.DataFrame()
    r["adm"], r["front"], r["rob"] = rd("admissibles.csv"), rd("pareto.csv"), rd("pareto_robuste.csv")
    return r


def fig_top(P, row, path):
    """Vues de dessus des deux montages avec le design recommande."""
    fig, axs = plt.subplots(1, 2, figsize=(7.4, 3.8))
    X = np.array([[row["L2"], row["L3"], row["alpha"], row["beta"], row["xi"]]])
    for ax, mount in zip(axs, ("dedans", "dehors")):
        Q = P.copy(); Q.raw["chassis"]["mount"] = mount; Q = Params.from_dict(Q.raw)
        g = geometry(X, Q); ch = Q["chassis"]
        ax.add_patch(Rectangle((-ch["N_ch"] / 2, -ch["M_ch"] / 2), ch["N_ch"], ch["M_ch"], fc="#e8e8e8", ec="#555"))
        feet = g["foot"][0]
        ax.fill(feet[[0, 2, 3, 1], 0], feet[[0, 2, 3, 1], 1], fc="none", ec=ORANGE, ls="--", lw=1.2)
        for i in range(4):
            C1 = g["C1"][0, i]; C2 = C1 + Q["leg"]["L1"] * g["u"][0, i]; f = feet[i]
            ax.plot([C1[0], C2[0]], [C1[1], C2[1]], c="k", lw=3)
            ax.plot([C2[0], f[0]], [C2[1], f[1]], c=RED, lw=2.4)
            ax.plot(*C1, "ko", ms=4); ax.plot(*f, "o", c=GREEN, ms=6)
        ax.add_patch(Rectangle((-200, -200), 400, 400, fc="none", ec=GREY, ls=":"))
        ax.set_aspect("equal"); ax.set_xlim(-215, 215); ax.set_ylim(-215, 215)
        ax.set_title(f"Coxa « {mount} »", fontsize=10, weight="bold")
        ax.set_xlabel("x (mm, avant)"); ax.set_ylabel("y (mm, gauche)")
        ax.text(-196, 188, "gabarit 400 × 400", fontsize=7, color="#666")
    fig.tight_layout(); fig.savefig(path, dpi=200); plt.close(fig)


def fig_design(P, row, path, title):
    """Vue de côté réelle du design retenu + vue de dessus (dedans)."""
    ch = P["chassis"]; L1 = P["leg"]["L1"]
    L2, L3 = row["L2"], row["L3"]
    al, be = np.radians(row["alpha"]), np.radians(row["beta"])
    H2, h = row["H2"], row["h"]
    fig, ax = plt.subplots(figsize=(6.6, 3.9))
    C2 = np.array([0.0, H2]); K = C2 + L2 * np.array([np.cos(al), np.sin(al)])
    F = K + L3 * np.array([np.cos(be), -np.sin(be)])
    ax.add_patch(Rectangle((-L1 - 70, h), L1 + 70 + 10, ch["e"] * 2.4, fc="#e8e8e8", ec="#555"))
    ax.plot([-L1, 0], [H2, H2], c="k", lw=3)
    ax.plot([C2[0], K[0]], [C2[1], K[1]], c=RED, lw=3.2)
    ax.plot([K[0], F[0]], [K[1], F[1]], c=GREEN, lw=3.2)
    for p in (np.array([-L1, H2]), C2, K, F):
        ax.plot(*p, "ko", ms=5)
    ax.axhline(0, c="#444", lw=1)
    ax.add_patch(Rectangle((F[0] + 40, 0), 70, 30, fc="#d8c7b0", ec="#7a5a3a"))
    ax.text(F[0] + 45, 10, "obstacle 30 mm", fontsize=8)
    ax.text(-L1 - 66, h - 11, f"garde au sol h = {h:.0f} mm", fontsize=8, color=ORANGE)
    ax.text(K[0] + 5, K[1] + 4, r"$C_3$", fontsize=10); ax.text(C2[0] - 8, C2[1] - 16, r"$C_2$", fontsize=10)
    ax.text(-L1 - 6, H2 + 6, r"$C_1$", fontsize=10)
    ax.text((C2[0] + K[0]) / 2 - 8, (C2[1] + K[1]) / 2 + 9, rf"$L_2={L2:.0f}$", color=RED, fontsize=9)
    ax.text((K[0] + F[0]) / 2 + 6, (K[1] + F[1]) / 2 + 2, rf"$L_3={L3:.0f}$", color=GREEN, fontsize=9)
    ax.text(C2[0] + 6, C2[1] - 17, rf"$\alpha={row['alpha']:.1f}^\circ$", color=RED, fontsize=9)
    ax.text(K[0] + 24, K[1] - 28, rf"$\beta={row['beta']:.1f}^\circ$", color=GREEN, fontsize=9)
    ax.plot([F[0] + 12] * 2, [0, H2], ":", c="#666"); ax.text(F[0] + 15, H2 / 2, rf"$H_2={H2:.0f}$", fontsize=8)
    ax.set_aspect("equal"); ax.set_xlabel("distance horizontale (mm)"); ax.set_ylabel("hauteur (mm)")
    ax.set_title(title, fontsize=9.5)
    fig.tight_layout(); fig.savefig(path, dpi=200); plt.close(fig)


def fig_pareto(data, path):
    fig, axs = plt.subplots(2, 2, figsize=(7.6, 6.2))
    vmin = min(r["adm"]["f1_masse_pattes"].min() for r in data.values())
    vmax = max(r["adm"]["f1_masse_pattes"].max() for r in data.values())
    for ax, case in zip(axs.ravel(), CASES):
        r = data[case]; a = r["adm"]
        sc = ax.scatter(a.f2_nesm, a.f3_couple_C2, c=a.f1_masse_pattes, s=5, cmap="Greys", vmin=vmin - .1, vmax=vmax, alpha=.5, lw=0)
        ax.scatter(r["front"].f2_nesm, r["front"].f3_couple_C2, s=22, fc="none", ec=BLUE, lw=.8, label="Pareto 3D (nominal)")
        ax.scatter(r["rob"].f2_nesm, r["rob"].f3_couple_C2, s=28, c=BLUE, label="Pareto robuste")
        c = r["choix"]
        ax.scatter(c["coude"]["f2_nesm"], c["coude"]["f3_couple_C2"], marker="*", s=190, c=RED, ec="k", lw=.6, zorder=5, label="compromis (coude)")
        ax.scatter(c["couple_min"]["f2_nesm"], c["couple_min"]["f3_couple_C2"], marker="D", s=48, c=ORANGE, ec="k", lw=.6, zorder=5, label="couple C2 min. (NESM ≥ 2 mm)")
        ax.set_title(case.replace("_", " – "), fontsize=10, weight="bold")
        ax.set_xlabel("$f_2$ : NESM (mm)"); ax.set_ylabel("$f_3$ : couple C2 requis (N·m)")
        ax.grid(alpha=.25)
    h, l = axs[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc="lower center", ncol=2, fontsize=8, frameon=False)
    fig.subplots_adjust(bottom=.14, hspace=.38, wspace=.3, right=.86)
    cax = fig.add_axes([0.89, 0.25, 0.018, 0.5])
    fig.colorbar(sc, cax=cax, label="$f_1$ : masse des pattes (kg)")
    fig.savefig(path, dpi=200); plt.close(fig)


def fig_servos(P, data, path):
    fig, ax = plt.subplots(figsize=(6.6, 3.9))
    SF = P["margins"]["SF"]
    for s in P.servos:
        usable = s.C_cont / SF
        ok = (s.stock is None) or s.stock >= 4
        m_g = s.mass * 1000 if s.mass < 5 else s.mass
        ax.scatter(m_g, usable, s=70, c=("#222" if ok else "#bbb"), zorder=3)
        ax.annotate(s.name + ("" if ok else " (stock 1)"), (m_g, usable), xytext=(5, 5), textcoords="offset points", fontsize=8,
                    color=("#222" if ok else "#888"))
    r = data["dedans_S2"]["choix"]["coude"]
    for j, col, lab in (("C1", "#7b3fa0", "C1"), ("C2", RED, "C2"), ("C3", GREEN, "C3")):
        ax.axhline(r[f"tau_{j}"], c=col, ls="--", lw=1.1)
        ax.text(168, r[f"tau_{j}"] * 1.04, f"couple requis {lab} (pos. NESM max) = {r[f'tau_{j}']:.2f} N·m", color=col, fontsize=8, ha="right")
    ax.set_yscale("log"); ax.set_xlim(0, 170)
    ax.set_xlabel("masse du servo (g)"); ax.set_ylabel(f"couple utilisable = C_stall / SF  (N·m)")
    ax.grid(alpha=.25, which="both"); ax.set_title("Catalogue : couple utilisable vs masse (recommandé dedans S2)", fontsize=9.5)
    fig.tight_layout(); fig.savefig(path, dpi=200); plt.close(fig)


def fig_sens(path):
    df = pd.concat([pd.read_csv(os.path.join(RES, f"sens_part{i}.csv")) for i in (0, 1)], ignore_index=True)
    fig, axs = plt.subplots(1, 3, figsize=(7.6, 3.0))
    a = df[df.parametre.str.startswith("incertitude")].sort_values("valeur")
    axs[0].plot(a.valeur, a.C2_requis_min, "o-", c=RED); axs[0].set_xlabel("ε_CdG (mm)"); axs[0].set_ylabel("C2 requis min. (N·m)")
    axs[0].set_title("Incertitude sur le CdG (S2)", fontsize=9)
    b = df[df.parametre.str.startswith("batterie") & (df.scenario == "S2")].sort_values("valeur")
    axs[1].plot(b.valeur, b.C2_requis_min, "o-", c=BLUE); axs[1].set_xlabel("x batterie (mm)")
    axs[1].set_title("Position de la batterie (S2)", fontsize=9); axs[1].set_ylim(0.8, 1.4)
    c = df[df.parametre.str.startswith("masse") & df.C2_requis_min.notna()].sort_values("valeur")
    axs[2].plot(c.valeur, c.C2_requis_min, "o-", c=GREEN); axs[2].set_xlabel("masse châssis (kg)")
    axs[2].set_title("Masse du châssis (S1)", fontsize=9)
    for x in axs: x.grid(alpha=.25)
    fig.tight_layout(); fig.savefig(path, dpi=200); plt.close(fig)


if __name__ == "__main__":
    P = Params.load(os.path.join(HERE, "config.yaml"))
    data = {c: load_case(c) for c in CASES}
    fig_leg_schema(os.path.join(OUT, "schema_patte.png"))
    kn = data["dedans_S2"]["choix"]["coude"]
    P1 = P.copy(); P1.raw["chassis"]["mount"] = "dedans"; P1 = Params.from_dict(P1.raw)
    fig_top(P1, kn, os.path.join(OUT, "vues_dessus.png"))
    fig_design(P1, kn, os.path.join(OUT, "design_cote.png"), "Design recommandé (dedans, pire cas S2) — plan de la patte")
    fig_pareto(data, os.path.join(OUT, "pareto.png"))
    fig_servos(P1, data, os.path.join(OUT, "servos.png"))
    fig_sens(os.path.join(OUT, "sensibilite.png"))
    print("ok", os.listdir(OUT))
