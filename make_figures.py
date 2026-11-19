"""Figures et tableaux de synthese a partir de results/ (pour le rapport)."""
import json, os, sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gr2.params import Params, LEG_SIGNS
from gr2.evaluate import geometry

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")
FIG = os.path.join(RES, "figures")
os.makedirs(FIG, exist_ok=True)
CASES = ["dedans_S1", "dedans_S2", "dehors_S1", "dehors_S2"]
COL = {"dedans_S1": "#2f6fb0", "dedans_S2": "#14457a", "dehors_S1": "#d9822b", "dehors_S2": "#9a4f0f"}


def load(case):
    d = os.path.join(RES, case)
    if not os.path.exists(os.path.join(d, "resume.json")):
        return None
    r = json.load(open(os.path.join(d, "resume.json")))
    r["adm"] = pd.read_csv(os.path.join(d, "admissibles.csv")) if os.path.getsize(os.path.join(d, "admissibles.csv")) > 1 else pd.DataFrame()
    r["front"] = pd.read_csv(os.path.join(d, "pareto.csv")) if os.path.getsize(os.path.join(d, "pareto.csv")) > 1 else pd.DataFrame()
    r["rob"] = pd.read_csv(os.path.join(d, "pareto_robuste.csv")) if os.path.getsize(os.path.join(d, "pareto_robuste.csv")) > 1 else pd.DataFrame()
    return r


def fronts_overlay(data):
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2), sharey=False)
    for case, r in data.items():
        if r is None or r["front"].empty:
            continue
        a = ax[0] if case.endswith("S1") else ax[1]
        f = r["front"]
        a.plot(f.f1_masse_pattes, f.f2_nesm, "o-", ms=3, c=COL[case], alpha=.5, label=f"{case} : front nominal")
        if not r["rob"].empty:
            a.plot(r["rob"].f1_masse_pattes, r["rob"].f2_nesm, "s", ms=6, c=COL[case], label=f"{case} : front robuste")
        for k, mk in (("coude", "*"), ("eps_contrainte", "P")):
            if k in r["choix"]:
                c = r["choix"][k]
                a.plot(c["f1_masse_pattes"], c["f2_nesm"], mk, ms=14, mec="k", mfc=COL[case])
    for a, t in zip(ax, ("Scenario S1 : CdG nominal (entrees)", "Scenario S2 : pire cas du CdG")):
        a.set_title(t, fontsize=10); a.grid(alpha=.3)
        a.set_xlabel("f1 : masse des 4 pattes (kg)"); a.set_ylabel("f2 : NESM minimal (mm)")
        a.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fronts.png"), dpi=140); plt.close(fig)


def funnel(data):
    fig, ax = plt.subplots(figsize=(10, 4))
    labels = ["echantillons", "apres prefiltre", "geometrie OK", "stabilite OK", "designs admissibles"]
    w = 0.2
    for j, (case, r) in enumerate(data.items()):
        if r is None:
            continue
        s = r["stats"]
        vals = [s["n_echantillons"], s["apres_prefiltre"], s["geometrie_ok"], s["stabilite_ok"],
                s["designs_geometriques_admissibles"]]
        ax.bar(np.arange(5) + (j - 1.5) * w, vals, w, color=COL[case], label=case)
    ax.set_xticks(range(5)); ax.set_xticklabels(labels, fontsize=8); ax.set_yscale("log")
    ax.set_ylabel("nombre de geometries (echelle log)"); ax.legend(fontsize=8); ax.grid(alpha=.3, axis="y")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "entonnoir.png"), dpi=140); plt.close(fig)


def rejections(data):
    fig, ax = plt.subplots(figsize=(8, 3.6))
    r = data.get("dedans_S2") or next(v for v in data.values() if v)
    rej = r["stats"]["rejets_prefiltre"]
    keys = list(rej)
    ax.barh(keys, [rej[k] for k in keys], color="#2f6fb0")
    ax.set_xlabel("geometries rejetees (sur %d echantillons)" % r["stats"]["n_echantillons"])
    ax.set_title("Contraintes de la posture de repos qui rejettent le plus (dedans, S2)", fontsize=9)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "rejets.png"), dpi=140); plt.close(fig)


def servo_usage(data):
    fig, ax = plt.subplots(1, 3, figsize=(11, 3.4))
    r = data.get("dedans_S2")
    if r is None or r["adm"].empty:
        return
    for a, j in zip(ax, ("C1", "C2", "C3")):
        vc = r["adm"]["servo_" + j].value_counts()
        a.bar(vc.index, vc.values, color="#14457a"); a.set_title(f"Servo choisi en {j} (designs admissibles, dedans S2)", fontsize=8)
        a.tick_params(axis="x", labelsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "servos.png"), dpi=140); plt.close(fig)


def design_drawing(P, row, mount, path, title):
    Q = P.copy(); Q.raw["chassis"]["mount"] = mount
    X = np.array([[row["L2"], row["L3"], row["alpha"], row["beta"], row["xi"]]])
    g = geometry(X, Q)
    ch = Q["chassis"]
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.6))
    a = ax[0]
    a.add_patch(plt.Rectangle((-ch["N_ch"] / 2, -ch["M_ch"] / 2), ch["N_ch"], ch["M_ch"], fc="#e6eef7", ec="#14457a"))
    feet = g["foot"][0]
    order = [0, 2, 3, 1]
    a.fill(feet[order, 0], feet[order, 1], fc="none", ec="#d9822b", ls="--", label="polygone d'appui (4 pieds)")
    for i in range(4):
        C1 = g["C1"][0, i]; f = feet[i]
        C2 = C1 + Q["leg"]["L1"] * g["u"][0, i]
        a.plot([C1[0], C2[0]], [C1[1], C2[1]], c="k", lw=3)
        a.plot([C2[0], f[0]], [C2[1], f[1]], c="#14457a", lw=2)
        a.plot(*C1, "ko", ms=5); a.plot(*f, "o", c="#d9822b", ms=7)
    a.add_patch(plt.Rectangle((-200, -200), 400, 400, fc="none", ec="#999", ls=":"))
    a.text(-195, 185, "gabarit 400 x 400 mm", fontsize=7, color="#666")
    a.set_aspect("equal"); a.set_xlim(-215, 215); a.set_ylim(-215, 215)
    a.set_xlabel("x (mm, vers l'avant)"); a.set_ylabel("y (mm, vers la gauche)")
    a.set_title(f"Vue de dessus - xi = {row['xi']:.1f} deg", fontsize=9); a.legend(fontsize=7, loc="lower left")
    b = ax[1]
    L1, L2, L3 = Q["leg"]["L1"], row["L2"], row["L3"]
    al, be = np.radians(row["alpha"]), np.radians(row["beta"])
    H2, h = row["H2"], row["h"]
    pts = np.array([[-L1, H2], [0, H2], [L2 * np.cos(al), H2 + L2 * np.sin(al)],
                    [L2 * np.cos(al) + L3 * np.cos(be), H2 + L2 * np.sin(al) - L3 * np.sin(be)]])
    b.add_patch(plt.Rectangle((-L1 - 60, h), 60 + L1 - 5, ch["e"] * 2.2, fc="#e6eef7", ec="#14457a"))
    b.plot(pts[:, 0], pts[:, 1], "-o", c="#14457a", lw=3, ms=6)
    b.axhline(0, c="#7a5a3a"); b.add_patch(plt.Rectangle((pts[3, 0] + 45, 0), 60, 30, fc="#d8c7b0", ec="#7a5a3a"))
    b.text(pts[3, 0] + 48, 12, "obstacle 30 mm", fontsize=7)
    b.annotate(f"alpha = {row['alpha']:.1f} deg", pts[1] + [8, 8], fontsize=8)
    b.annotate(f"beta = {row['beta']:.1f} deg", pts[2] + [8, -12], fontsize=8)
    b.annotate("C1", pts[0] + [-4, 8], fontsize=8); b.annotate("C2", pts[1] + [-4, -14], fontsize=8)
    b.annotate("genou (C3)", pts[2] + [-20, 8], fontsize=8)
    b.plot([pts[3, 0] + 8] * 2, [0, H2], ":", c="#666"); b.text(pts[3, 0] + 10, H2 / 2, f"H2 = {H2:.0f}", fontsize=7)
    b.text(-L1 - 55, h - 10, f"garde au sol h = {h:.0f} mm", fontsize=7)
    b.set_aspect("equal"); b.set_xlabel("distance horizontale (mm)"); b.set_ylabel("hauteur (mm)")
    b.set_title(f"Vue de cote (plan de la patte) - L1={L1:.0f}, L2={L2:.0f}, L3={L3:.0f} mm", fontsize=9)
    b.grid(alpha=.3)
    fig.suptitle(title, fontsize=10); fig.tight_layout(); fig.savefig(path, dpi=140); plt.close(fig)


def merge_sensitivity():
    parts = [os.path.join(RES, f"sensibilite_part{i}.csv") for i in (0, 1)]
    if not all(os.path.exists(p) for p in parts):
        return
    out = pd.concat([pd.read_csv(p) for p in parts], ignore_index=True)
    order = ["couple continu servo EQUIPE (x)", "masse chassis (kg)", "course R_s (mm)",
             "coefficient de securite", "incertitude CdG (mm)", "NESM min (mm)"]
    out["o"] = out.parametre.map({k: i for i, k in enumerate(order)})
    out = out.sort_values(["o", "valeur"]).drop(columns="o")
    out.to_csv(os.path.join(RES, "sensibilite.csv"), index=False)
    fig, axs = plt.subplots(2, 3, figsize=(11, 6))
    for ax, (name, g) in zip(axs.flat, out.groupby("parametre", sort=False)):
        ax.bar([str(v) for v in g.valeur], g.n_admissibles, color="#2f6fb0")
        ax.set_title(name, fontsize=9); ax.set_ylabel("couples (design, servos) admissibles", fontsize=7)
        for i, (v, nm) in enumerate(zip(g.n_admissibles, g.nesm_max)):
            ax.text(i, v, f"NESM max {nm:.1f}" if v else "aucun", ha="center", va="bottom", fontsize=6)
    fig.tight_layout(); fig.savefig(os.path.join(RES, "sensibilite.png"), dpi=130); plt.close(fig)


def main():
    merge_sensitivity()
    P = Params.load(os.path.join(HERE, "config.yaml"))
    data = {c: load(c) for c in CASES}
    data = {k: v for k, v in data.items() if v is not None}
    fronts_overlay(data); funnel(data); rejections(data); servo_usage(data)
    for case, r in data.items():
        for k in ("coude", "eps_contrainte"):
            if k in r["choix"]:
                design_drawing(P, r["choix"][k], case.split("_")[0],
                               os.path.join(FIG, f"design_{case}_{k}.png"), f"{case} - choix '{k}'")
    print("figures :", sorted(os.listdir(FIG)))


if __name__ == "__main__":
    main()
