"""Chaine complete : exploration -> Pareto -> raffinement -> robustesse ->
decision, pour les 2 montages de coxa x 2 scenarios de CdG, puis etude de
sensibilite. Usage : python run_all.py [--quick]"""
import argparse, json, os, sys, time
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gr2.params import Params
from gr2 import explore as ex

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")


def run_case(P, mount, sc, quick):
    Q = P.copy(); Q.raw["chassis"]["mount"] = mount
    n = 2048 if quick else Q["search"]["n_sobol"]
    t = time.time()
    X = ex.sobol(Q, n, Q["search"]["seed"])
    df, stats = ex.evaluate_all(Q, sc, X)
    if not df.empty:
        df = ex.refine(Q, sc, df, 1 if quick else Q["search"]["refine_rounds"],
                       Q["search"]["refine_per_point"], Q["search"]["seed"] + 7)
    front = ex.pareto(df)
    if quick:
        Q.raw["robust"]["n_mc"] = 30
    # 1) candidats : 6 premieres couches de non-dominance (<= 60 points)
    cand = ex.pareto_layers(df, 6, 60) if not df.empty else df
    rob = ex.robustness(Q, sc, cand, Q["search"]["seed"] + 11) if not cand.empty else cand
    rob_ok = rob[rob["taux_reussite"] >= Q["robust"]["pass_rate"]] if not rob.empty else rob
    rob_front = ex.pareto(rob_ok) if not rob_ok.empty else rob_ok
    # 2) test de resistance 'tout au pire en meme temps' (information)
    Qp = ex.pessimistic(Q)
    dfp, stats_p = ex.evaluate_all(Qp, sc, X)
    choice = {}
    if not rob_front.empty:
        kp = ex.knee_point(rob_front)
        choice["coude"] = rob_front.loc[kp].to_dict()
        ec = ex.eps_choice(rob_front, Q["margins"]["nesm_cible"])
        if ec is not None:
            choice["eps_contrainte"] = rob_front.loc[ec].to_dict()
    if rob_front.empty and not rob.empty:
        # aucun point robuste a 95 % : on donne le candidat le plus robuste (signale)
        best = rob.sort_values(["taux_reussite", "f3_couple_C2"], ascending=[False, True]).index[0]
        choice["meilleur_non_robuste"] = rob.loc[best].to_dict()
    lo, hi = ex.bounds(Q)
    for kk, row in choice.items():   # alerte si le choix touche une borne de recherche
        x = np.array([row[v] for v in ex.VARS])
        row["bornes_actives"] = [v for v, xv, l, u in zip(ex.VARS, x, lo, hi) if abs(xv - l) < 0.5 or abs(xv - u) < 0.5]
    d = os.path.join(OUT, f"{mount}_{sc}")
    os.makedirs(d, exist_ok=True)
    df.to_csv(os.path.join(d, "admissibles.csv"), index=False)
    front.to_csv(os.path.join(d, "pareto.csv"), index=False)
    rob.to_csv(os.path.join(d, "candidats_robustesse.csv"), index=False)
    dfp.to_csv(os.path.join(d, "admissibles_tout_au_pire.csv"), index=False)
    rob_front.to_csv(os.path.join(d, "pareto_robuste.csv"), index=False)
    stats.update(dict(admissibles=len(df), pareto=len(front),
                      candidats_testes=len(cand),
                      taux_reussite_front_nominal=(rob.merge(front[list(ex.VARS)+["servo_C1","servo_C2","servo_C3"]])["taux_reussite"].round(3).tolist() if len(rob) else []),
                      candidats_robustes=int((rob["taux_reussite"] >= Q["robust"]["pass_rate"]).sum()) if len(rob) else 0,
                      admissibles_tout_au_pire=len(dfp), stats_tout_au_pire=stats_p,
                      pareto_robuste=len(rob_front),
                      duree_s=round(time.time() - t, 1)))
    with open(os.path.join(d, "resume.json"), "w") as f:
        json.dump(dict(stats=stats, choix=choice), f, indent=2, default=float)
    plot_case(df, front, rob_front, choice, f"{mount} / {sc}", os.path.join(d, "pareto.png"))
    print(f"[{mount} {sc}] {stats}")
    return df, front, rob_front, choice, stats


def plot_case(df, front, robf, choice, title, path):
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.6))
    for a, (x, xl) in zip(ax, (("f3_couple_C2", "f3 : couple requis en C2 (N.m)  [a minimiser]"),
                               ("f1_masse_pattes", "f1 : masse des 4 pattes (kg)  [a minimiser]"))):
        if not df.empty:
            a.scatter(df[x], df.f2_nesm, s=3, c="#c8c8c8", label="admissibles")
        if not front.empty:
            sc = a.scatter(front[x], front.f2_nesm, s=10, c=front.f1_masse_pattes if x != "f1_masse_pattes" else front.f3_couple_C2,
                           cmap="viridis", label="front de Pareto (3 objectifs)")
            fig.colorbar(sc, ax=a, label="f1 masse (kg)" if x != "f1_masse_pattes" else "f3 couple C2 (N.m)")
        if not robf.empty:
            a.plot(robf[x], robf.f2_nesm, "s", mfc="none", mec="#d9822b", ms=8, label="front robuste (>=95 %)")
        for k, mk in (("coude", "*"), ("eps_contrainte", "P")):
            if k in choice:
                a.plot(choice[k][x], choice[k]["f2_nesm"], mk, c="k", ms=14, label=f"choix : {k}")
        a.set_xlabel(xl); a.set_ylabel("f2 : NESM minimal (mm)  [a maximiser]")
        a.grid(alpha=.3); a.legend(fontsize=7)
    fig.suptitle(title); fig.tight_layout(); fig.savefig(path, dpi=130); plt.close(fig)


def sensitivity(P, quick, part=None):
    """Etude un-facteur-a-la-fois sur le COUPLE REQUIS (montage dedans, S1 et S2).
    On ajoute un servo fictif 'ILLIMITE' (couple 100 N.m, masse du S3306) en C2 et
    C3 : aucune limite de catalogue, on mesure le couple minimal necessaire
    (f3 en C2, tau_min en C3) parmi les geometries stables. Ce couple est
    independant du catalogue : on le compare ensuite aux servos disponibles."""
    base = P.copy(); base.raw["chassis"]["mount"] = "dedans"
    ref = next(s for s in base.raw["servos"] if s["name"] == base.raw["reference_servo"])
    base.raw["servos"].append({"name": "ILLIMITE", "C_stall": 100.0, "mass": ref["mass"], "range": 180.0, "stock": None})
    n = 512 if quick else 1536
    tests = {
        "masse chassis + electronique (kg)": (("body", "chassis", "m"), [1.0, 1.25, 1.5, 2.0]),
        "incertitude CdG (mm)": (("body", "eps_cdg_xy"), [0.0, 5.0, 10.0, 20.0]),
        "course R_s (mm)": (("task", "stroke"), [20.0, 40.0, 60.0]),
        "s_min (mm)": (("margins", "s_min"), [0.0, 5.0, 10.0]),
        "balancement max (mm)": (("task", "sway_max"), [40.0, 60.0, 80.0]),
    }
    X = ex.sobol(base, n, 5)
    rows = []
    items = [(nm, t, sc) for nm, t in tests.items() for sc in ("S1", "S2")]
    if part is not None:            # execution en 2 morceaux paralleles
        items = items[part::2]
    for name, (path, vals), sc in items:
        for v in vals:
            Q = base.copy()
            if len(path) == 2:
                Q.raw[path[0]][path[1]] = v
            else:
                Q.raw[path[0]][path[1]][path[2]] = v
            if path == ("body", "eps_cdg_xy"):
                Q.raw["body"]["eps_cdg_z"] = v
            Q = Params.from_dict(Q.raw)
            df, st = ex.evaluate_all(Q, sc, X)
            if len(df):
                df = df[(df.servo_C2 == "ILLIMITE") & (df.servo_C3 == "ILLIMITE")]
            rows.append(dict(parametre=name, scenario=sc, valeur=v, n_geometries_stables=len(df),
                             couple_C2_requis_min=df.f3_couple_C2.min() if len(df) else np.nan,
                             couple_C3_requis_min=df.tau_min_C3.min() if len(df) else np.nan,
                             nesm_max=df.f2_nesm.max() if len(df) else np.nan,
                             M_total=df.M_total.min() if len(df) else np.nan))
            print("sensibilite", rows[-1], flush=True)
    out = pd.DataFrame(rows)
    out.to_csv(os.path.join(OUT, f"sensibilite_part{part}.csv" if part is not None else "sensibilite.csv"), index=False)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--only", default=None, help="ex : dedans_S2")
    ap.add_argument("--no-sens", action="store_true")
    ap.add_argument("--sens-only", action="store_true")
    ap.add_argument("--part", type=int, default=None)
    a = ap.parse_args()
    P = Params.load(os.path.join(HERE, "config.yaml"))
    os.makedirs(OUT, exist_ok=True)
    if a.sens_only:
        sensitivity(P, a.quick, a.part)
        return
    summary = {}
    for mount in ("dedans", "dehors"):
        for sc in ("S1", "S2"):
            if a.only and a.only != f"{mount}_{sc}":
                continue
            _, front, robf, choice, stats = run_case(P, mount, sc, a.quick)
            summary[f"{mount}_{sc}"] = dict(stats=stats, choix=choice)
    if not a.no_sens and not a.only:
        sensitivity(P, a.quick)
    with open(os.path.join(OUT, "synthese.json"), "w") as f:
        json.dump(summary, f, indent=2, default=float)


if __name__ == "__main__":
    main()
