"""Courbe de compromis : que coute une empreinte au sol imposee ?

Pour une serie d'empreintes imposees E (mm), on calcule avec un servo ILLIMITE
aux trois axes (masse minimale du catalogue) :
  - le plus petit couple requis en C2 (et C1, C3) : PLANCHER physique, car aucun
    servo du catalogue ne peut faire mieux ;
  - le plus grand NESM atteignable ;
  - la plus petite masse de pattes.

Lecture : si le plancher de C2 depasse le couple utilisable du S3306
(2,354 / 1,2 = 1,962 N.m), l'empreinte E est IMPOSSIBLE avec le catalogue du
FabLab. L'inverse n'est pas vrai : le plancher est optimiste (servo de 14 g au
lieu de 126 g), donc un plancher sous la limite ne garantit pas l'admissibilite.

Usage : python sweep_empreinte.py [S2|S1] [dedans|dehors]
"""
import copy, json, os, sys
import numpy as np
from scipy.stats import qmc

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gr2.params import Params
from gr2 import pipeline
import run_final as RF

HERE = os.path.dirname(os.path.abspath(__file__))
CIBLES = [240.0, 270.0, 300.0, 330.0, 360.0, 380.0, 400.0]
BAND = 4.0


def sample(raw, target, n, seed, xi_fixed=None):
    b = raw["search"]["bounds"]
    lo = np.array([b["L2"][0], b["L3"][0], b["alpha"][0], b["xi"][0] if xi_fixed is None else xi_fixed])
    hi = np.array([b["L2"][1], b["L3"][1], b["alpha"][1], b["xi"][1] if xi_fixed is None else xi_fixed + 1e-9])
    S = qmc.scale(qmc.Sobol(d=4, scramble=True, seed=seed).random(n), lo, hi)
    L2, L3, al, xi = S.T
    base, k = RF.footprint_coeff(raw, xi)
    D0 = (target - 0.5 - base) / (2 * k)
    cb = (D0 - L2 * np.cos(np.radians(al))) / L3
    ok = (np.abs(cb) <= 1.0) & (D0 > 0)
    be = np.degrees(np.arccos(np.clip(cb, -1, 1)))
    ok &= (be >= b["beta"][0]) & (be <= b["beta"][1])
    return np.stack([L2, L3, al, be, xi], 1)[ok]


def main(sc="S2", mount="dedans", xi_fixed=None, sway=None):
    P0 = Params.load(os.path.join(HERE, "config.yaml"))
    out = []
    for E in CIBLES:
        raw = copy.deepcopy(P0.raw)
        raw["chassis"]["mount"] = mount
        if sway:                                # test de sensibilite a la borne de balancement
            raw["task"]["sway_max"] = float(sway)
            raw["task"]["sway_n"] = 13
        raw["gabarit"] = max(E, 400.0)        # plafond du cahier des charges
        raw["gabarit_min"] = E - BAND
        if xi_fixed is not None:
            raw["search"]["bounds"]["xi"] = [xi_fixed, xi_fixed + 1e-6]
        X = sample(raw, E, 1 << 16, 1, xi_fixed)
        if len(X) > 4096:
            X = X[np.linspace(0, len(X) - 1, 4096).astype(int)]
        mmin = min(s["mass"] for s in raw["servos"])
        raw["servos"] = raw["servos"] + [{"name": "ILLIMITE", "C_stall": 1000.0, "mass": mmin,
                                          "range": 360.0, "stock": None}]
        i = len(raw["servos"]) - 1
        o = pipeline.optimize(raw, mount, sc, combos=[(i, i, i)], n_mc=0, refine_rounds=1, X=X,
                              log=lambda m: None)
        d = o["df"]
        r = dict(empreinte=E, echantillon=len(X), admissibles=int(len(d)))
        if len(d):
            r.update(couple_C2_min=float(d.f3_couple_C2.min()),
                     couple_C3_min=float(d.tau_min_C3.min()),
                     couple_C1_min=float(d.tau_min_C1.min()),
                     nesm_max=float(d.f2_nesm.max()),
                     masse_pattes_min=float(d.f1_masse_pattes.min()),
                     L2_L3_min=float((d.L2 + d.L3).min()),
                     xi_median=float(d.xi.median()), D_min=float(d.D.min()))
        out.append(r)
        print(json.dumps(r, ensure_ascii=False), flush=True)
    tag = f"{mount}_{sc}" + (f"_xi{int(xi_fixed)}" if xi_fixed is not None else "_xilibre") \
          + (f"_sway{int(sway)}" if sway else "")
    os.makedirs(os.path.join(HERE, "results_final"), exist_ok=True)
    json.dump(out, open(os.path.join(HERE, "results_final", f"sweep_empreinte_{tag}.json"), "w"),
              indent=1, ensure_ascii=False)


if __name__ == "__main__":
    a = sys.argv[1:]
    main(a[0] if a else "S2", a[1] if len(a) > 1 else "dedans",
         float(a[2]) if len(a) > 2 and a[2] != "libre" else None,
         float(a[3]) if len(a) > 3 else None)
