"""Diagnostic du cas "empreinte imposee a 400 mm" : de combien manque-t-il ?

1) Plancher de couple avec un servo de couple ILLIMITE mais de MASSE REELLE
   (126 g, celle du S3306) : la masse des servos compte dans le couple, donc ce
   plancher est beaucoup plus honnete que celui du servo de 14 g.
2) Facteur k minimal a appliquer au couple du S3306 en C2 (masse inchangee)
   pour qu'au moins un design devienne admissible -> "il faudrait un servo de
   k x 2,354 N.m". Recherche par dichotomie.
3) Pour le cas optimum (empreinte libre), meme mesure de marge : facteur k
   maximal de REDUCTION du couple encore admissible.

Usage : python diag_400.py
"""
import copy, json, os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gr2.params import Params
from gr2 import explore as ex
import run_final as RF

HERE = os.path.dirname(os.path.abspath(__file__))
M_S3306 = 0.126
SF = 1.2


def prepare(case, xi_fixed=None, forced=True, n=1 << 16, nkeep=4096):
    P0 = Params.load(os.path.join(HERE, "config.yaml"))
    raw = copy.deepcopy(P0.raw)
    raw["chassis"]["mount"] = "dedans"
    if xi_fixed is not None:
        raw["search"]["bounds"]["xi"] = [xi_fixed, xi_fixed + 1e-6]
    if forced:
        raw["gabarit_min"] = RF.GAB - RF.BAND
        X = RF.sample_on_footprint(raw, n, 1, xi_fixed)
    else:
        X = ex.sobol(Params.from_dict(raw), nkeep, 1)
    if len(X) > nkeep:
        X = X[np.linspace(0, len(X) - 1, nkeep).astype(int)]
    return raw, X


def n_admissibles(raw, X, c2_scale, c3_name="S3306", mass=M_S3306):
    """Nombre de designs admissibles avec, en C2, un servo de masse `mass` et de
    couple c2_scale x 2,354 N.m ; C1 = SG90 ; C3 = c3_name."""
    r = copy.deepcopy(raw)
    r["servos"] = r["servos"] + [{"name": "C2X", "C_stall": 2.354 * c2_scale, "mass": mass,
                                  "range": 180.0, "stock": None}]
    nm = {s["name"]: i for i, s in enumerate(r["servos"])}
    P = Params.from_dict(r)
    df, _ = ex.evaluate_all(P, "S2", X, exact_combo=(nm["SG90"], nm["C2X"], nm[c3_name]))
    return len(df), df


def dicho(raw, X, lo, hi, c3="S3306"):
    """Plus petit facteur k dans [lo, hi] tel qu'un design soit admissible."""
    if n_admissibles(raw, X, hi, c3)[0] == 0:
        return None
    for _ in range(9):
        mid = 0.5 * (lo + hi)
        if n_admissibles(raw, X, mid, c3)[0] > 0:
            hi = mid
        else:
            lo = mid
    return hi


def main():
    out = {}

    # ---------- cas E400 (xi libre) et E400 + xi = 45
    for tag, xi in (("E400_xilibre", None), ("E400_xi45", 45.0)):
        raw, X = prepare(tag, xi, forced=True)
        res = {"echantillon": len(X)}
        # plancher de couple a MASSE REELLE
        n_ill, d_ill = n_admissibles(raw, X, 1000.0 / 2.354, "S3306", M_S3306)
        res["plancher_masse_reelle"] = dict(
            admissibles=int(n_ill),
            couple_C2_min=float(d_ill.f3_couple_C2.min()) if n_ill else None,
            couple_C3_min=float(d_ill.tau_min_C3.min()) if n_ill else None,
            nesm_max=float(d_ill.f2_nesm.max()) if n_ill else None,
            masse_pattes_min=float(d_ill.f1_masse_pattes.min()) if n_ill else None)
        # S3306 reel
        res["avec_S3306"] = dict(C3_S3306=n_admissibles(raw, X, 1.0, "S3306")[0],
                                 C3_SU400=n_admissibles(raw, X, 1.0, "S-U400")[0])
        k = dicho(raw, X, 1.0, 6.0, "S3306")
        res["facteur_couple_C2_necessaire"] = k
        res["couple_C2_necessaire_Nm"] = round(2.354 * k, 3) if k else None
        out[tag] = res
        print(tag, json.dumps(res, ensure_ascii=False), flush=True)

    # ---------- cas optimum : marge disponible
    raw, X = prepare("OPT", None, forced=False, nkeep=4096)
    k = dicho(raw, X, 0.1, 1.0, "S-U400")
    out["OPT_xilibre"] = dict(echantillon=len(X),
                              facteur_couple_C2_minimal=k,
                              couple_C2_minimal_Nm=round(2.354 * k, 3) if k else None,
                              admissibles_S3306=n_admissibles(raw, X, 1.0, "S-U400")[0])
    print("OPT", json.dumps(out["OPT_xilibre"], ensure_ascii=False), flush=True)

    json.dump(out, open(os.path.join(HERE, "results_final", "diag_400.json"), "w"),
              indent=1, ensure_ascii=False, default=float)


if __name__ == "__main__":
    main()
