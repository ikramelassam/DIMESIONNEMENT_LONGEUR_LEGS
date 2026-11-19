"""Couple requis (sans limite de catalogue) avec CdG des pattes EXACT.
Servo fictif ILLIMITE (100 N.m, masse du S3306) en C2 et C3, S-U400 en C1."""
import os, sys, numpy as np, pandas as pd
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gr2.params import Params
from gr2 import explore as ex
HERE = os.path.dirname(os.path.abspath(__file__))
part = int(sys.argv[1])
base = Params.load(os.path.join(HERE, "config.yaml")); base.raw["chassis"]["mount"] = "dedans"
base.raw["servos"].append({"name": "ILLIMITE", "C_stall": 100.0, "mass": 0.126, "range": 180.0, "stock": None})
X = ex.sobol(base, 2048, 5)
tests = []
for v in (0.0, 5.0, 10.0, 20.0): tests.append(("incertitude CdG (mm)", "S2", v, lambda r, v=v: r["body"].update(eps_cdg_xy=v, eps_cdg_z=v)))
for sc in ("S1", "S2"):
    for v in (-80.0, -20.0, 10.0):
        def f(r, v=v, sc=sc):
            r["body"]["battery"]["x"] = v
            if sc == "S2": r["body"]["battery_zone"] = {"x": [v, v], "y": [0.0, 0.0], "z_rel": [15.0, 15.0]}
        tests.append(("batterie x (S2 : fixee)", sc, v, f))
for v in (1.0, 1.5, 2.0): tests.append(("masse chassis (kg)", "S1", v, lambda r, v=v: r["body"]["chassis"].update(m=v)))
for v in (0.0, 5.0, 10.0): tests.append(("s_min (mm)", "S1", v, lambda r, v=v: r["margins"].update(s_min=v)))
rows = []
for name, sc, v, fn in tests[part::2]:
    Q = base.copy(); fn(Q.raw); Q = Params.from_dict(Q.raw)
    nm = {s.name: i for i, s in enumerate(Q.servos)}
    df, st = ex.evaluate_all(Q, sc, X, exact_combo=(nm["S-U400"], nm["ILLIMITE"], nm["ILLIMITE"]))
    rows.append(dict(parametre=name, scenario=sc, valeur=v, n=len(df),
                     C2_requis_min=df.f3_couple_C2.min() if len(df) else np.nan,
                     C3_requis_min=df.tau_min_C3.min() if len(df) else np.nan,
                     nesm_max=df.f2_nesm.max() if len(df) else np.nan))
    print(rows[-1], flush=True)
pd.DataFrame(rows).to_csv(os.path.join(HERE, "results_exact", f"sens_part{part}.csv"), index=False)
