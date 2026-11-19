"""Extrait de results_exact/ les fronts de reference (Pareto 3D, front robuste, choix) pour
le simulateur : sim/ref_fronts.json. Usage : python sim/make_ref_fronts.py"""
import json, os
import pandas as pd
H = os.path.dirname(os.path.abspath(__file__)); R = os.path.dirname(H)
COLS = ["L2", "L3", "alpha", "beta", "xi", "D", "h", "servo_C1", "servo_C2", "servo_C3", "f1_masse_pattes", "f2_nesm", "f3_couple_C2",
        "frac_bloc_C2", "M_total", "phase_critique", "taux_reussite"]
def rows(df):
    out = []
    for _, r in df.iterrows():
        out.append({c: (None if c not in r or pd.isna(r[c]) else (float(r[c]) if not isinstance(r[c], str) else r[c])) for c in COLS})
    return out
out = {}
for m in ("dedans", "dehors"):
    for s in ("S1", "S2"):
        d = os.path.join(R, "results_exact", f"{m}_{s}")
        res = json.load(open(os.path.join(d, "resume.json")))
        ad = pd.read_csv(os.path.join(d, "admissibles.csv"))
        ch = {k: {c: (None if c not in v or v[c] is None else (v[c] if not isinstance(v[c], str) else v[c])) for c in COLS} for k, v in res["choix"].items()}
        out[f"{m}_{s}"] = dict(front=rows(pd.read_csv(os.path.join(d, "pareto.csv"))), rob=rows(pd.read_csv(os.path.join(d, "pareto_robuste.csv"))), choix=ch,
                               admissibles=res["admissibles"], extremes=dict(f1_min=float(ad.f1_masse_pattes.min()), f2_max=float(ad.f2_nesm.max()), f3_min=float(ad.f3_couple_C2.min())))
json.dump(out, open(os.path.join(H, "ref_fronts.json"), "w"), separators=(",", ":"))
print({k: (len(v["front"]), len(v["rob"])) for k, v in out.items()}, os.path.getsize(os.path.join(H, "ref_fronts.json")))
