"""Diagnostic corrige : pourquoi le 400 mm ne passe pas.

Trois passes, de la plus optimiste a la plus realiste, avec aux TROIS axes un
servo de couple illimite (1000 N.m) mais de masse donnee :
  masse 14 g   = la plus legere du catalogue (SG90)  -> plancher absolu
  masse 126 g  = celle du S3306, le seul servo >= 1,8 N.m du labo
Si la passe a 126 g ne donne aucun design, le 400 mm est impossible avec le
catalogue, quel que soit le couple : le seul servo assez fort pese 126 g.

Rapporte aussi la contrainte qui tue, et pour l'optimum la meme mesure comme
point de comparaison.
"""
import copy, json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gr2.params import Params
from gr2 import explore as ex
from gr2.evaluate import Evaluator, prefilter
import run_final as RF

HERE = os.path.dirname(os.path.abspath(__file__))

def pack(raw, X, mass, tag):
    r = copy.deepcopy(raw)
    r["servos"] = r["servos"] + [{"name": "ILL", "C_stall": 1000.0, "mass": mass, "range": 360.0, "stock": None}]
    nm = {s["name"]: i for i, s in enumerate(r["servos"])}
    P = Params.from_dict(r)
    c = (nm["ILL"],) * 3
    df, st = ex.evaluate_all(P, "S2", X, exact_combo=c)
    out = dict(tag=tag, masse_servo_g=mass*1000, apres_prefiltre=st["apres_prefiltre"],
               geometrie_ok=st["geometrie_ok"], stabilite_ok=st["stabilite_ok"],
               admissibles=int(len(df)))
    if len(df):
        out.update(C1_min=float(df.tau_min_C1.min()), C2_min=float(df.f3_couple_C2.min()),
                   C3_min=float(df.tau_min_C3.min()), nesm_max=float(df.f2_nesm.max()),
                   masse_pattes_min=float(df.f1_masse_pattes.min()),
                   masse_CDC_min=float(df.f1_masse_pattes.min()+2.04),
                   span_C2_max=float(df.span_C2.max()), L2L3_min=float((df.L2+df.L3).min()))
    print(json.dumps(out, ensure_ascii=False), flush=True)
    return out

res = {}
for tag, xi, forced in (("E400_xilibre", None, True), ("E400_xi45", 45.0, True), ("OPT", None, False)):
    P0 = Params.load(os.path.join(HERE, "config.yaml"))
    raw = copy.deepcopy(P0.raw); raw["chassis"]["mount"] = "dedans"
    if xi is not None: raw["search"]["bounds"]["xi"] = [xi, xi + 1e-6]
    if forced:
        raw["gabarit_min"] = RF.GAB - RF.BAND
        X = RF.sample_on_footprint(raw, 1 << 16, 1, xi)
    else:
        X = ex.sobol(Params.from_dict(raw), 4096, 1)
    if len(X) > 4096: X = X[np.linspace(0, len(X)-1, 4096).astype(int)]
    res[tag] = [pack(raw, X, m, tag) for m in (0.014, 0.126)]
json.dump(res, open(os.path.join(HERE, "results_final", "diag2.json"), "w"), indent=1, ensure_ascii=False, default=float)
