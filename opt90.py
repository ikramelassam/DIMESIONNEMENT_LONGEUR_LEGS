"""Cas optimum recalcule avec une borne de balancement de 90 mm (grille 13x13)
au lieu de 60 mm : le pic de stabilite vers 290 mm d'empreinte se deplace-t-il ?"""
import copy, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gr2.params import Params
from gr2 import pipeline
import run_final as RF
HERE = os.path.dirname(os.path.abspath(__file__))
P = Params.load(os.path.join(HERE, "config.yaml"))
raw = copy.deepcopy(P.raw)
raw["task"]["sway_max"] = 90.0
raw["task"]["sway_n"] = 13
nm = {s["name"]: i for i, s in enumerate(raw["servos"])}
combos = [tuple(nm[x] for x in cb) for cb in RF.COMBOS]
o = pipeline.optimize(raw, "dedans", "S2", combos=combos, n_mc=0,
                      log=lambda m: print("OPT90", m, flush=True))
d = os.path.join(HERE, "results_final", "OPT90_dedans_S2"); os.makedirs(d, exist_ok=True)
o["df"].to_csv(os.path.join(d, "admissibles.csv"), index=False)
o["front"].to_csv(os.path.join(d, "pareto.csv"), index=False)
json.dump(o["res"], open(os.path.join(d, "resume.json"), "w"), indent=1, default=float)
print("FIN", {k: v for k, v in o["res"].items() if k not in ("choix", "stats")}, flush=True)
