"""Recherche avec CdG des pattes EXACT pour chaque combinaison de servos
(eps_legs = 0), au lieu de la borne prudente eps_legs. Combinaisons testees :
C2 = C3 = S3306 (seul servo du stock assez fort), C1 parmi SG90 / S-U400 / S3306,
plus une combinaison temoin avec S-U400 en C3.
Usage : python run_exact.py <montage> <scenario>"""
import json, os, sys, time
import numpy as np, pandas as pd
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gr2.params import Params
from gr2 import explore as ex, pipeline
from gr2.evaluate import Evaluator, prefilter

HERE = os.path.dirname(os.path.abspath(__file__))
# C2 : S3306 obligatoire (couple C2 requis min >= 0,70 N.m > 0,58 N.m utilisable du S-U400).
# C3 : SG90 exclu (couple C3 requis min >= 0,22 N.m > 0,12 N.m). Voir sens_exact.py.
COMBOS = [(c1, "S3306", c3) for c3 in ("S-U400", "S3306") for c1 in ("SG90", "S-U400", "S3306")]


def main(mount, sc):
    import copy
    P = Params.load(os.path.join(HERE, "config.yaml"))
    names = {s.name: i for i, s in enumerate(P.servos)}
    combos = [tuple(names[n] for n in cb) for cb in COMBOS]
    out = pipeline.optimize(P.raw, mount, sc, combos=combos, log=lambda m: print(mount, sc, m, flush=True))
    res = out["res"]
    d = os.path.join(HERE, "results_exact", f"{mount}_{sc}"); os.makedirs(d, exist_ok=True)
    out["df"].to_csv(os.path.join(d, "admissibles.csv"), index=False); out["front"].to_csv(os.path.join(d, "pareto.csv"), index=False)
    out["rob"].to_csv(os.path.join(d, "candidats_robustesse.csv"), index=False); out["rob_front"].to_csv(os.path.join(d, "pareto_robuste.csv"), index=False)
    json.dump(res, open(os.path.join(d, "resume.json"), "w"), indent=1, default=float)
    print("FIN", mount, sc, {k: v for k, v in res.items() if k not in ("choix", "stats")}, flush=True)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
