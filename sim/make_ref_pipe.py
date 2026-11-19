"""Reference Python du pipeline de bout en bout sur un echantillon FIXE (sans raffinement ni Monte-Carlo, qui sont aleatoires)."""
import json, os, sys
import numpy as np, yaml
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from gr2.params import Params
from gr2 import pipeline, explore as ex
import run_exact
cfg = yaml.safe_load(open(os.path.join(ROOT, "config.yaml"), encoding="utf-8"))
P = Params.from_dict(cfg)
X = ex.sobol(P, 384, 3)
allowed = [["SG90", "S-U400", "S3306"], ["S3306"], ["S-U400", "S3306"]]
out = []
for mount, sc in (("dedans", "S2"), ("dehors", "S1")):
    o = pipeline.optimize(cfg, mount, sc, allowed=allowed, X=X, refine_rounds=0, n_mc=0)
    r = o["res"]
    key = lambda d: [[round(float(q[k]), 5) for k in ("L2", "L3", "alpha", "beta", "xi")] + [q["servo_C1"], q["servo_C2"], q["servo_C3"]] for _, q in d.iterrows()]
    out.append(dict(mount=mount, sc=sc, adm=len(o["df"]), pareto=key(o["front"]), robust_front=len(o["rob_front"]),
                    knee=o["res"]["choix"]["coude"]["L2"], f=[o["res"]["choix"]["coude"][k] for k in ("f1_masse_pattes", "f2_nesm", "f3_couple_C2")],
                    eps=o["res"]["choix"].get("eps_contrainte", {}).get("L2"), n_cand=r["candidats"], stats=r["stats"]))
json.dump(dict(config=cfg, X=X.tolist(), allowed=allowed, cases=out), open(os.path.join(HERE, "ref_pipe.json"), "w"), default=float)
print([(c["mount"], c["sc"], c["adm"], len(c["pareto"]), c["knee"]) for c in out])
