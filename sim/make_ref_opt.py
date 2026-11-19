"""Donnees de reference Python pour valider sim/opt.js : lignes admissibles reelles
(results_exact), Pareto 3D, couches, coude, epsilon-contrainte calcules par gr2/explore.py."""
import json, os, sys
import numpy as np, pandas as pd, yaml
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from gr2 import explore as ex
cfg = yaml.safe_load(open(os.path.join(ROOT, "config.yaml"), encoding="utf-8"))
cases = []
for case in ("dedans_S2", "dehors_S1"):
    df = pd.read_csv(os.path.join(ROOT, "results_exact", case, "admissibles.csv"))
    rng = np.random.default_rng(3)
    sub = df.iloc[np.sort(rng.choice(len(df), 700, replace=False))].reset_index(drop=True)
    sub = sub.assign(_i=np.arange(len(sub)))
    fr = ex.pareto(sub); ly = ex.pareto_layers(sub, 6, 60)
    ky = ex.knee_point(fr); ec = ex.eps_choice(fr, 2.0)
    cases.append(dict(case=case, mount=case.split("_")[0], sc=case.split("_")[1],
                      rows=json.loads(sub.drop(columns=["_i"]).to_json(orient="records")),
                      pareto=[int(i) for i in fr["_i"]], layers=[int(i) for i in ly["_i"]],
                      knee=int(fr.loc[ky, "_i"]), eps=None if ec is None else int(fr.loc[ec, "_i"])))
json.dump(dict(config=cfg, cases=cases), open(os.path.join(HERE, "ref_opt.json"), "w"), default=float)
print("ok", [(c["case"], len(c["pareto"]), len(c["layers"]), c["knee"], c["eps"]) for c in cases])
