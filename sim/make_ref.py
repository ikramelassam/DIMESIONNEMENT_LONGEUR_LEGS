"""Genere des valeurs de reference avec le programme Python (gr2/) pour
valider le portage JavaScript (sim/model.js). Usage : python sim/make_ref.py"""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from gr2.params import Params
from gr2.evaluate import Evaluator

P0 = Params.load(os.path.join(os.path.dirname(HERE), "config.yaml"))
rng = np.random.default_rng(42)
designs = [
    ("dehors", "S1", [40.0, 102.67, 42.07, 80.33, 80.88]),
    ("dedans", "S1", [40.0, 114.87, 45.2, 78.01, 79.2]),
    ("dedans", "S2", [78.74, 125.48, 48.23, 72.63, 17.4]),
    ("dehors", "S2", [60.21, 129.0, 34.93, 82.33, 75.33]),
]
b = P0["search"]["bounds"]
lo = np.array([b[k][0] for k in ("L2", "L3", "alpha", "beta", "xi")]); hi = np.array([b[k][1] for k in ("L2", "L3", "alpha", "beta", "xi")])
for i in range(8):
    x = lo + rng.random(5) * (hi - lo)
    designs.append((["dedans", "dehors"][i % 2], ["S1", "S2"][(i // 2) % 2], x.round(4).tolist()))
out = []
for mount, sc, x in designs:
    P = P0.copy(); P.raw["chassis"]["mount"] = mount; P = Params.from_dict(P.raw)
    X = np.array([x], float)
    E = Evaluator(P, sc)
    names = {s.name: i for i, s in enumerate(P.servos)}
    rows, _ = E.run(X)
    L = E.last
    # mode exact (CdG des pattes avec les servos choisis), combinaison mixte
    cbx = (names["S-U400"], names["S3306"], names["S-U400"])
    Ex = Evaluator(P, sc, cbx); rx, _ = Ex.run(X)
    exact = dict(combo=["S-U400", "S3306", "S-U400"], nesm=Ex.last["nesm"][0].tolist(), k=Ex.last["k"][0].tolist(),
                 rows=[{k: (float(v) if isinstance(v, np.floating) else v) for k, v in r.items()} for r in rx])
    out.append(dict(mount=mount, scenario=sc, x=dict(zip(("L2", "L3", "alpha", "beta", "xi"), x)),
                    geo_ok=L["geo_ok"][0].tolist(), nesm=L["nesm"][0].tolist(), ssm_c=L["ssm_c"][0].tolist(),
                    k=L["k"][0].tolist(), swA2=L["swA2"][0].tolist(), eps_legs=float(L["eps_legs_xy"][0]),
                    rows=[{k: (v if not isinstance(v, (np.floating,)) else float(v)) for k, v in r.items()} for r in rows],
                    combo_index=names, exact=exact))
cfg = P0.raw
json.dump(dict(config=cfg, cases=out), open(os.path.join(HERE, "ref.json"), "w"), default=float)
print("cas :", len(out), " lignes admissibles :", sum(len(c["rows"]) for c in out))
