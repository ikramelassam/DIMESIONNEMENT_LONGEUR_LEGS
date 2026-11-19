"""Audit de coherence, cote Python : produit audit/ref_audit.json.
Pour un grand echantillon de designs (points des fronts sauvegardes, admissibles
quelconques, designs volontairement HORS contraintes), ecrit ce que le programme
Python repond : prefiltre geometrique (ok + violations), admissibilite pour chaque
combinaison de servos, valeurs des objectifs et couples, et le nombre de points du
front de Pareto de reference qui dominent le design.
Usage : python audit/audit_make.py [dedans_S2|dedans_S1|dehors_S2|dehors_S1]   (sans argument : les 4 cas ; avec argument : un cas, a lancer en parallele)"""
import json, os, sys, itertools
import numpy as np, pandas as pd
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from gr2.params import Params
from gr2.evaluate import Evaluator, prefilter
from gr2 import explore as ex

VARS = ex.VARS
CASES = [("dedans", "S2"), ("dedans", "S1"), ("dehors", "S2"), ("dehors", "S1")]
SERVOS = ["SG90", "S-U400", "S3306"]
rng = np.random.default_rng(2026)
P0 = Params.load(os.path.join(ROOT, "config.yaml"))
lo, hi = ex.bounds(P0)
KEYS = ["f1_masse_pattes", "f2_nesm", "f3_couple_C2", "tau_C1", "tau_C2", "tau_C3", "tau_min_C1", "tau_min_C2", "tau_min_C3",
        "span_C1", "span_C2", "span_C3", "M_total", "frac_bloc_C1", "frac_bloc_C2", "frac_bloc_C3", "ssm_corr_min", "phase_critique"]

def dominators(front, f):
    """nombre de points du front qui dominent f=(f1,f2,f3) (f1,f3 a minimiser, f2 a maximiser)"""
    a = (front.f1_masse_pattes.values <= f[0]) & (front.f2_nesm.values >= f[1]) & (front.f3_couple_C2.values <= f[2])
    b = (front.f1_masse_pattes.values < f[0]) | (front.f2_nesm.values > f[1]) | (front.f3_couple_C2.values < f[2])
    return int((a & b).sum())

out = []
ONLY = sys.argv[1] if len(sys.argv) > 1 else None
for mount, sc in CASES:
    if ONLY and ONLY != f"{mount}_{sc}": continue
    d = os.path.join(ROOT, "results_exact", f"{mount}_{sc}")
    fr = pd.read_csv(os.path.join(d, "pareto.csv")); rf = pd.read_csv(os.path.join(d, "pareto_robuste.csv")); ad = pd.read_csv(os.path.join(d, "admissibles.csv"))
    # --- designs : (a) fronts, (b) admissibles tires au hasard, (c) hors contraintes
    X = [("front_robuste", r[list(VARS)].values.astype(float)) for _, r in rf.iterrows()]
    X += [("front", r[list(VARS)].values.astype(float)) for _, r in fr.sample(min(30, len(fr)), random_state=1).iterrows()]
    X += [("admissible", r[list(VARS)].values.astype(float)) for _, r in ad.sample(40, random_state=2).iterrows()]
    span = hi - lo
    for _ in range(40):                       # uniformes dans les bornes (la plupart non admissibles)
        X.append(("alea_bornes", lo + rng.random(5) * span))
    for _ in range(40):                       # hors des bornes de recherche (+-35 %)
        X.append(("hors_bornes", np.clip(lo - 0.35 * span + rng.random(5) * 1.7 * span, [10, 20, -15, 0, 0], [220, 280, 89, 130, 90])))
    base = rf.iloc[0][list(VARS)].values.astype(float)
    for k in range(5):                        # perturbations d'un seul parametre autour d'un optimum
        for f in (-0.5, -0.25, -0.1, 0.1, 0.25, 0.5):
            x = base.copy(); x[k] = x[k] * (1 + f) if k < 2 else x[k] + f * 60
            X.append((f"perturb_{VARS[k]}", x))
    Xa = np.array([x for _, x in X]); tags = [t for t, _ in X]
    P = P0.copy(); P.raw["chassis"]["mount"] = mount; P = Params.from_dict(P.raw)
    ok, viol, _ = prefilter(Xa, P)
    names = {s.name: i for i, s in enumerate(P.servos)}
    res = [dict(case=f"{mount}_{sc}", mount=mount, sc=sc, tag=tags[i], x=dict(zip(VARS, map(float, Xa[i]))), pre_ok=bool(ok[i]),
                pre_viol={k: float(v[i]) for k, v in viol.items()}, combos={}) for i in range(len(Xa))]
    for combo in itertools.product(SERVOS, repeat=3):
        cidx = tuple(names[n] for n in combo)
        E = Evaluator(P, sc, cidx)
        idx = np.where(ok)[0]
        rows, _ = E.run(Xa[idx])
        got = {tuple(round(r[v], 6) for v in VARS): r for r in rows}      # cle = les CINQ variables (L2, L3, alpha seuls : collisions)
        assert len(got) == len(rows) or len(set(map(tuple, np.round(Xa[idx], 6)))) < len(idx)
        for i in range(len(Xa)):
            key = tuple(round(float(Xa[i, j]), 6) for j in range(5))
            r = got.get(key) if ok[i] else None
            ent = dict(adm=r is not None)
            if r is not None:
                ent["v"] = {k: (float(r[k]) if k != "phase_critique" else r[k]) for k in KEYS}
                ent["dom"] = dominators(fr, (r["f1_masse_pattes"], r["f2_nesm"], r["f3_couple_C2"]))
            res[i]["combos"]["/".join(combo)] = ent
    n_adm = sum(1 for r in res for c in r["combos"].values() if c["adm"])
    print(mount, sc, "designs", len(res), "paires admissibles", n_adm, flush=True)
    out += res
json.dump(out, open(os.path.join(HERE, f"ref_audit_{ONLY}.json" if ONLY else "ref_audit.json"), "w"))
print("total designs", len(out))
