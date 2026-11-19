"""Exploration, front de Pareto, raffinement, robustesse, sensibilite, decision."""
from __future__ import annotations
import json
import numpy as np
import pandas as pd
from scipy.stats import qmc
from .params import Params
from .evaluate import Evaluator, prefilter

VARS = ("L2", "L3", "alpha", "beta", "xi")


def bounds(P):
    b = P["search"]["bounds"]
    return np.array([b[k][0] for k in VARS]), np.array([b[k][1] for k in VARS])


def sobol(P, n, seed):
    lo, hi = bounds(P)
    s = qmc.Sobol(d=5, scramble=True, seed=seed).random(n)
    return qmc.scale(s, lo, hi)


def evaluate_all(P, scenario, X, chunk=256, exact_combo=None):
    ok, viol, _ = prefilter(X, P)
    E = Evaluator(P, scenario, exact_combo)
    rows, n_geo, n_stab = [], 0, 0
    Xo = X[ok]
    for i in range(0, len(Xo), chunk):
        r, st = E.run(Xo[i:i + chunk])
        rows += r
        n_geo += st["geo_any"]
        n_stab += st["stab_any"]
    stats = dict(n_echantillons=len(X), apres_prefiltre=int(ok.sum()),
                 rejets_prefiltre={k: int((v > 0).sum()) for k, v in viol.items()},
                 geometrie_ok=n_geo, stabilite_ok=n_stab,
                 couples_designs=len(rows),
                 designs_geometriques_admissibles=len({tuple(np.round([r[k] for k in VARS], 6)) for r in rows}))
    return pd.DataFrame(rows), stats


OBJ = ("f1_masse_pattes", "f2_nesm", "f3_couple_C2")   # min, max, min


def _F(df):
    """Matrice des objectifs, tous ramenes a minimiser."""
    return np.stack([df["f1_masse_pattes"].values, -df["f2_nesm"].values,
                     df["f3_couple_C2"].values], 1)


def pareto_mask_F(F):
    """Non-dominance (minimisation de toutes les colonnes). Algorithme 'skyline' :
    apres tri lexicographique, un point ne peut etre domine que par un point
    place avant lui ; on le compare au front courant (par transitivite, il
    suffit de comparer aux points non domines)."""
    order = np.lexsort(F.T[::-1])
    keep = np.zeros(len(F), bool)
    front = np.empty((0, F.shape[1]))
    for i in order:
        f = F[i]
        if len(front):
            dom = np.all(front <= f + 1e-12, 1) & np.any(front < f - 1e-12, 1)
            if dom.any():
                continue
            if np.any(np.all(np.abs(front - f) <= 1e-12, 1)):
                continue                       # doublon exact
        keep[i] = True
        front = np.vstack([front, f])
    return keep


def pareto(df):
    if df.empty:
        return df
    m = pareto_mask_F(_F(df))
    return df[m].sort_values("f3_couple_C2").reset_index(drop=True)


def pareto_layers(df, k, cap):
    """Union des k premiers fronts de non-dominance (couches de NSGA-II).
    Les points du 1er front sont souvent colles aux contraintes donc fragiles :
    on teste aussi les couches suivantes. Si trop de points : sous-echantillon
    regulier en couple requis."""
    rest = df.copy()
    layers = []
    for _ in range(k):
        if rest.empty:
            break
        m = pareto_mask_F(_F(rest))
        layers.append(rest[m])
        rest = rest[~m]
    out = pd.concat(layers).drop_duplicates() if layers else df.iloc[:0]
    if len(out) > cap:
        out = out.sort_values("f3_couple_C2").iloc[np.linspace(0, len(out) - 1, cap).astype(int)]
    return out.reset_index(drop=True)


def refine(P, scenario, df, rounds, per_point, seed):
    """Raffinement local : on re-echantillonne autour des points du front
    (boite de +-5 % de la plage de chaque variable) et on fusionne."""
    lo, hi = bounds(P)
    rng = np.random.default_rng(seed)
    for r in range(rounds):
        front = pareto(df)
        if front.empty:
            break
        X0 = front[list(VARS)].drop_duplicates().values
        if len(X0) > 150:          # front 3D large : on garde 150 geometries regulierement
            X0 = X0[np.linspace(0, len(X0) - 1, 150).astype(int)]
        X = np.repeat(X0, per_point, 0) + rng.uniform(-1, 1, (len(X0) * per_point, 5)) * 0.05 * (hi - lo)
        X = np.clip(X, lo, hi)
        new, _ = evaluate_all(P, scenario, X)
        df = pd.concat([df, new], ignore_index=True)
    return df


# ------------------------------------------------------------- robustesse
def perturbed(P, rng):
    """Une realisation aleatoire des incertitudes (entrees perturbees)."""
    R = P["robust"]
    Q = P.copy()
    raw = Q.raw
    b = raw["body"]
    for key in ("chassis", "battery"):
        b[key]["m"] *= 1 + rng.uniform(-R["rel_mass"], R["rel_mass"])
        b[key]["x"] += rng.uniform(-R["abs_cdg"], R["abs_cdg"])
        b[key]["y"] += rng.uniform(-R["abs_cdg"], R["abs_cdg"])
        b[key]["z_rel"] += rng.uniform(-R["abs_cdg"], R["abs_cdg"])
    am = b["arm_model"]
    for key in ("m_base", "m_elbow", "m_tip"):
        am[key] *= 1 + rng.uniform(-R["rel_mass"], R["rel_mass"])
    am["mount_from_front"] += rng.uniform(-R["abs_cdg"], R["abs_cdg"])   # tout le bras se deplace
    am["mount_y"] += rng.uniform(-R["abs_cdg"], R["abs_cdg"])
    am["mount_z_rel"] += rng.uniform(-R["abs_cdg"], R["abs_cdg"])
    for s in raw["servos"]:
        s["C_stall"] *= 1 - rng.uniform(0, R["rel_torque"])
        s["mass"] *= 1 + rng.uniform(-R["rel_mass"], R["rel_mass"])
    lg = raw["leg"]
    lg["seg_lin_mass"] *= 1 + rng.uniform(-R["rel_mass"], R["rel_mass"])
    lg["L1"] += rng.uniform(-R["abs_len"], R["abs_len"])
    raw["chassis"]["e"] += rng.uniform(-R["abs_len"], R["abs_len"])
    return Params.from_dict(raw)


def robustness(P, scenario, front, seed):
    """Taux de reussite de chaque point du front sous incertitudes
    (le balancement du corps est re-optimise : c'est un choix de commande)."""
    R = P["robust"]
    rng = np.random.default_rng(seed)
    n = R["n_mc"]
    X0 = front[list(VARS)].values
    key = list(zip(front.servo_C1, front.servo_C2, front.servo_C3))
    passes = np.zeros(len(front))
    for _ in range(n):
        Q = perturbed(P, rng)
        dX = np.concatenate([rng.uniform(-R["abs_len"], R["abs_len"], (len(X0), 2)),
                             rng.uniform(-R["abs_ang"], R["abs_ang"], (len(X0), 3))], 1)
        X = X0 + dX
        ok, _, _ = prefilter(X, Q)
        E = Evaluator(Q, scenario)
        names = {s.name: i for i, s in enumerate(Q.servos)}
        E.combos = sorted({(names[a], names[b], names[c]) for a, b, c in key})
        idx = np.nonzero(ok)[0]
        if len(idx) == 0:
            continue
        rows, _ = E.run(X[idx])
        got = {(round(r["L2"], 6), round(r["L3"], 6), r["servo_C1"], r["servo_C2"], r["servo_C3"]) for r in rows}
        for j, gi in enumerate(idx):
            if (round(X[gi, 0], 6), round(X[gi, 1], 6), *key[gi]) in got:
                passes[gi] += 1
    out = front.copy()
    out["taux_reussite"] = passes / n
    return out


def pessimistic(P):
    """Contrepartie robuste par 'recul des contraintes' : on remplace chaque
    entree incertaine par sa valeur defavorable (couple -10 %, masses +10 %,
    CdG +-10 mm de plus, garde au sol, butees et portee reculees) ; le front
    obtenu est ensuite VERIFIE par Monte-Carlo sur les entrees nominales."""
    R = P["robust"]
    Q = P.copy()
    raw = Q.raw
    for s in raw["servos"]:
        s["C_stall"] *= 1 - R["rel_torque"]
        s["mass"] *= 1 + R["rel_mass"]
    b = raw["body"]
    b["chassis"]["m"] *= 1 + R["rel_mass"]
    for key in ("m_base", "m_elbow", "m_tip"):
        b["arm_model"][key] *= 1 + R["rel_mass"]
    raw["leg"]["seg_lin_mass"] *= 1 + R["rel_mass"]
    b["eps_cdg_xy"] += R["abs_cdg"]
    b["eps_cdg_z"] += R["abs_cdg"]
    b["eps_s1"] = R["abs_cdg"]
    m = raw["margins"]
    raw["task"]["h_margin"] += R.get("backoff_h", 10.0)
    m["joint_margin"] += R["abs_ang"]
    m["reach_margin"] += R["abs_len"]
    m["s_min"] = m.get("s_min", 0.0) + R.get("backoff_nesm", 0.5)
    return Params.from_dict(raw)


# ------------------------------------------------------------- decision
def knee_point(front):
    """Point de compromis (programmation de compromis, Zeleny 1973 ; Miettinen
    1999 ch. 3.2) : objectifs normalises sur le front ([0,1], 0 = meilleur),
    on prend le point le plus proche (distance euclidienne) du point ideal."""
    if len(front) < 3:
        return front.index[0] if len(front) else None
    F = _F(front)
    Z = (F - F.min(0)) / np.maximum(np.ptp(F, 0), 1e-12)
    return front.index[int(np.linalg.norm(Z, axis=1).argmin())]


def eps_choice(front, nesm_target):
    """Epsilon-contrainte : le plus petit couple requis en C2 parmi les points
    ayant NESM >= cible (egalite departagee par la masse)."""
    ok = front[front["f2_nesm"] >= nesm_target]
    if ok.empty:
        return None
    return ok.sort_values(["f3_couple_C2", "f1_masse_pattes"]).index[0]
