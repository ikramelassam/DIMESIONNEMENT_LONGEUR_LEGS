"""Pipeline d'optimisation COMPLET (mode exact), appelable depuis un programme,
le serveur local (serve.py) ou la ligne de commande (run_exact.py).

Etapes (voir le rapport) : Sobol -> pre-filtre -> evaluation exacte de chaque
combinaison de servos -> raffinement local -> Pareto 3D -> couches de
non-dominance -> Monte-Carlo -> front robuste -> choix (coude, epsilon-contrainte).
"""
from __future__ import annotations
import copy, itertools, time
import numpy as np
import pandas as pd
from .params import Params
from . import explore as ex
from .evaluate import Evaluator, prefilter


def deep_merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def stock_ok(P: Params, combo) -> bool:
    for i in set(combo):
        st = P.servos[i].stock
        if st is not None and 4 * list(combo).count(i) > st:
            return False
    return True


def combos_from_allowed(P: Params, allowed):
    """allowed = [[noms C1], [noms C2], [noms C3]] -> combinaisons compatibles avec le stock."""
    names = {s.name: i for i, s in enumerate(P.servos)}
    lists = [[names[n] for n in a if n in names] for a in allowed]
    return [c for c in itertools.product(*lists) if stock_ok(P, c)]


def screen_servos(P: Params, sc: str, n=512, seed=5, log=None):
    """Pre-selection INDICATIVE des servos possibles par articulation : couple
    requis minimal (sur n designs de Sobol) calcule avec un servo fictif illimite
    de masse minimale (borne inferieure), compare au couple utilisable de chaque
    servo du catalogue. Un servo est retenu pour l'articulation j s'il atteint ce minimum."""
    Q = P.copy()
    mmin = min(s.mass for s in Q.servos)
    Q.raw["servos"].append({"name": "ILLIMITE", "C_stall": 100.0, "mass": mmin, "range": 360.0, "stock": None})
    Q = Params.from_dict(Q.raw)
    nm = {s.name: i for i, s in enumerate(Q.servos)}
    X = ex.sobol(Q, n, seed)
    c = (nm["ILLIMITE"],) * 3
    df, _ = ex.evaluate_all(Q, sc, X, exact_combo=c)
    if df.empty:
        return dict(ok=False, minima=[None] * 3, allowed=[[], [], []], n_adm=0)
    mins = [float(df.tau_min_C1.min()), float(df.f3_couple_C2.min()), float(df.tau_min_C3.min())]
    SF = P["margins"]["SF"]
    allowed = [[s.name for s in P.servos if (s.stock is None or s.stock >= 4) and s.C_cont / SF >= 0.98 * mins[j]] for j in range(3)]
    return dict(ok=True, minima=mins, allowed=allowed, n_adm=int(len(df)))


def robust_exact(P, sc, cand, seed, log=None, stop=None):
    R = P["robust"]; rng = np.random.default_rng(seed)
    lim = R.get("masse_max_mc")          # limite de masse relachee DANS LE MONTE-CARLO seulement
    if lim:                              # (le design nominal reste juge a mass_robot_max_sans_bras)
        import copy as _cp
        raw2 = _cp.deepcopy(P.raw); raw2["body"]["mass_robot_max_sans_bras"] = float(lim)
        P = type(P).from_dict(raw2)
    X0 = cand[list(ex.VARS)].values
    keys = list(zip(cand.servo_C1, cand.servo_C2, cand.servo_C3))
    passes = np.zeros(len(cand))
    for it in range(R["n_mc"]):
        if stop and stop():
            break
        Q = ex.perturbed(P, rng)
        dX = np.concatenate([rng.uniform(-R["abs_len"], R["abs_len"], (len(X0), 2)),
                             rng.uniform(-R["abs_ang"], R["abs_ang"], (len(X0), 3))], 1)
        X = X0 + dX
        ok, _, _ = prefilter(X, Q)
        names = {s.name: i for i, s in enumerate(Q.servos)}
        for key in set(keys):
            idx = [i for i, k in enumerate(keys) if k == key and ok[i]]
            if not idx:
                continue
            rows, _ = Evaluator(Q, sc, tuple(names[n] for n in key)).run(X[idx])
            got = {(round(r["L2"], 6), round(r["L3"], 6)) for r in rows}
            for i in idx:
                if (round(X[i, 0], 6), round(X[i, 1], 6)) in got:
                    passes[i] += 1
        if log and (it + 1) % 10 == 0:
            log(f"Monte-Carlo : {it + 1}/{R['n_mc']} tirages")
    out = cand.copy(); out["taux_reussite"] = passes / R["n_mc"] if R["n_mc"] > 0 else 1.0   # n_mc = 0 : pas de Monte-Carlo
    return out


def optimize(raw: dict, mount: str, sc: str, allowed=None, combos=None, *, n_sobol=None, refine_rounds=None,
             refine_per_point=None, n_mc=None, seed=None, X=None, log=None, stop=None, progress=None):
    """Optimisation complete. `raw` = configuration (dict, comme config.yaml).
    `allowed` = [[noms C1], [noms C2], [noms C3]] ou `combos` = liste d'indices.
    `X` : echantillon explicite (n,5), sinon Sobol. Retourne un dict (dataframes inclus)."""
    t0 = time.time()
    log = log or (lambda s: None)
    raw = copy.deepcopy(raw); raw["chassis"]["mount"] = mount
    if n_sobol: raw["search"]["n_sobol"] = int(n_sobol)
    if refine_rounds is not None: raw["search"]["refine_rounds"] = int(refine_rounds)
    if refine_per_point: raw["search"]["refine_per_point"] = int(refine_per_point)
    if n_mc is not None: raw["robust"]["n_mc"] = int(n_mc)
    if seed is not None: raw["search"]["seed"] = int(seed)
    P = Params.from_dict(raw)
    if combos is None:
        combos = combos_from_allowed(P, allowed) if allowed else P.combos()
    combos = [tuple(c) for c in combos]
    if not combos:
        raise ValueError("aucune combinaison de servos compatible avec le stock")
    lo, hi = ex.bounds(P)
    if X is None:
        X = ex.sobol(P, P["search"]["n_sobol"], P["search"]["seed"])
    X = np.asarray(X, float)
    rng = np.random.default_rng(7)
    parts, stats = [], {}
    tot = len(combos) + 2
    for ci, ci_combo in enumerate(combos):
        if stop and stop():
            break
        cb = tuple(P.servos[i].name for i in ci_combo)
        df, st = ex.evaluate_all(P, sc, X, exact_combo=ci_combo)
        for _ in range(P["search"]["refine_rounds"]):
            fr = ex.pareto(df) if len(df) else df
            if fr.empty:
                break
            X0 = fr[list(ex.VARS)].drop_duplicates().values[:150]
            n_pp = P["search"]["refine_per_point"]
            Xr = np.clip(np.repeat(X0, n_pp, 0) + rng.uniform(-1, 1, (len(X0) * n_pp, 5)) * 0.05 * (hi - lo), lo, hi)
            new, _ = ex.evaluate_all(P, sc, Xr, exact_combo=ci_combo)
            df = pd.concat([df, new], ignore_index=True)
        stats["/".join(cb)] = dict(geometries_admissibles=st["designs_geometriques_admissibles"], apres_raffinement=len(df))
        parts.append(df)
        log(f"{'/'.join(cb)} : {stats['/'.join(cb)]}")
        if progress: progress((ci + 1) / tot)
    df = pd.concat([d for d in parts if len(d)], ignore_index=True) if any(len(d) for d in parts) else pd.DataFrame()
    front = ex.pareto(df) if len(df) else df
    cand = ex.pareto_layers(df, 6, 60) if len(df) else df
    rob = robust_exact(P, sc, cand, 11, log, stop) if len(cand) else cand
    if progress: progress((len(combos) + 1) / tot)
    rob_ok = rob[rob.taux_reussite >= P["robust"]["pass_rate"]] if len(rob) else rob
    rob_front = ex.pareto(rob_ok) if len(rob_ok) else rob_ok
    choice = {}
    if len(rob_front):
        choice["coude"] = rob_front.loc[ex.knee_point(rob_front)].to_dict()
        ec = ex.eps_choice(rob_front, P["margins"]["nesm_cible"])
        if ec is not None:
            choice["eps_contrainte"] = rob_front.loc[ec].to_dict()
        choice["couple_min"] = rob_front.loc[rob_front.f3_couple_C2.idxmin()].to_dict()
    elif len(rob):
        choice["meilleur_non_robuste"] = rob.sort_values(["taux_reussite", "f3_couple_C2"], ascending=[False, True]).iloc[0].to_dict()
    for row in choice.values():
        x = np.array([row[v] for v in ex.VARS])
        row["bornes_actives"] = [v for v, xv, l, u in zip(ex.VARS, x, lo, hi) if abs(xv - l) < 0.5 or abs(xv - u) < 0.5]
    res = dict(stats=stats, admissibles=len(df), pareto=len(front), candidats=len(cand), robustes=int(len(rob_ok)),
               pareto_robuste=len(rob_front),
               min_couple_C2=float(df.f3_couple_C2.min()) if len(df) else None,
               max_nesm=float(df.f2_nesm.max()) if len(df) else None,
               choix=choice, duree_s=round(time.time() - t0), combos=[[P.servos[i].name for i in c] for c in combos])
    return dict(res=res, df=df, front=front, cand=cand, rob=rob, rob_front=rob_front)
