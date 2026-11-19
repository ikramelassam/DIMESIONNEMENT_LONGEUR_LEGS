"""Plus grande empreinte au sol tenable avec les servos du FabLab (sans Dynamixel).

Trois reponses, parce qu'elles ne tombent pas au meme endroit :
  E_faisable : il existe AU MOINS UN design admissible (toutes contraintes, 14 phases)
  E_utile    : il existe un design admissible de NESM >= margins.nesm_cible (2 mm)
  E_robuste  : un tel design passe aussi >= robust.pass_rate (95 %) du Monte-Carlo

Methode, pour chaque empreinte cible E :
  1. Sobol sur (L2, L3, alpha, xi), puis on RESOUT beta pour tomber exactement sur
     la surface {empreinte = E}  (montage dedans : E = 190 + 2 D max(sin xi, cos xi)).
  2. Evaluation exacte (14 phases x 3 etats x grille de balancement) pour chacune
     des 6 combinaisons de servos du labo (C2 = S3306 obligatoire).
  3. Un tour de raffinement local autour des designs admissibles, re-projetes sur
     la surface (pousse la frontiere au-dela de ce que Sobol a trouve).
  4. Monte-Carlo sur jusqu'a 40 designs de NESM >= cible, pris dans les 6 premieres couches
     de non-dominance (meme selection que le pipeline). Pour ce test, la
     contrainte d'empreinte minimale est relachee : on teste si le robot MARCHE
     malgre les tolerances, pas si son empreinte reste dans la bande a 1 mm pres.

Usage :
  python empreinte_max.py                     # 300 a 390 mm par pas de 10, xi libre
  python empreinte_max.py 345 350 375         # empreintes au choix (mm)
  python empreinte_max.py --xi45 260 280      # xi impose a 45 deg
  python empreinte_max.py --rapide 345        # sans raffinement ni Monte-Carlo
  python empreinte_max.py --servos S3306,S3306,S3306 300 320   # un seul triplet C1,C2,C3
Sortie : results_final/empreinte_max[_xi45]_<E...>.json, et un resume a l'ecran
avec les designs a recopier dans le simulateur (sim/gr2_simulateur.html).
"""
import copy, json, os, sys
import numpy as np
import pandas as pd
from scipy.stats import qmc

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gr2.params import Params
from gr2 import explore as ex
from gr2.pipeline import robust_exact
import run_final as RF

HERE = os.path.dirname(os.path.abspath(__file__))
BAND = RF.BAND            # empreinte acceptee dans [E - 4, E]
N_ECH = 2048              # geometries par empreinte
N_PARENTS, N_ENFANTS = 64, 16
N_ROBUSTE = 40          # candidats testes au Monte-Carlo (6 couches de non-dominance, 40 au plus)


def config(E, xi_fixed):
    raw = copy.deepcopy(Params.load(os.path.join(HERE, "config.yaml")).raw)
    raw["chassis"]["mount"] = "dedans"
    raw["gabarit"] = max(E, raw.get("gabarit", 400.0))
    raw["gabarit_min"] = E - BAND
    if xi_fixed is not None:
        raw["search"]["bounds"]["xi"] = [xi_fixed, xi_fixed + 1e-6]
    if RF.MASSE_MC:
        raw["robust"]["masse_max_mc"] = RF.MASSE_MC
    return raw


def projeter(raw, S4, E):
    """(L2, L3, alpha, xi) -> (L2, L3, alpha, beta, xi) sur la surface {empreinte = E}."""
    b = raw["search"]["bounds"]
    L2, L3, al, xi = S4.T
    base, k = RF.footprint_coeff(raw, xi)
    D0 = (E - 0.5 - base) / (2 * k)
    cb = (D0 - L2 * np.cos(np.radians(al))) / L3
    ok = (np.abs(cb) <= 1.0) & (D0 > 0)
    be = np.degrees(np.arccos(np.clip(cb, -1, 1)))
    ok &= (be >= b["beta"][0]) & (be <= b["beta"][1])
    return np.stack([L2, L3, al, be, xi], 1)[ok]


def essai(E, xi_fixed=None, rapide=False, seed=1):
    raw = config(E, xi_fixed)
    b = raw["search"]["bounds"]
    lo = np.array([b["L2"][0], b["L3"][0], b["alpha"][0], b["xi"][0]])
    hi = np.array([b["L2"][1], b["L3"][1], b["alpha"][1], b["xi"][1]])
    X = projeter(raw, qmc.scale(qmc.Sobol(d=4, scramble=True, seed=seed).random(1 << 15), lo, hi), E)
    if len(X) > N_ECH:
        X = X[np.linspace(0, len(X) - 1, N_ECH).astype(int)]
    P = Params.from_dict(raw)
    nm = {s["name"]: i for i, s in enumerate(raw["servos"])}
    rng = np.random.default_rng(7)
    frames, lignes = [], []
    for names in RF.COMBOS:
        c = tuple(nm[x] for x in names)
        df, _ = ex.evaluate_all(P, "S2", X, exact_combo=c)
        n0 = len(df)
        if not rapide and n0:            # raffinement local, re-projete sur la surface
            par = df.nlargest(min(N_PARENTS, n0), "f2_nesm")[["L2", "L3", "alpha", "xi"]].values
            span = hi - lo
            kids = np.repeat(par, N_ENFANTS, 0) + rng.uniform(-0.05, 0.05, (len(par) * N_ENFANTS, 4)) * span
            kids = np.clip(kids, lo, hi)
            Xk = projeter(raw, kids, E)
            if len(Xk):
                dk, _ = ex.evaluate_all(P, "S2", Xk, exact_combo=c)
                df = dk if not n0 else pd.concat([df, dk], ignore_index=True)
        cible = raw["margins"]["nesm_cible"]
        r = dict(E=E, combo="/".join(names), echantillon=len(X), admissibles_sobol=n0,
                 admissibles=int(len(df)))
        if len(df):
            r.update(nesm_max=round(float(df.f2_nesm.max()), 3),
                     n_nesm_cible=int((df.f2_nesm >= cible).sum()),
                     C2_min=round(float(df.f3_couple_C2.min()), 3),
                     masse_pattes_min=round(float(df.f1_masse_pattes.min()), 3))
            frames.append(df)
        print(json.dumps(r, ensure_ascii=False), flush=True)
        lignes.append(r)

    res = dict(E=E, xi=xi_fixed, combos=lignes, faisable=any(l["admissibles"] for l in lignes))
    if not frames:
        res.update(utile=False, robuste=False)
        return res

    allf = pd.concat(frames, ignore_index=True)
    cible = raw["margins"]["nesm_cible"]
    res["meilleur_nesm"] = _design(allf.loc[allf.f2_nesm.idxmax()])
    utiles = allf[allf.f2_nesm >= cible]
    res["utile"] = bool(len(utiles))
    res["robuste"] = False
    if len(utiles):
        res["meilleur_utile"] = _design(utiles.loc[utiles.f2_nesm.idxmax()])
        if not rapide:
            relache = copy.deepcopy(raw); relache["gabarit_min"] = 0.0
            # meme selection que le pipeline : couches de non-dominance successives
            # (les points du 1er front sont souvent colles aux limites, donc fragiles)
            cand = ex.pareto_layers(utiles, 6, N_ROBUSTE)
            rob = robust_exact(Params.from_dict(relache), "S2", cand, seed=11)
            ok = rob[rob.taux_reussite >= raw["robust"]["pass_rate"]]
            res["taux_reussite_meilleurs"] = [round(float(t), 2) for t in rob.taux_reussite]
            res["robuste"] = bool(len(ok))
            if len(ok):
                res["meilleur_robuste"] = _design(ok.loc[ok.f2_nesm.idxmax()])
    return res


def _design(row):
    keys = ("L2", "L3", "alpha", "beta", "xi", "D", "h", "f1_masse_pattes", "f2_nesm",
            "f3_couple_C2", "servo_C1", "servo_C2", "servo_C3", "taux_reussite")
    return {k: (round(float(row[k]), 3) if isinstance(row[k], (int, float, np.floating)) else row[k])
            for k in keys if k in row.index}


def main(argv):
    argv = list(argv)
    suffixe = RF.impose_servos(argv)   # --servos S3306,S3306,S3306 : un seul triplet
    suffixe += RF.impose_masse_mc(argv)  # --masse_mc 4.3 : limite de masse relachee au Monte-Carlo
    xi = 45.0 if "--xi45" in argv else None
    rapide = "--rapide" in argv
    cibles = [float(a) for a in argv if not a.startswith("--")] or [300.0 + 10 * i for i in range(10)]
    out = []
    for E in cibles:
        r = essai(E, xi, rapide)
        out.append(r)
        print(f"  >> E = {E:.0f} mm : faisable {r['faisable']}, utile {r['utile']}, robuste {r['robuste']}", flush=True)
    seuil = lambda key: max((r["E"] for r in out if r[key]), default=None)
    res = dict(xi=xi or "libre", E_faisable=seuil("faisable"), E_utile=seuil("utile"),
               E_robuste=seuil("robuste"), detail=out)
    tag = ("_xi45" if xi else "") + suffixe + "_" + "-".join(str(int(e)) for e in cibles)
    os.makedirs(os.path.join(HERE, "results_final"), exist_ok=True)
    json.dump(res, open(os.path.join(HERE, "results_final", f"empreinte_max{tag}.json"), "w"),
              indent=1, ensure_ascii=False, default=float)
    print(f"\nPARMI LES EMPREINTES TESTEES : faisable jusqu'a {res['E_faisable']} mm, "
          f"utile (NESM >= cible) jusqu'a {res['E_utile']} mm, robuste jusqu'a {res['E_robuste']} mm")
    for r in out:
        d = r.get("meilleur_robuste") or r.get("meilleur_utile") or r.get("meilleur_nesm")
        if d:
            print(f"  E = {r['E']:.0f} : L2 {d['L2']:.1f}  L3 {d['L3']:.1f}  alpha {d['alpha']:.1f}  "
                  f"beta {d['beta']:.1f}  xi {d['xi']:.1f}  | {d['servo_C1']}/{d['servo_C2']}/{d['servo_C3']}  "
                  f"| masse {d['f1_masse_pattes']:.3f} kg  NESM {d['f2_nesm']:.2f} mm  C2 {d['f3_couple_C2']:.3f} N.m")


if __name__ == "__main__":
    main(sys.argv[1:])
