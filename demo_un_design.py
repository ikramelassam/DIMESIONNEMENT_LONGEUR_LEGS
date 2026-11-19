"""Suivre UN design pas a pas dans le code, pour comprendre ce qu'il fait.

On donne une patte (L2, L3, alpha, beta, xi) et trois servos ; le script affiche
chaque etape exactement comme le programme la calcule :
  0. ce qui est demande (et les reglages de config.yaml modifies)
  1. geometrie        -> gr2/evaluate.py, geometry()
  2. pre-filtre       -> gr2/evaluate.py, prefilter()
  3. les 14 phases    -> Evaluator.run() puis _resolve_servos()
  4. courses et masse -> _resolve_servos()
  5. verdict et objectifs f1, f2, f3

Exemples :
  python demo_un_design.py                         # le design retenu (xi = 77,3)
  python demo_un_design.py --xi 45                 # la meme patte, tournee a 45 deg
  python demo_un_design.py --xi 45 --servos SG90,S3306,S3306
  python demo_un_design.py --set task.obstacle=40  # n'importe quel reglage de config.yaml
  python demo_un_design.py --set margins.SF=1.5 --set frac_couple=0.7
"""
import argparse, copy, os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gr2.params import Params, G
from gr2.evaluate import geometry, prefilter, Evaluator

HERE = os.path.dirname(os.path.abspath(__file__))
PATTES = ["avant-gauche FL", "avant-droite FR", "arriere-gauche RL", "arriere-droite RR"]
EXPLIQUE = {
    "G5_garde_sol": "garde au sol h >= obstacle + marge",
    "G3_theta3_repos": "angle de genou au repos dans [theta3_min ; theta3_max]",
    "G6_gabarit": "empreinte <= gabarit (400 mm)",
    "G6b_empreinte_min": "empreinte >= empreinte imposee",
    "G7_axes_C1": "axes C1 assez ecartes",
    "G10_chassis": "chassis <= gabarit",
}


def regle(raw, texte):
    """--set task.obstacle=40 : modifie raw['task']['obstacle'].
    --set servos.S-U400.C_stall=0.638 : dans une liste, on designe l'element par son nom."""
    cle, val = texte.split("=", 1)
    d = raw
    parts = cle.split(".")
    for p in parts[:-1]:
        d = next(e for e in d if e.get("name") == p) if isinstance(d, list) else d[p]
    ancien = d.get(parts[-1])
    try:
        val = float(val)
    except ValueError:
        pass
    d[parts[-1]] = val
    return cle, ancien, val


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--L2", type=float, default=60.084)
    ap.add_argument("--L3", type=float, default=71.924)
    ap.add_argument("--alpha", type=float, default=10.548)
    ap.add_argument("--beta", type=float, default=99.762)
    ap.add_argument("--xi", type=float, default=77.32)
    ap.add_argument("--servos", default="SG90,S3306,S-U400", help="C1,C2,C3")
    ap.add_argument("--sc", default="S2", choices=["S1", "S2"])
    ap.add_argument("--mount", default="dedans", choices=["dedans", "dehors"])
    ap.add_argument("--set", action="append", default=[], help="cle.de.config=valeur")
    a = ap.parse_args()

    # ------------------------------------------------------------ etape 0
    raw = copy.deepcopy(Params.load(os.path.join(HERE, "config.yaml")).raw)
    raw["chassis"]["mount"] = a.mount
    print("=" * 78)
    print("ETAPE 0 - CE QUE TU DEMANDES")
    print(f"  design x = (L2, L3, alpha, beta, xi) = ({a.L2}, {a.L3}, {a.alpha}, {a.beta}, {a.xi})")
    print("  -> ce vecteur est UNE ligne du tableau X. Dans l'optimiseur, X a 8 192 lignes tirees")
    print("     par Sobol entre les bornes search.bounds ; ici on en donne une a la main.")
    for t in a.set:
        cle, ancien, val = regle(raw, t)
        print(f"  reglage modifie : {cle} = {val}   (valeur de config.yaml : {ancien})")
    P = Params.from_dict(raw)
    names = a.servos.split(",")
    nm = {s.name: i for i, s in enumerate(P.servos)}
    combo = tuple(nm[n] for n in names)
    s1, s2, s3 = (P.servos[i] for i in combo)
    print(f"  servos C1 / C2 / C3 = {s1.name} / {s2.name} / {s3.name} ; scenario {a.sc} ; montage {a.mount}")
    X = np.array([[a.L2, a.L3, a.alpha, a.beta, a.xi]])

    # ------------------------------------------------------------ etape 1
    g = geometry(X, P)
    print("=" * 78)
    print("ETAPE 1 - GEOMETRIE  (geometry(), evaluate.py l.25)")
    print(f"  D  = L2 cos(alpha) + L3 cos(beta)  = {g['D0'][0]:7.2f} mm   portee horizontale")
    print(f"  H2 = L3 sin(beta) - L2 sin(alpha)  = {g['H2'][0]:7.2f} mm   hauteur de l'axe C2")
    print(f"  h  = H2 - e                        = {g['h'][0]:7.2f} mm   garde au sol (e = {raw['chassis']['e']})")
    print(f"  xi = {a.xi} deg -> direction de la patte vue de dessus u = (sin xi, cos xi) = "
          f"({np.sin(np.radians(a.xi)):.3f}, {np.cos(np.radians(a.xi)):.3f})")
    print("  pied = coin du chassis + D x u :")
    for i, nom in enumerate(PATTES):
        f = g["foot"][0, i]
        print(f"     {nom:18s} x = {f[0]:7.1f}  y = {f[1]:7.1f} mm")
    ex = 2 * np.abs(g["foot"][0, :, 0]).max() + raw["leg"]["foot_diam"]
    ey = 2 * np.abs(g["foot"][0, :, 1]).max() + raw["leg"]["foot_diam"]
    print(f"  empreinte au sol = {ex:.1f} x {ey:.1f} mm")

    # ------------------------------------------------------------ etape 2
    ok0, viol, _ = prefilter(X, P)
    print("=" * 78)
    print("ETAPE 2 - PRE-FILTRE  (prefilter(), evaluate.py l.49) : chaque critere = un nombre <= 0")
    for k_, v in viol.items():
        v = float(np.atleast_1d(v)[0])
        print(f"  {k_:20s} = {v:+.3f}  {'OK ' if v <= 0 else 'NON'}  ({EXPLIQUE.get(k_, '')})")
    print(f"  => pre-filtre {'PASSE' if ok0[0] else 'ECHOUE : le design est rejete ici'}")

    # ------------------------------------------------------------ etape 3
    E = Evaluator(P, a.sc, exact_combo=combo)
    rows, _ = E.run(X)
    L = E.last
    geo_ok, nesm, ssm_c, k = L["geo_ok"][0], L["nesm"][0], L["ssm_c"][0], L["k"][0]
    mg = raw["margins"]
    legm = P.leg_mass(a.L2, a.L3, s1.mass, s2.mass, s3.mass)
    Mc = E.m_body_max + 4 * legm                                   # comme l.309
    tau = Mc * G * k / 1000.0                                      # comme l.310
    Cu = np.array([s1.C_cont, s2.C_cont, s3.C_cont]) / mg["SF"]    # comme l.311
    tq = (np.all(tau <= Cu, -1) & (L["swA2"][0] + L["swB2"][0] * s3.mass <= Cu[1])
          & (L["swA3"][0] <= Cu[2]))                               # comme l.312
    stab = geo_ok & (nesm >= mg["nesm_min"]) & (ssm_c >= mg.get("s_min", 0.0))   # comme l.301
    feas = stab & tq
    nS = len(E.sw)
    print("=" * 78)
    print(f"ETAPE 3 - LES 14 PHASES  (Evaluator.run + _resolve_servos)  : {nS} positions du corps par phase")
    print(f"  masse du robot en charge M = {Mc:.3f} kg ; limites utilisables C1/C2/C3 = "
          f"{Cu[0]:.3f} / {Cu[1]:.3f} / {Cu[2]:.3f} N.m (= frac x blocage / SF)")
    print(f"  {'phase':14s} {'geom':>5s} {'+stab':>6s} {'+couple':>8s}   meilleure position : NESM et couples C1/C2/C3")
    best_idx = []
    for p_, ph in enumerate(E.phases):
        ng, ns, nf = int(geo_ok[p_].sum()), int(stab[p_].sum()), int(feas[p_].sum())
        if nf:
            b = int(np.where(feas[p_], nesm[p_], -np.inf).argmax())      # comme l.315
            best_idx.append(b)
            t = tau[p_, b]
            print(f"  {ph.name:14s} {ng:5d} {ns:6d} {nf:8d}   corps decale de ({E.sw[b][0]:+.0f}, {E.sw[b][1]:+.0f}) mm, "
                  f"NESM {nesm[p_, b]:5.2f} mm, {t[0]:.3f} / {t[1]:.3f} / {t[2]:.3f}")
        else:
            best_idx.append(None)
            if ng == 0:
                pourquoi = "aucune position geometriquement possible (portee, genou, collisions)"
            elif ns == 0:
                pourquoi = "aucune position stable"
            else:
                r = np.where(stab[p_][:, None], tau[p_] / Cu, np.inf).min(0)
                j = int(np.argmax(r))
                pourquoi = (f"couple : C{j + 1} demande au mieux {r[j] * Cu[j]:.3f} N.m "
                            f"pour une limite de {Cu[j]:.3f}")
            print(f"  {ph.name:14s} {ng:5d} {ns:6d} {nf:8d}   ECHEC -> {pourquoi}")
    toutes = all(b is not None for b in best_idx)
    print(f"  => {'les 14 phases passent' if toutes else 'au moins une phase echoue : design REJETE'}")

    # ------------------------------------------------------------ etape 4
    print("=" * 78)
    print("ETAPE 4 - COURSES ANGULAIRES ET MASSE  (_resolve_servos, l.320-325)")
    if toutes:
        amin = np.stack([L["ang_min"][0, p_, b] for p_, b in enumerate(best_idx)])   # (P,4,3)
        amax = np.stack([L["ang_max"][0, p_, b] for p_, b in enumerate(best_idx)])
        span = (amax.max(0) - amin.min(0)).max(0)
        rng = np.array([s1.range, s2.range, s3.range]) - 2 * mg["joint_margin"]
        for j in range(3):
            print(f"  course C{j + 1} = {span[j]:6.1f} deg  (disponible {rng[j]:.0f})  {'OK' if span[j] <= rng[j] else 'NON'}")
    else:
        print("  (non verifie : le design est deja rejete a l'etape 3)")
    b = raw["body"]
    mrob = b["chassis"]["m"] + b["battery"]["m"] + 4 * legm
    print(f"  masse chassis + batterie + 4 pattes = {mrob:.3f} kg  (max {b['mass_robot_max_sans_bras']})"
          f"  {'OK' if mrob <= b['mass_robot_max_sans_bras'] else 'NON'}")

    # ------------------------------------------------------------ etape 5
    print("=" * 78)
    print("ETAPE 5 - VERDICT DU PROGRAMME")
    if rows:
        r = rows[0]
        print("  ADMISSIBLE")
        print(f"  f1 masse des 4 pattes  = {r['f1_masse_pattes']:.3f} kg")
        print(f"  f2 NESM (pire phase)   = {r['f2_nesm']:.2f} mm   phase critique : {r['phase_critique']}")
        print(f"  f3 couple C2 requis    = {r['f3_couple_C2']:.3f} N.m")
    else:
        print("  REJETE : ce design ne sera jamais dans les resultats de l'optimiseur.")
    print("=" * 78)


if __name__ == "__main__":
    main()
