"""Etude finale (17/11) : catalogue du FabLab SANS Dynamixel.

Cas etudies (montage 'dedans', scenario S2 pire cas par defaut) :
  OPT        xi libre,  empreinte libre (<= 400 mm)        -> le cas optimum
  XI45       xi = 45,   empreinte libre                    -> decision d'equipe
  E400       xi libre,  empreinte IMPOSEE a ~400 mm
  E400_XI45  xi = 45,   empreinte IMPOSEE a ~400 mm

Pour chaque cas, deux passes :
  (a) servo ILLIMITE aux 3 axes -> plancher de couple requis et NESM max
      atteignable, independamment du catalogue (dit si le cas est possible) ;
  (b) les 6 combinaisons reellement disponibles (C2 = S3306 obligatoire,
      C3 dans {S-U400, S3306}, C1 dans {SG90, S-U400, S3306}).

Echantillonnage du cas a empreinte imposee : l'empreinte vaut
  E = 2*(N/2 - c) + foot_diam + 2*D0*max(sin xi, cos xi)   [montage dedans]
donc imposer E = 400 revient a imposer D0 = D0*(xi). On tire (L2, L3, alpha, xi)
par Sobol et on RESOUT beta pour tomber exactement sur la surface, au lieu de
tirer les 5 variables et de rejeter 99 % des points.

Usage : python run_final.py <OPT|XI45|E<mm>|E<mm>_XI45> [S1|S2] [dedans|dehors]
  ex. : python run_final.py E340          -> empreinte imposee a 340 mm, xi libre
        python run_final.py E300_XI45     -> empreinte imposee a 300 mm, xi = 45 deg
"""
import copy, json, os, sys
import numpy as np
from scipy.stats import qmc

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gr2.params import Params
from gr2 import explore as ex, pipeline

HERE = os.path.dirname(os.path.abspath(__file__))
LAB = ["SG90", "S-U400", "S3306"]                 # catalogue FabLab, >= 4 unites
COMBOS = [(c1, "S3306", c3) for c3 in ("S-U400", "S3306") for c1 in LAB]


def impose_servos(argv):
    """--servos S3306,S3306,S3306 : n'etudie QUE ce triplet C1,C2,C3 (meme moteur partout, etc.).
    Retire l'option de argv et renvoie le suffixe a ajouter aux noms de fichiers."""
    global COMBOS
    if "--servos" not in argv:
        return ""
    i = argv.index("--servos")
    trio = tuple(argv[i + 1].split(","))
    assert len(trio) == 3, "--servos attend 3 noms : C1,C2,C3"
    del argv[i:i + 2]
    COMBOS = [trio]
    return "_" + "-".join(trio)


MASSE_MC = None


def impose_masse_mc(argv):
    """--masse_mc 4.3 : dans le Monte-Carlo, la limite de 4 kg devient 4,3 kg (masses +-10 %).
    Sert a separer un echec 'mecanique' d'un simple depassement du budget de masse."""
    global MASSE_MC
    if "--masse_mc" in argv:
        i = argv.index("--masse_mc")
        MASSE_MC = float(argv[i + 1])
        del argv[i:i + 2]
        return f"_mc{MASSE_MC:g}kg"
    return ""
GAB = 400.0
BAND = 4.0            # l'empreinte imposee est acceptee dans [GAB-BAND, GAB]


def footprint_coeff(raw, xi_deg):
    """E = base + 2*D0*k(xi) ; retourne (base, k)."""
    ch, lg = raw["chassis"], raw["leg"]
    s, c = np.sin(np.radians(xi_deg)), np.cos(np.radians(xi_deg))
    k = np.maximum(s, c)
    if ch["mount"] == "dedans":
        base_x = 2 * (ch["N_ch"] / 2 - ch["c"]) + lg["foot_diam"]
    else:                                  # dehors : le pied est a L1 + D0 du coin
        base_x = 2 * (ch["N_ch"] / 2 - ch["c"]) + lg["foot_diam"] + 2 * lg["L1"] * k
    return base_x, k


def sample_on_footprint(raw, n, seed, xi_fixed=None, target=None):
    """Tirage sur la surface {empreinte = target} (defaut GAB = 400) : Sobol sur (L2, L3, alpha, xi),
    beta resolu. Retourne (n_valides, 5)."""
    b = raw["search"]["bounds"]
    lo = np.array([b["L2"][0], b["L3"][0], b["alpha"][0], b["xi"][0] if xi_fixed is None else xi_fixed])
    hi = np.array([b["L2"][1], b["L3"][1], b["alpha"][1], b["xi"][1] if xi_fixed is None else xi_fixed + 1e-9])
    S = qmc.scale(qmc.Sobol(d=4, scramble=True, seed=seed).random(n), lo, hi)
    L2, L3, al, xi = S.T
    base, k = footprint_coeff(raw, xi)
    E = GAB if target is None else float(target)
    D0 = (E - 0.5 - base) / (2 * k)                   # D0 exige par l'empreinte (0,5 mm sous la cible)
    cb = (D0 - L2 * np.cos(np.radians(al))) / L3      # cos(beta)
    ok = np.abs(cb) <= 1.0
    be = np.degrees(np.arccos(np.clip(cb, -1, 1)))
    ok &= (be >= b["beta"][0]) & (be <= b["beta"][1])
    X = np.stack([L2, L3, al, be, xi], 1)[ok]
    return X


def build_case(name, sc, mount):
    P0 = Params.load(os.path.join(HERE, "config.yaml"))
    raw = copy.deepcopy(P0.raw)
    raw["chassis"]["mount"] = mount
    if MASSE_MC:
        raw["robust"]["masse_max_mc"] = MASSE_MC
    if "6V" in name:                                   # une seule alimentation 6 V (voir config.yaml)
        next(s for s in raw["servos"] if s["name"] == "S-U400")["C_stall"] = 0.638
    xi_fixed = 45.0 if name.endswith("XI45") else None
    import re
    m = re.match(r"E(\d+(?:\.\d+)?)", name)       # E400, E340, E340_XI45 ...
    forced = m is not None
    target = float(m.group(1)) if forced else None
    if xi_fixed is not None:
        raw["search"]["bounds"]["xi"] = [xi_fixed, xi_fixed + 1e-6]
    if forced:
        raw["gabarit_min"] = target - BAND
        n = raw["search"]["n_sobol"]
        X = sample_on_footprint(raw, 1 << 17, raw["search"]["seed"], xi_fixed, target)
        if len(X) > n:                                 # sous-echantillon regulier
            X = X[np.linspace(0, len(X) - 1, n).astype(int)]
        print(f"[{name}] echantillon sur la surface empreinte={target:.0f} mm : {len(X)} points", flush=True)
    else:
        X = None
    return raw, X


def run(name, sc="S2", mount="dedans"):
    raw, X = build_case(name, sc, mount)
    out_dir = os.path.join(HERE, "results_final", f"{name}_{mount}_{sc}")
    os.makedirs(out_dir, exist_ok=True)
    report = {"cas": name, "montage": mount, "scenario": sc,
              "empreinte_imposee_mm": (raw["gabarit_min"] + BAND) if raw.get("gabarit_min") else None,
              "bande_empreinte_mm": [raw["gabarit_min"], raw["gabarit_min"] + BAND] if raw.get("gabarit_min") else None}

    # ---- (a) servo ILLIMITE : le cas est-il possible, et a quel prix en couple ?
    ill = copy.deepcopy(raw)
    mmin = min(s["mass"] for s in ill["servos"])
    ill["servos"] = ill["servos"] + [{"name": "ILLIMITE", "C_stall": 1000.0, "mass": mmin,
                                      "range": 360.0, "stock": None}]
    names = {s["name"]: i for i, s in enumerate(ill["servos"])}
    o = pipeline.optimize(ill, mount, sc, combos=[(names["ILLIMITE"],) * 3], n_mc=0, X=X,
                          log=lambda m: print(f"[{name}/ILLIMITE] {m}", flush=True))
    d = o["df"]
    report["illimite"] = dict(
        admissibles=int(len(d)),
        couple_C2_min=float(d.f3_couple_C2.min()) if len(d) else None,
        couple_C3_min=float(d.tau_min_C3.min()) if len(d) else None,
        couple_C1_min=float(d.tau_min_C1.min()) if len(d) else None,
        nesm_max=float(d.f2_nesm.max()) if len(d) else None,
        masse_pattes_min=float(d.f1_masse_pattes.min()) if len(d) else None,
        D_min=float(d.D.min()) if len(d) else None, D_max=float(d.D.max()) if len(d) else None,
    )
    if len(d):
        d.to_csv(os.path.join(out_dir, "illimite_admissibles.csv"), index=False)
    print(f"[{name}] ILLIMITE -> {report['illimite']}", flush=True)

    # ---- (b) catalogue reel du FabLab
    nm = {s["name"]: i for i, s in enumerate(raw["servos"])}
    combos = [tuple(nm[x] for x in cb) for cb in COMBOS]
    o2 = pipeline.optimize(raw, mount, sc, combos=combos, X=X,
                           log=lambda m: print(f"[{name}/LAB] {m}", flush=True))
    for key, fn in (("df", "admissibles"), ("front", "pareto"),
                    ("rob", "candidats_robustesse"), ("rob_front", "pareto_robuste")):
        if len(o2[key]):
            o2[key].to_csv(os.path.join(out_dir, fn + ".csv"), index=False)
    report["lab"] = o2["res"]
    json.dump(report, open(os.path.join(out_dir, "resume.json"), "w"), indent=1,
              default=float, ensure_ascii=False)
    print("FIN", name, {k: v for k, v in o2["res"].items() if k not in ("choix", "stats")}, flush=True)


if __name__ == "__main__":
    args = sys.argv[1:]
    impose_servos(args)              # ex. python run_final.py OPT_MONO S2 dedans --servos S3306,S3306,S3306
    impose_masse_mc(args)            #     ... --masse_mc 4.3
    run(args[0], *(args[1:3] or ["S2", "dedans"]))
