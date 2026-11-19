"""Etude ciblee : optimisation COMPLETE (pipeline.optimize) avec xi libre ou verrouille,
et catalogue standard ou avec Dynamixels disponibles en 4 exemplaires.
Usage : python run_study.py <nom> <montage> <scenario> <xi|libre> <std|dyn> [n_mc]"""
import json, os, sys, copy
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gr2.params import Params
from gr2 import pipeline

HERE = os.path.dirname(os.path.abspath(__file__))
STD = [("SG90", "S3306", "S-U400"), ("S-U400", "S3306", "S-U400"), ("S3306", "S3306", "S-U400"),
       ("SG90", "S3306", "S3306"), ("S-U400", "S3306", "S3306"), ("S3306", "S3306", "S3306")]   # = run_exact.py
DYN = dict(allowed=[["SG90", "S-U400"], ["S3306", "MX-28T", "MX-64AT"], ["S-U400", "S3306", "MX-28T", "MX-64AT"]])


def main(name, mount, sc, xi, cat, n_mc=None):
    P = Params.load(os.path.join(HERE, "config.yaml"))
    raw = copy.deepcopy(P.raw)
    if xi != "libre":
        raw["search"]["bounds"]["xi"] = [float(xi), float(xi) + 1e-6]   # qmc.scale exige lo < hi
    kw = {}
    if cat == "dyn":
        for s in raw["servos"]:
            if s["name"].startswith(("AX", "MX")):
                s["stock"] = None          # hypothese : 4 exemplaires disponibles
        kw = dict(allowed=DYN["allowed"])
    else:
        names = {s["name"]: i for i, s in enumerate(raw["servos"])}
        kw = dict(combos=[tuple(names[n] for n in cb) for cb in STD])
    out = pipeline.optimize(raw, mount, sc, n_mc=n_mc, log=lambda m: print(name, m, flush=True), **kw)
    d = os.path.join(HERE, "results_study", name); os.makedirs(d, exist_ok=True)
    out["df"].to_csv(os.path.join(d, "admissibles.csv"), index=False); out["front"].to_csv(os.path.join(d, "pareto.csv"), index=False)
    out["rob"].to_csv(os.path.join(d, "candidats_robustesse.csv"), index=False); out["rob_front"].to_csv(os.path.join(d, "pareto_robuste.csv"), index=False)
    json.dump(out["res"], open(os.path.join(d, "resume.json"), "w"), indent=1, default=float)
    print("FIN", name, {k: v for k, v in out["res"].items() if k not in ("choix", "stats")}, flush=True)


if __name__ == "__main__":
    main(*sys.argv[1:6], n_mc=(int(sys.argv[6]) if len(sys.argv) > 6 else None))
