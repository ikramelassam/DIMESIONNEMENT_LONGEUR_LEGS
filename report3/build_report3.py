"""Construit le rapport v3 (HTML + PDF). Usage: python build_report3.py <figdir> <outdir> <katexdir>"""
import json, os, sys, base64, subprocess, re
import numpy as np, pandas as pd
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
FIG, OUT, KATEX = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(OUT, exist_ok=True)
A = lambda n: json.load(open(f"{ROOT}/audit/{n}"))
res = A("audit_result.json"); E = {}
for part in ("A", "B", "C", "E"):
    if os.path.exists(f"{ROOT}/audit/experiences_{part}.json"):
        d = json.load(open(f"{ROOT}/audit/experiences_{part}.json")); E.update({k: v for k, v in d.items() if k != "reference" or "reference" not in E})
E["Eseeds"] = E.pop("E", [])
val = json.load(open(f"{ROOT}/sim/validation.json")); vopt = json.load(open(f"{ROOT}/sim/validation_opt.json"))
refs = json.load(open(f"{ROOT}/sim/ref_fronts.json"))
SUP = str.maketrans("-0123456789", "⁻⁰¹²³⁴⁵⁶⁷⁸⁹")
def sci(v):
    if v == 0: return "0"
    e = int(np.floor(np.log10(abs(v)))); m = v / 10 ** e
    return f"{m:.1f}".replace(".", ",") + "·10" + str(e).translate(SUP)
f = lambda x, n=2: "—" if x is None or (isinstance(x, float) and not np.isfinite(x)) else f"{x:.{n}f}".replace(".", ",")
def table(head, rows, cls=""):
    h = "".join(f"<th>{c}</th>" for c in head)
    b = "".join(("<tr class='%s'>" % r[0] if isinstance(r, tuple) else "<tr>") + "".join(f"<td>{c}</td>" for c in (r[1] if isinstance(r, tuple) else r)) + "</tr>" for r in rows)
    return f'<div class="tw"><table class="{cls}"><thead><tr>{h}</tr></thead><tbody>{b}</tbody></table></div>'
t = res["tot"]
# --- reproductibilite des CSV : front robuste sauvegarde vs reevaluation Python (ref_audit)
RA = [d for c in ("dedans_S2", "dedans_S1", "dehors_S2", "dehors_S1") for d in json.load(open(f"{ROOT}/audit/ref_audit_{c}.json"))]; worst = 0; n = 0
for case in ("dedans_S2", "dedans_S1", "dehors_S2", "dehors_S1"):
    rf = pd.read_csv(f"{ROOT}/results_exact/{case}/pareto_robuste.csv")
    ents = [d for d in RA if d["case"] == case and d["tag"] == "front_robuste"]
    assert len(ents) == len(rf), (case, len(ents), len(rf))
    for d, (_, r) in zip(ents, rf.iterrows()):
        c = d["combos"][f"{r.servo_C1}/{r.servo_C2}/{r.servo_C3}"]
        assert c["adm"], (case, r.L2)
        for k in ("f1_masse_pattes", "f2_nesm", "f3_couple_C2"):
            worst = max(worst, abs(c["v"][k] - r[k]) / max(1, abs(r[k]))); n += 1
        # dominance du front robuste sauvegarde
ties = A("ties_result.json"); keys_ties = ("tau_C1", "tau_C3", "span_C3")
err_dec = max(v for k, v in res["perKey"].items() if k not in keys_ties and not k.startswith("span"))
err_span = max(res["perKey"]["span_C1"], res["perKey"]["span_C2"])
assert ties["explained_by_ties"] == ties["cases_with_diff"], ties
V = dict(ties_n=ties["cases_with_diff"], ties_pct=f"{100 * ties['cases_with_diff'] / t['both_adm']:.1f}".replace(".", ","), span_err=sci(err_span), designs=res["tot"]["designs"], pairs=f"{t['pairs']:,}".replace(",", " "), pre_diff=t["pre_diff"], viol_err=sci(t["viol_err"]) if t["viol_err"] else "0", adm_diff=t["adm_diff"],
         both_adm=f"{t['both_adm']:,}".replace(",", " "), both_inf=f"{t['both_inf']:,}".replace(",", " "), pre_inf=f"{t['pre_inf']:,}".replace(",", " "), val_err="≤ " + sci(err_dec),
         dom_agree=f"{t['dom_agree']:,}".replace(",", " "), crit_diff=t["crit_diff"], dom_strict=t["dom_strict_vs_tol"], csv_n=n // 3, csv_err=sci(worst) if worst else "0",
         valid_model=sci(val["ecart_relatif_max"]["nesm"]) + " sur les valeurs internes", valid_opt_err=sci(vopt["ecart_relatif_max"]), ref_regret=f(E["reference"]["regret"], 2))
# --- tableaux d'audit
NAMES = {"front_robuste": "Front robuste sauvegardé", "front": "Front de Pareto (30 points par cas)", "admissible": "Admissibles tirés au hasard", "alea_bornes": "Aléatoires dans les bornes", "hors_bornes": "Hors des bornes (jusqu'à 35 %)", "perturb": "Perturbation d'une variable"}
rows = []
for k in NAMES:
    tg = {kk: vv for kk, vv in res["byTag"].items() if kk == k}
    d = sum(v["designs"] for v in tg.values()); p = sum(v["pairs"] for v in tg.values()); pre = sum(v["pre_ok"] for v in tg.values()); a = sum(v["adm"] for v in tg.values()); df = sum(v["diff"] for v in tg.values())
    rows.append([NAMES[k], d, f"{pre} ({100 * pre / max(d, 1):.0f} %)", p, a, df])
rows.append(("ref", ["<b>Total</b>", t["designs"], "", t["pairs"], t["both_adm"], t["adm_diff"]]))
TBL_AUDIT_TAGS = (lambda h: h.replace('<div class="tw">', '<div class="tw keep">'))(table(["Famille de designs", "Designs", "Passent le pré-filtre", "Cas (× 27 servos)", "Admissibles dans les deux", "Désaccords d'admissibilité"], rows))
LAB = {"f1_masse_pattes": "$f_1$ masse des pattes", "f2_nesm": "$f_2$ NESM", "f3_couple_C2": "$f_3$ couple $C_2$ requis", "tau_C1": "couple max $C_1$", "tau_C2": "couple max $C_2$", "tau_C3": "couple max $C_3$",
       "tau_min_C1": "couple requis min. $C_1$", "tau_min_C3": "couple requis min. $C_3$", "span_C1": "course utilisée $C_1$", "span_C2": "course utilisée $C_2$", "span_C3": "course utilisée $C_3$", "M_total": "masse portée",
       "frac_bloc_C1": "part de blocage $C_1$", "frac_bloc_C2": "part de blocage $C_2$", "frac_bloc_C3": "part de blocage $C_3$"}
TBL_AUDIT_ERR = (lambda h: h.replace('<div class="tw">', '<div class="tw keep">'))(table(["Grandeur comparée", "Écart relatif maximal (simulateur / Python)", "Remarque"], [[LAB[k], sci(v) if v else "0", ("ex æquo de position (§3.2) : " + str(sum(1 for d in ties["details"] if k in d["bad"] or (k.startswith("span") and d["badSpan"]))) + " cas") if k in keys_ties else ("arrondi simple précision des fichiers Python" if k.startswith("span") else "")] for k, v in res["perKey"].items()]))
# --- experiences
r0 = E["reference"]; x0 = r0["x"]
cons_n = {"G5": "G5 garde au sol", "G3": "G3 θ₃", "G6": "G6 gabarit", "G7": "G7 axes C1", "G10": "G10 châssis", "PH": "PH phases", "NESM": "NESM", "T1": "T1", "T2": "T2", "T3": "T3", "R1": "R1", "R2": "R2", "R3": "R3", "M": "M masse"}
def causes(r):
    ids = [b["id"] for b in r["bad"]]
    root = [i for i in ids if i[0] in "GMTR" and i != "PH"]
    if root: return ", ".join(root) + (" (+ phases)" if "PH" in ids else "")
    if "PH" in ids: return "PH : phases irréalisables"
    return ", ".join(ids) or "aucune"
TBL_REF = table(["$L_2$", "$L_3$", "α", "β", "ξ", "Servos C1/C2/C3", "$f_1$ (kg)", "$f_2$ (mm)", "$f_3$ (N·m)", "garde au sol $h$ (mm)", "MC", "distance"],
                [[f(x0["L2"], 1), f(x0["L3"], 1), f(x0["alpha"], 1), f(x0["beta"], 1), f(x0["xi"], 1), r0["combo"], f(r0["f1"], 3), f(r0["f2"]), f(r0["f3"]), f(r0["h"], 1), f"{100 * r0['mc']:.0f} %", f(r0["regret"])]])
UN = {"L2": "$L_2$ (mm)", "L3": "$L_3$ (mm)", "alpha": "α (°)", "beta": "β (°)", "xi": "ξ (°)"}
rows = []
for r in E["A"]:
    cls = "" if r["adm"] else "bad"
    rows.append((cls, [UN[r["var"]], ("+" if r["delta"] > 0 else "−") + f(abs(r["delta"]), 0), f(r["value"], 1), "oui" if r["adm"] else "non", causes(r) if not r["adm"] else "—", f(r["f1"], 3), f(r["f2"]), f(r["f3"]),
                       f"{100 * r['mc']:.0f}&nbsp;%" if r["mc"] is not None else "—", r["dom"] if r["dom"] is not None else "—", f(r["regret"]) if r["regret"] is not None else "—"]))
rows = [(c, [x for x in rr]) for c, rr in rows]
TBL_OAT = table(["Variable", "Variation", "Valeur", "Admissible", "Contrainte(s) cassée(s)", "$f_1$ (kg)", "$f_2$ (mm)", "$f_3$ (N·m)", "MC", "Dominé par", "Distance"], rows, "small")
rows = []
for r in E["B"]:
    rows.append(("" if r["adm"] else "bad", [r["combo"], "oui" if r["adm"] else "non", "—" if r["adm"] else ", ".join(b["id"] for b in r["bad"]), f(r["f1"], 3), f(r["f2"]), f(r["f3"]),
                                             "" if r["adm"] else f(r["f2_notq"]) + " / " + f(r["f3_notq"]), r["dom"] if r["dom"] is not None else "—"]))
TBL_SERVO = table(["Servos C1 / C2 / C3", "Admissible", "Contraintes violées", "$f_1$ (kg)", "$f_2$ (mm)", "$f_3$ (N·m)", "$f_2$ / $f_3$ sans limite de couple", "Dominé par"], rows)
C = E.get("C", [])
rows = []
base = C[0] if C else None
for c in C:
    if c["label"].startswith("marge NESM"): continue
    ec = c.get("eps"); d = lambda k: "" if c is base else f" ({100 * (c[k] / base[k] - 1):+.0f} %)".replace("-", "−")
    rows.append(("ref" if c is base else "", [c["label"], f"{c['adm']}{d('adm')}", f"{f(c['min_f3'])}{d('min_f3')}", f"{f(c['max_f2'], 1)}{d('max_f2')}", f(ec["f3"]) if ec else "—"]))
for e in E.get("Eseeds", []):
    rows.append(("", [f"<i>bruit : référence, graine {e['seed']}</i>", f"{e['adm']} ({100 * (e['adm'] / base['adm'] - 1):+.0f} %)".replace("-", "−"), f"{f(e['min_f3'])} ({100 * (e['min_f3'] / base['min_f3'] - 1):+.0f} %)".replace("-", "−"), f"{f(e['max_f2'], 1)} ({100 * (e['max_f2'] / base['max_f2'] - 1):+.0f} %)".replace("-", "−"), ""]))
TBL_WHATIF = table(["Hypothèse modifiée", "Designs admissibles", "$f_3$ minimal du cas (N·m)", "$f_2$ maximal du cas (mm)", "$f_3$ minimal avec NESM ≥ 2 mm (N·m)"], rows)
READ = json.load(open(f"{HERE}/reads.json")) if os.path.exists(f"{HERE}/reads.json") else {}
body = open(f"{HERE}/body.html", encoding="utf-8").read()
for k, v in dict(TBL_AUDIT_TAGS=TBL_AUDIT_TAGS, TBL_AUDIT_ERR=TBL_AUDIT_ERR, TBL_REF=TBL_REF, TBL_OAT=TBL_OAT, TBL_SERVO=TBL_SERVO, TBL_WHATIF=TBL_WHATIF, **{k.upper(): v for k, v in READ.items()}).items():
    body = body.replace("{{" + k + "}}", v)
for k, v in V.items(): body = body.replace("{{V:" + k + "}}", str(v))
body = re.sub(r"\{\{IMG:(\w+)\}\}", lambda m: "data:image/png;base64," + base64.b64encode(open(f"{FIG}/{m.group(1)}.png", "rb").read()).decode(), body)
assert "{{" not in body, re.findall(r"\{\{[^}]*\}\}", body)[:5]
open(f"{OUT}/body_tmp.html", "w", encoding="utf-8").write(body)
subprocess.check_call(["node", f"{HERE}/render_math.js", f"{OUT}/body_tmp.html", f"{OUT}/body_math.html", KATEX])
css = open(f"{HERE}/style.css", encoding="utf-8").read(); math = open(f"{OUT}/body_math.html", encoding="utf-8").read()
def page(kc): return f'<!doctype html><html lang="fr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Cohérence simulateur–optimiseur Gr2</title>{kc}<style>{css}</style></head><body><main>{math}</main></body></html>'
open(f"{OUT}/rapport3_pdf.html", "w", encoding="utf-8").write(page(f'<link rel="stylesheet" href="file://{KATEX}/katex/dist/katex.min.css">'))
open(f"{OUT}/rapport3_web.html", "w", encoding="utf-8").write(page('<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/KaTeX/0.16.11/katex.min.css">'))
json.dump(V, open(f"{OUT}/values.json", "w")); print("ok", V)
