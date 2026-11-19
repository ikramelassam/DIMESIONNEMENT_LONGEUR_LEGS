"""Construit le rapport v2 (HTML + PDF) a partir de results_exact/ et config.yaml.
Usage: python build_report2.py <figdir> <outdir> <katexdir>"""
import json, os, sys, base64, subprocess, html
import pandas as pd, yaml
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
FIG, OUT, KATEX = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(OUT, exist_ok=True)
CASES = [("dedans", "S2"), ("dedans", "S1"), ("dehors", "S2"), ("dehors", "S1")]
R = {f"{m}_{s}": json.load(open(f"{ROOT}/results_exact/{m}_{s}/resume.json")) for m, s in CASES}
cfg = yaml.safe_load(open(f"{ROOT}/config.yaml"))
f = lambda x, n=1: f"{x:.{n}f}".replace(".", ",")
def table(head, rows, cls=""):
    h = "".join(f"<th>{c}</th>" for c in head)
    b = "".join("<tr>" + "".join(f"<td>{c}</td>" for c in r) + "</tr>" for r in rows)
    return f'<div class="tw"><table class="{cls}"><thead><tr>{h}</tr></thead><tbody>{b}</tbody></table></div>'
def sv(c): return f"{c['servo_C1']} / {c['servo_C2']} / {c['servo_C3']}"
def name(m, s): return f"{m} {s}" + (" ★" if (m, s) == ("dedans", "S2") else "")

rows = []
for m, s in CASES:
    c = R[f"{m}_{s}"]["choix"]["coude"]
    rows.append([name(m, s), f"{f(c['L2'])} / {f(c['L3'])}", f"{f(c['alpha'])} / {f(c['beta'])} / {f(c['xi'])}", sv(c),
                 f(c['f1_masse_pattes'], 2), f(c['f2_nesm']), f(c['f3_couple_C2'], 2), f(c['tau_C2'], 2), f"{c['taux_reussite']*100:.0f} %"])
TBL_RESUME = table(["Cas (coude)", "L₂ / L₃ (mm)", "α / β / ξ (°)", "C1 / C2 / C3", "f₁ masse pattes (kg)", "f₂ NESM (mm)", "f₃ C₂ requis (N·m)", "τ_C2 à NESM max (N·m)", "Réussite MC"], rows)

rows = []
for m, s in CASES:
    r = R[f"{m}_{s}"]
    for lab, key in (("coude", "coude"), ("C₂ min. (ε-contrainte)", "couple_min")):
        c = r["choix"][key]
        rows.append([name(m, s), lab, f(c['L2']), f(c['L3']), f(c['alpha']), f(c['beta']), f(c['xi']), f(c['D']), f(c['h']), sv(c),
                     f(c['f1_masse_pattes'], 2), f(c['f2_nesm']), f(c['f3_couple_C2'], 2), f"{c['frac_bloc_C2']*100:.0f} %", f(c['M_total'], 2), f"{c['taux_reussite']*100:.0f} %"])
TBL_CHOIX = table(["Cas", "Règle", "L₂", "L₃", "α", "β", "ξ", "D", "h", "C1 / C2 / C3", "f₁", "f₂", "f₃", "τ_C2 / blocage", "M_tot (kg)", "MC"], rows, "small")

rows = [[name(m, s), r["admissibles"], r["pareto"], r["candidats"], r["robustes"], r["pareto_robuste"], f(r["min_couple_C2"], 2), f(r["max_nesm"]), f"{r['duree_s']/60:.0f} min"]
        for (m, s) in CASES for r in [R[f"{m}_{s}"]]]
TBL_STATS = table(["Cas", "Designs admissibles (6 combinaisons, après raffinement)", "Front de Pareto 3D", "Candidats testés (MC)", "Robustes (≥ 95 %)", "Front robuste", "min f₃ (N·m)", "max f₂ (mm)", "Durée"], rows)
frac = cfg.get("frac_couple", 1.0); SF = cfg["margins"]["SF"]
rows = [[s["name"], f(s["C_stall"], 3), f(s["C_stall"] * frac / SF, 2), f(s["mass"] * 1000, 0), f"{s['range']:.0f}°",
         "inconnu (≥ 4 supposé)" if s.get("stock") is None else f"{s['stock']} → exclu"] for s in cfg["servos"]]
TBL_SERVOS = table(["Servo", "C_stall (N·m)", "Utilisable = C_stall/SF (N·m)", "Masse (g)", "Course", "Stock"], rows)
sens = pd.concat([pd.read_csv(f"{ROOT}/results_exact/sens_part{i}.csv") for i in (0, 1)], ignore_index=True)
sens = sens.sort_values(["parametre", "scenario", "valeur"])
rows = [[r.parametre, r.scenario, f(r.valeur, 1), int(r.n), "—" if pd.isna(r.C2_requis_min) else f(r.C2_requis_min, 2), "—" if pd.isna(r.C3_requis_min) else f(r.C3_requis_min, 2),
         "aucun design" if pd.isna(r.nesm_max) else f(r.nesm_max, 1)] for r in sens.itertuples()]
sens["parametre"] = sens["parametre"].replace({"batterie x (S2 : fixee)": "position x de la batterie (mm)"})
rows = [[r.parametre, r.scenario, f(r.valeur, 1), int(r.n), "—" if pd.isna(r.C2_requis_min) else f(r.C2_requis_min, 2), "—" if pd.isna(r.C3_requis_min) else f(r.C3_requis_min, 2),
         "aucun design" if pd.isna(r.nesm_max) else f(r.nesm_max, 1)] for r in sens.sort_values(["parametre","scenario","valeur"]).itertuples()]
TBL_SENS = table(["Paramètre", "Scénario", "Valeur", "n admissibles", "C₂ requis min. (N·m)", "C₃ requis min. (N·m)", "NESM max (mm)"], rows)
ch, bd, ar, tk, mg = cfg["chassis"], cfg["body"], cfg["body"]["arm_model"], cfg["task"], cfg["margins"]
rows = [["Châssis", f"{ch['N_ch']} × {ch['M_ch']} mm (carré, hypothèse) ; retrait c = {ch['c']} ; e = {ch['e']} ; débord C₂ ≤ {ch['overhang_C2']} mm"],
        ["Coxa", f"L₁ = {cfg['leg']['L1']} mm ; segments aluminium 20×3 mm, {cfg['leg']['seg_lin_mass']} kg/m ; équerres {cfg['leg']['bracket_mass']*1000:.0f} g ; pied {cfg['leg']['foot_mass']*1000:.0f} g, ⌀ {cfg['leg']['foot_diam']} mm"],
        ["Masses", f"châssis {bd['chassis']['m']} kg ; batterie {bd['battery']['m']} kg ; objet {cfg['body']['object_mass']} kg ; bras {ar['m_base']+ar['m_elbow']+ar['m_tip']:.2f} kg"],
        ["Batterie", "position nominale (−20, 0, 15) mm ; zone x∈[−80, 40], y∈[−30, 30], z∈[10, 25]"],
        ["Bras", f"fixation à {ar['mount_from_front']} mm de l'avant, {ar['mount_z_rel']} mm au-dessus ; a₁ = {ar['a1']}, a₂ = {ar['a2']} mm ; postures replié, transport, tendu"],
        ["Incertitudes CdG", "ε_CdG = 10 mm (corps), ε_pied = 5 mm"],
        ["Tâches", f"obstacle {tk['obstacle']} mm ; Δh {tk['dh_lift']} ; pas {tk['stroke']} mm ; lacet {tk['yaw_step']}° ; a = {tk['accel']} m/s² ; balancement ±{tk['sway_max']} mm, grille {tk['sway_n']}×{tk['sway_n']}"],
        ["Marges", f"SF {mg['SF']} ; θ₃ ∈ [{mg['theta3_min']}°, {mg['theta3_max']}°] ; marge d'articulation {mg['joint_margin']}° ; portée {mg['reach_margin']} mm ; écart des axes C₁ {mg['servo_spacing']} mm"],
        ["Budget masse", f"{cfg['body']['chassis']['m']} + batterie + 4 pattes ≤ {cfg['margins'].get('mass_robot_max_sans_bras', 4.0) if 'mass_robot_max_sans_bras' in cfg['margins'] else 4.0} kg (sans bras)"],
        ["Recherche", f"Sobol {cfg['search']['n_sobol']} points ; Monte-Carlo {cfg['robust']['n_mc']} tirages, seuil {cfg['robust']['pass_rate']*100:.0f} %"]]
TBL_INPUTS = table(["Entrée", "Valeur (config.yaml v2)"], rows)

TBL_RESUME = TBL_RESUME.replace('<div class="tw">', '<div class="tw keep">')
body = open(f"{HERE}/body.html", encoding="utf-8").read()
for k, v in dict(TBL_RESUME=TBL_RESUME, TBL_CHOIX=TBL_CHOIX, TBL_STATS=TBL_STATS, TBL_SERVOS=TBL_SERVOS, TBL_SENS=TBL_SENS, TBL_INPUTS=TBL_INPUTS).items():
    body = body.replace("{{" + k + "}}", v)
def img(m):
    p = f"{FIG}/{m}.png"
    return "data:image/png;base64," + base64.b64encode(open(p, "rb").read()).decode()
import re
body = re.sub(r"\{\{IMG:(\w+)\}\}", lambda m: img(m.group(1)), body)
open(f"{OUT}/body_tmp.html", "w", encoding="utf-8").write(body)
subprocess.check_call(["node", f"{HERE}/render_math.js", f"{OUT}/body_tmp.html", f"{OUT}/body_math.html", KATEX])
css = open(f"{HERE}/style.css", encoding="utf-8").read()
math = open(f"{OUT}/body_math.html", encoding="utf-8").read()
def page(katex_css):
    return f'<!doctype html><html lang="fr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Dimensionnement des pattes Gr2</title>{katex_css}<style>{css}</style></head><body><main>{math}</main></body></html>'
open(f"{OUT}/rapport_pdf.html", "w", encoding="utf-8").write(page(f'<link rel="stylesheet" href="file://{KATEX}/katex/dist/katex.min.css">'))
open(f"{OUT}/rapport_web.html", "w", encoding="utf-8").write(page('<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/KaTeX/0.16.11/katex.min.css">'))
print("ok")
