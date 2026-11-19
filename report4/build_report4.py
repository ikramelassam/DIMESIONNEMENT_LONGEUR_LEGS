"""Rapport v4. Usage: python build_report4.py <figdir> <outdir> <katex_node_modules_dir>"""
import json, os, sys, re, base64, subprocess
import numpy as np, pandas as pd
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
FIG, OUT, KATEX = sys.argv[1:4]
os.makedirs(OUT, exist_ok=True)
from gr2.params import Params

def f(x, n=2):
    return "—" if x is None or (isinstance(x, float) and not np.isfinite(x)) else f"{x:.{n}f}".replace(".", ",")
def table(head, rows, cls=""):
    h = "".join(f"<th>{c}</th>" for c in head)
    b = "".join(("<tr class='%s'>" % r[0] if isinstance(r, tuple) else "<tr>") + "".join(f"<td>{c}</td>" for c in (r[1] if isinstance(r, tuple) else r)) + "</tr>" for r in rows)
    return f'<div class="tw keep"><table class="{cls}"><thead><tr>{h}</tr></thead><tbody>{b}</tbody></table></div>'
J = lambda p: json.load(open(f"{ROOT}/{p}"))
RS = lambda n: f"{ROOT}/results_study/{n}"
def load(n):
    d = RS(n)
    if not os.path.exists(d + "/resume.json"): return None
    return dict(res=J(f"results_study/{n}/resume.json"), adm=pd.read_csv(d + "/admissibles.csv"), par=pd.read_csv(d + "/pareto.csv"),
                rob=pd.read_csv(d + "/pareto_robuste.csv"))
A, B, D, E = (load(n) for n in ("A_xi_libre", "B_xi_45", "D_dyn_xi_libre", "E_dyn_xi_45"))
trip = lambda r: f"{r.servo_C1} / {r.servo_C2} / {r.servo_C3}"

def dominated(P1, P2):
    """nb de points de P2 dominés par au moins un point de P1 (f1 min, f2 max, f3 min)."""
    n = 0
    for _, q in P2.iterrows():
        if ((P1.f1_masse_pattes <= q.f1_masse_pattes + 1e-9) & (P1.f2_nesm >= q.f2_nesm - 1e-9) & (P1.f3_couple_C2 <= q.f3_couple_C2 + 1e-9) &
            ((P1.f1_masse_pattes < q.f1_masse_pattes - 1e-9) | (P1.f2_nesm > q.f2_nesm + 1e-9) | (P1.f3_couple_C2 < q.f3_couple_C2 - 1e-9))).any():
            n += 1
    return n

coA, coB = A["res"]["choix"]["coude"], B["res"]["choix"]["coude"]
domBA = dominated(A["par"], B["par"]); domAB = dominated(B["par"], A["par"])
print("B dominé par A:", domBA, "/", len(B["par"]), "; A dominé par B:", domAB)
V = dict(minC2_A=f(A["res"]["min_couple_C2"], 3), nB_front=f"{len(B['par'])}" if domBA == len(B["par"]) else f"{domBA} sur {len(B['par'])}",
         coudeA_f1=f(coA["f1_masse_pattes"], 3), coudeA_f2=f(coA["f2_nesm"], 2), coudeB_f1=f(coB["f1_masse_pattes"], 3), coudeB_f2=f(coB["f2_nesm"], 2),
         reproA=f"{A['res']['admissibles']:,} admissibles, front de {A['res']['pareto']} points, {A['res']['pareto_robuste']} robustes, couple min {f(A['res']['min_couple_C2'], 4)} N·m, NESM max {f(A['res']['max_nesm'], 4)} mm".replace(",", " "),
         cost_mass=f"+{f(coB['f1_masse_pattes'] - coA['f1_masse_pattes'], 2)} kg", cost_nesm=f"{f(coA['f2_nesm'] - coB['f2_nesm'], 1)} mm (×{f(coA['f2_nesm'] / coB['f2_nesm'], 1)})",
         cost_tau=f"+{f(coB['f3_couple_C2'] - coA['f3_couple_C2'], 2)} N·m")
assert domAB == 0, "des points du front libre sont dominés"

# ---------------- phases
PH = [("avant_RL / FL / RR / FR", "4", "RL, FL, RR, FR", "marche avant, pieds au sol plat"),
      ("arriere_FR / RR / FL / RL", "4", "miroir avant↔arrière", "marche arrière (cahier des charges)"),
      ("obstacle_RL / FL / RR / FR", "4", "pieds avant sur l'obstacle (z = 30 mm)", "franchissement"),
      ("rotation", "1", "4 pieds, lacet ± yaw/2", "rotation sur place"), ("saisie", "1", "4 pieds, bras tendu avec objet", "manipulation")]
TBL_PHASES = table(["Phases", "Nombre", "Patte levée / configuration", "Rôle"], [list(r) for r in PH]) + \
    table(["Patte qui se lève $k$", "$k$", "précédente", "d'avant", "suivante"], [["décalage du pied le long de $x$ (mm)", "−20", "+10", "0", "−10"]])

# ---------------- combinaisons de CdG
P0 = Params.load(f"{ROOT}/config.yaml")
z = P0.raw["body"]["battery_zone"]; rows = []
corner_name = lambda bx, by, bz: ("devant" if bx > 0 else "derrière") + "/" + ("gauche" if by > 0 else "droite") + "/" + ("haut" if bz > 15 else "bas")
for post in ("replie", "transport", "tendu"):
    pts = []
    for bx in z["x"]:
        for by in z["y"]:
            for bz in z["z_rel"]:
                m, g = P0._body_cdg([bx, by, bz], post); pts.append((g, (bx, by, bz), m))
    gx = [p[0][0] for p in pts]; gz = [p[0][2] for p in pts]; gy = [p[0][1] for p in pts]
    imin, imax, izm = int(np.argmin(gx)), int(np.argmax(gx)), int(np.argmax(gz))
    rows.append([post, f(pts[0][2], 2), f"{f(min(gx), 1)} ({corner_name(*pts[imin][1])})", f"{f(max(gx), 1)} ({corner_name(*pts[imax][1])})",
                 f"{f(min(gy), 1)} … {f(max(gy), 1)}", f"{f(max(gz), 1)} ({corner_name(*pts[izm][1])})"])
TBL_COMBOS = table(["Posture du bras", "Masse corps (kg)", "$x_G$ min (mm) (batterie)", "$x_G$ max (mm) (batterie)", "$y_G$ (mm)", "$z_G$ max (mm) (batterie)"], rows)
for post in ("replie", "transport", "tendu"): pass
mw, cw, hw = P0.body_box(["replie", "transport"]); ms, cs, hs = P0.body_box(["replie", "transport", "tendu"])
TBL_COMBOS += table(["Boîte", "Centre $(x,y,z)$ (mm)", "Demi-largeurs (mm)", "Masse max (kg)"],
                    [["marche (replié, transport)", f"({f(cw[0], 1)} ; {f(cw[1], 1)} ; {f(cw[2], 1)})", f"({f(hw[0], 1)} ; {f(hw[1], 1)} ; {f(hw[2], 1)})", f(mw, 2)],
                     ["saisie (+ tendu)", f"({f(cs[0], 1)} ; {f(cs[1], 1)} ; {f(cs[2], 1)})", f"({f(hs[0], 1)} ; {f(hs[1], 1)} ; {f(hs[2], 1)})", f(ms, 2)]])

# ---------------- reconstruction à la main
d3 = J("diag/diag3.json"); Mg = d3["45"]["Mc"] * 9.81
rows = []
for xi, lab in (("45", "45°"), ("77.32", "77,3°")):
    for lg in d3[xi]["states"][0]["legs"]:
        t2 = Mg * lg["lam"] * lg["D"] / 1000; t3 = Mg * lg["lam"] * lg["x3"] / 1000
        rows.append([lab, lg["leg"], f(lg["lam"], 3), f(lg["D"], 1), f(lg["beta"], 1), f(lg["x3"], 1), f(t2, 3), f(t3, 3)])
    t = d3[xi]["tau"]
    rows.append(("ref", [lab, "<b>max</b>", "", "", "", "", f"<b>{f(t[1], 3)}</b>", f"<b>{f(t[2], 3)}</b>"]))
TBL_RECON = table(["ξ", "Patte", "$\\lambda$", "$D$ (mm)", "$\\beta$ (°)", "$L_3|\\cos\\beta|$ (mm)", "$\\tau_2=M_cg\\lambda D$ (N·m)", "$\\tau_3=M_cg\\lambda L_3|\\cos\\beta|$ (N·m)"], rows)

# ---------------- attribution
at = J("diag/attr.json")
TBL_ATTR = table(["Hypothèse relâchée", "$f_2$ à 45° (mm)", "$f_2$ à 77,3° (mm)", "NESM avant_RL 45° (mm)", "NESM avant_RL 77,3° (mm)"],
                 [[r["name"], f(r["45"]["f2"], 2), f(r["77.32"]["f2"], 2), f(r["45"]["a"], 2), f(r["77.32"]["a"], 2)] for r in at])

# ---------------- grille
g = J("diag/grid.json"); rows = []
for n in (11, 21, 41):
    for xi, lab in (("45", "45°"), ("77.32", "77,3°")):
        r = g[f"{n}_{xi}"]; rows.append([lab, f"{n}×{n}", f(120 / (n - 1), 0), f(r["f2"], 2), f(r["tmin"][2], 3), r["crit"]])
TBL_GRID = table(["ξ", "Grille", "Pas (mm)", "$f_2$ (mm)", "$\\tau_3$ requis (N·m)", "Phase critique"], rows)

# ---------------- A contre B
def kpi(X):
    r = X["res"]; c = r["choix"]["coude"]
    return [f"{r['admissibles']:,}".replace(",", " "), r["pareto"], f"{r['robustes']} / {r['pareto_robuste']}", f(X["par"].f1_masse_pattes.min(), 3), f(X["par"].f2_nesm.max(), 2), f(r["min_couple_C2"], 3),
            f"{f(c['f1_masse_pattes'], 3)} / {f(c['f2_nesm'], 2)} / {f(c['f3_couple_C2'], 3)}", trip(pd.Series(c)), f"{f(c['xi'], 1)}°"]
lab = ["Designs admissibles", "Points du front de Pareto", "Candidats robustes / front robuste", "Masse minimale du front (kg)", "NESM maximal du front (mm)", "Couple C2 minimal (N·m)",
       "Coude : $f_1$ (kg) / $f_2$ (mm) / $f_3$ (N·m)", "Coude : servos C1 / C2 / C3", "Coude : ξ"]
ka, kb = kpi(A), kpi(B)
TBL_AB = table(["Indicateur", "ξ libre", "ξ = 45°"], [[l, a, b] for l, a, b in zip(lab, ka, kb)]) + \
    f"<p>Dominance : {domBA} points sur {len(B['par'])} du front à 45° sont dominés par le front libre ; {domAB} points sur {len(A['par'])} du front libre sont dominés par celui à 45°.</p>"

rows = []
for cap in (1.05, 1.15, 1.25, 1.40, 1.60, 2.0):
    ra, rb = A["adm"][A["adm"].f1_masse_pattes <= cap], B["adm"][B["adm"].f1_masse_pattes <= cap]
    rows.append([f"≤ {f(cap, 2)}", f(ra.f2_nesm.max(), 2) if len(ra) else "aucun design", f(rb.f2_nesm.max(), 2) if len(rb) else "aucun design",
                 f(ra.f3_couple_C2.min(), 3) if len(ra) else "—", f(rb.f3_couple_C2.min(), 3) if len(rb) else "—"])
TBL_CAP = table(["Masse de pattes $f_1$ (kg)", "meilleur $f_2$, ξ libre (mm)", "meilleur $f_2$, ξ = 45° (mm)", "$f_3$ min, ξ libre (N·m)", "$f_3$ min, ξ = 45° (N·m)"], rows)

# ---------------- classes de ξ
Aa = A["adm"].copy(); Aa["bin"] = (Aa.xi // 10 * 10).clip(upper=80)
Aa["geo"] = Aa[["L2", "L3", "alpha", "beta", "xi"]].round(4).astype(str).agg("|".join, axis=1)
gb = Aa.groupby("bin").agg(n=("xi", "size"), geo=("geo", "nunique"), f2=("f2_nesm", "max"), f1=("f1_masse_pattes", "min"), t=("f3_couple_C2", "min"))
TBL_BINS = table(["ξ", "Lignes", "Géométries distinctes", "meilleur $f_2$ (mm)", "$f_1$ min (kg)", "$f_3$ min (N·m)"],
                 [[f"{int(b)}–{int(b) + 10}°", int(r.n), int(r.geo), f(r.f2, 2), f(r.f1, 3), f(r.t, 3)] for b, r in gb.iterrows()])

# ---------------- servos
def servo_rows(X, tag):
    out = []
    for k, d in X["adm"].groupby(["servo_C1", "servo_C2", "servo_C3"]):
        out.append([tag, " / ".join(k), len(d), f(d.f1_masse_pattes.min(), 3), f(d.f2_nesm.max(), 2), f(d.f3_couple_C2.min(), 3)])
    return out
rows = servo_rows(A, "libre") + servo_rows(B, "45°") + [["45°", "(tous) / S3306 / S-U400", 0, "—", "—", "—"]]
TBL_SERVOS = table(["ξ", "C1 / C2 / C3", "Lignes admissibles", "$f_1$ min (kg)", "$f_2$ max (mm)", "$f_3$ min (N·m)"], rows)

# ---------------- catalogue (figure 5 table)
cat = {s["name"]: s for s in P0.raw["servos"]}
TBL_CAP = TBL_CAP
rows = []
for n in ("SG90", "S-U400", "S3306", "AX-12A", "MX-28T", "MX-64AT", "MX-106T"):
    s = cat[n]; rows.append([n, f(s["C_stall"], 3), f(s["C_stall"] / 1.2, 3), f(s["mass"] * 1000, 0), s["stock"] if s["stock"] is not None else "—"])
TBL_SERVOS += table(["Servo", "Blocage (N·m)", "Utilisable ÷ 1,2 (N·m)", "Masse (g)", "Stock"], rows)

# ---------------- SECTION 5 (Dynamixel)
def sec5():
    if not (D and E):
        return "<p><i>Variante Dynamixel non disponible.</i></p>"
    h = ["<h2>5. Et avec un autre moteur ?</h2>"]
    h.append("<p>Hypothèse de calcul : on suppose qu'on pourrait disposer de <b>4 exemplaires</b> de MX-28T (2,5 N·m, 72 g) ou MX-64AT (6,0 N·m, 126 g), alors que le stock est de 1 unité. Ces servos fonctionnent en 12 V (autre bus d'alimentation que S3306/S-U400) et ont une plage de 360°. Même pipeline complet (ξ libre, puis ξ = 45°), C2 ∈ {S3306, MX-28T, MX-64AT}, C3 ∈ {S-U400, S3306, MX-28T, MX-64AT}.</p>")
    rows = []
    for X, tag in ((A, "S3306 seul — ξ libre"), (D, "avec Dynamixel — ξ libre"), (B, "S3306 seul — 45°"), (E, "avec Dynamixel — 45°")):
        c = X["res"]["choix"]["coude"] if "coude" in X["res"]["choix"] else None
        rows.append([tag, f"{X['res']['admissibles']:,}".replace(",", " "), f(X["par"].f1_masse_pattes.min(), 3), f(X["par"].f2_nesm.max(), 2), f(X["res"]["min_couple_C2"], 3),
                     (f"{f(c['f1_masse_pattes'], 3)} / {f(c['f2_nesm'], 2)} / {f(c['f3_couple_C2'], 3)}" if c else "—"), (trip(pd.Series(c)) if c else "—")])
    h.append(table(["Cas", "Admissibles", "$f_1$ min (kg)", "$f_2$ max (mm)", "$f_3$ min (N·m)", "Coude $f_1$ / $f_2$ / $f_3$", "Coude servos"], rows))
    rows = []
    for X, tag in ((D, "libre"), (E, "45°")):
        for k, d in X["adm"].groupby("servo_C2"):
            rows.append([tag, k, len(d), f(d.f1_masse_pattes.min(), 3), f(d.f2_nesm.max(), 2), f(d.f3_couple_C2.min(), 3)])
    h.append(table(["ξ", "Servo en C2", "Lignes admissibles", "$f_1$ min (kg)", "$f_2$ max (mm)", "$f_3$ min (N·m)"], rows))
    # verdict
    def pick(X):
        r = X["rob"]; return {"léger": r.loc[r.f1_masse_pattes.idxmin()], "coude": pd.Series(X["res"]["choix"]["coude"]), "max NESM": r.loc[r.f2_nesm.idxmax()]}
    rows = []
    for X, tag in ((A, "S3306 seul, ξ libre"), (D, "avec Dynamixel, ξ libre"), (B, "S3306 seul, 45°"), (E, "avec Dynamixel, 45°")):
        for k, c in pick(X).items():
            rows.append([tag, k, f"{f(c['L2'], 0)} / {f(c['L3'], 0)}", f"{f(c['xi'], 0)}°", trip(c), f(c["f1_masse_pattes"], 3), f(c["f2_nesm"], 2), f(c["f3_couple_C2"], 2)])
    h.append("<p>Points du front <b>robuste</b> (≥ 95 % de réussite Monte-Carlo) : le plus léger, le coude, et celui de NESM maximal.</p>")
    h.append(table(["Cas", "Choix", "$L_2$ / $L_3$ (mm)", "ξ", "C1 / C2 / C3", "$f_1$ (kg)", "$f_2$ (mm)", "$f_3$ (N·m)"], rows))
    h.append("<p><b>Lecture physique.</b> Le couple de hanche vaut $\\tau_2=M_cg\\,\\lambda\\,D$ : il croît avec la portée horizontale $D$ de la patte. Or le NESM croît avec la place laissée au corps pour se décaler, donc avec la longueur des pattes. Avec le S3306 (limite utilisable 1,96 N·m) l'optimiseur doit garder $L_2\\approx 45$–$60$ mm, donc de faibles NESM (6,4 mm au mieux). Avec un MX-64AT en C2 (limite utilisable 5,0 N·m) il peut allonger les pattes ($L_2\\approx 85$–$110$ mm) et atteint 9 à 13 mm, y compris à ξ = 45° (12,7 mm pour un point robuste de 1,55 kg ; 9,5 mm pour 1,33 kg). Le prix est la masse (≥ 1,3 kg de pattes contre 1,03) et un couple C2 requis de 2,2 à 2,8 N·m, supérieur à ce que le S3306 sait fournir.</p>")
    h.append("<p><b>Deuxième effet : la masse.</b> Le MX-28T (2,5 N·m, 72 g) est plus fort et plus léger que le S3306 (2,35 N·m, 126 g) : à 45° le meilleur compromis robuste devient 0,97 kg / 3,0 mm / 1,46 N·m (SG90 / MX-28T / MX-28T) contre 1,39 kg / 1,29 mm / 1,47 N·m avec le catalogue actuel, et le front ne descend plus qu'à 0,95 kg.</p>")
    h.append("<p><b>Réponse à « si on prend l'autre moteur de 2,35 N·m ? »</b> Le seul moteur à 2,35 N·m du catalogue est le S3306 lui-même. L'équivalent le plus proche est le MX-28T (2,5 N·m) : il améliore la masse (−0,4 kg de pattes) et, à 45°, le NESM robuste (1,3 → 3,0 mm). Pour un gain de stabilité important il faut un servo de classe 6 N·m en hanche (MX-64AT) : NESM ×2 à ×4, avec des pattes plus longues et plus lourdes.</p>")
    h.append("<p><b>Réserves.</b> Stock de 1 unité par référence Dynamixel (il en faudrait 4 identiques ou 8 au total selon l'architecture), alimentation 12 V (S3306/S-U400 : 6–7,4 V), bus TTL, masses du catalogue « à vérifier », et la plage de 360° n'est pas exploitée. Ce scénario est un calcul de sensibilité, pas une proposition d'achat. Le NESM de 13,6 mm est atteint avec un balancement saturé à la borne de 60 mm : la grille limite donc aussi ce maximum.</p>")
    return "".join(h)
SEC5 = sec5()

# ---------------- assemblage
body = open(f"{HERE}/body.html", encoding="utf-8").read()
for k, v in V.items(): body = body.replace("{{V:%s}}" % k, str(v))
for k in [k for k in list(globals()) if k.startswith("TBL_")]: body = body.replace("{{%s}}" % k, globals()[k])
body = body.replace("{{SEC5}}", SEC5)
body = re.sub(r"\{\{IMG:(\w+)\}\}", lambda m: "data:image/png;base64," + base64.b64encode(open(f"{FIG}/{m.group(1)}.png", "rb").read()).decode(), body)
left = re.findall(r"\{\{[^}]*\}\}", body); assert not left, left
open(f"{OUT}/body_tmp.html", "w", encoding="utf-8").write(body)
subprocess.check_call(["node", f"{HERE}/render_math.js", f"{OUT}/body_tmp.html", f"{OUT}/body_math.html", KATEX])
math = open(f"{OUT}/body_math.html", encoding="utf-8").read(); css = open(f"{HERE}/style.css", encoding="utf-8").read()
def page(kc): return f'<!doctype html><html lang="fr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Stabilité et servos Gr2</title>{kc}<style>{css}</style></head><body><main>{math}</main></body></html>'
open(f"{OUT}/rapport4_pdf.html", "w", encoding="utf-8").write(page(f'<link rel="stylesheet" href="file://{KATEX}/katex/dist/katex.min.css">'))
open(f"{OUT}/rapport4_web.html", "w", encoding="utf-8").write(page('<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/KaTeX/0.16.11/katex.min.css">'))
print("ok")
