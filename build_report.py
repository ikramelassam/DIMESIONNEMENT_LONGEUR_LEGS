"""Construit le rapport HTML (results/rapport_gr2.html) a partir des resultats.
Tous les chiffres du rapport sont lus dans results/ : rien n'est recopie a la main."""
import base64, html, json, os, sys
import numpy as np
import pandas as pd
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")
FIG = os.path.join(RES, "figures")
CASES = ["dedans_S1", "dedans_S2", "dehors_S1", "dehors_S2"]
CASE_TXT = {"dedans_S1": "L1 dedans, CdG nominal (S1)", "dedans_S2": "L1 dedans, CdG pire cas (S2)",
            "dehors_S1": "L1 dehors, CdG nominal (S1)", "dehors_S2": "L1 dehors, CdG pire cas (S2)"}


def img(name, alt):
    p = os.path.join(FIG, name)
    if not os.path.exists(p):
        return f"<p class='muted'>(figure {html.escape(name)} absente)</p>"
    b = base64.b64encode(open(p, "rb").read()).decode()
    return f"<figure><img src='data:image/png;base64,{b}' alt='{html.escape(alt)}'><figcaption>{html.escape(alt)}</figcaption></figure>"


def f(x, n=1):
    try:
        return f"{float(x):.{n}f}".replace(".", ",")
    except Exception:
        return html.escape(str(x))


def load():
    out = {}
    for c in CASES:
        p = os.path.join(RES, c, "resume.json")
        if os.path.exists(p):
            out[c] = json.load(open(p))
            cand = os.path.join(RES, c, "candidats_robustesse.csv")
            out[c]["cand"] = pd.read_csv(cand) if os.path.exists(cand) and os.path.getsize(cand) > 1 else pd.DataFrame()
    sens = os.path.join(RES, "sensibilite.csv")
    return out, (pd.read_csv(sens) if os.path.exists(sens) else pd.DataFrame())


def choice_table(data):
    rows = []
    keys = [("L1", "L1 (mm)", 0), ("L2", "L2 (mm)", 1), ("L3", "L3 (mm)", 1), ("alpha", "alpha repos (deg)", 1),
            ("beta", "beta repos (deg)", 1), ("xi", "xi repos (deg)", 1), ("D", "D repos (mm)", 1),
            ("H2", "H2 (mm)", 1), ("h", "garde au sol h (mm)", 1), ("servo_C1", "servo C1", None),
            ("servo_C2", "servo C2", None), ("servo_C3", "servo C3", None),
            ("f1_masse_pattes", "f1 masse 4 pattes (kg)", 3), ("f2_nesm", "f2 NESM min (mm)", 2),
            ("ssm_corr_min", "SSM corrigee min (mm)", 1),
            ("tau_C1", "couple C1 a la position choisie (N.m)", 2), ("tau_C2", "couple C2 (N.m)", 2), ("tau_C3", "couple C3 (N.m)", 2),
            ("tau_min_C2", "couple C2 minimal necessaire (N.m)", 2),
            ("u_C1", "utilisation C1 (/ couple continu)", 2), ("u_C2", "utilisation C2", 2), ("u_C3", "utilisation C3", 2),
            ("span_C1", "course utilisee C1 (deg)", 0), ("span_C2", "course C2 (deg)", 0), ("span_C3", "course C3 (deg)", 0),
            ("M_total", "masse totale portee (kg)", 2), ("phase_critique", "phase critique", None),
            ("taux_reussite", "taux de reussite Monte-Carlo", 2), ("bornes_actives", "bornes de recherche atteintes", None)]
    cols = []
    for c, d in data.items():
        for k in ("coude", "eps_contrainte"):
            if k in d.get("choix", {}):
                cols.append((f"{CASE_TXT[c]}<br><small>choix « {'point de coude' if k=='coude' else 'plus leger avec NESM ≥ 2 mm'} »</small>", d["choix"][k]))
    if not cols:
        return "<p>Aucun design robuste trouve avec les entrees actuelles.</p>"
    h = "<div class='tw'><table class='num'><thead><tr><th>grandeur</th>" + "".join(f"<th>{t}</th>" for t, _ in cols) + "</tr></thead><tbody>"
    for key, lab, n in keys:
        h += f"<tr><td>{lab}</td>"
        for _, r in cols:
            v = r.get(key, "")
            if isinstance(v, list):
                v = ", ".join(v) if v else "aucune"
            h += f"<td>{f(v, n) if n is not None else html.escape(str(v))}</td>"
        h += "</tr>"
    return h + "</tbody></table></div>"


def stats_table(data):
    h = ("<div class='tw'><table class='num'><thead><tr><th>cas</th><th>echantillons</th><th>apres prefiltre</th>"
         "<th>geometrie OK</th><th>stabilite OK</th><th>geometries admissibles</th><th>couples (design, servos) admissibles</th>"
         "<th>points du front</th><th>candidats testes (MC)</th><th>candidats robustes (≥ 95 %)</th><th>front robuste</th>"
         "<th>admissibles « tout au pire »</th></tr></thead><tbody>")
    for c, d in data.items():
        s = d["stats"]
        h += (f"<tr><td>{CASE_TXT[c]}</td><td>{s['n_echantillons']}</td><td>{s['apres_prefiltre']}</td><td>{s['geometrie_ok']}</td>"
              f"<td>{s['stabilite_ok']}</td><td>{s['designs_geometriques_admissibles']}</td><td>{s['admissibles']}</td>"
              f"<td>{s['pareto']}</td><td>{s.get('candidats_testes','')}</td><td>{s.get('candidats_robustes','')}</td>"
              f"<td>{s['pareto_robuste']}</td><td>{s.get('admissibles_tout_au_pire','')}</td></tr>")
    return h + "</tbody></table></div>"


def front_robust_rates(data):
    h = "<ul>"
    for c, d in data.items():
        r = d["stats"].get("taux_reussite_front_nominal", [])
        if r:
            h += f"<li><b>{CASE_TXT[c]}</b> : taux de reussite des points du front nominal testes = {', '.join(f(x, 2) for x in r)}</li>"
    return h + "</ul>"


def sens_table(sens):
    if sens.empty:
        return "<p class='muted'>(etude de sensibilite non executee)</p>"
    h = ("<div class='tw'><table class='num'><thead><tr><th>parametre</th><th>valeur</th><th>couples admissibles</th>"
         "<th>masse min des pattes (kg)</th><th>NESM max (mm)</th></tr></thead><tbody>")
    for _, r in sens.iterrows():
        h += (f"<tr><td>{html.escape(r.parametre)}</td><td>{f(r.valeur, 2)}</td><td>{int(r.n_admissibles)}</td>"
              f"<td>{f(r.masse_min, 3)}</td><td>{f(r.nesm_max, 2)}</td></tr>")
    return h + "</tbody></table></div>"


def config_table(cfg):
    rows = []

    def walk(d, pre=""):
        for k, v in d.items():
            if isinstance(v, dict):
                walk(v, pre + k + ".")
            elif k == "servos":
                pass
            else:
                rows.append((pre + k, v))
    walk({k: v for k, v in cfg.items() if k != "servos"})
    h = "<div class='tw'><table><thead><tr><th>parametre</th><th>valeur</th></tr></thead><tbody>"
    for k, v in rows:
        h += f"<tr><td><code>{html.escape(k)}</code></td><td>{html.escape(str(v))}</td></tr>"
    h += "</tbody></table></div><h4>Catalogue de servos (exemples fictifs sauf EQUIPE)</h4><div class='tw'><table class='num'><thead><tr><th>nom</th><th>couple continu (N.m)</th><th>masse (kg)</th><th>course (deg)</th></tr></thead><tbody>"
    for s in cfg["servos"]:
        h += f"<tr><td>{s['name']}</td><td>{f(s['C_cont'],2)}</td><td>{f(s['mass'],3)}</td><td>{f(s['range'],0)}</td></tr>"
    return h + "</tbody></table></div>"


def main():
    data, sens = load()
    cfg = yaml.safe_load(open(os.path.join(HERE, "config.yaml"), encoding="utf-8"))
    tpl = open(os.path.join(HERE, "report_template.html"), encoding="utf-8").read()
    figs = {
        "{{FIG_FRONTS}}": img("fronts.png", "Fronts de Pareto : nominal (lignes) et robuste (carres) ; etoile = point de coude, croix = plus leger avec NESM >= 2 mm"),
        "{{FIG_ENTONNOIR}}": img("entonnoir.png", "Entonnoir : nombre de geometries qui survivent a chaque etape (echelle log)"),
        "{{FIG_REJETS}}": img("rejets.png", "Contraintes de la posture de repos qui eliminent le plus de geometries"),
        "{{FIG_SERVOS}}": img("servos.png", "Servos retenus par articulation parmi les designs admissibles"),
        "{{FIG_SENS}}": img("sensibilite.png", "Etude de sensibilite (un facteur a la fois, montage dedans, scenario S2)") if os.path.exists(os.path.join(RES, "sensibilite.png")) else "",
    }
    if os.path.exists(os.path.join(RES, "sensibilite.png")):
        b = base64.b64encode(open(os.path.join(RES, "sensibilite.png"), "rb").read()).decode()
        figs["{{FIG_SENS}}"] = f"<figure><img src='data:image/png;base64,{b}' alt='sensibilite'><figcaption>Etude de sensibilite : nombre de couples (design, servos) admissibles quand un seul parametre varie (montage dedans, scenario S2)</figcaption></figure>"
    drawings = ""
    for c in CASES:
        for k in ("coude", "eps_contrainte"):
            n = f"design_{c}_{k}.png"
            if os.path.exists(os.path.join(FIG, n)):
                drawings += img(n, f"{CASE_TXT[c]} : design « {k} »")
    rep = {
        "{{TABLE_STATS}}": stats_table(data),
        "{{TABLE_CHOIX}}": choice_table(data),
        "{{FRONT_ROBUST}}": front_robust_rates(data),
        "{{TABLE_SENS}}": sens_table(sens),
        "{{TABLE_CONFIG}}": config_table(cfg),
        "{{DESSINS}}": drawings,
    }
    rep.update(figs)
    for key, fn in (("{{CONCLUSIONS}}", "conclusions.html"), ("{{CRITIQUE}}", "critique.html")):
        p = os.path.join(HERE, fn)
        rep[key] = open(p, encoding="utf-8").read() if os.path.exists(p) else ""
    for k, v in rep.items():
        tpl = tpl.replace(k, v)
    out = os.path.join(RES, "rapport_gr2.html")
    open(out, "w", encoding="utf-8").write(tpl)
    print("rapport :", out, round(os.path.getsize(out) / 1e6, 2), "Mo")


if __name__ == "__main__":
    main()
