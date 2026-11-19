# Cohérence simulateur ↔ optimiseur Gr2 — synthèse

Rapport complet (PDF, 15 p.) livré dans la conversation ; version web publiée en artifact. Guide d'usage : `claude/roadmap-utilisation.md`.

## Principe
Optimiseur : trouve x=(L2,L3,α,β,ξ) + servos tels que g≤0, minimise f1 (masse 4 pattes), maximise f2 (NESM), minimise f3 (couple C2).
Simulateur = lecture inverse : on donne (x, servos, config) → marge de chaque contrainte, ce qui casse, valeurs f1/f2/f3, position vs front de Pareto (dominance, distance normalisée à l'idéal, points nommés coude / ε-contrainte / couple min / extrêmes).

## Audit (3 niveaux)
- A. model.js (JS) vs evaluate.py : 21 627 paires comparées — 0 désaccord de pré-filtre, 0 désaccord d'admissibilité, 0 désaccord de dominance ; objectifs f1/f2/f3 égaux à ~3e-14.
- B. opt.js vs pipeline.py : fronts, couches, coude, ε-contrainte identiques sur le même échantillon ; RNG différents (Sobol brouillé, raffinement, MC) → fronts proches mais pas identiques.
- C. CSV sauvegardés vs ré-évaluation : 81 lignes, 1e-15.
- Contrôle physique indépendant : tests/test_crosscheck.py (cinématique à la main + linprog), 16 tests pytest.
- 9 cas résiduels, tous expliqués par des égalités exactes (pattes symétriques, phase « saisie » à deux positions symétriques de même NESM).
- Bug trouvé et corrigé dans mon script d'audit (clé de correspondance sur 3 variables au lieu de 5).

## Limites
Cohérence d'implémentation ≠ validité physique : équations partagées, entrées [HYP], catalogue réduit, modèle statique. Le moteur JS tourne à résolution réduite ; l'artifact ne joint pas localhost (mode Python = `python serve.py`) ; exports inopérants dans l'artifact. Châssis 260 mm → +31 % de couple : non expliqué.

## Enseignements des expériences
- L'optimum est adossé à G5 (garde au sol) et T1 (couple).
- Falaises de robustesse à l'intérieur du domaine admissible.
- Le « coude » n'est pas un minimum exact (β+5° donne un meilleur compromis).
- S3306 obligatoire en C2 ; pattes tout-S3306 plus lourdes = pires.
- Hypothèses (batterie, SF, frac, accél., masse bras, obstacle, châssis) : −33 à −97 % de designs admissibles, +18 à +41 % de couple C2 min ; frac 0,7 critique (20 designs restants).
- Bruit du moteur JS mesuré : adm ±7 %, min f3 ±7 %, max f2 jusqu'à +27 %.

## Entrées à obtenir de l'équipe
Stock SG90 / S-U400 / S3306, dimensions châssis, données bras, masses/CdG CAO, couple continu utilisable.
