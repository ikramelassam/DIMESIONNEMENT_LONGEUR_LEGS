# Liaison simulateur ↔ code complet d'optimisation (Gr2)

## Ce qui a été fait
Le simulateur contient une carte « Optimisation complète » qui exécute le MÊME pipeline que `gr2/pipeline.py` :
Sobol → pré-filtre → évaluation exacte de chaque combinaison de servos → raffinement local → Pareto 3D (f1 masse, f2 NESM, f3 couple C2) → couches de non-dominance → Monte-Carlo → front robuste → choix (coude, ε-contrainte NESM ≥ 2 mm).
Tous les intrants sont modifiables : bornes de recherche, servos autorisés par articulation, nombre de points Sobol, tours de raffinement, tirages MC, grille de balancement, graine. Les paramètres de la carte « Patte », « Châssis », « CdG », « Servos » (catalogue) sont repris. Bouton « Charger » : le design choisi est chargé dans le simulateur pour inspection phase par phase.

## Deux moteurs
- **Navigateur (JS)** : `sim/opt.js`, port du pipeline, résolution réduite par défaut (1024 points, MC 30). Fonctionne dans l'artefact.
- **Python complet** : `python serve.py` puis http://localhost:8765 (8192 points, 2 tours, MC 100). Code de référence. Ne fonctionne PAS depuis la page de l'artefact (pas d'accès à localhost) : il faut ouvrir la page servie par serve.py.

## Validation
- `evalRow` JS vs Python : 128 lignes, écart relatif max 2,1e-7.
- Pareto, couches, coude, ε-choix identiques sur 2 cas (dedans S2, dehors S1).
- Bout en bout sur même échantillon (sans raffinement ni MC) : 2/2 cas identiques (front, stats, choix).
- Reproductibilité `pipeline.optimize` vs `results_exact/dedans_S2` : stats par combinaison identiques (4 combos sur 6 vérifiés avant interruption).
- 16 tests pytest passent. Test navigateur headless : moteur JS (23 s) et moteur Python (50 s) OK, pas d'erreur JS, pas de débordement horizontal à 390 px.

## Limites
- Sobol (embrouillage), raffinement et MC : générateurs différents JS/Python → fronts voisins, pas identiques.
- La pré-sélection des servos est indicative (borne inférieure avec servo fictif).
- Les boutons d'export CSV/JSON ne fonctionnent pas dans l'artefact (téléchargement bloqué) ; ils fonctionnent via serve.py.
