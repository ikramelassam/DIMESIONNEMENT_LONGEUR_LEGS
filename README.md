# gr2_dim — dimensionnement des pattes du robot Gr2

Optimiseur Python + simulateur JavaScript reliés, avec audit de cohérence.

* **Par où commencer** : `ROADMAP.md` (ordre d'utilisation, lecture des sorties, checklist avant commande des servos).
* **Simulateur** : ouvrir `sim/gr2_simulateur.html` (ou `python serve.py` puis http://localhost:8765 pour le moteur Python complet).
* **Entrées** : `config.yaml` (`[HYP]` hypothèse, `[EQ]` décision d'équipe, `[CDC]` cahier des charges).
* **Résultats de référence** : `results_exact/` (4 cas : dedans/dehors × S1/S2).
* **Rapports** : `report2/` (dimensionnement, équations, résultats) et `report3/` (cohérence simulateur–optimiseur, lecture inverse, expériences, feuille de route).
* **Vérifications** : `python -m pytest -q`, `node sim/validate.js`, `audit/` (voir `ROADMAP.md` §6).

## Empreinte imposée et empreinte maximale (ajout du 17/11/2026)

L'empreinte au sol se règle maintenant dans le code, avec la clé `gabarit_min` (mm) :
la plus grande dimension de l'empreinte doit être ≥ `gabarit_min` (et toujours ≤ `gabarit`, 400 mm).
`gabarit_min: 0` (défaut) = contrainte inactive, résultats inchangés.

* **Une empreinte précise, étude complète** (Pareto + Monte-Carlo) :
  `python run_final.py E340` (ξ libre) ou `python run_final.py E300_XI45` (ξ = 45°).
  L'empreinte est imposée dans la bande [E − 4, E] ; les géométries sont tirées directement
  sur la surface « empreinte = E » (formule : E = 190 + 2·D·max(sin ξ, cos ξ), montage dedans).
* **La plus grande empreinte tenable** : `python empreinte_max.py 340 350 360`
  (ou `--xi45`, `--rapide`). Pour chaque E : faisable (≥ 1 design admissible), utile
  (NESM ≥ `margins.nesm_cible`), robuste (≥ 95 % au Monte-Carlo), et le meilleur design.
  Résultats dans `results_final/empreinte_max_*.json`.
* **Dans le simulateur** : groupe « Limites d'angles et marges », réglage
  « Empreinte minimale imposée (0 = libre) ». Le verdict signale `G6b` si le design est
  trop petit ; l'onglet d'optimisation respecte la contrainte (tirage par rejet : pour une
  borne proche de 400 mm, préférer `empreinte_max.py`, qui tire sur la surface).
* Test associé : `tests/test_model.py::test_empreinte_imposee`.

## Comprendre le code sur un exemple : `demo_un_design.py`

Suit UN design à travers tout le programme et affiche chaque étape (géométrie, pré-filtre,
les 14 phases avec les 121 positions du corps, couples, courses, masse, verdict, f1 f2 f3) :

    python demo_un_design.py                          # design retenu
    python demo_un_design.py --xi 45                  # même patte, ξ = 45°
    python demo_un_design.py --servos SG90,S3306,S3306
    python demo_un_design.py --set task.obstacle=40   # n'importe quelle clé de config.yaml
    python demo_un_design.py --set servos.S-U400.C_stall=0.638   # un servo de la liste, par son nom

## Servos vérifiés et tension d'alimentation (ajout du 17/11/2026)

Les couples et masses de `config.yaml` ont été comparés au catalogue du FabLab et aux fiches
des constructeurs (détail en commentaire dans `config.yaml` et dans l'onglet « Servos » du Sheet).
Tout est exact, sauf le SG90 : 14 g au catalogue, 9 g sur la fiche TowerPro ; on garde 14 g
(prudent, effet négligeable).

Point à trancher : le S3306 et le SG90 acceptent 6 V au maximum, mais le couple du S-U400
(0,696 N·m) est donné à 6,6 V. Les résultats supposent donc deux rails d'alimentation.
Sur un seul rail de 6 V (S-U400 ≈ 0,638 N·m) :

    python run_final.py OPT_6V S2 dedans    # « 6V » dans le nom du cas = S-U400 à 0,638 N·m

Résultat : le design retenu reste admissible mais sa NESM tombe de 4,10 à 2,36 mm ; le meilleur
compromis à 6 V atteint 3,00 mm (L2 60,0 ; L3 75,7 ; α 13,9° ; β 103,5° ; ξ 82,5° ; SG90/S3306/S-U400).

## Un seul moteur dans les trois articulations (ajout du 17/11/2026)

    python run_final.py OPT_MONO43 S2 dedans --servos S3306,S3306,S3306 --masse_mc 4.3
    python empreinte_max.py --servos S3306,S3306,S3306 --masse_mc 4.3 300 310 320 330 340

`--servos C1,C2,C3` n'étudie que ce triplet. `--masse_mc 4.3` relâche la limite de 4 kg
DANS LE MONTE-CARLO seulement (le design nominal reste jugé à 4 kg) : avec 12 S3306 le robot
sans bras pèse 3,90 kg, et les tirages à ±10 % de masse dépassent 4 kg dans 20 à 30 % des cas,
ce qui masquerait tout le reste.

Seul le S3306 peut tenir C2 (même la géométrie la plus favorable demande au moins 0,85 N·m en C2,
le S-U400 n'en donne que 0,58 utilisable). Résultats (S2, montage dedans) :
- meilleur design robuste : L2 71,3 ; L3 89,4 ; α 23,3° ; β 94,8° ; ξ 70,8° ;
  empreinte 300 × 228 mm ; pattes 1,856 kg ; NESM 5,13 mm ; C2 1,632 N·m ; 99 % au Monte-Carlo ;
- empreinte : robuste jusqu'à 300 mm (310 : 89 % au mieux), NESM ≥ 2 mm jusqu'à 340 mm,
  faisable jusqu'à 340-345 mm (350 : aucun design).
