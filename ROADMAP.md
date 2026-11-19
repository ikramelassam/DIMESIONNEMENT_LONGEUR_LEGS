# Gr2 — feuille de route d'utilisation du code (optimiseur + simulateur)

Ce document dit **dans quel ordre** utiliser les outils, **quoi lire** dans les résultats et **quand s'arrêter**. Il suppose le dossier `gr2_dim/` (voir l'arbre en fin de document).

## 0. Les trois outils, et à quoi ils servent

| Outil | Question à laquelle il répond | Sens |
|---|---|---|
| **Optimiseur** (`gr2/pipeline.py`, `run_exact.py`, `serve.py`) | « Quelles longueurs, quelle posture et quels servos sont les meilleurs ? » | entrées → *meilleurs x* |
| **Simulateur** (`sim/gr2_simulateur.html`) | « Que vaut *ce* design ? Quelles contraintes casse-t-il, de combien, et à quelle distance est-il de l'optimum ? » | *x donné* → contraintes, f₁ f₂ f₃, écart à l'optimum |
| **Tests / audit** (`tests/`, `sim/validate*.js`, `audit/`) | « Les deux programmes calculent-ils la même chose ? » | preuve de cohérence |

Règle d'or : **l'optimiseur propose, le simulateur vérifie et explique, l'audit garantit que les deux parlent du même modèle.**

## 1. Installation (une fois)

```bash
pip install numpy pandas pyyaml scipy matplotlib pytest      # Python ≥ 3.10
python -m pytest -q                                           # 16 tests, ~40 s : tout doit passer
node -v                                                       # Node ≥ 18 : seulement pour les validations JS et l'audit
```

Le simulateur seul ne demande rien : ouvrir `sim/gr2_simulateur.html` dans un navigateur.

## 2. Parcours recommandé (8 étapes)

### Étape 1 — Remplacer les hypothèses par les vraies valeurs : `config.yaml`
Chaque valeur est étiquetée `[HYP]` (hypothèse), `[EQ]` (décision d'équipe) ou `[CDC]` (cahier des charges, **ne pas modifier**). Par ordre d'influence sur le résultat :

1. **Stock réel** de SG90, S-U400, S3306 (≥ 4 de chaque type utilisé) : `servos[].stock`.
2. **Masses et CdG issus de la CAO** : `body.chassis` (masse, position), `body.battery` et `battery_zone`, `body.arm_model` (masses, longueurs). Le CdG est ce qui pèse le plus sur la stabilité.
3. **Dimensions du châssis** `N_ch`, `M_ch` (aujourd'hui 200 × 200 supposé carré) et `c`, `e`, `overhang_C2`.
4. **Fraction de couple utilisable** `frac_couple` et facteur de sécurité `SF` (aujourd'hui 1,0 et 1,2 : optimiste, à mesurer sur banc).
5. Matériau des segments (`seg_lin_mass`), équerres, pied.
6. Allure : `accel`, `stroke`, `sway_max`, `dh_lift`.

Après chaque modification : `python -m pytest -q`.

### Étape 2 — Regarder où l'on en est : simulateur, mode « exact »
Ouvrir `sim/gr2_simulateur.html`. Choisir un préréglage (groupe « Préréglages et configuration »), le montage (dedans/dehors) et le scénario de CdG (S1 nominal, S2 pire cas). Lire dans l'ordre :

1. **Verdict** : admissible oui/non, et pourquoi.
2. **Contraintes et comparaison à l'optimum** : chaque contrainte avec valeur, limite, **marge** ; les trois objectifs ; la position par rapport au front de Pareto.
3. **Couples par articulation** : barre « requis » contre « limite » (rouge = couple de blocage, gris = 1/5 du blocage, recommandation Robotis).

> Si l'avertissement orange « la configuration a été modifiée par rapport aux résultats sauvegardés » apparaît, la comparaison à l'optimum n'est plus valable : passez à l'étape 4.

### Étape 3 — Pré-sélectionner les servos (indicatif)
Carte **Optimisation complète → Pré-sélectionner les servos possibles**. Elle calcule le couple minimal que *chaque* articulation devra fournir (avec un servo fictif illimité) et ne garde que les servos du catalogue qui peuvent y arriver. C'est une **borne inférieure** : un servo écarté l'est à coup sûr, un servo retenu n'est pas forcément suffisant.

### Étape 4 — Explorer vite (moteur navigateur), puis ajuster les bornes
Dans la même carte, moteur **Navigateur (JS)** : 1 024 points, 1 tour de raffinement, 30 tirages Monte-Carlo (≈ 20–60 s). Lire le résultat, puis regarder la colonne **« bornes actives »** :

* si un optimum est collé à une borne de recherche (ex. `alpha`), **élargir cette borne** (`Bornes de recherche`) et relancer ; ne jamais conclure sur un optimum collé à une borne ;
* si aucun design n'est admissible : élargir les bornes, autoriser d'autres servos, ou revoir une entrée trop sévère (le tableau des contraintes du simulateur dit laquelle).

### Étape 5 — Calculer pour de bon (moteur Python complet)
Deux voies équivalentes :

```bash
python serve.py                    # puis ouvrir http://localhost:8765 ; moteur « Python complet » dans la carte
# ou, sans interface, un cas à la fois (≈ 13–16 min chacun sur 2 cœurs) :
python run_exact.py dedans S2      # écrit results_exact/dedans_S2/{admissibles,pareto,pareto_robuste,candidats_robustesse}.csv + resume.json
python run_exact.py dedans S1 ; python run_exact.py dehors S2 ; python run_exact.py dehors S1
python sim/make_ref_fronts.py      # met à jour les fronts de référence du simulateur
cd sim && python build.py          # reconstruit gr2_simulateur.html
```

Défauts du moteur complet : 8 192 points de Sobol, 2 tours de raffinement, 24 points par point du front, 100 tirages Monte-Carlo, grille de balancement 11 × 11.

### Étape 6 — Décider (règles explicites, pas de « au feeling »)
Travailler sur le **front robuste** (réussite ≥ 95 % au Monte-Carlo) du scénario **S2** (pire cas) :

* **Compromis (coude)** : le point le plus proche de l'idéal (distance normalisée L², Zeleny 1973). Choix par défaut.
* **ε-contrainte** : le plus petit couple C₂ avec NESM ≥ `nesm_cible` (2 mm). À prendre si le couple de C₂ est le facteur limitant.
* **Plus petit C₂ du front** : borne basse de couple, au détriment de la stabilité.

Comparer les 4 cas (dedans/dehors × S1/S2). S2 donne le design sûr ; S1 dit ce que l'on perd par prudence.

### Étape 7 — Revenir au simulateur : vérifier, perturber, regarder les marges
Cliquer **Charger** à côté du choix retenu : le design passe dans le simulateur. Puis :

1. **Marges** (tableau des contraintes) : une marge < quelques % signale une contrainte presque saturée ; c'est là que la réalité (tolérances, masses réelles) cassera d'abord.
2. **Perturber une variable à la fois** (curseurs, ou carte « Impact d'un paramètre ») : voir quelle contrainte casse en premier et combien f₂, f₃ se dégradent. Comparer à l'écart à l'optimum.
3. **Changer un servo** : bouton « Servos les plus légers admissibles » (teste toutes les combinaisons compatibles avec le stock) ou choix manuel.
4. **Phase par phase** : sélecteurs Phase / État : polygone d'appui, CdG, pied sur l'obstacle, part de poids par pied.
5. **Changer une hypothèse** (batterie plus lourde, accélération, SF, châssis) : relancer l'optimisation (étape 4 ou 5) et comparer ; ne pas supposer que l'optimum reste le même (voir le rapport, §7 : il bouge).

### Étape 8 — Geler, puis recaler avec la CAO et le banc
1. Reporter L₁, L₂, L₃, α, β, ξ et les servos dans la CAO.
2. **Remettre dans `config.yaml` les masses et CdG réels de la CAO**, relancer étapes 5–7 : la décision doit rester valable (même servo en C₂, marge de NESM > 0).
3. **Essai sur banc d'une patte** : couple réellement tenu par le S3306 en C₂ à la posture de repos (le design demande 50–65 % du blocage, au-dessus de la recommandation Robotis de 20 %), température après 5 minutes. Si le servo chauffe : réduire `frac_couple`, relancer.

## 3. Checklist avant de commander les servos

- [ ] `pytest` passe ; `node sim/validate.js` et `node audit/audit_run.js` : 0 désaccord.
- [ ] Le stock est confirmé pour **4 unités** de chaque type retenu.
- [ ] Le design retenu est **admissible dans les deux montages ou le montage choisi**, en **S2**, avec réussite Monte-Carlo ≥ 95 %.
- [ ] Aucune « borne active » dans le choix.
- [ ] Marges du tableau des contraintes relues ; la plus petite est connue et acceptée.
- [ ] Les entrées `[HYP]` qui pèsent le plus (étape 1) sont remplacées ou, à défaut, testées en sensibilité.
- [ ] Un essai thermique du S3306 en C₂ est prévu.

## 4. Dictionnaire des sorties (colonnes des CSV)

| Colonne | Sens |
|---|---|
| `L1, L2, L3` | longueurs coxa, fémur, tibia (mm) |
| `alpha, beta, xi` | posture de repos (°) : élévation du fémur, inclinaison du tibia, orientation vue de dessus |
| `D, H2, h` | portée horizontale du pied, hauteur de C2, garde au sol (mm) |
| `servo_C1/C2/C3` | servo retenu par articulation |
| `f1_masse_pattes` | **f₁** : masse des 4 pattes (kg), à minimiser |
| `f2_nesm` | **f₂** : NESM garanti dans la pire phase (mm), à maximiser |
| `f3_couple_C2` | **f₃** : couple minimal requis en C₂ dans la pire phase (N·m), à minimiser |
| `tau_C1..3` | couple max aux positions retenues (N·m) |
| `tau_min_C1..3` | couple minimal requis (N·m) |
| `frac_bloc_C1..3` | part du couple de blocage requise |
| `span_C1..3` | course angulaire utilisée (°) |
| `M_total` | masse portée par les pattes (kg) |
| `phase_critique` | phase qui limite f₂ |
| `taux_reussite` | fraction de tirages Monte-Carlo où le design reste admissible |
| `ssm_corr_min` | marge statique corrigée (mm) |

## 5. Pièges connus

1. **Comparer à périmètre égal.** Le front de référence du simulateur correspond à `config.yaml` tel que sauvegardé. Toute modification (même la grille de balancement) rend la comparaison approximative ; l'avertissement orange le signale.
2. **Grille de balancement** : 11 × 11 par défaut (Python), 7 × 7 en JS rapide. Une grille plus fine donne un f₂ un peu plus élevé ; comparer à grille égale.
3. **Monte-Carlo différent JS/Python** : mêmes algorithmes, générateurs aléatoires différents. Deux exécutions complètes donnent des fronts voisins, pas identiques. Pour une décision, utiliser le moteur Python et noter la graine.
4. **L'artefact en ligne** n'atteint pas `localhost` : le moteur Python n'est utilisable que depuis la page servie par `serve.py`. Les exports CSV/JSON ne marchent aussi que là.
5. **Ex æquo de phase critique** (pattes symétriques) : plusieurs phases ont le même NESM ; le nom de la phase critique peut différer entre deux programmes sans que la valeur change. C'est normal.
6. **Une valeur hors domaine n'est pas un résultat** : f₂ et f₃ d'un design qui viole une contrainte sont affichés à titre indicatif, jamais comparables à ceux de l'optimiseur.
7. **Le modèle est statique** : pas d'inertie des pattes ni de chocs ; le mouvement final viendra du RL. Le dimensionnement garantit qu'*un* mouvement admissible existe, pas que le RL le trouvera.

## 6. Maintenance : tout revérifier après un changement de code

```bash
python -m pytest -q                          # modèle Python
node sim/validate.js                         # simulateur vs Python (12 designs, 324 combinaisons)
python sim/make_ref_opt.py && node sim/validate_opt.js      # optimiseur JS vs Python (lignes de résultat)
python sim/make_ref_pipe.py && node sim/validate_pipe.js    # pipeline JS vs Python de bout en bout
python audit/audit_make.py && node audit/audit_run.js       # audit large (≈ 800 designs × 27 servos), ≈ 45 min
cd sim && python build.py                    # reconstruit le simulateur (intègre les validations)
```

## 7. Arbre du dossier

```
gr2_dim/
├─ config.yaml            entrées (HYP / EQ / CDC)
├─ gr2/                   modèle et optimiseur Python
│  ├─ params.py kinematics.py gait.py statics.py evaluate.py   modèle
│  ├─ explore.py          Sobol, Pareto, couches, coude, ε-contrainte
│  └─ pipeline.py         optimisation complète (appelable)
├─ run_exact.py           un cas (montage × scénario) → results_exact/
├─ serve.py               serveur local : simulateur + moteur Python
├─ results_exact/         résultats de référence (4 cas)
├─ sim/                   simulateur : model.js opt.js compare.js ui.html build.py → gr2_simulateur.html
│                         ref_fronts.json (fronts de référence), validate*.js, experiences.js
├─ audit/                 audit de cohérence Python ↔ simulateur et expériences inverses
├─ tests/                 pytest
└─ report2/ report3/      rapports (PDF + HTML)
```
