"""Definition des etats a verifier (taches du cahier des charges).

Allure de reference pour les verifications : crawl DISCONTINU (le corps ne
bouge que lorsque les 4 pieds sont au sol ; Gonzalez de Santos 1998 discute ce
type d'allure). Ce n'est PAS l'allure finale (elle sera apprise) : c'est un
temoin d'existence d'au moins un mouvement admissible (contrainte G9).

Cycle (ordre de levee RL, FL, RR, FR) ; chaque patte avance de R_s quand elle
est en l'air ; le corps avance de R_s/4 avant chaque levee. En regime etabli,
juste avant la levee de la patte k, les decalages (le long de x, par rapport
a la position de repos) des 4 pieds sont :
    patte k : -R_s/2   (au fond de sa course)
    k-1     : +R_s/4   (posee le plus recemment)
    k-2     :  0
    k-3     : -R_s/4
Pendant le vol de k (3 pieds au sol) ces decalages ne changent pas ; k passe
par l'apex (decalage 0, hauteur obstacle + dh) et se pose a +R_s/2.

Chaque PHASE regroupe des etats qui partagent la meme position du corps
(balancement cherche par l'optimiseur interne, G9) :
  avant k   : etat pre-levee (4 pieds), etat en vol (3 pieds + patte k a l'apex),
              etat post-pose (4 pieds)
  arriere k : idem, miroir avant/arriere (marche arriere, cahier des charges)
  obstacle k: comme 'avant' mais pieds AVANT sur l'obstacle (z = 30 mm)
  rotation  : 4 pieds, corps tourne de +-yaw/2 (rotation sur place)
  saisie    : 4 pieds au repos, bras tendu avec objet
"""
from dataclasses import dataclass, field

FL, FR, RL, RR = 0, 1, 2, 3
MIRROR = {FL: RL, RL: FL, FR: RR, RR: FR}
FRONT = (FL, FR)


@dataclass
class State:
    phase: str
    ground: list           # [(patte, decalage_x, z_pied)]
    swing: tuple = None    # (patte, decalage_x, z_pied) ou None
    yaw: float = 0.0       # deg
    cdg: str = "walk"      # 'walk' ou 'manip'
    label: str = ""


@dataclass
class Phase:
    name: str
    states: list = field(default_factory=list)


def build_phases(R, lift_z, obstacle, yaw_step):
    order = [RL, FL, RR, FR]
    phases = []

    def cycle(name, mirror, on_obst):
        for k_i, k in enumerate(order):
            prev1, prev2, prev3 = order[k_i - 1], order[k_i - 2], order[k_i - 3]
            offs = {k: -R / 2, prev1: +R / 4, prev2: 0.0, prev3: -R / 4}
            if mirror:  # marche arriere : symetrie avant/arriere
                offs = {MIRROR[l]: -o for l, o in offs.items()}
                kk = MIRROR[k]
            else:
                kk = k

            def z(l):
                return obstacle if (on_obst and l in FRONT) else 0.0
            ph = Phase(f"{name}_{['FL','FR','RL','RR'][kk]}")
            ph.states.append(State(ph.name, [(l, o, z(l)) for l, o in offs.items()], label="pre-levee (4 pieds)"))
            ph.states.append(State(ph.name, [(l, o, z(l)) for l, o in offs.items() if l != kk],
                                   swing=(kk, 0.0, lift_z),  # apex a obstacle+dh au-dessus du sol
                                   label="vol (3 pieds)"))
            post = dict(offs)
            post[kk] = -post[kk]  # -R/2 -> +R/2 (ou miroir)
            ph.states.append(State(ph.name, [(l, o, z(l)) for l, o in post.items()], label="post-pose (4 pieds)"))
            phases.append(ph)

    cycle("avant", False, False)
    cycle("arriere", True, False)
    cycle("obstacle", False, True)
    rot = Phase("rotation")
    for s in (-1, 1):
        rot.states.append(State("rotation", [(l, 0.0, 0.0) for l in range(4)], yaw=s * yaw_step / 2,
                                label=f"lacet {s*yaw_step/2:+.1f} deg"))
    phases.append(rot)
    man = Phase("saisie")
    man.states.append(State("saisie", [(l, 0.0, 0.0) for l in range(4)], cdg="manip", label="bras tendu"))
    phases.append(man)
    return phases
