"""Chargement et pre-traitement des entrees (config.yaml).

Toutes les grandeurs sont en mm, kg, N.m, degres. Ce module ne fait aucun
calcul de dimensionnement : il transforme le fichier d'entrees en objets
directement utilisables (masses du corps, boite du CdG, catalogue de servos).
"""
from __future__ import annotations
import copy
import itertools
from dataclasses import dataclass, field
import numpy as np
import yaml

G = 9.81  # m/s2

# Ordre des pattes : FL, FR, RL, RR (avant-gauche, avant-droite, arriere-gauche, arriere-droite)
LEG_NAMES = ["FL", "FR", "RL", "RR"]
LEG_SIGNS = np.array([[+1, +1], [+1, -1], [-1, +1], [-1, -1]], dtype=float)


@dataclass
class Servo:
    name: str
    C_stall: float  # N.m : couple max / de blocage du catalogue
    mass: float     # kg
    range: float    # deg
    stock: int | None = None   # unites disponibles (None = inconnu)
    frac: float = 1.0          # fraction du couple de blocage utilisable en continu

    @property
    def C_cont(self) -> float:
        """Couple utilisable en continu (N.m), avant coefficient de securite."""
        return self.frac * self.C_stall


@dataclass
class Params:
    raw: dict
    servos: list = field(default_factory=list)

    # ------------------------------------------------------------------ io
    @staticmethod
    def load(path: str) -> "Params":
        with open(path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        return Params.from_dict(raw)

    @staticmethod
    def from_dict(raw: dict) -> "Params":
        p = Params(raw=copy.deepcopy(raw))
        fr = raw.get("frac_couple", 1.0)
        p.servos = [Servo(**s, frac=fr) for s in raw["servos"]]
        return p

    def copy(self) -> "Params":
        return Params.from_dict(copy.deepcopy(self.raw))

    # -------------------------------------------------------- raccourcis
    def __getitem__(self, k):
        return self.raw[k]

    @property
    def ref_servo(self) -> Servo:
        name = self.raw["reference_servo"]
        return next(s for s in self.servos if s.name == name)

    def combos(self):
        """Affectations (servo C1, servo C2, servo C3) compatibles avec le stock :
        un modele utilise sur k types d'articulation demande 4k unites."""
        out = []
        for c in itertools.product(range(len(self.servos)), repeat=3):
            ok = True
            for i in set(c):
                st = self.servos[i].stock
                if st is not None and 4 * c.count(i) > st:
                    ok = False
            if ok:
                out.append(c)
        return out

    # ---------------------------------------------------- masses de patte
    def leg_mass(self, L2, L3, m1, m2, m3):
        """Masse d'UNE patte (kg) : 3 servos + segments + equerres + pied."""
        lg = self.raw["leg"]
        lam = lg["seg_lin_mass"] / 1000.0  # kg/mm
        return (m1 + m2 + m3 + lam * (lg["L1"] + L2 + L3)
                + 3 * lg["bracket_mass"] + lg["foot_mass"])

    # ---------------------------------------------------- corps (hors pattes)
    def body_mass(self, with_arm=True):
        b = self.raw["body"]
        m = b["chassis"]["m"] + b["battery"]["m"]
        if with_arm:
            m += self.arm_mass() + b["object_mass"]
        return m

    # ---------------------------------------------------------------- bras
    def arm_mass(self):
        a = self.raw["body"]["arm_model"]
        return a["m_base"] + a["m_elbow"] + a["m_tip"]

    def arm_points(self, posture):
        """Masses ponctuelles du bras (et de l'objet s'il est porte) dans le
        repere corps, z en 'z_rel'. Modele plan a 3 masses :
          epaule S = (N/2 - retrait, y, z_fix)
          coude  E = S + a1 (cos phi1, 0, sin phi1)
          pince  T = E + a2 (cos(phi1+phi2), 0, sin(phi1+phi2))"""
        b = self.raw["body"]
        a = b["arm_model"]
        p = a["postures"][posture]
        f1, f2 = np.radians(p["phi"][0]), np.radians(p["phi"][0] + p["phi"][1])
        S = np.array([self.raw["chassis"]["N_ch"] / 2 - a["mount_from_front"], a["mount_y"], a["mount_z_rel"]])
        E = S + a["a1"] * np.array([np.cos(f1), 0.0, np.sin(f1)])
        T = E + a["a2"] * np.array([np.cos(f2), 0.0, np.sin(f2)])
        pts = [(a["m_base"], S), (a["m_elbow"], E), (a["m_tip"], T)]
        if p["obj"]:
            pts.append((b["object_mass"], T.copy()))
        return pts

    def arm_cdg(self, posture):
        """(masse, CdG) du bras seul et de l'objet s'il est porte."""
        pts = self.arm_points(posture)
        m = sum(mi for mi, _ in pts)
        return m, sum(mi * pi for mi, pi in pts) / m

    def _body_cdg(self, battery_xyz, posture):
        """CdG du corps (chassis + batterie + bras [+ objet]) et sa masse.
        z en 'z_rel' (au-dessus du dessous du chassis)."""
        b = self.raw["body"]
        ch = b["chassis"]
        items = [(ch["m"], np.array([ch["x"], ch["y"], ch["z_rel"]])),
                 (b["battery"]["m"], np.asarray(battery_xyz, float))]
        # bras (3 masses) + objet s'il est porte ; sans objet, sa masse n'est pas comptee
        items += self.arm_points(posture)
        m = sum(mi for mi, _ in items)
        gpos = sum(mi * pi for mi, pi in items) / m
        return m, gpos

    def body_nominal(self, posture):
        b = self.raw["body"]["battery"]
        return self._body_cdg([b["x"], b["y"], b["z_rel"]], posture)

    def body_box(self, postures=None):
        """Boite (min, max) du CdG du corps sur toutes les postures du bras et
        toute la zone possible de la batterie. Le CdG est affine en la position
        de la batterie : les extremes sont atteints aux coins de la zone.
        Retourne (m_max, centre, demi-largeurs) ; m_max = masse avec objet."""
        z = self.raw["body"]["battery_zone"]
        pts, masses = [], []
        for post in (postures or self.raw["body"]["arm_model"]["postures"]):
            for bx in z["x"]:
                for by in z["y"]:
                    for bz in z["z_rel"]:
                        m, gp = self._body_cdg([bx, by, bz], post)
                        pts.append(gp)
                        masses.append(m)
        pts = np.array(pts)
        lo, hi = pts.min(0), pts.max(0)
        return max(masses), (lo + hi) / 2, (hi - lo) / 2
