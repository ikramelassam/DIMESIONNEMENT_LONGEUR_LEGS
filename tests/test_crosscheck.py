"""Verification croisee : couple de C2 calcule par l'evaluateur vectorise ==
calcul naif independant (cinematique a la main + programme lineaire)."""
import sys, os
import numpy as np
from scipy.optimize import linprog
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from gr2.params import Params, G, LEG_SIGNS
from gr2.evaluate import Evaluator

CFG = os.path.join(os.path.dirname(__file__), "..", "config.yaml")


def test_couple_C2_independant():
    P = Params.load(CFG)
    mref = P.ref_servo.mass
    for s in P.raw["servos"]:
        s["mass"] = mref            # eps_legs = 0 pour comparer exactement
    P = Params.from_dict(P.raw)
    P.raw["chassis"]["mount"] = "dedans"
    L1, L2, L3, A, B, XI = 30.0, 100.0, 130.0, 35.0, 80.0, 40.0
    X = np.array([[L2, L3, A, B, XI]])
    E = Evaluator(P, "S1"); E.run(X); L = E.last
    si = int(np.argmin(np.linalg.norm(E.sw, axis=1)))
    a, b, xi = np.radians([A, B, XI])
    D0 = L2 * np.cos(a) + L3 * np.cos(b); H2 = L3 * np.sin(b) - L2 * np.sin(a)
    ch = P["chassis"]; h = H2 - ch["e"]
    feet, C1 = {}, {}
    for i, (sx, sy) in enumerate(LEG_SIGNS):
        u = np.array([sx * np.sin(xi), sy * np.cos(xi)])
        P0 = np.array([sx * (ch["N_ch"] / 2 - ch["c"]), sy * (ch["M_ch"] / 2 - ch["c"])])
        C1[i] = P0 - L1 * u; feet[i] = P0 + D0 * u
    mb, gb = P.body_nominal("transport")
    lam = P["leg"]["seg_lin_mass"] / 1000; br = P["leg"]["bracket_mass"]; mf = P["leg"]["foot_mass"]

    def pose(i, off, zf):
        f = feet[i] + np.array([off, 0]); v = f - C1[i]; rh = np.linalg.norm(v); dr = v / rh
        D = rh - L1; Hz = H2 - zf; d = np.hypot(D, Hz)
        psi = np.arccos((L2 ** 2 + d ** 2 - L3 ** 2) / (2 * L2 * d)); al = psi - np.arctan2(Hz, D)
        C2 = np.r_[C1[i] + L1 * dr, H2]; kn = np.r_[C1[i] + L1 * dr + L2 * np.cos(al) * dr, H2 + L2 * np.sin(al)]
        ft = np.r_[f, zf]; c1 = np.r_[C1[i], H2]
        mom = (mref * c1 + (lam * L1 + br) * (c1 + C2) / 2 + (mref + br) * C2 + lam * L2 * (C2 + kn) / 2
               + (mref + br) * kn + lam * L3 * (kn + ft) / 2 + mf * ft)
        return dict(D=D, mom=mom, f=f)
    M = mb + 4 * P.leg_mass(L2, L3, mref, mref, mref)
    kmax = 0.0
    for st_idx in range(3):        # les 3 etats de la phase 'avant_RL'
        _, st = L["states"][st_idx]
        ps = {l: pose(l, o, z) for (l, o, z) in st.ground}
        if st.swing:
            l, o, z = st.swing; ps[l] = pose(l, o, z)
        Gc = (mb * np.r_[gb[0], gb[1], h + gb[2]] + sum(p["mom"] for p in ps.values())) / M
        gl = [g_[0] for g_ in st.ground]; T = np.array([ps[i]["f"] for i in gl])
        Aeq = np.vstack([np.ones(len(gl)), T[:, 0], T[:, 1]])
        for j, i in enumerate(gl):
            c = np.zeros(len(gl)); c[j] = -1
            r = linprog(c, A_eq=Aeq, b_eq=np.r_[1, Gc[0], Gc[1]], bounds=[(0, None)] * len(gl))
            kmax = max(kmax, -r.fun * ps[i]["D"])
    assert abs(kmax - L["k"][0, 0, si, 1]) < 1e-6
