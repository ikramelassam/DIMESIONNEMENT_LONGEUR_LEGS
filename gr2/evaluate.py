"""Evaluation vectorisee d'un lot de designs (geometrie + posture de repos)
pour toutes les affectations de servos du catalogue.

Variables d'un design x = (L2, L3, alpha, beta, xi) [mm, mm, deg, deg, deg].
Scenario de CdG :
  'S1' nominal  : CdG calcule a partir des entrees (batterie nominale, bras en
                  posture de marche ou de saisie) ; seule la petite incertitude
                  due au choix des servos (eps_legs, bornee) est ajoutee.
  'S2' pire cas : le CdG du corps peut etre n'importe ou dans la boite
                  (toutes postures du bras x toute la zone batterie) elargie de
                  eps_cdg + eps_legs ; on prend le pire point (exact, D6).
"""
from __future__ import annotations
import numpy as np
from .params import Params, G, LEG_SIGNS
from .kinematics import ik
from .statics import (barycentric, box_max, polygon_margins, nesm_3d,
                      four_feet_max_lambda, seg_seg_dist)
from .gait import build_phases

CYCLIC = [0, 2, 3, 1]   # FL, RL, RR, FR : ordre cyclique autour du chassis


# ----------------------------------------------------------------- geometrie
def geometry(X, P: Params):
    L1 = P["leg"]["L1"]
    ch = P["chassis"]
    L2, L3 = X[:, 0], X[:, 1]
    a, b, xi = np.radians(X[:, 2]), np.radians(X[:, 3]), np.radians(X[:, 4])
    D0 = L2 * np.cos(a) + L3 * np.cos(b)
    H2 = L3 * np.sin(b) - L2 * np.sin(a)
    h = H2 - ch["e"]
    u = np.stack([LEG_SIGNS[None, :, 0] * np.sin(xi)[:, None],
                  LEG_SIGNS[None, :, 1] * np.cos(xi)[:, None]], -1)          # (g,4,2)
    P0 = np.stack([LEG_SIGNS[:, 0] * (ch["N_ch"] / 2 - ch["c"]),
                   LEG_SIGNS[:, 1] * (ch["M_ch"] / 2 - ch["c"])], -1)        # (4,2)
    P0 = np.broadcast_to(P0, u.shape)
    if ch["mount"] == "dehors":       # axe C1 au coin, coxa vers l'exterieur
        C1 = P0.copy()
        foot = C1 + (L1 + D0)[:, None, None] * u
    elif ch["mount"] == "dedans":     # axe C2 au coin, C1 a L1 vers l'interieur
        C1 = P0 - L1 * u
        foot = P0 + D0[:, None, None] * u
    else:
        raise ValueError("mount doit etre 'dedans' ou 'dehors'")
    return dict(L2=L2, L3=L3, a=a, b=b, xi=xi, D0=D0, H2=H2, h=h, u=u, C1=C1, foot=foot)


def prefilter(X, P: Params):
    """Contraintes qui ne dependent que de la posture de repos (rapides).
    Retourne (masque, dict des violations normalisees)."""
    g = geometry(X, P)
    tk, mg, ch, lg = P["task"], P["margins"], P["chassis"], P["leg"]
    viol = {}
    h_min = tk["obstacle"] + tk["h_margin"]
    viol["G5_garde_sol"] = (h_min - g["h"]) / h_min
    th3 = np.degrees(g["a"] + g["b"])
    viol["G3_theta3_repos"] = np.maximum(mg["theta3_min"] - th3, th3 - mg["theta3_max"]) / 90.0
    ext_x = 2 * np.abs(g["foot"][..., 0]).max(1) + lg["foot_diam"]
    ext_y = 2 * np.abs(g["foot"][..., 1]).max(1) + lg["foot_diam"]
    gab = float(P.raw.get("gabarit", 400.0))      # cote du carre de gabarit (cahier des charges : 400 mm)
    viol["G6_gabarit"] = np.maximum(ext_x, ext_y) / gab - 1
    # EMPREINTE IMPOSEE (optionnelle, ajout 17/11) : "occuper tout le gabarit".
    # gabarit_min = 0 (defaut) -> contrainte inactive, comportement inchange.
    gab_min = float(P.raw.get("gabarit_min", 0.0))
    if gab_min > 0:
        viol["G6b_empreinte_min"] = (gab_min - np.maximum(ext_x, ext_y)) / gab_min
    # axes C1 distincts et espaces (encombrement des servos C1)
    sp = mg.get("servo_spacing", 40.0)
    dx = g["C1"][:, 0, 0] - g["C1"][:, 2, 0]     # FL - RL
    dy = g["C1"][:, 0, 1] - g["C1"][:, 1, 1]     # FL - FR
    viol["G7_axes_C1"] = (sp - np.minimum(dx, dy)) / sp
    viol["G10_chassis"] = np.full(len(X), max(ch["N_ch"], ch["M_ch"]) / gab - 1)
    ok = np.all(np.stack([v <= 0 for v in viol.values()]), 0)
    return ok, viol, g


# --------------------------------------------------------------- evaluation
class Evaluator:
    def __init__(self, P: Params, scenario: str, exact_combo=None):
        """exact_combo = (i1, i2, i3) : CdG des pattes calcule EXACTEMENT avec ces
        servos (eps_legs = 0) et seule cette combinaison est evaluee. Sinon :
        servo de reference + borne prudente eps_legs pour tout le catalogue."""
        assert scenario in ("S1", "S2")
        self.P, self.sc = P, scenario
        self.exact_combo = tuple(exact_combo) if exact_combo is not None else None
        tk = P["task"]
        self.phases = build_phases(tk["stroke"], tk["obstacle"] + tk["dh_lift"],
                                   tk["obstacle"], tk["yaw_step"])
        n, smax = tk["sway_n"], tk["sway_max"]
        lin = np.linspace(-smax, smax, n)
        self.sw = np.stack(np.meshgrid(lin, lin, indexing="ij"), -1).reshape(-1, 2)
        self.combos = [self.exact_combo] if self.exact_combo else P.combos()
        b = P["body"]
        if scenario == "S1":
            mw, gw = P.body_nominal(b["walk_posture"])
            mm, gm = P.body_nominal(b["manip_posture"])
            self.body = {"walk": (mw, gw, np.zeros(3)), "manip": (mm, gm, np.zeros(3))}
            es = b.get("eps_s1", 0.0)   # 0 en nominal ; > 0 dans la contrepartie robuste
            self.eps = np.array([es, es, es])
        else:
            mw, cw, hw = P.body_box(b.get("walk_postures_S2"))
            mm, cm, hm = P.body_box(b.get("manip_postures_S2"))
            self.body = {"walk": (mw, cw, hw), "manip": (mm, cm, hm)}
            self.eps = np.array([b["eps_cdg_xy"], b["eps_cdg_xy"], b["eps_cdg_z"]])
        self.m_body_max = max(v[0] for v in self.body.values())

    # ------------------------------------------------------------ une patte
    def _leg_pose(self, geo, i, off, zf, yaw):
        """Pose d'une patte pour toutes les positions du corps (balancements)."""
        L1 = self.P["leg"]["L1"]
        foot_w = geo["foot"][:, i, :] + np.array([off, 0.0])           # monde (g,2)
        rel = foot_w[:, None, :] - self.sw[None, :, :]                  # (g,S,2)
        if yaw != 0.0:
            c, s = np.cos(np.radians(-yaw)), np.sin(np.radians(-yaw))
            rel = np.stack([c * rel[..., 0] - s * rel[..., 1], s * rel[..., 0] + c * rel[..., 1]], -1)
        v = rel - geo["C1"][:, None, i, :]
        rh = np.linalg.norm(v, axis=-1)
        dirv = v / np.maximum(rh, 1e-9)[..., None]
        D = rh - L1
        Hz = geo["H2"][:, None] - zf
        L2, L3 = geo["L2"][:, None], geo["L3"][:, None]
        al, be, d, reach = ik(L2, L3, D, Hz)
        m = self.P["margins"]["reach_margin"]
        reach = reach & (d <= L2 + L3 - m) & (d >= np.abs(L2 - L3) + m) & (D > 0)
        th3 = np.degrees(al + be)
        u0 = geo["u"][:, None, i, :]
        th1 = np.degrees(np.arctan2(u0[..., 0] * dirv[..., 1] - u0[..., 1] * dirv[..., 0],
                                    np.sum(u0 * dirv, -1)))
        C2 = geo["C1"][:, None, i, :] + L1 * dirv
        knee_xy = C2 + (L2 * np.cos(al))[..., None] * dirv
        z2 = np.broadcast_to(geo["H2"][:, None], D.shape)
        knee = np.concatenate([knee_xy, (z2 + L2 * np.sin(al))[..., None]], -1)
        C2_3 = np.concatenate([C2, z2[..., None]], -1)
        C1_3 = np.concatenate([np.broadcast_to(geo["C1"][:, None, i, :], C2.shape), z2[..., None]], -1)
        foot3 = np.concatenate([rel, np.full(D.shape + (1,), zf)], -1)
        return dict(rel=rel, D=D, rh=rh, al=al, be=be, reach=reach, th1=th1, th2=np.degrees(al),
                    th3=th3, C1=C1_3, C2=C2_3, knee=knee, foot=foot3, L2=L2, L3=L3)

    def _leg_moment(self, pose, m_s1, m_s2, m_s3):
        """Somme m*position des pieces d'une patte (kg.mm), masses de reference."""
        lg = self.P["leg"]
        lam = lg["seg_lin_mass"] / 1000.0
        br = lg["bracket_mass"]
        L1 = lg["L1"]
        mom = (m_s1 * pose["C1"] + (lam * L1 + br) * 0.5 * (pose["C1"] + pose["C2"])
               + (m_s2 + br) * pose["C2"] + lam * pose["L2"][..., None] * 0.5 * (pose["C2"] + pose["knee"])
               + (m_s3 + br) * pose["knee"] + lam * pose["L3"][..., None] * 0.5 * (pose["knee"] + pose["foot"])
               + lg["foot_mass"] * pose["foot"])
        return mom

    # --------------------------------------------------------------- lot
    def run(self, X):
        P = self.P
        ok0, viol0, geo = prefilter(X, P)
        g_n, S = len(X), len(self.sw)
        if self.exact_combo:
            mr = [P.servos[i].mass for i in self.exact_combo]
        else:
            mr = [P.ref_servo.mass] * 3
        mref = P.ref_servo.mass
        legm_ref = P.leg_mass(geo["L2"], geo["L3"], *mr)
        tk, mg, ch, lg = P["task"], P["margins"], P["chassis"], P["leg"]
        acc = tk["accel"] / G

        # ---------- passe 1 : poses + CdG de reference + borne eps_legs
        states = [(pi, st) for pi, ph in enumerate(self.phases) for st in ph.states]
        poses, Gref, Mref_s = [], [], []
        E_xy = np.zeros(g_n)
        E_z = np.zeros(g_n)
        for pi, st in states:
            mb, gb, _ = self.body[st.cdg]
            M_ref = mb + 4 * legm_ref
            legs = {}
            for (l, off, zf) in st.ground:
                legs[l] = self._leg_pose(geo, l, off, zf, st.yaw)
            if st.swing is not None:
                l, off, zf = st.swing
                legs[l] = self._leg_pose(geo, l, off, zf, st.yaw)
            mom = np.zeros((g_n, S, 3))
            sP = {t: np.zeros((g_n, S, 3)) for t in ("C1", "C2", "knee")}
            for l, ps in legs.items():
                mom += self._leg_moment(ps, *mr)
                for t in sP:
                    sP[t] += ps[t]
            gb_abs = np.array([gb[0], gb[1], 0.0])[None, :] + np.stack(
                [np.zeros(g_n), np.zeros(g_n), geo["h"] + gb[2]], -1)
            Gr = (mb * gb_abs[:, None, :] + mom) / M_ref[:, None, None]
            for t in sP:
                diff = sP[t] - 4 * Gr
                E_xy = np.maximum(E_xy, np.linalg.norm(diff[..., :2], axis=-1).max(1))
                E_z = np.maximum(E_z, np.abs(diff[..., 2]).max(1))
            Gref.append(Gr)
            Mref_s.append(M_ref)
        masses = np.array([s.mass for s in P.servos])
        dm = 0.0 if self.exact_combo else np.abs(masses - mref).max()
        M_min = self.m_body_max * 0 + min(v[0] for v in self.body.values()) + 4 * P.leg_mass(
            geo["L2"], geo["L3"], masses.min(), masses.min(), masses.min())
        eps_legs_xy = 3 * dm * E_xy / M_min        # 3 types d'articulation
        eps_legs_z = 3 * dm * E_z / M_min

        # ---------- passe 2 : stabilite, couples, geometrie, angles par etat
        nP = len(self.phases)
        geo_ok = np.ones((g_n, nP, S), bool)
        nesm = np.full((g_n, nP, S), np.inf)
        ssm_c = np.full((g_n, nP, S), np.inf)
        k = np.zeros((g_n, nP, S, 3))            # coefficients de couple (mm) C1,C2,C3
        swA2 = np.zeros((g_n, nP, S))
        swB2 = np.zeros((g_n, nP, S))
        swA3 = np.zeros((g_n, nP, S))
        ang_min = np.full((g_n, nP, S, 4, 3), np.inf, dtype=np.float32)
        ang_max = np.full((g_n, nP, S, 4, 3), -np.inf, dtype=np.float32)
        lam_kg = lg["seg_lin_mass"] / 1000.0
        for (pi, st), Gr, M_ref in zip(states, Gref, Mref_s):
            mb, gb, hb = self.body[st.cdg]
            legs = {}
            for (l, off, zf_) in st.ground:
                legs[l] = self._leg_pose(geo, l, off, zf_, st.yaw)
            if st.swing is not None:
                l, off, zf_ = st.swing
                legs[l] = self._leg_pose(geo, l, off, zf_, st.yaw)
            hxy = hb[:2] * (mb / M_ref)[:, None] + self.eps[:2] + eps_legs_xy[:, None]
            hz = hb[2] * mb / M_ref + self.eps[2] + eps_legs_z
            hxy = np.broadcast_to(hxy[:, None, :], (g_n, S, 2))
            Gc = Gr[..., :2]
            zmax = Gr[..., 2] + hz[:, None]
            ground = [l for (l, _, _) in st.ground]
            order = [l for l in CYCLIC if l in ground]
            feet = np.stack([legs[l]["rel"] for l in order], -2)            # (g,S,n,2)
            zf = np.array([z for (l, _, z) in sorted(st.ground, key=lambda t: order.index(t[0]))])
            ssm, nes = polygon_margins(feet, Gc, hxy, zmax)
            if np.ptp(zf) > 0:   # pieds a des hauteurs differentes : NESM 3D aux 8 coins
                feet3 = np.concatenate([feet, np.broadcast_to(zf, feet.shape[:-1])[..., None]], -1)
                n3 = np.full((g_n, S), np.inf)
                for sx in (-1, 1):
                    for sy in (-1, 1):
                        for sz in (-1, 1):
                            Gk = np.concatenate([Gc + np.stack([sx * hxy[..., 0], sy * hxy[..., 1]], -1),
                                                 (Gr[..., 2] + sz * hz[:, None])[..., None]], -1)
                            n3 = np.minimum(n3, nesm_3d(feet3, Gk))
                nes = n3
            # reactions robustes (parts de poids max par pied)
            if len(order) == 3:
                lam, grad = barycentric(Gc, feet[..., 0, :], feet[..., 1, :], feet[..., 2, :])
                lmax = np.clip(box_max(lam, grad, hxy[..., None, :]), 0, 1)
                lam_leg = {l: lmax[..., j] for j, l in enumerate(order)}
            else:
                f4 = np.stack([legs[l]["rel"] for l in range(4)], -2)
                l4, _ = four_feet_max_lambda(f4, Gc, hxy)
                lam_leg = {l: l4[..., l] for l in range(4)}
            kk = np.zeros((g_n, S, 3))
            ok = np.ones((g_n, S), bool)
            for l in ground:
                ps = legs[l]
                kk[..., 0] = np.maximum(kk[..., 0], lam_leg[l] * acc * ps["rh"])
                kk[..., 1] = np.maximum(kk[..., 1], lam_leg[l] * ps["D"])
                kk[..., 2] = np.maximum(kk[..., 2], lam_leg[l] * ps["L3"] * np.abs(np.cos(ps["be"])))
            for l, ps in legs.items():
                ok &= ps["reach"] & (ps["th3"] >= mg["theta3_min"]) & (ps["th3"] <= mg["theta3_max"])
                # pieds hors du contour du chassis (pas de pied ni de tibia sous le ventre)
                fx, fy = ps["rel"][..., 0], ps["rel"][..., 1]
                cl = mg["chassis_clearance"]
                ok &= (np.abs(fx) >= ch["N_ch"] / 2 + cl) | (np.abs(fy) >= ch["M_ch"] / 2 + cl)
                if ch["mount"] == "dedans":   # axe C2 : debord limite hors du chassis
                    o = ch["overhang_C2"]
                    ok &= (np.abs(ps["C2"][..., 0]) <= ch["N_ch"] / 2 + o) & (np.abs(ps["C2"][..., 1]) <= ch["M_ch"] / 2 + o)
                for j, key in enumerate(("th1", "th2", "th3")):
                    ang_min[:, pi, :, l, j] = np.minimum(ang_min[:, pi, :, l, j], ps[key])
                    ang_max[:, pi, :, l, j] = np.maximum(ang_max[:, pi, :, l, j], ps[key])
            # pattes d'un meme cote (vue de dessus) : distance entre segments C1->pied
            for la, lb in ((0, 2), (1, 3)):
                pa, pb = legs[la], legs[lb]
                dseg = seg_seg_dist(pa["C1"][..., :2], pa["rel"], pb["C1"][..., :2], pb["rel"])
                ok &= dseg >= mg["feet_clearance"]
            if st.swing is not None:
                ps = legs[st.swing[0]]
                xk = ps["L2"] * np.cos(ps["al"])
                xf = ps["D"]
                swA2[:, pi] = np.maximum(swA2[:, pi], G / 1000 * np.abs(
                    lam_kg * ps["L2"] * xk / 2 + lg["bracket_mass"] * xk
                    + lam_kg * ps["L3"] * (xk + xf) / 2 + lg["foot_mass"] * xf))
                swB2[:, pi] = np.maximum(swB2[:, pi], G / 1000 * np.abs(xk))
                swA3[:, pi] = np.maximum(swA3[:, pi], G / 1000 * np.abs(
                    (lam_kg * ps["L3"] / 2 + lg["foot_mass"]) * (xf - xk)))
            geo_ok[:, pi] &= ok
            nesm[:, pi] = np.minimum(nesm[:, pi], nes)
            ssm_c[:, pi] = np.minimum(ssm_c[:, pi], ssm - zmax * acc - mg["eps_foot"])
            k[:, pi] = np.maximum(k[:, pi], kk)

        self.last = dict(geo=geo, geo_ok=geo_ok, nesm=nesm, ssm_c=ssm_c, k=k, swA2=swA2,
                         swB2=swB2, swA3=swA3, ang_min=ang_min, ang_max=ang_max,   # (ajout 17/11 : demo_un_design.py)
                         eps_legs_xy=eps_legs_xy, Gref=Gref, states=states)
        return self._resolve_servos(X, geo, ok0, viol0, geo_ok, nesm, ssm_c, k,
                                    swA2, swB2, swA3, ang_min, ang_max, eps_legs_xy)

    # ------------------------------------------------- choix exact des servos
    def _resolve_servos(self, X, geo, ok0, viol0, geo_ok, nesm, ssm_c, k,
                        swA2, swB2, swA3, ang_min, ang_max, eps_legs):
        P = self.P
        mg = P["margins"]
        SF, nmin = mg["SF"], mg["nesm_min"]
        stab_ok = geo_ok & (nesm >= nmin) & (ssm_c >= mg.get("s_min", 0.0))
        b = P["body"]
        m_nolimb = b["chassis"]["m"] + b["battery"]["m"]
        rows = []
        g_n = len(X)
        for (i1, i2, i3) in self.combos:
            s1, s2, s3 = P.servos[i1], P.servos[i2], P.servos[i3]
            legm = P.leg_mass(geo["L2"], geo["L3"], s1.mass, s2.mass, s3.mass)
            Mc = self.m_body_max + 4 * legm                                  # kg
            tau = Mc[:, None, None, None] * G * k / 1000.0                   # N.m
            Cu = np.array([s1.C_cont, s2.C_cont, s3.C_cont]) / SF
            tq_ok = np.all(tau <= Cu, -1) & (swA2 + swB2 * s3.mass <= Cu[1]) & (swA3 <= Cu[2])
            feas = stab_ok & tq_ok
            score = np.where(feas, nesm, -np.inf)
            best = score.argmax(-1)                                           # (g,P)
            ph_ok = feas.any(-1)
            nes_best = np.take_along_axis(nesm, best[..., None], -1)[..., 0]
            tau_best = np.take_along_axis(tau, best[..., None, None], 2)[:, :, 0, :]
            ssm_best = np.take_along_axis(ssm_c, best[..., None], -1)[..., 0]
            amin = np.take_along_axis(ang_min, best[..., None, None, None], 2)[:, :, 0]   # (g,P,4,3)
            amax = np.take_along_axis(ang_max, best[..., None, None, None], 2)[:, :, 0]
            span = amax.max(1) - amin.min(1)                                  # (g,4,3)
            rng = np.array([s1.range, s2.range, s3.range]) - 2 * mg["joint_margin"]
            span_ok = np.all(span <= rng, (-1, -2))
            mass_ok = m_nolimb + 4 * legm <= b["mass_robot_max_sans_bras"]
            all_ok = ok0 & ph_ok.all(-1) & span_ok & mass_ok
            f2 = np.where(ph_ok, nes_best, np.inf).min(-1)
            u = (tau_best / np.array([s1.C_cont, s2.C_cont, s3.C_cont])).max(1)   # (g,3)
            sw_best = self.sw[best]                                           # (g,P,2)
            # couple minimal necessaire (par articulation) si l'on accepte juste NESM >= NESM_min
            tmin = np.where(feas[..., None], tau, np.inf).min(2).max(1)          # (g,3)
            # f3 : couple REQUIS en C2 (independant du servo C2 choisi) = pour chaque
            # phase, le plus petit tau2 sur les balancements stables ou C1 et C3 tiennent ;
            # on prend aussi le couple C2 de la patte en vol ; puis le max sur les phases.
            tq13 = (tau[..., 0] <= Cu[0]) & (tau[..., 2] <= Cu[2]) & (swA3 <= Cu[2])
            t2 = np.maximum(tau[..., 1], swA2 + swB2 * s3.mass)                       # (g,P,S)
            f3 = np.where(stab_ok & tq13, t2, np.inf).min(-1).max(-1)                 # (g,)
            Cst = np.array([s1.C_stall, s2.C_stall, s3.C_stall])
            for gi in np.nonzero(all_ok)[0]:
                rows.append(dict(
                    L1=P["leg"]["L1"], L2=X[gi, 0], L3=X[gi, 1], alpha=X[gi, 2], beta=X[gi, 3], xi=X[gi, 4],
                    D=geo["D0"][gi], H2=geo["H2"][gi], h=geo["h"][gi],
                    servo_C1=s1.name, servo_C2=s2.name, servo_C3=s3.name,
                    f1_masse_pattes=4 * legm[gi], f2_nesm=f2[gi], f3_couple_C2=f3[gi],
                    ssm_corr_min=ssm_best[gi].min(),
                    frac_bloc_C1=tmin[gi, 0] / Cst[0], frac_bloc_C2=f3[gi] / Cst[1], frac_bloc_C3=tmin[gi, 2] / Cst[2],
                    u_C1=u[gi, 0], u_C2=u[gi, 1], u_C3=u[gi, 2],
                    tau_C1=tau_best[gi, :, 0].max(), tau_C2=tau_best[gi, :, 1].max(), tau_C3=tau_best[gi, :, 2].max(),
                    tau_min_C1=tmin[gi, 0], tau_min_C2=tmin[gi, 1], tau_min_C3=tmin[gi, 2],
                    span_C1=span[gi, :, 0].max(), span_C2=span[gi, :, 1].max(), span_C3=span[gi, :, 2].max(),
                    M_total=Mc[gi], eps_legs=eps_legs[gi],
                    sway_max=np.abs(sw_best[gi]).max(),
                    phase_critique=self.phases[int(np.where(ph_ok[gi], nes_best[gi], np.inf).argmin())].name,
                ))
        # statistiques des contraintes (pour les cartes de contraintes actives)
        stats = dict(n=g_n, prefilter_ok=int(ok0.sum()),
                     viol0={kk: int((v > 0).sum()) for kk, v in viol0.items()},
                     geo_any=int(geo_ok.any(-1).all(-1).sum()),
                     stab_any=int(stab_ok.any(-1).all(-1).sum()))
        return rows, stats
