"""Tests des proprietes demontrees dans le rapport (lancer : python -m pytest -q)."""
import sys, os
import numpy as np
from scipy.optimize import linprog
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from gr2.kinematics import fk, ik, jacobian_det
from gr2.statics import (barycentric, box_max, box_min, polygon_margins, nesm_3d,
                         four_feet_max_lambda, seg_seg_dist)
from gr2.params import Params
from gr2.evaluate import geometry, Evaluator
from gr2.gait import build_phases

rng = np.random.default_rng(0)
CFG = os.path.join(os.path.dirname(__file__), "..", "config.yaml")


def test_fk_ik_aller_retour():
    """D0 : cinematique directe puis inverse = identite (branche genou haut)."""
    L2 = rng.uniform(60, 140, 5000); L3 = rng.uniform(80, 200, 5000)
    a = np.radians(rng.uniform(-30, 80, 5000))
    th3 = np.radians(rng.uniform(5, 175, 5000))
    b = th3 - a
    D, Hz = fk(L2, L3, a, b)
    keep = D > 1
    a2, b2, d, ok = ik(L2[keep], L3[keep], D[keep], Hz[keep])
    assert ok.all()
    D2, Hz2 = fk(L2[keep], L3[keep], a2, b2)
    assert np.abs(D2 - D[keep]).max() < 1e-8 and np.abs(Hz2 - Hz[keep]).max() < 1e-8
    assert np.abs(a2 - a[keep]).max() < 1e-8


def test_jacobien_singularite():
    """D8 : det J = -L2 L3 sin(theta3) (differences finies)."""
    L2, L3 = 90.0, 120.0
    for _ in range(200):
        a, t3 = rng.uniform(-1, 1.3), rng.uniform(0.1, 3.0)
        e = 1e-6
        f = lambda a_, t_: np.array(fk(L2, L3, a_, t_ - a_))
        J = np.column_stack([(f(a + e, t3) - f(a - e, t3)) / (2 * e), (f(a, t3 + e) - f(a, t3 - e)) / (2 * e)])
        assert abs(np.linalg.det(J) - jacobian_det(L2, L3, a, t3 - a)) < 1e-5


def test_reactions_equilibre():
    """D1 : R_i = M g lambda_i verifie l'equilibre des forces et des moments."""
    for _ in range(500):
        T = rng.uniform(-200, 200, (3, 2))
        lam, _ = barycentric(np.mean(T, 0), T[0], T[1], T[2])
        p = (lam[:, None] * T).sum(0)
        assert abs(lam.sum() - 1) < 1e-9 and np.allclose(p, np.mean(T, 0))


def test_pire_cas_coin_boite():
    """D6 : le max d'une fonction affine sur une boite est donne exactement par
    la fonction d'appui, et aucun point interieur ne le depasse."""
    for _ in range(300):
        T = rng.uniform(-200, 200, (3, 2)); c = T.mean(0); h = rng.uniform(1, 20, 2)
        lam, grad = barycentric(c, T[0], T[1], T[2])
        bound = box_max(lam, grad, h[None, :])
        pts = c + rng.uniform(-1, 1, (400, 2)) * h
        lamp, _ = barycentric(pts, T[0][None], T[1][None], T[2][None])
        assert (lamp <= bound + 1e-9).all()
        corners = np.array([[sx, sy] for sx in (-1, 1) for sy in (-1, 1)]) * h + c
        lc, _ = barycentric(corners, T[0][None], T[1][None], T[2][None])
        assert np.allclose(lc.max(0), bound)


def test_nesm_valeurs():
    """D4 : NESM sol plat = sqrt(d^2+z^2)-z ; exemple du cours : d=60, z=80 -> 20."""
    feet = np.array([[[-100, 60], [100, 60], [0, -200]]], float)
    # d = distance du centre au bord y=60 -> choisir G a 60 mm du bord
    G = np.array([[0.0, 0.0]])
    ssm, nesm = polygon_margins(feet, G, np.zeros((1, 2)), np.array([80.0]))
    d = ssm[0]
    assert abs(nesm[0] - (np.hypot(d, 80) - 80)) < 1e-9
    assert abs((np.hypot(60, 80) - 80) - 20) < 1e-12


def test_nesm_3d_egal_plat():
    """Le NESM 3D (Hirose) se reduit au NESM plat quand les pieds sont au meme niveau."""
    for _ in range(300):
        T = rng.uniform(-200, 200, (3, 2)); c = T.mean(0) + rng.uniform(-10, 10, 2); z = rng.uniform(40, 150)
        ssm, n2 = polygon_margins(T[None], c[None], np.zeros((1, 2)), np.array([z]))
        n3 = nesm_3d(np.concatenate([T, np.zeros((3, 1))], 1)[None], np.array([[c[0], c[1], z]]))
        if ssm[0] > 0:
            assert abs(n2[0] - n3[0]) < 1e-8


def test_quatre_pieds_borne_exacte():
    """D12 : avec 4 pieds, le max de R_i sur toutes les repartitions admissibles
    (programme lineaire) egale le max des solutions a 3 pieds."""
    for _ in range(200):
        sx, sy = rng.uniform(80, 200), rng.uniform(80, 200)
        F = np.array([[sx, sy], [sx, -sy], [-sx, sy], [-sx, -sy]]) + rng.uniform(-30, 30, (4, 2))
        Gp = rng.uniform(-40, 40, 2)
        bound, _ = four_feet_max_lambda(F[None], Gp[None], np.zeros((1, 2)))
        A_eq = np.vstack([np.ones(4), F[:, 0], F[:, 1]]); b_eq = np.array([1.0, Gp[0], Gp[1]])
        for i in range(4):
            c = np.zeros(4); c[i] = -1
            r = linprog(c, A_eq=A_eq, b_eq=b_eq, bounds=[(0, None)] * 4)
            if r.status == 0:
                assert abs(-r.fun - bound[0, i]) < 1e-6


def test_convexite_extremites_course():
    """D7 : la distance hanche-pied le long d'une course rectiligne est maximale a une extremite."""
    for _ in range(500):
        p0 = rng.uniform(-150, 150, 2); u = rng.normal(size=2); u /= np.linalg.norm(u)
        t = np.linspace(-1, 1, 201)[:, None] * rng.uniform(5, 60)
        d = np.linalg.norm(p0 + t * u, axis=1)
        assert d.max() <= max(d[0], d[-1]) + 1e-9


def test_distance_segments():
    assert seg_seg_dist(np.array([0., 0]), np.array([1., 1]), np.array([0., 1]), np.array([1., 0])) == 0.0
    assert abs(seg_seg_dist(np.array([0., 0]), np.array([1., 0]), np.array([0., 2]), np.array([1., 2])) - 2) < 1e-12


def test_geometrie_montages():
    """Dedans / dehors : le pied est a L1 + D de l'axe C1 dans la direction xi."""
    P = Params.load(CFG)
    X = np.array([[90, 120, 40, 75, 40.0]])
    for mount in ("dedans", "dehors"):
        P.raw["chassis"]["mount"] = mount
        g = geometry(X, P)
        dist = np.linalg.norm(g["foot"] - g["C1"], axis=-1)
        assert np.allclose(dist, P["leg"]["L1"] + g["D0"][:, None])


def test_allure_decalages():
    """Crawl discontinu : avant la levee de k, decalages {-R/2, +R/4, 0, -R/4}."""
    R = 40.0
    for ph in build_phases(R, 50, 30, 10)[:4]:
        pre = sorted(o for (_, o, _) in ph.states[0].ground)
        assert np.allclose(pre, [-R / 2, -R / 4, 0, R / 4])
        assert len(ph.states[1].ground) == 3 and ph.states[1].swing is not None


def test_monotonie_couple():
    """Plus de couple disponible ne peut pas reduire le nombre de solutions."""
    P = Params.load(CFG)
    rng2 = np.random.default_rng(3)
    b = P["search"]["bounds"]
    lo = np.array([b[k][0] for k in ("L2", "L3", "alpha", "beta", "xi")])
    hi = np.array([b[k][1] for k in ("L2", "L3", "alpha", "beta", "xi")])
    X = lo + rng2.random((300, 5)) * (hi - lo)
    n1 = len(Evaluator(P, "S1").run(X)[0])
    for s in P.raw["servos"]:
        s["C_stall"] *= 1.3
    n2 = len(Evaluator(Params.from_dict(P.raw), "S1").run(X)[0])
    assert n2 >= n1


def test_f3_coherent_et_stock():
    """f3 (couple requis C2) <= couple C2 au balancement retenu et <= couple utilisable ;
    les modeles en stock 1 ne sont jamais affectes (il en faut 4)."""
    P = Params.load(CFG)
    rng2 = np.random.default_rng(5)
    b = P["search"]["bounds"]
    lo = np.array([b[k][0] for k in ("L2", "L3", "alpha", "beta", "xi")])
    hi = np.array([b[k][1] for k in ("L2", "L3", "alpha", "beta", "xi")])
    X = lo + rng2.random((300, 5)) * (hi - lo)
    rows, _ = Evaluator(P, "S1").run(X)
    cap = {s.name: s.C_cont / P["margins"]["SF"] for s in P.servos}
    for r in rows:
        assert r["f3_couple_C2"] <= r["tau_C2"] + 1e-9
        assert r["f3_couple_C2"] <= cap[r["servo_C2"]] + 1e-9
    stock1 = {s.name for s in P.servos if s.stock is not None and s.stock < 4}
    assert all(r["servo_C" + j] not in stock1 for r in rows for j in "123")


def test_cdg_bras_a_la_main():
    """Bras tendu : epaule (N/2-15, 0, 40), coude +a1, pince +a1+a2, objet dans la pince."""
    P = Params.load(CFG)
    a = P["body"]["arm_model"]
    xs = P["chassis"]["N_ch"] / 2 - a["mount_from_front"]
    m = a["m_base"] + a["m_elbow"] + a["m_tip"] + P["body"]["object_mass"]
    x = (a["m_base"] * xs + a["m_elbow"] * (xs + a["a1"]) + (a["m_tip"] + P["body"]["object_mass"]) * (xs + a["a1"] + a["a2"])) / m
    mm, g = P.arm_cdg("tendu")
    assert abs(mm - m) < 1e-12 and abs(g[0] - x) < 1e-9 and abs(g[2] - a["mount_z_rel"]) < 1e-9


def test_mode_exact():
    """Mode exact avec S3306 partout = mode borne avec un catalogue d'un seul servo
    (alors eps_legs = 0) ; et le mode exact n'est jamais plus pessimiste que la borne."""
    P = Params.load(CFG); P.raw["chassis"]["mount"] = "dedans"; P = Params.from_dict(P.raw)
    X = np.array([[70.0, 120.0, 40.0, 75.0, 30.0], [55.0, 90.0, 20.0, 95.0, 70.0]])
    i = next(k for k, s in enumerate(P.servos) if s.name == "S3306")
    Ex = Evaluator(P, "S2", (i, i, i)); Ex.run(X)
    Q = P.copy(); Q.raw["servos"] = [s for s in Q.raw["servos"] if s["name"] == "S3306"]; Q = Params.from_dict(Q.raw)
    Eb = Evaluator(Q, "S2"); Eb.run(X)
    assert np.allclose(Ex.last["nesm"], Eb.last["nesm"]) and np.allclose(Ex.last["k"], Eb.last["k"])
    Eg = Evaluator(P, "S2"); Eg.run(X)          # borne prudente, catalogue complet
    m = np.isfinite(Eg.last["nesm"]) & Eg.last["geo_ok"]
    assert np.all(Ex.last["nesm"][m] >= Eg.last["nesm"][m] - 1e-9)


def test_empreinte_imposee():
    """Ajout 17/11 : la contrainte gabarit_min (empreinte imposee) est inactive par
    defaut, et la formule E = 190 + 2 D max(sin xi, cos xi) (montage dedans) est exacte."""
    import copy
    from gr2.evaluate import prefilter
    P0 = Params.load(CFG)
    x = np.array([[60.084, 71.924, 10.548, 99.762, 77.32]])        # coude du cas optimum
    raw = copy.deepcopy(P0.raw); raw["chassis"]["mount"] = "dedans"
    ok, viol, g = prefilter(x, Params.from_dict(raw))
    assert ok[0] and "G6b_empreinte_min" not in viol                  # inactive par defaut
    E = max(2 * np.abs(g["foot"][0, :, 0]).max(), 2 * np.abs(g["foot"][0, :, 1]).max()) + raw["leg"]["foot_diam"]
    k = max(np.sin(np.radians(77.32)), np.cos(np.radians(77.32)))
    assert abs(E - (190 + 2 * g["D0"][0] * k)) < 1e-9                 # formule de l'empreinte
    raw["gabarit_min"] = E - 1.0
    assert prefilter(x, Params.from_dict(raw))[0][0]                  # juste au-dessus : passe
    raw["gabarit_min"] = E + 1.0
    ok2, viol2, _ = prefilter(x, Params.from_dict(raw))
    assert not ok2[0] and viol2["G6b_empreinte_min"][0] > 0          # juste en dessous : rejete
