"""Statique du robot : reactions des pieds, marges de stabilite.

Resultats demontres dans le rapport et testes dans tests/ :
 D1  3 pieds : R_i = M g lambda_i (coordonnees barycentriques du CdG), affines en p_G.
 D3  R_i >= 0  <=>  CdG dans le polygone d'appui.
 D6  Sur une boite de CdG, max lambda_i et min distance sont atteints a un coin ;
     on les calcule exactement par la fonction d'appui : f(c) +- |grad| . h.
 D12 4 pieds (hyperstatique) : l'ensemble des repartitions admissibles est un
     segment dont les extremites sont les solutions a 3 pieds ; le max de R_i est
     donc atteint sur une solution a 3 pieds.
"""
import numpy as np


def barycentric(p, a, b, c):
    """Coordonnees barycentriques de p dans le triangle (a, b, c) et leurs
    gradients. Formes (..., 2). Retourne lam (...,3), grad (...,3,2)."""
    x, y = p[..., 0], p[..., 1]
    x1, y1 = a[..., 0], a[..., 1]
    x2, y2 = b[..., 0], b[..., 1]
    x3, y3 = c[..., 0], c[..., 1]
    det = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3)
    det = np.where(np.abs(det) < 1e-9, 1e-9, det)
    g1 = np.stack([(y2 - y3) / det, (x3 - x2) / det], -1)
    g2 = np.stack([(y3 - y1) / det, (x1 - x3) / det], -1)
    l1 = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / det
    l2 = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / det
    l3 = 1.0 - l1 - l2
    lam = np.stack([l1, l2, l3], -1)
    grad = np.stack([g1, g2, -g1 - g2], -2)
    return lam, grad


def box_max(f_c, grad, h):
    """Max sur la boite |dx|<=hx, |dy|<=hy d'une fonction affine (valeur au
    centre f_c, gradient grad[...,2]). Fonction d'appui : exact (D6)."""
    return f_c + np.abs(grad[..., 0]) * h[..., 0] + np.abs(grad[..., 1]) * h[..., 1]


def box_min(f_c, grad, h):
    return f_c - np.abs(grad[..., 0]) * h[..., 0] - np.abs(grad[..., 1]) * h[..., 1]


def edge_inward(pa, pb, centroid):
    """Normale unitaire interieure a l'arete (pa, pb) et distance signee
    d(p) = n . (p - pa) (positive a l'interieur)."""
    e = pb - pa
    n = np.stack([-e[..., 1], e[..., 0]], -1)
    n = n / np.maximum(np.linalg.norm(n, axis=-1, keepdims=True), 1e-12)
    s = np.sign(np.sum(n * (centroid - pa), -1))
    s = np.where(s == 0, 1.0, s)
    return n * s[..., None]


def polygon_margins(feet, Gc, h, zG_max):
    """Marges robustes d'un polygone d'appui CONVEXE (pieds au meme niveau).
    feet : (..., n, 2) dans l'ordre cyclique ; Gc : (...,2) centre de la boite ;
    h : (...,2) demi-largeurs ; zG_max : (...) hauteur max du CdG.
    Retourne (SSM_min, NESM_min) sur la boite (mm). NESM sol plat (D4) :
    sqrt(d^2 + z^2) - z, croissant en d et decroissant en z -> coin le pire."""
    n = feet.shape[-2]
    centroid = feet.mean(-2)
    ssm = np.full(Gc.shape[:-1], np.inf)
    nesm = np.full(Gc.shape[:-1], np.inf)
    for k in range(n):
        pa, pb = feet[..., k, :], feet[..., (k + 1) % n, :]
        nrm = edge_inward(pa, pb, centroid)
        d_c = np.sum(nrm * (Gc - pa), -1)
        d_min = box_min(d_c, nrm, h)
        ne = np.where(d_min >= 0, np.hypot(d_min, zG_max) - zG_max, d_min)
        ssm = np.minimum(ssm, d_min)
        nesm = np.minimum(nesm, ne)
    return ssm, nesm


def nesm_3d(feet3, G3):
    """NESM (Hirose et al.) pour des pieds a des hauteurs differentes.
    feet3 : (..., n, 3) ordre cyclique ; G3 : (..., 3).
    Pour chaque arete P1P2 : le CdG tourne autour de l'axe P1P2 ; Q = projection
    de G sur l'axe ; rho = |G - Q| ; point le plus haut du cercle a la hauteur
    Q_z + rho * sqrt(1 - a_z^2). Hauteur de barriere = (sommet) - G_z.
    Si la projection horizontale de G est hors du polygone : marge negative
    (distance horizontale signee)."""
    n = feet3.shape[-2]
    xy = feet3[..., :2]
    centroid = xy.mean(-2)
    out = np.full(G3.shape[:-1], np.inf)
    for k in range(n):
        P1, P2 = feet3[..., k, :], feet3[..., (k + 1) % n, :]
        nrm = edge_inward(P1[..., :2], P2[..., :2], centroid)
        dh = np.sum(nrm * (G3[..., :2] - P1[..., :2]), -1)
        a = P2 - P1
        a = a / np.linalg.norm(a, axis=-1, keepdims=True)
        t = np.sum((G3 - P1) * a, -1)
        Q = P1 + t[..., None] * a
        rho = np.linalg.norm(G3 - Q, axis=-1)
        top = Q[..., 2] + rho * np.sqrt(np.clip(1 - a[..., 2] ** 2, 0, 1))
        rise = top - G3[..., 2]
        out = np.minimum(out, np.where(dh >= 0, rise, dh))
    return out


def four_feet_max_lambda(feet, Gc, h):
    """Borne exacte (sur la boite) de la part de poids max portee par chaque
    pied quand 4 pieds sont au sol (D12). Pour chaque pied i : max sur les
    triangles contenant i (et pouvant contenir le CdG) de max_boite lambda_i.
    feet (...,4,2). Retourne (...,4) et un booleen 'au moins un triangle valide'."""
    out = np.zeros(feet.shape[:-1])
    any_ok = np.zeros(feet.shape[:-2], bool)
    for omit in range(4):
        idx = [i for i in range(4) if i != omit]
        lam, grad = barycentric(Gc, feet[..., idx[0], :], feet[..., idx[1], :], feet[..., idx[2], :])
        lmax = box_max(lam, grad, h[..., None, :])
        lmin = box_min(lam, grad, h[..., None, :])
        # le triangle peut contenir un point de la boite seulement si aucune
        # coordonnee n'est negative partout (condition necessaire -> borne prudente)
        plausible = np.all(lmax >= 0, -1)
        any_ok |= np.all(lmin >= 0, -1) | plausible
        for j, leg in enumerate(idx):
            val = np.where(plausible, np.clip(lmax[..., j], 0, 1), 0.0)
            out[..., leg] = np.maximum(out[..., leg], val)
    return out, any_ok


def seg_seg_dist(p1, p2, q1, q2):
    """Distance minimale entre deux segments 2D (formes (...,2))."""
    def pt_seg(p, a, b):
        ab = b - a
        t = np.clip(np.sum((p - a) * ab, -1) / np.maximum(np.sum(ab * ab, -1), 1e-12), 0, 1)
        return np.linalg.norm(p - (a + t[..., None] * ab), axis=-1)

    def cross(u, v):
        return u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]
    d = np.minimum(np.minimum(pt_seg(p1, q1, q2), pt_seg(p2, q1, q2)),
                   np.minimum(pt_seg(q1, p1, p2), pt_seg(q2, p1, p2)))
    r, s = p2 - p1, q2 - q1
    den = cross(r, s)
    den_s = np.where(np.abs(den) < 1e-12, 1e-12, den)
    t = cross(q1 - p1, s) / den_s
    u = cross(q1 - p1, r) / den_s
    inter = (np.abs(den) > 1e-12) & (t >= 0) & (t <= 1) & (u >= 0) & (u <= 1)
    return np.where(inter, 0.0, d)
