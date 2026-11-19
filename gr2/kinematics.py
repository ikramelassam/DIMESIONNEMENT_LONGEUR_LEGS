"""Cinematique d'une patte d'architecture 2 (coxa horizontale, femur vers le
haut, tibia vers le bas), dans le plan vertical de la patte.

Conventions (plan de la patte, origine sur l'axe C2) :
  alpha : elevation du femur AU-DESSUS de l'horizontale (alpha > 0 : genou haut)
  beta  : inclinaison du tibia SOUS l'horizontale
  D     : distance horizontale axe C2 -> pied
  Hz    : hauteur de l'axe C2 au-dessus du pied (Hz > 0 : pied plus bas)
  theta3 = alpha + beta : rotation relative au genou (angle interieur = 180 - theta3)
Liens avec l'etude cinematique de l'equipe : theta2 = 90 - alpha, theta3 = alpha + beta.
"""
import numpy as np


def fk(L2, L3, alpha, beta):
    """Cinematique directe (angles en rad). Retourne (D, Hz)."""
    D = L2 * np.cos(alpha) + L3 * np.cos(beta)
    Hz = L3 * np.sin(beta) - L2 * np.sin(alpha)
    return D, Hz


def ik(L2, L3, D, Hz):
    """Cinematique inverse, branche 'genou haut' (architecture 2/3).
    Retourne (alpha, beta, d, reach_ok) ; angles en rad.
    reach_ok = triangle C2-genou-pied constructible (inegalite triangulaire)."""
    d = np.hypot(D, Hz)
    reach_ok = (d <= L2 + L3) & (d >= np.abs(L2 - L3)) & (d > 1e-9)
    cpsi = (L2 ** 2 + d ** 2 - L3 ** 2) / (2 * L2 * np.maximum(d, 1e-9))
    psi = np.arccos(np.clip(cpsi, -1.0, 1.0))
    alpha = psi - np.arctan2(Hz, D)
    beta = np.arctan2(Hz + L2 * np.sin(alpha), D - L2 * np.cos(alpha))
    return alpha, beta, d, reach_ok


def jacobian_det(L2, L3, alpha, beta):
    """Determinant du jacobien (D, Hz) / (alpha, theta3 relatif).
    Avec q2 = alpha et q3 = theta3 = alpha + beta : det = -L2 L3 sin(theta3)."""
    return -L2 * L3 * np.sin(alpha + beta)
