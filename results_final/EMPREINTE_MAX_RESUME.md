# Empreinte maximale tenable avec les servos du FabLab (17/11/2026)

Montage dedans, scenario S2, catalogue SG90 / S-U400 / S3306 (C2 = S3306 obligatoire),
balancement 60 mm, frac_couple = 1,0, SF = 1,2, chassis 200 x 200 mm.
Script : `python empreinte_max.py <E...> [--xi45]`. Bande d'empreinte acceptee : [E - 4, E].

## xi libre
| E (mm) | faisable | NESM >= 2 | robuste (>= 95 % MC) | remarque |
|---|---|---|---|---|
| 300 | oui | oui | oui (10/40) | |
| 310 | oui | oui | oui (15/40) | |
| 320 | oui | oui | oui (8/40)  | design : L2 81,8  L3 80,5  a 5,1  b 101,3  xi 79,7  S-U400/S3306/S-U400, 1,161 kg, NESM 4,27, C2 1,664, MC 100 % |
| 330 | oui | oui | oui (2/40)  | design : L2 89,6  L3 82,3  a 12,3  b 100,9  xi 76,0  S-U400/S3306/S-U400, 1,167 kg, NESM 2,91, C2 1,708, MC 97 % |
| 340 | oui | oui | non (0/40, 73 % au mieux) | |
| 345 | oui | oui | non (*) | |
| 350 | oui | oui | non (0/28, 40 % au mieux) | |
| 355 | oui | oui (NESM 2,36) | non (*) | C2 a 98 % de la limite |
| 360 | oui | non (NESM max 0,91) | non | |
| 375 | oui (NESM 0,6) | non | non | C2 a 99,7 % de la limite |
| 380 | non | non | non | |
(*) Monte-Carlo sur 5 a 12 candidats seulement (premiere version du script).

=> faisable jusqu'a ~375 mm, NESM >= 2 jusqu'a ~355 mm, ROBUSTE jusqu'a ~330 mm.

## xi = 45 deg
| E (mm) | faisable | NESM >= 2 | robuste |
|---|---|---|---|
| 265 | oui | oui (5 designs, tout S3306) | non |
| 280 | oui | oui (NESM 2,9) | non |
| 290 | oui | non (NESM max 1,7) | non |
| 300 | oui | non | non |
| 310 | non | non | non |
=> faisable ~300 mm, NESM >= 2 ~280 mm, aucun design a la fois NESM >= 2 et robuste.

Fichiers : `empreinte_max_*.json` (detail par combinaison de servos et meilleurs designs), `emax_*.log`.
