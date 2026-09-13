#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Campagne G : distance et duree necessaires pour atteindre un seuil de
surface cartographiee DANS LA ZONE.

Meme modele geometrique que resultats_G.py (CELL, PORTEE, ZONE, trajectoire
vraie), mais applique de maniere cumulative le long du parcours : on releve
la distance parcourue et le temps ecoule au moment ou la surface couverte
atteint pour la premiere fois chaque seuil.

C'est la mesure d'efficacite de deplacement : a surface egale, combien de
metres et de secondes ont ete necessaires.

Donnees manquantes : un run qui n'atteint jamais le seuil dans son budget
de 1200 m est CENSURE, pas ignore. Dans le test des signes il perd face a
un run qui atteint le seuil, ce qui est le traitement conservateur correct.
Le jeter reviendrait a supprimer les echecs de la strategie de reference.

Seules les graines 1 a 5 entrent dans les tests (plan pre-enregistre).

Sortie : tableaux a l'ecran + ~/logs_campagne2/resultats_seuils_G.csv
"""
import csv
import math
import os

BASE = os.environ.get("ACTIVE_SLAM_METRICS",
                      os.path.expanduser("~/active_slam_carla/metrics"))
SORTIE = os.path.expanduser("~/logs_campagne2/resultats_seuils_G.csv")
CELL, PORTEE, ZONE = 10.0, 40.0, 150.0
BRAS = ["weighted", "random", "predefinie"]
GRAINES = [1, 2, 3, 4, 5]
SEUILS = [10000.0, 12000.0, 14000.0, 15140.0]
INF = float("inf")


def signes(paires, plus_petit_est_mieux=True):
    """paires : liste de (a, b), INF autorise pour un run censure."""
    if plus_petit_est_mieux:
        gagne = sum(1 for a, b in paires if a < b)
        perd = sum(1 for a, b in paires if a > b)
    else:
        gagne = sum(1 for a, b in paires if a > b)
        perd = sum(1 for a, b in paires if a < b)
    n = gagne + perd
    if n == 0:
        return 0, 0, 1.0
    k = max(gagne, perd)
    p = sum(math.comb(n, i) for i in range(k, n + 1)) / 2.0 ** n
    return gagne, n, p


def verdict(a, b, gagne, n, p):
    """Nomme le bras gagnant. Un compteur seul se lit a l'envers."""
    if n == 0:
        return "aucune graine departageable"
    if gagne * 2 > n:
        vain, k = a, gagne
    elif gagne * 2 < n:
        vain, k = b, n - gagne
    else:
        return "egalite %d/%d  p=%.3f" % (gagne, n, p)
    note = "CONCLUANT" if p < 0.05 else ("tendance" if k == n else "non concluant")
    return "%s meilleur %d/%d  p=%.3f  %s" % (vain, k, n, p, note)


def profil(nom):
    """Courbe cumulative (distance, temps, surface_en_zone) d'un run."""
    f = os.path.join(BASE, "ate_%s.csv" % nom)
    if not os.path.exists(f):
        return None
    pts = []
    with open(f, newline='') as fh:
        for l in csv.DictReader(fh):
            try:
                t = float(l['timestamp'])
                x, y = float(l['gt_x']), float(l['gt_y'])
            except (KeyError, TypeError, ValueError):
                continue
            if x == 0.0 and y == 0.0:
                continue
            pts.append((t, x, y))
    if len(pts) < 2:
        return None

    n = int(PORTEE / CELL) + 1
    zone = set()
    courbe = []
    d = 0.0
    t0 = pts[0][0]
    px, py = pts[0][1], pts[0][2]
    for t, x, y in pts:
        d += math.hypot(x - px, y - py)
        px, py = x, y
        cx, cy = int(round(x / CELL)), int(round(y / CELL))
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                ux, uy = (cx + i) * CELL, (cy + j) * CELL
                if math.hypot(ux - x, uy - y) > PORTEE:
                    continue
                if abs(ux) <= ZONE and abs(uy) <= ZONE:
                    zone.add((cx + i, cy + j))
        courbe.append((d, t - t0, len(zone) * CELL * CELL))
    return courbe


def atteinte(courbe, seuil):
    """(distance, duree) au premier franchissement, ou (INF, INF)."""
    for d, t, s in courbe:
        if s >= seuil:
            return d, t
    return INF, INF


# --------------------------------------------------------------- lecture

print()
print("Lecture des runs ...")
C = {b: {} for b in BRAS}
for b in BRAS:
    for g in GRAINES:
        c = profil("%sG_%d" % (b, g))
        if c:
            C[b][g] = c
            print("  %-11s graine %d : %5d echantillons, %7.1f m, "
                  "%6.0f m2 en zone" % (b, g, len(c), c[-1][0], c[-1][2]))
print()

# --------------------------------------------------------- par seuil

lignes_csv = []

for seuil in SEUILS:
    A = {b: {g: atteinte(C[b][g], seuil) for g in sorted(C[b])} for b in BRAS}

    print("=" * 64)
    print("SEUIL %s m2" % ("%d" % seuil).rjust(6))
    print()
    print("%7s%13s%13s%13s" % ("graine", "weighted", "random", "predefinie"))
    print("-" * 46)
    for g in GRAINES:
        ligne = "%7d" % g
        for b in BRAS:
            if g not in A[b]:
                ligne += "%13s" % "-"
            elif A[b][g][0] == INF:
                ligne += "%13s" % "non atteint"
            else:
                ligne += "%13.0f" % A[b][g][0]
        print(ligne)
    print("-" * 46)

    for etiq, idx, unite in (("dist.", 0, "m"), ("duree", 1, "s")):
        ligne = "%7s" % etiq
        for b in BRAS:
            v = [A[b][g][idx] for g in sorted(A[b]) if A[b][g][idx] != INF]
            ligne += ("%13.0f" % (sum(v) / len(v))) if v else "%13s" % "-"
        print(ligne + ("   moyenne sur les runs qui atteignent  [%s]" % unite))

    ligne = "%7s" % "atteint"
    for b in BRAS:
        k = sum(1 for g in sorted(A[b]) if A[b][g][0] != INF)
        ligne += "%13s" % ("%d/%d" % (k, len(A[b])))
    print(ligne)
    print()

    for etiq, a, b in (("PRINCIPALE ", "weighted", "predefinie"),
                       ("controle   ", "random", "predefinie"),
                       ("secondaire ", "weighted", "random")):
        com = sorted(set(A[a]) & set(A[b]))
        if not com:
            continue
        for quoi, idx in (("distance", 0), ("duree   ", 1)):
            paires = [(A[a][g][idx], A[b][g][idx]) for g in com]
            k, n, p = signes(paires)
            cens = sum(1 for x, y in paires if INF in (x, y))
            note = "  (%d paire(s) censuree(s))" % cens if cens else ""
            print("  %s %s : %-10s vs %-10s  ->  %s%s"
                  % (etiq, quoi, a, b, verdict(a, b, k, n, p), note))
        print()

    for b in BRAS:
        for g in sorted(A[b]):
            d, t = A[b][g]
            lignes_csv.append([b, g, "%d" % seuil,
                               "" if d == INF else "%.1f" % d,
                               "" if t == INF else "%.1f" % t,
                               "non" if d == INF else "oui"])

# ------------------------------------------------------------- lecture

print("=" * 64)
print("LECTURE")
print()
print("  L'avantage depend du seuil. En dessous d'environ 12 000 m2 les")
print("  deux strategies se valent : au debut du run, avancer tout droit")
print("  decouvre autant de terrain neuf que decider ou aller. L'ecart")
print("  apparait quand la zone commence a etre couverte et qu'il faut")
print("  choisir ou aller chercher ce qui manque.")
print()
print("  Ce comportement doit etre ecrit tel quel dans le rapport : une")
print("  affirmation non conditionnee au seuil serait fausse.")

os.makedirs(os.path.dirname(SORTIE), exist_ok=True)
with open(SORTIE, "w", newline='') as fh:
    w = csv.writer(fh)
    w.writerow(["bras", "graine", "seuil_m2", "distance_m", "duree_s", "atteint"])
    w.writerows(lignes_csv)

print()
print("  ecrit : %s" % SORTIE)
