#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Campagne G : surface cartographiee DANS LA ZONE, par graine.

Metrique pre-enregistree : surface couverte a l'interieur du carre
de 300 m de cote centre sur le point d'apparition, a budget de
1200 m. Calculee sur la trajectoire VRAIE, comme analyse_couverture.py.
"""
import csv, glob, math, os, re

BASE = os.environ.get("ACTIVE_SLAM_METRICS",
                      os.path.expanduser("~/active_slam_carla/metrics"))
CELL, PORTEE, ZONE = 10.0, 40.0, 150.0
BRAS = ["weighted", "random", "predefinie"]


def mesure(nom):
    f = f"{BASE}/ate_{nom}.csv"
    if not os.path.exists(f):
        return None
    pts = []
    for l in csv.DictReader(open(f, newline='')):
        try:
            x, y = float(l['gt_x']), float(l['gt_y'])
        except (KeyError, TypeError, ValueError):
            continue
        if x == 0.0 and y == 0.0:
            continue
        pts.append((x, y))
    if len(pts) < 2:
        return None
    n = int(PORTEE / CELL) + 1
    total, zone = set(), set()
    for x, y in pts:
        cx, cy = int(round(x / CELL)), int(round(y / CELL))
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                ux, uy = (cx + i) * CELL, (cy + j) * CELL
                if math.hypot(ux - x, uy - y) > PORTEE:
                    continue
                total.add((cx + i, cy + j))
                if abs(ux) <= ZONE and abs(uy) <= ZONE:
                    zone.add((cx + i, cy + j))
    d = sum(math.dist(pts[i - 1], pts[i]) for i in range(1, len(pts)))
    return len(zone) * CELL * CELL, len(total) * CELL * CELL, d


def signes(paires):
    """paires : liste de (a, b). Compte les graines ou a > b."""
    pos = sum(1 for a, b in paires if a > b)
    neg = sum(1 for a, b in paires if a < b)
    n = pos + neg
    if n == 0:
        return 0, 0, 1.0
    k = max(pos, neg)
    p = sum(math.comb(n, i) for i in range(k, n + 1)) / 2.0 ** n
    return pos, n, p


R = {b: {} for b in BRAS}
for b in BRAS:
    for g in range(1, 6):
        m = mesure(f"{b}G_{g}")
        if m:
            R[b][g] = m

print("SURFACE CARTOGRAPHIEE DANS LA ZONE (m2)")
print()
print(f"{'graine':>7}" + "".join(f"{b:>13}" for b in BRAS))
print("-" * (7 + 13 * len(BRAS)))
for g in range(1, 6):
    ligne = f"{g:>7}"
    for b in BRAS:
        ligne += f"{R[b][g][0]:>13,.0f}".replace(",", " ") if g in R[b] else f"{'-':>13}"
    print(ligne)
print("-" * (7 + 13 * len(BRAS)))
ligne = f"{'moyenne':>7}"
for b in BRAS:
    v = [R[b][g][0] for g in R[b]]
    ligne += f"{sum(v)/len(v):>13,.0f}".replace(",", " ") if v else f"{'-':>13}"
print(ligne + "\n")

TESTS = [("PRINCIPALE ", "weighted", "predefinie"),
         ("controle   ", "random", "predefinie"),
         ("secondaire ", "weighted", "random")]

print("COMPARAISONS PRE-ENREGISTREES (graines completes seulement)")
print()
for etiq, a, b in TESTS:
    com = sorted(set(R[a]) & set(R[b]))
    if not com:
        print(f"  {etiq} {a} vs {b} : aucune graine complete")
        continue
    paires = [(R[a][g][0], R[b][g][0]) for g in com]
    k, n, p = signes(paires)
    ma = sum(x for x, _ in paires) / len(paires)
    mb = sum(y for _, y in paires) / len(paires)
    verdict = "CONCLUANT" if p < 0.05 else ("tendance" if k == n else "")
    print(f"  {etiq} {a:>10} {ma:>9,.0f}  vs {b:>10} {mb:>9,.0f}"
          .replace(",", " ")
          + f"   ->  {k}/{n}  p={p:.3f}  {verdict}")

fait = sum(len(R[b]) for b in BRAS)
print(f"\n  {fait} / 15 runs archives")
if fait < 15:
    print("  Resultats PARTIELS : le verdict se lit a 15 runs.")
