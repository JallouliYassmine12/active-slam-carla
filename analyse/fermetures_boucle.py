#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Campagne G : fermetures de boucle par run, lues dans les bases RTAB-Map.

Chaque run ecrit sa propre base dans ~/active_slam_carla/maps/ :
    active_slam_<date>_<heure>.db   pour weighted et random
    slam_eval_<date>_<heure>.db     pour predefinie

Le nom de la base ne contient pas le bras ni la graine. L'appariement se
fait donc par PREFIXE (qui donne le bras) et par HORODATAGE : la base d'un
run est fermee quelques secondes avant l'ecriture de son CSV de metriques.
L'ecart mesure sur la campagne est de 5 a 15 s ; la tolerance par defaut
est de 180 s. Le tableau d'appariement est imprime pour verification.

Table Link de RTAB-Map, colonne type :
    0 = lien de voisinage (odometrie, entre poses consecutives)
    1 = fermeture de boucle globale (le lieu est reconnu, le graphe est
        corrige)
    2 = lien de proximite (deux poses proches dans l'espace)

Ce sont les liens de type 1 et 2 qui corrigent la derive : ils expliquent
l'ecart d'ATE mesure par analyse_ate_appariee.py.

Seules les graines 1 a 5 entrent dans les tests (plan pre-enregistre).

Sortie : tableaux a l'ecran + ~/logs_campagne2/resultats_fermetures_G.csv
"""
import csv
import glob
import math
import os
import sqlite3
import time

METRICS = os.environ.get("ACTIVE_SLAM_METRICS",
                         os.path.expanduser("~/active_slam_carla/metrics"))
MAPS = os.environ.get("ACTIVE_SLAM_MAPS",
                      os.path.expanduser("~/active_slam_carla/maps"))
SORTIE = os.path.expanduser("~/logs_campagne2/resultats_fermetures_G.csv")

BRAS = ["weighted", "random", "predefinie"]
PREFIXE = {"weighted": "active_slam", "random": "active_slam",
           "predefinie": "slam_eval"}
GRAINES = [1, 2, 3, 4, 5]
TOLERANCE = 180.0          # secondes entre la fin de la base et le CSV


def signes(paires, plus_grand_est_mieux=True):
    if plus_grand_est_mieux:
        gagne = sum(1 for a, b in paires if a > b)
        perd = sum(1 for a, b in paires if a < b)
    else:
        gagne = sum(1 for a, b in paires if a < b)
        perd = sum(1 for a, b in paires if a > b)
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


def horodatage(t):
    return time.strftime("%H:%M:%S", time.localtime(t))


def liens(chemin):
    """Compte les liens par type. Retourne None si la base est illisible."""
    try:
        cx = sqlite3.connect("file:%s?mode=ro" % chemin, uri=True)
    except sqlite3.Error:
        return None
    try:
        c = dict(cx.execute("SELECT type, COUNT(*) FROM Link GROUP BY type"))
        try:
            noeuds = cx.execute("SELECT COUNT(*) FROM Node").fetchone()[0]
        except sqlite3.Error:
            noeuds = 0
    except sqlite3.Error:
        cx.close()
        return None
    cx.close()
    return {
        "voisinage": c.get(0, 0),
        "globales": c.get(1, 0),
        "proximite": c.get(2, 0),
        "autres": sum(v for k, v in c.items() if k not in (0, 1, 2)),
        "noeuds": noeuds,
    }


# ------------------------------------------------------------ appariement

bases = {}
for p in set(PREFIXE.values()):
    bases[p] = sorted(
        ((os.path.getmtime(f), f) for f in glob.glob("%s/%s_*.db" % (MAPS, p))),
        key=lambda t: t[0])

pris = set()
APP = {b: {} for b in BRAS}
orphelins = []

for b in BRAS:
    for g in GRAINES:
        csv_run = os.path.join(METRICS, "ate_%sG_%d.csv" % (b, g))
        if not os.path.exists(csv_run):
            continue
        tc = os.path.getmtime(csv_run)
        cands = [(abs(tm - tc), tm, f) for tm, f in bases[PREFIXE[b]]
                 if abs(tm - tc) <= TOLERANCE and f not in pris]
        if not cands:
            orphelins.append((b, g, tc))
            continue
        cands.sort()
        ecart, tm, f = cands[0]
        pris.add(f)
        APP[b][g] = {"base": f, "ecart": ecart, "t_base": tm, "t_csv": tc}

print()
print("APPARIEMENT RUN -> BASE RTAB-MAP")
print()
print("%-11s %-7s %-10s %-10s %7s  %s"
      % ("bras", "graine", "fin base", "fin csv", "ecart", "base"))
print("-" * 96)
for b in BRAS:
    for g in sorted(APP[b]):
        a = APP[b][g]
        print("%-11s %-7d %-10s %-10s %6.0fs  %s"
              % (b, g, horodatage(a["t_base"]), horodatage(a["t_csv"]),
                 a["ecart"], os.path.basename(a["base"])))
print()
if orphelins:
    print("SANS BASE APPARIEE (tolerance %.0f s) :" % TOLERANCE)
    for b, g, tc in orphelins:
        print("  %s graine %d  (csv a %s)" % (b, g, horodatage(tc)))
    print("  -> augmenter TOLERANCE, ou la base a ete supprimee.")
    print()

# ---------------------------------------------------------------- lecture

R = {b: {} for b in BRAS}
for b in BRAS:
    for g in sorted(APP[b]):
        d = liens(APP[b][g]["base"])
        if d is None:
            print("  base illisible : %s" % APP[b][g]["base"])
            continue
        d["base"] = APP[b][g]["base"]
        R[b][g] = d

print("LIENS DU GRAPHE DE POSES, PAR RUN")
print()
print("%-11s %-7s %10s %12s %11s %12s %9s"
      % ("bras", "graine", "voisinage", "fermetures", "proximite",
         "corrections", "noeuds"))
print("%-11s %-7s %10s %12s %11s %12s %9s"
      % ("", "", "(type 0)", "globales (1)", "(type 2)", "1 + 2", ""))
print("-" * 80)
for b in BRAS:
    for g in sorted(R[b]):
        d = R[b][g]
        print("%-11s %-7d %10d %12d %11d %12d %9d"
              % (b, g, d["voisinage"], d["globales"], d["proximite"],
                 d["globales"] + d["proximite"], d["noeuds"]))
print("-" * 80)

MOY = {}
for b in BRAS:
    if not R[b]:
        continue
    n = len(R[b])
    MOY[b] = {k: sum(R[b][g][k] for g in R[b]) / n
              for k in ("voisinage", "globales", "proximite", "noeuds")}
    MOY[b]["corrections"] = MOY[b]["globales"] + MOY[b]["proximite"]
    print("%-11s %-7s %10.1f %12.1f %11.1f %12.1f %9.0f"
          % (b, "moyenne", MOY[b]["voisinage"], MOY[b]["globales"],
             MOY[b]["proximite"], MOY[b]["corrections"], MOY[b]["noeuds"]))
print()

# ------------------------------------------------------------------ ratio

if "predefinie" in MOY:
    ref = MOY["predefinie"]
    print("RAPPORT A LA STRATEGIE DE REFERENCE")
    print()
    for b in ("weighted", "random"):
        if b not in MOY:
            continue
        for cle, nom in (("globales", "fermetures globales"),
                         ("proximite", "liens de proximite"),
                         ("corrections", "corrections totales")):
            r = MOY[b][cle] / ref[cle] if ref[cle] else float("inf")
            print("  %-10s %-22s %7.1f  contre %6.1f   ->  x%.1f"
                  % (b, nom, MOY[b][cle], ref[cle], r))
        print()

    # Les graphes n'ont pas la meme taille : la strategie de reference roule
    # plus vite et produit donc moins de noeuds. Comparer des comptages bruts
    # prete le flanc a l'objection "ils avaient plus de noeuds". On donne donc
    # aussi le taux normalise, qui repond a l'objection sans rien conceder.
    print("NORMALISE PAR LA TAILLE DU GRAPHE  (fermetures globales / 100 noeuds)")
    print()
    for b in BRAS:
        if b not in MOY or not MOY[b]["noeuds"]:
            continue
        print("  %-11s %6.1f fermetures / 100 noeuds   (%.0f noeuds en moyenne)"
              % (b, 100.0 * MOY[b]["globales"] / MOY[b]["noeuds"],
                 MOY[b]["noeuds"]))
    if ref["noeuds"] and ref["globales"]:
        base = 100.0 * ref["globales"] / ref["noeuds"]
        for b in ("weighted", "random"):
            if b in MOY and MOY[b]["noeuds"]:
                t = 100.0 * MOY[b]["globales"] / MOY[b]["noeuds"]
                print("  %-11s -> x%.1f apres normalisation" % (b, t / base))
    print()

# ------------------------------------------------------------------ tests

print("COMPARAISONS  (fermetures : plus grand est meilleur)")
print()
for etiq, a, b in (("PRINCIPALE ", "weighted", "predefinie"),
                   ("controle   ", "random", "predefinie"),
                   ("secondaire ", "weighted", "random")):
    com = sorted(set(R[a]) & set(R[b]))
    if not com:
        continue
    paires = [(R[a][g]["globales"], R[b][g]["globales"]) for g in com]
    k, n, p = signes(paires)
    print("  %s %-10s vs %-10s  ->  %s" % (etiq, a, b, verdict(a, b, k, n, p)))
print()
print("  Ce test n'etait PAS pre-enregistre : il a ete decide apres avoir vu")
print("  les resultats d'ATE, pour en expliquer la cause. Il doit etre")
print("  presente comme une analyse explicative, pas comme une confirmation.")

# ------------------------------------------------------------------ sortie

os.makedirs(os.path.dirname(SORTIE), exist_ok=True)
with open(SORTIE, "w", newline='') as fh:
    w = csv.writer(fh)
    w.writerow(["bras", "graine", "base", "liens_voisinage",
                "fermetures_globales", "liens_proximite", "corrections",
                "noeuds"])
    for b in BRAS:
        for g in sorted(R[b]):
            d = R[b][g]
            w.writerow([b, g, os.path.basename(d["base"]), d["voisinage"],
                        d["globales"], d["proximite"],
                        d["globales"] + d["proximite"], d["noeuds"]])

print()
print("  ecrit : %s" % SORTIE)
