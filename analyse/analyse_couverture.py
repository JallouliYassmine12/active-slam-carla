#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Couverture de la campagne appariee : ce que chaque strategie a
reellement CARTOGRAPHIE, a budget de distance identique.

POURQUOI CETTE MESURE
---------------------
Le sujet s'intitule "cartographie autonome". Le livrable d'un systeme
d'Active SLAM est une CARTE ; l'erreur de localisation n'en mesure que
la justesse, pas l'etendue.

Or 'predefinie' suit une trajectoire ecrite d'avance : elle ne decide
rien et ne peut pas explorer un environnement inconnu. La comparer aux
quatre strategies de decision sur la seule ATE revient a juger un
explorateur sur sa precision de navigation sans regarder ce qu'il a
explore.

A budget EGAL -- 1200 m parcourus pour les cinq -- la question est :
combien de terrain chacune a-t-elle couvert ?

CE QUI EST MESURE
-----------------
Trois grandeurs, toutes issues de la trajectoire VRAIE (gt_x, gt_y),
donc independantes de la qualite du SLAM :

  parcourue   longueur de la trajectoire, en metres. Controle que le
              budget a bien ete respecte par les cinq.

  visitees    cellules de 10 x 10 m contenant au moins un point de la
              trajectoire. Mesure l'etalement du parcours : une
              methode qui tourne en rond en visite peu.

  cartographiee  cellules de 10 x 10 m dont le CENTRE passe a moins de
              40 m d'un point de la trajectoire. 40 m est la portee de
              la grille d'occupation (Grid/RangeMax dans les deux
              fichiers de lancement), donc c'est la surface que les
              capteurs ont effectivement pu observer.

  rendement   surface cartographiee divisee par la distance parcourue,
              en m2 par metre roule. C'est la mesure d'EFFICACITE
              d'exploration : combien de carte par metre depense.

CE QUE CETTE MESURE N'EST PAS
-----------------------------
Ce n'est pas la grille d'occupation de RTAB-Map. Celle-ci vit dans les
bases .db et depend de la qualite du SLAM -- une methode qui derive
produit une carte etendue mais fausse, et paraitrait avantagee. La
mesure ci-dessus part de la verite terrain : elle dit quelle surface
le vehicule a physiquement pu observer, sans recompenser la derive.

Les deux se completent et se disent dans le rapport : l'ATE pour la
justesse, la couverture pour l'etendue.

LE TEST
-------
Le meme que pour l'ATE : test des signes, exact, unilateral, sur les
cinq graines appariees. Pour la couverture, PLUS GRAND vaut mieux --
c'est l'inverse de l'ATE, et le script en tient compte.

USAGE
-----
    python3 ~/analyse_couverture.py
    python3 ~/analyse_couverture.py --csv
"""

import csv
import math
import os
import sys

METRICS = os.environ.get("ACTIVE_SLAM_METRICS",
                         os.path.expanduser("~/active_slam_carla/metrics"))

STRATEGIES = ["random", "distance", "info_gain", "weighted", "predefinie"]
GRAINES = [1, 2, 3, 4, 5]

CELLULE = 10.0
# Portee de la grille d'occupation, identique dans active_slam.launch.py
# et slam_evaluation.launch.py (Grid/RangeMax = 40.0).
PORTEE = 40.0

PRINCIPALE = ("weighted", "predefinie")
SECONDAIRES = [("weighted", "random"),
               ("weighted", "distance"),
               ("weighted", "info_gain")]

# Decalages de cellules dont le centre tombe dans la portee du capteur.
# Calcules une seule fois : le meme motif s'applique a chaque point.
_RAYON_CELLULES = int(math.ceil(PORTEE / CELLULE))
MOTIF = [(di, dj)
         for di in range(-_RAYON_CELLULES, _RAYON_CELLULES + 1)
         for dj in range(-_RAYON_CELLULES, _RAYON_CELLULES + 1)
         if math.hypot(di * CELLULE, dj * CELLULE) <= PORTEE]


def lire_trajectoire(nom):
    """Points (t, x, y) de la trajectoire vraie, tries chronologiquement."""
    chemin = os.path.join(METRICS, "ate_%s.csv" % nom)
    if not os.path.exists(chemin):
        return None

    points = []
    with open(chemin, newline="", encoding="utf-8", errors="replace") as f:
        for l in csv.DictReader(f):
            try:
                t = float(l["timestamp"])
                x = float(l["gt_x"])
                y = float(l["gt_y"])
            except (KeyError, TypeError, ValueError):
                continue
            # Echantillon d'amorcage : tout a zero, ce n'est pas une position.
            if x == 0.0 and y == 0.0:
                continue
            points.append((t, x, y))

    if len(points) < 2:
        return None
    # Les lignes ne sont pas ecrites dans l'ordre chronologique.
    points.sort()
    return points


def couverture(nom):
    points = lire_trajectoire(nom)
    if points is None:
        return None

    longueur = 0.0
    for i in range(1, len(points)):
        longueur += math.hypot(points[i][1] - points[i - 1][1],
                               points[i][2] - points[i - 1][2])

    visitees = set()
    cartographiees = set()
    for _, x, y in points:
        ci = int(math.floor(x / CELLULE))
        cj = int(math.floor(y / CELLULE))
        visitees.add((ci, cj))
        for di, dj in MOTIF:
            cartographiees.add((ci + di, cj + dj))

    surface = len(cartographiees) * CELLULE * CELLULE
    return {
        "parcourue": longueur,
        "visitees": len(visitees),
        "cartographiees": len(cartographiees),
        "surface": surface,
        "rendement": surface / longueur if longueur > 0 else 0.0,
        "n": len(points),
    }


def collecter(suffixe="P"):
    donnees = {}
    for s in STRATEGIES:
        donnees[s] = {}
        for g in GRAINES:
            donnees[s][g] = couverture("%s%s_%d" % (s, suffixe, g))
    return donnees


def moyenne(v):
    return sum(v) / len(v) if v else None


def test_des_signes(ecarts, plus_grand_vaut_mieux):
    """Retourne (a_mieux, b_mieux, n, k, p).

    'ecarts' vaut b - a. Si plus grand vaut mieux (couverture), un
    ecart positif signifie que b fait mieux. Si plus petit vaut mieux
    (erreur), c'est l'inverse.
    """
    positifs = sum(1 for e in ecarts if e > 0)
    negatifs = sum(1 for e in ecarts if e < 0)
    if plus_grand_vaut_mieux:
        a_mieux, b_mieux = negatifs, positifs
    else:
        a_mieux, b_mieux = positifs, negatifs
    n = a_mieux + b_mieux
    if n == 0:
        return 0, 0, 0, 0, 1.0
    k = max(a_mieux, b_mieux)
    p = sum(math.comb(n, i) for i in range(k, n + 1)) / (2.0 ** n)
    return a_mieux, b_mieux, n, k, p


def afficher_tableau(donnees, cle, titre, unite, decimales=0):
    print("  %s (%s)" % (titre, unite))
    print("")
    print("  %-12s %8s %8s %8s %8s %8s   %9s"
          % ("strategie", "g1", "g2", "g3", "g4", "g5", "moyenne"))
    print("  " + "-" * 68)
    fmt = "%%.%df" % decimales
    for s in STRATEGIES:
        cases, valeurs = [], []
        for g in GRAINES:
            d = donnees[s][g]
            if d is None:
                cases.append("-")
            else:
                cases.append(fmt % d[cle])
                valeurs.append(d[cle])
        print("  %-12s %8s %8s %8s %8s %8s   %9s"
              % (s, cases[0], cases[1], cases[2], cases[3], cases[4],
                 "-" if not valeurs else fmt % moyenne(valeurs)))
    print("")


def comparer(donnees, a, b, cle, titre, unite, principale):
    ecarts = []
    lignes = []
    for g in GRAINES:
        da, db = donnees[a][g], donnees[b][g]
        if da is None or db is None:
            lignes.append((g, None, None))
            continue
        lignes.append((g, da[cle], db[cle]))
        ecarts.append(db[cle] - da[cle])

    if principale:
        print("  %s : %s contre %s" % (titre, a, b))
        print("")
        print("  %6s %12s %12s %12s"
              % ("graine", a[:12], b[:12], "ecart"))
        print("  " + "-" * 46)
        for g, va, vb in lignes:
            if va is None or vb is None:
                print("  %6d %12s %12s %12s"
                      % (g,
                         "-" if va is None else "%.0f" % va,
                         "-" if vb is None else "%.0f" % vb,
                         "incomplet"))
            else:
                print("  %6d %12.0f %12.0f %+12.0f" % (g, va, vb, vb - va))
        print("")

    if not ecarts:
        print("    aucune paire complete")
        return

    a_mieux, b_mieux, n, k, p = test_des_signes(ecarts, True)
    m = moyenne(ecarts)
    gagnant = a if a_mieux > b_mieux else b

    if principale:
        print("  Ecart moyen %s - %s : %+.0f %s" % (b, a, m, unite))
        print("  %s couvre plus sur %d graine(s) sur %d ; %s sur %d"
              % (a, a_mieux, len(ecarts), b, b_mieux))
        print("  Test des signes : %d/%d, p = %.3f" % (k, n, p))
        print("")
        if n == 0:
            print("  LECTURE : couverture identique.")
        elif p <= 0.05:
            print("  LECTURE : %s couvre systematiquement plus de terrain"
                  % gagnant)
            print("  que l'autre, a budget de distance egal, sur les cinq")
            print("  trafics testes.")
        elif k == n:
            print("  LECTURE : les %d ecarts vont dans le meme sens, en" % n)
            print("  faveur de %s, mais %d paires ne suffisent pas a"
                  % (gagnant, n))
            print("  exclure le hasard.")
        else:
            print("  LECTURE : pas d'avantage systematique de couverture.")
    else:
        print("    %-24s ecart moyen %+8.0f %s  |  %s meilleure %d/%d"
              "  |  p = %.3f"
              % ("%s contre %s" % (a, b), m, unite, a, a_mieux,
                 len(ecarts), p))


def exporter(donnees):
    sortie = csv.writer(sys.stdout)
    sortie.writerow(["strategie", "graine", "distance_parcourue_m",
                     "cellules_visitees", "cellules_cartographiees",
                     "surface_m2", "rendement_m2_par_m", "echantillons"])
    for s in STRATEGIES:
        for g in GRAINES:
            d = donnees[s][g]
            if d is None:
                sortie.writerow([s, g, "", "", "", "", "", ""])
                continue
            sortie.writerow([s, g, "%.1f" % d["parcourue"], d["visitees"],
                             d["cartographiees"], "%.0f" % d["surface"],
                             "%.1f" % d["rendement"], d["n"]])


def main():
    if not os.path.isdir(METRICS):
        print("Dossier introuvable : %s" % METRICS)
        return 1

    donnees = collecter()

    if "--csv" in sys.argv[1:]:
        exporter(donnees)
        return 0

    print("=== COUVERTURE : CE QUI A ETE CARTOGRAPHIE ===")
    print("")
    print("  Budget identique pour les cinq : 1200 m parcourus.")
    print("  Cellules de %.0f x %.0f m ; portee capteur %.0f m"
          % (CELLULE, CELLULE, PORTEE))
    print("  (Grid/RangeMax des deux fichiers de lancement).")
    print("  Tout est calcule sur la trajectoire VRAIE : la derive du")
    print("  SLAM ne peut pas gonfler artificiellement la couverture.")
    print("")

    afficher_tableau(donnees, "parcourue",
                     "Distance parcourue -- controle du budget", "m", 0)
    afficher_tableau(donnees, "visitees",
                     "Cellules visitees -- etalement du parcours",
                     "cellules", 0)
    afficher_tableau(donnees, "surface",
                     "Surface cartographiee", "m2", 0)
    afficher_tableau(donnees, "rendement",
                     "Rendement d'exploration", "m2 par metre roule", 1)

    print("=== COMPARAISON PRINCIPALE (couverture) ===")
    print("")
    comparer(donnees, PRINCIPALE[0], PRINCIPALE[1], "surface",
             "Surface cartographiee", "m2", True)
    print("")

    print("=== COMPARAISONS SECONDAIRES (descriptives) ===")
    print("")
    for a, b in SECONDAIRES:
        comparer(donnees, a, b, "surface", "", "m2", False)
    print("")
    print("  Comme pour l'ATE, ces trois-la n'etaient pas")
    print("  pre-enregistrees et se lisent comme des indications.")
    print("")

    manquants = [(s, g) for s in STRATEGIES for g in GRAINES
                 if donnees[s][g] is None]
    if manquants:
        print("=== RUNS MANQUANTS ===")
        print("")
        for s, g in manquants:
            print("  %sP_%d" % (s, g))
        print("")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BrokenPipeError:
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, sys.stdout.fileno())
        sys.exit(0)
