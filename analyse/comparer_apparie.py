#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Compare deux reglages de la fonction de decision par PAIRES.

POURQUOI UN PLAN APPARIE
------------------------
Campagne du 3 septembre, cinq repetitions par strategie :

    distance    27,5 m   (10,7 - 38,8)
    predefinie  32,6 m   (19,7 - 47,3)
    weighted    36,0 m   (19,3 - 56,5)
    info_gain   36,7 m   ( 8,7 - 122,1)

Les etendues se recouvrent toutes. Avec un ecart-type de 14,5 m sur
'weighted', separer deux moyennes distantes de 8 m demanderait une
trentaine de repetitions par configuration.

Le plan apparie contourne cela. Les deux configurations tournent sur le
MEME trafic, graine par graine :

    seed        1      2      3      4      5
    config A   A1     A2     A3     A4     A5
    config B   B1     B2     B3     B4     B5
    ecart    B1-A1  B2-A2  B3-A3  B4-A4  B5-A5

La variabilite du trafic est presente dans A comme dans B, donc elle
DISPARAIT de la difference. Ce qui reste est l'effet du reglage.

LE TEST UTILISE
---------------
Le test des SIGNES, et lui seul. Il ne demande aucune hypothese sur la
distribution des ecarts -- ce qui compte quand on n'en a que cinq. Il
repond exactement a la question posee : les differences vont-elles
toutes dans le meme sens ?

    5 ecarts de meme signe  ->  p = 0,031 (unilateral)
    4 sur 5                 ->  p = 0,19
    3 sur 5                 ->  aucune conclusion

Aucune p-valeur issue d'un test t n'est affichee : sur cinq paires,
elle supposerait une normalite invérifiable et donnerait une apparence
de rigueur que les donnees ne portent pas.

CE QUE LE SCRIPT NE DIT PAS
---------------------------
Un resultat apparie vaut pour LE TRAFIC TESTE, graines 1 a 5. Il ne
garantit pas que l'effet se retrouve sur d'autres conditions. C'est le
prix de la puissance gagnee : on a supprime une source de variabilite
de la comparaison, pas du monde.

USAGE
-----
    python3 ~/comparer_apparie.py
    python3 ~/comparer_apparie.py --csv
"""

import csv
import math
import os
import sys

METRICS = os.path.expanduser("~/active_slam_carla/metrics")
GRAINES = [1, 2, 3, 4, 5]

CONFIGS = [
    ("A", "weightedA", "info 0.30 / distance 0.15  (reglage actuel)"),
    ("B", "weightedB", "info 0.20 / distance 0.25  (sauts plus courts)"),
]


def erreur_moyenne(nom):
    """ATE moyenne d'un run, ou None s'il n'est pas archive."""
    chemin = os.path.join(METRICS, "ate_%s.csv" % nom)
    if not os.path.exists(chemin):
        return None

    erreurs = []
    with open(chemin, newline="", encoding="utf-8", errors="replace") as f:
        for ligne in csv.DictReader(f):
            try:
                gx = float(ligne["gt_x"])
                gy = float(ligne["gt_y"])
                err = float(ligne["error"])
            except (KeyError, TypeError, ValueError):
                continue
            # Echantillon d'amorcage : tout a zero, ce n'est pas une mesure.
            if gx == 0.0 and gy == 0.0 and err == 0.0:
                continue
            erreurs.append(err)

    if not erreurs:
        return None
    return sum(erreurs) / len(erreurs)


def collecter():
    paires = []
    for graine in GRAINES:
        a = erreur_moyenne("weightedA_%d" % graine)
        b = erreur_moyenne("weightedB_%d" % graine)
        paires.append((graine, a, b))
    return paires


def afficher(paires):
    print("=== COMPARAISON APPARIEE A / B ===")
    print("")
    for lettre, prefixe, description in CONFIGS:
        print("  config %s : %s" % (lettre, description))
    print("")
    print("  %6s %12s %12s %12s" % ("graine", "A (m)", "B (m)", "B - A"))
    print("  " + "-" * 46)

    ecarts = []
    for graine, a, b in paires:
        if a is None or b is None:
            print("  %6d %12s %12s %12s"
                  % (graine,
                     "-" if a is None else "%.1f" % a,
                     "-" if b is None else "%.1f" % b,
                     "incomplet"))
            continue
        ecart = b - a
        ecarts.append(ecart)
        print("  %6d %12.1f %12.1f %+12.1f" % (graine, a, b, ecart))

    print("")

    if len(ecarts) < len(GRAINES):
        print("  %d paire(s) sur %d completes : resultat provisoire."
              % (len(ecarts), len(GRAINES)))
        print("")

    if not ecarts:
        print("  Aucune paire complete. Lancer les runs :")
        print("    VARIANTE=A bash ~/un_scenario.sh weighted 1   (etc.)")
        return

    moyenne = sum(ecarts) / len(ecarts)
    negatifs = sum(1 for e in ecarts if e < 0)
    positifs = sum(1 for e in ecarts if e > 0)

    print("  Ecart moyen B - A : %+.1f m" % moyenne)
    print("  Signes : %d amelioration(s), %d degradation(s)"
          % (negatifs, positifs))
    print("")

    # Test des signes, unilateral, sur les paires non nulles.
    n = negatifs + positifs
    k = max(negatifs, positifs)
    if n == 0:
        print("  Aucune difference : les deux reglages donnent le meme")
        print("  resultat sur ce trafic.")
        return

    p = sum(math.comb(n, i) for i in range(k, n + 1)) / (2.0 ** n)

    sens = "AMELIORE" if negatifs > positifs else "DEGRADE"
    print("  Test des signes : %d/%d dans le meme sens, p = %.3f"
          % (k, n, p))
    print("")

    if p <= 0.05:
        print("  Lecture : la configuration B %s la localisation de facon" % sens)
        print("  systematique sur les cinq trafics testes. L'effet vaut pour")
        print("  ces conditions ; il reste a confirmer sur d'autres graines.")
    elif k == n:
        print("  Lecture : les %d ecarts vont dans le meme sens, mais %d"
              % (n, n))
        print("  paires ne suffisent pas a exclure le hasard (p = %.3f)." % p)
        print("  Deux graines de plus trancheraient.")
    else:
        print("  Lecture : les ecarts ne vont pas tous dans le meme sens.")
        print("  Aucun effet systematique n'est mis en evidence -- ce qui")
        print("  est un resultat, pas un echec : le reglage teste ne")
        print("  gouverne pas la precision de localisation.")


def exporter(paires):
    sortie = csv.writer(sys.stdout)
    sortie.writerow(["graine", "ate_A", "ate_B", "ecart_B_moins_A"])
    for graine, a, b in paires:
        sortie.writerow([
            graine,
            "" if a is None else "%.4f" % a,
            "" if b is None else "%.4f" % b,
            "" if (a is None or b is None) else "%.4f" % (b - a),
        ])


def main():
    if not os.path.isdir(METRICS):
        print("Dossier introuvable : %s" % METRICS)
        return 1
    paires = collecter()
    if "--csv" in sys.argv[1:]:
        exporter(paires)
    else:
        afficher(paires)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BrokenPipeError:
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, sys.stdout.fileno())
        sys.exit(0)
