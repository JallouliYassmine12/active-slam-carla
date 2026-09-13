#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Agrege les runs de la campagne par strategie : moyennes, dispersion, et
tableaux prets pour le chapitre resultats.

POURQUOI CE SCRIPT
------------------
resume_run.py donne un run par ligne. Pour conclure quoi que ce soit il
faut regrouper par strategie et regarder DEUX choses, jamais une seule :

  - la moyenne, qui dit quelle strategie fait mieux ;
  - la dispersion entre repetitions, qui dit si cette difference veut
    dire quelque chose.

Une moyenne sans dispersion est un chiffre qu'on ne peut pas defendre.
Mesure sur la campagne du 3 septembre : 'random' va de 14,9 m a 272,7 m
d'erreur moyenne selon la repetition, avec un code identique. Annoncer
"random = 137 m" sans dire "de 15 a 273" serait trompeur.

USAGE
-----
    python3 ~/agreger.py                 # tableaux a l'ecran
    python3 ~/agreger.py --csv           # export pour le rapport
    python3 ~/agreger.py --csv > ~/resultats_campagne.csv

Il lit les memes fichiers que resume_run.py :
    metrics/ate_<nom>.csv        erreur de localisation
    metrics/campagne_<nom>.csv   journal des decisions

CE QUE LE SCRIPT NE FAIT PAS
----------------------------
Aucun test statistique. Avec un si petit nombre de repetitions par
strategie, aucun test n'aurait de puissance : annoncer une p-valeur
donnerait une
apparence de rigueur a une conclusion que les donnees ne portent pas. Le
script donne les valeurs, l'etendue et l'ecart-type ; l'interpretation
se fait a la main, en regardant si les etendues se chevauchent.
"""

import csv
import glob
import math
import os
import sys

METRICS = os.path.expanduser("~/active_slam_carla/metrics")
PAS_GRILLE = 10.0

STRATEGIES = ["random", "distance", "info_gain", "weighted", "predefinie"]
REPETITIONS = [1, 2, 3, 4, 5]
METEOS = ["rain", "fog"]


def lire_csv(chemin):
    with open(chemin, newline="", encoding="utf-8", errors="replace") as f:
        return list(csv.DictReader(f))


def flottant(ligne, cle, defaut=float("nan")):
    v = ligne.get(cle)
    if v is None or v == "":
        return defaut
    try:
        return float(v)
    except ValueError:
        return defaut


def moyenne(valeurs):
    valeurs = [v for v in valeurs if not math.isnan(v)]
    return sum(valeurs) / len(valeurs) if valeurs else float("nan")


def ecart_type(valeurs):
    valeurs = [v for v in valeurs if not math.isnan(v)]
    if len(valeurs) < 2:
        return 0.0
    m = sum(valeurs) / len(valeurs)
    return math.sqrt(sum((v - m) ** 2 for v in valeurs) / (len(valeurs) - 1))


def mesures_run(nom):
    """Toutes les mesures d'un run, ou None s'il n'est pas archive."""
    chemin_ate = os.path.join(METRICS, "ate_%s.csv" % nom)
    if not os.path.exists(chemin_ate):
        return None

    echantillons = []
    for l in lire_csv(chemin_ate):
        t = flottant(l, "timestamp")
        gx, gy = flottant(l, "gt_x"), flottant(l, "gt_y")
        err = flottant(l, "error")
        if any(math.isnan(v) for v in (t, gx, gy, err)):
            continue
        if gx == 0.0 and gy == 0.0 and err == 0.0:
            continue          # echantillon d'amorcage
        echantillons.append((t, gx, gy, err))

    if not echantillons:
        return None

    echantillons.sort()       # les lignes ne sont pas ecrites triees

    erreurs = [e[3] for e in echantillons]
    cellules = {
        (int(math.floor(gx / PAS_GRILLE)), int(math.floor(gy / PAS_GRILLE)))
        for _, gx, gy, _ in echantillons
    }
    longueur = sum(
        math.hypot(echantillons[i][1] - echantillons[i - 1][1],
                   echantillons[i][2] - echantillons[i - 1][2])
        for i in range(1, len(echantillons))
    )

    m = {
        "n": len(echantillons),
        "ate_moy": moyenne(erreurs),
        "ate_max": max(erreurs),
        "rmse": math.sqrt(sum(e * e for e in erreurs) / len(erreurs)),
        "cellules": float(len(cellules)),
        "longueur": longueur,
        "duree": echantillons[-1][0] - echantillons[0][0],
    }

    chemin_dec = os.path.join(METRICS, "campagne_%s.csv" % nom)
    if os.path.exists(chemin_dec):
        lignes = lire_csv(chemin_dec)
        if lignes:
            def col(c):
                return [flottant(l, c) for l in lignes]

            def cumul(c):
                v = [x for x in col(c) if not math.isnan(x)]
                return max(v) if v else 0.0

            m.update({
                "decisions": float(len(lignes)),
                "gain": moyenne(col("information_gain")),
                "boucle": moyenne(col("loop_closure")),
                "incertitude": moyenne(col("localization_uncertainty")),
                "securite": moyenne(col("safety")),
                "dist_candidat": moyenne(col("distance")),
                "collisions": (cumul("collisions_vehicules")
                               + cumul("collisions_pietons")
                               + cumul("collisions_obstacles")),
            })
    return m


COLONNES = [
    ("ate_moy", "ATE moyenne (m)", 1),
    ("rmse", "RMSE (m)", 1),
    ("ate_max", "ATE max (m)", 1),
    ("cellules", "cellules 10 m", 0),
    ("longueur", "trajectoire (m)", 0),
    ("decisions", "decisions", 0),
    ("gain", "gain d'info", 3),
    ("boucle", "fermeture boucle", 3),
    ("securite", "securite", 3),
    ("dist_candidat", "dist. candidat (m)", 1),
    ("collisions", "collisions (total)", 0),
]


def collecter():
    par_strategie = {}
    for s in STRATEGIES:
        runs = []
        for rep in REPETITIONS:
            m = mesures_run("%s_%d" % (s, rep))
            if m is not None:
                runs.append(("%s_%d" % (s, rep), m))
        if runs:
            par_strategie[s] = runs

    meteo = {}
    for mto in METEOS:
        nom = "weighted_%s_1" % mto
        m = mesures_run(nom)
        if m is not None:
            meteo[nom] = m
    return par_strategie, meteo


def afficher(par_strategie, meteo):
    if not par_strategie:
        print("Aucun run archive dans %s" % METRICS)
        return

    print("=== ERREUR DE LOCALISATION PAR STRATEGIE ===")
    print("")
    print("  %-12s %10s %10s %10s %10s %6s"
          % ("strategie", "moyenne", "min", "max", "ecart-type", "n"))
    print("  " + "-" * 62)
    for s in STRATEGIES:
        if s not in par_strategie:
            continue
        v = [m["ate_moy"] for _, m in par_strategie[s]]
        print("  %-12s %10.1f %10.1f %10.1f %10.1f %6d"
              % (s, moyenne(v), min(v), max(v), ecart_type(v), len(v)))
    print("")
    print("  Toutes valeurs en metres. 'min' et 'max' sont les")
    print("  repetitions extremes, pas des bornes de confiance.")
    print("")

    print("=== DISPERSION ENTRE REPETITIONS ===")
    print("")
    print("  Si l'etendue d'une strategie recouvre la moyenne d'une")
    print("  autre, les deux ne sont pas separables au nombre de")
    print("  repetitions dont on dispose.")
    print("")
    for s in STRATEGIES:
        if s not in par_strategie:
            continue
        v = [m["ate_moy"] for _, m in par_strategie[s]]
        print("  %-12s etendue %7.1f m   (%s)"
              % (s, max(v) - min(v), ", ".join("%.1f" % x for x in v)))
    print("")

    print("=== MOYENNES PAR CRITERE ===")
    print("")
    entete = "  %-18s" % "critere"
    for s in STRATEGIES:
        if s in par_strategie:
            entete += " %12s" % s
    print(entete)
    print("  " + "-" * (18 + 13 * len(par_strategie)))
    for cle, libelle, dec in COLONNES:
        ligne = "  %-18s" % libelle
        vide = True
        for s in STRATEGIES:
            if s not in par_strategie:
                continue
            v = [m[cle] for _, m in par_strategie[s] if cle in m]
            if not v:
                ligne += " %12s" % "-"
            else:
                total = sum(v) if cle == "collisions" else moyenne(v)
                ligne += " %12.*f" % (dec, total)
                vide = False
        if not vide:
            print(ligne)
    print("")
    print("  'collisions' est un TOTAL sur les repetitions ; les autres")
    print("  colonnes sont des moyennes.")
    print("")

    if meteo:
        print("=== SCENARIOS METEO (weighted, n = 1 chacun) ===")
        print("")
        reference = None
        if "weighted" in par_strategie:
            reference = moyenne(
                [m["ate_moy"] for _, m in par_strategie["weighted"]]
            )
            print("  weighted clair (moyenne des %d repetitions) : %.1f m"
                  % (len(par_strategie["weighted"]), reference))
        for nom, m in meteo.items():
            suffixe = ""
            if reference and reference > 0:
                suffixe = "   soit x%.1f" % (m["ate_moy"] / reference)
            print("  %-18s %8.1f m%s" % (nom, m["ate_moy"], suffixe))
        print("")
        print("  Une seule repetition par condition : la direction de")
        print("  l'effet est lisible, son amplitude ne l'est pas.")
        print("")


def exporter(par_strategie, meteo):
    sortie = csv.writer(sys.stdout)
    sortie.writerow(["strategie", "repetition", "run"]
                    + [cle for cle, _, _ in COLONNES])
    for s in STRATEGIES:
        for nom, m in par_strategie.get(s, []):
            rep = nom.rsplit("_", 1)[1]
            sortie.writerow([s, rep, nom]
                            + ["%.4f" % m[c] if c in m else ""
                               for c, _, _ in COLONNES])
    for nom, m in meteo.items():
        sortie.writerow(["weighted_meteo", "1", nom]
                        + ["%.4f" % m[c] if c in m else ""
                           for c, _, _ in COLONNES])


def main():
    if not os.path.isdir(METRICS):
        print("Dossier introuvable : %s" % METRICS)
        return 1
    par_strategie, meteo = collecter()
    if "--csv" in sys.argv[1:]:
        exporter(par_strategie, meteo)
    else:
        afficher(par_strategie, meteo)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BrokenPipeError:
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, sys.stdout.fileno())
        sys.exit(0)
