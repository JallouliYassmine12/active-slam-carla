#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Compte les fermetures de boucle REELLES de chaque run et les met en
regard du critere qui etait cense les provoquer.

LE MAILLON MANQUANT
-------------------
Le critere 'loop_closure' du decideur mesure un POTENTIEL : la
proximite d'un candidat aux noeuds du graphe RTAB-Map. Il ne dit pas
si un recalage a effectivement eu lieu. La chaine que le sujet suppose
est pourtant :

    critere eleve  ->  fermetures reelles  ->  erreur reduite

Les deux extremites sont mesurees depuis le debut ; le milieu ne l'a
jamais ete. Ce script le mesure, sur des journaux deja archives, sans
relancer un seul run.

CE QUI A DECLENCHE CETTE ANALYSE
--------------------------------
Releve sur weighted_1 (campagne du 3 septembre) :

    12 candidats de fermeture proposes par RTAB-Map
    12 rejetes : "Cannot compute transform"
     0 accepte
    meilleur corrRatio atteint : 0.049066

Or Icp/CorrespondenceRatio vaut 0.05 dans le launch. La meilleure
fermeture du run a donc ete refusee pour 0,0009.

Si ce constat vaut sur l'ensemble des runs, alors aucune strategie ne
pouvait beneficier d'un recalage : toutes accumulaient de la derive
d'odometrie pure. Cela expliquerait d'un seul coup pourquoi les quatre
strategies sont indiscernables, pourquoi l'erreur croit de facon
monotone, et pourquoi le critere de fermeture de boucle n'a produit
aucun effet mesurable.

DEUX MESURES INDEPENDANTES
--------------------------
1. LE JOURNAL. Nombre de lignes "Rejected loop closure" et meilleur
   corrRatio atteint. C'est ce que RTAB-Map dit avoir refuse.

2. LA TRAJECTOIRE. Nombre de CHUTES ABRUPTES de l'erreur de
   localisation entre deux echantillons consecutifs.

   Cette seconde mesure ne depend d'aucun message : une erreur qui
   diminue brusquement de plusieurs metres en une seconde ne peut pas
   venir de la derive, qui est graduelle. C'est la signature d'une
   optimisation du graphe de poses, donc d'une fermeture ACCEPTEE.

   Le seuil est fixe a 5 m : au-dela de ce que la derive peut defaire
   en une seconde, en deca de ce qu'un vrai recalage corrige.

Les deux mesures se controlent l'une l'autre. Des rejets sans aucune
chute confirment que rien n'a jamais ete accepte ; des chutes sans
rejet montreraient l'inverse.

USAGE
-----
    python3 ~/analyse_boucles.py
    python3 ~/analyse_boucles.py --csv
"""

import csv
import glob
import math
import os
import re
import sys

METRICS = os.path.expanduser("~/active_slam_carla/metrics")
LOGS = os.path.expanduser("~/logs_campagne2")

# Une chute de l'erreur superieure a ce seuil entre deux echantillons
# consecutifs ne peut pas s'expliquer par la derive d'odometrie.
SEUIL_CHUTE = 5.0

STRATEGIES = ["random", "distance", "info_gain", "weighted", "predefinie"]
REPETITIONS = [1, 2, 3, 4, 5]
AUTRES = ["weighted_rain_1", "weighted_fog_1"]
APPARIES = (["weightedA_%d" % i for i in REPETITIONS]
            + ["weightedB_%d" % i for i in REPETITIONS])

MOTIF_REJET = re.compile(r"Rejected loop closure")
MOTIF_RATIO = re.compile(r"corrRatio=([0-9.]+)")


def runs_attendus():
    noms = []
    for s in STRATEGIES:
        for r in REPETITIONS:
            noms.append("%s_%d" % (s, r))
    return noms + AUTRES + APPARIES


def analyser_journal(nom):
    """Rejets de fermeture et meilleur ratio, depuis le journal du run."""
    chemin = os.path.join(LOGS, "%s.log" % nom)
    if not os.path.exists(chemin):
        return None

    rejets = 0
    ratios = []
    with open(chemin, encoding="utf-8", errors="replace") as f:
        for ligne in f:
            if MOTIF_REJET.search(ligne):
                rejets += 1
            for m in MOTIF_RATIO.finditer(ligne):
                try:
                    ratios.append(float(m.group(1)))
                except ValueError:
                    pass

    return {
        "rejets": rejets,
        "ratio_max": max(ratios) if ratios else None,
        "nb_ratios": len(ratios),
    }


def analyser_trajectoire(nom):
    """Chutes abruptes de l'erreur : signature d'un recalage accepte."""
    chemin = os.path.join(METRICS, "ate_%s.csv" % nom)
    if not os.path.exists(chemin):
        return None

    echantillons = []
    with open(chemin, newline="", encoding="utf-8", errors="replace") as f:
        for l in csv.DictReader(f):
            try:
                t = float(l["timestamp"])
                x = float(l["gt_x"])
                y = float(l["gt_y"])
                e = float(l["error"])
            except (KeyError, TypeError, ValueError):
                continue
            if x == 0.0 and y == 0.0 and e == 0.0:
                continue
            echantillons.append((t, e))

    if len(echantillons) < 2:
        return None

    # Les lignes ne sont pas ecrites dans l'ordre chronologique.
    echantillons.sort()

    chutes = []
    for i in range(1, len(echantillons)):
        delta = echantillons[i][1] - echantillons[i - 1][1]
        if delta <= -SEUIL_CHUTE:
            chutes.append(-delta)

    return {
        "n": len(echantillons),
        "chutes": len(chutes),
        "correction_totale": sum(chutes),
        "correction_max": max(chutes) if chutes else 0.0,
        "erreur_finale": echantillons[-1][1],
    }


def critere_moyen(nom):
    """Moyenne du critere 'loop_closure' sur les decisions du run."""
    chemin = os.path.join(METRICS, "campagne_%s.csv" % nom)
    if not os.path.exists(chemin):
        return None
    valeurs = []
    with open(chemin, newline="", encoding="utf-8", errors="replace") as f:
        for l in csv.DictReader(f):
            try:
                valeurs.append(float(l["loop_closure"]))
            except (KeyError, TypeError, ValueError):
                continue
    return sum(valeurs) / len(valeurs) if valeurs else None


def collecter():
    lignes = []
    for nom in runs_attendus():
        traj = analyser_trajectoire(nom)
        if traj is None:
            continue
        lignes.append({
            "run": nom,
            "journal": analyser_journal(nom),
            "traj": traj,
            "critere": critere_moyen(nom),
        })
    return lignes


def afficher(lignes):
    if not lignes:
        print("Aucun run archive trouve.")
        return

    print("=== FERMETURES DE BOUCLE : POTENTIEL CONTRE REALITE ===")
    print("")
    print("  %-16s %8s %9s %8s %10s %10s"
          % ("run", "rejets", "ratio max", "chutes", "corrige m", "critere"))
    print("  " + "-" * 68)

    total_rejets = 0
    total_chutes = 0
    ratios = []
    sans_journal = 0

    for l in lignes:
        j = l["journal"]
        t = l["traj"]
        if j is None:
            sans_journal += 1
            rejets, ratio = "-", "-"
        else:
            total_rejets += j["rejets"]
            rejets = "%d" % j["rejets"]
            if j["ratio_max"] is not None:
                ratios.append(j["ratio_max"])
                ratio = "%.4f" % j["ratio_max"]
            else:
                ratio = "-"
        total_chutes += t["chutes"]
        print("  %-16s %8s %9s %8d %10.1f %10s"
              % (l["run"], rejets, ratio, t["chutes"],
                 t["correction_totale"],
                 "-" if l["critere"] is None else "%.3f" % l["critere"]))

    print("")
    print("  rejets = fermetures proposees puis refusees par RTAB-Map")
    print("  ratio max = meilleur corrRatio atteint (seuil du launch : 0.05)")
    print("  chutes = baisses d'erreur > %.0f m entre deux echantillons,"
          % SEUIL_CHUTE)
    print("           signature d'une optimisation du graphe de poses")
    print("  corrige m = total des metres repris par ces chutes")
    print("  critere = moyenne du critere loop_closure sur les decisions")
    print("")

    print("=== SYNTHESE ===")
    print("")
    print("  %d runs analyses" % len(lignes))
    if sans_journal:
        print("  %d sans journal archive" % sans_journal)
    print("  %d fermetures proposees puis REJETEES au total" % total_rejets)
    print("  %d chutes d'erreur observees au total" % total_chutes)
    if ratios:
        print("  meilleur corrRatio tous runs confondus : %.6f"
              % max(ratios))
        au_dessus = sum(1 for r in ratios if r >= 0.05)
        print("  runs dont le meilleur ratio atteint 0.05 : %d / %d"
              % (au_dessus, len(ratios)))
    print("")

    if total_chutes == 0 and total_rejets > 0:
        print("  LECTURE : aucune fermeture de boucle n'a jamais ete")
        print("  acceptee. Le critere de potentiel optimisait vers un")
        print("  recalage que la chaine SLAM ne delivrait pas. Toutes les")
        print("  strategies accumulaient de la derive d'odometrie pure,")
        print("  ce qui explique qu'aucune ne se distingue.")
    elif total_chutes > 0:
        print("  LECTURE : des recalages ont bien eu lieu (%d chutes)."
              % total_chutes)
        print("  Le critere n'est donc pas inoperant ; comparer la colonne")
        print("  'chutes' a la colonne 'critere' dit s'il les predit.")

        # Correlation entre le critere et les recalages reels.
        paires = [(l["critere"], l["traj"]["chutes"])
                  for l in lignes if l["critere"] is not None]
        if len(paires) >= 3:
            xs = [p[0] for p in paires]
            ys = [float(p[1]) for p in paires]
            mx = sum(xs) / len(xs)
            my = sum(ys) / len(ys)
            sx = math.sqrt(sum((a - mx) ** 2 for a in xs))
            sy = math.sqrt(sum((b - my) ** 2 for b in ys))
            if sx > 0 and sy > 0:
                r = sum((a - mx) * (b - my)
                        for a, b in zip(xs, ys)) / (sx * sy)
                print("")
                print("  Correlation critere <-> recalages reels : %.2f"
                      " (sur %d runs)" % (r, len(paires)))
                if abs(r) < 0.3:
                    print("  Faible : le critere ne predit pas les recalages")
                    print("  qu'il est cense provoquer.")


def exporter(lignes):
    sortie = csv.writer(sys.stdout)
    sortie.writerow(["run", "rejets", "ratio_max", "chutes",
                     "correction_totale_m", "correction_max_m",
                     "erreur_finale_m", "critere_loop_closure"])
    for l in lignes:
        j = l["journal"] or {}
        t = l["traj"]
        sortie.writerow([
            l["run"],
            j.get("rejets", ""),
            "" if j.get("ratio_max") is None else "%.6f" % j["ratio_max"],
            t["chutes"],
            "%.2f" % t["correction_totale"],
            "%.2f" % t["correction_max"],
            "%.2f" % t["erreur_finale"],
            "" if l["critere"] is None else "%.4f" % l["critere"],
        ])


def main():
    if not os.path.isdir(METRICS):
        print("Dossier introuvable : %s" % METRICS)
        return 1
    lignes = collecter()
    if "--csv" in sys.argv[1:]:
        exporter(lignes)
    else:
        afficher(lignes)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BrokenPipeError:
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, sys.stdout.fileno())
        sys.exit(0)
