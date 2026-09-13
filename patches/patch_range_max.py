#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Releve Grid/RangeMax de 25 m a 40 m dans les DEUX fichiers de lancement.

LE DEFAUT
---------
    Grid/RangeMax          = 25 m   (portee de la grille d'occupation)
    min_candidate_distance = 25 m   (distance minimale d'un candidat)

Les deux valeurs sont EGALES. Tout candidat se trouve donc au bord ou
au-dela de la zone cartographiee, et compute_information_gain -- qui
mesure la proportion de cellules inconnues autour du candidat -- renvoie
la meme chose pour tous.

Mesure sur la campagne complete (tableau 2, moyenne par strategie) :

    random 0.92    distance 0.96    info_gain 0.96    weighted 0.96

Un critere qui vaut 0,96 pour tout le monde ne classe rien. Or c'est
celui qui porte le poids le plus fort de la fonction de decision : 0,30
sur 1,00. Trente pour cent de la ponderation multi-criteres est donc
inerte.

Pire, il n'est pas parfaitement constant : les candidats proches
retombent parfois a 0,90, les lointains restent a 1,00 -- non parce
qu'ils sont plus informatifs, mais parce qu'ils sont plus loin. Le
critere pousse vers les destinations eloignees sans rien mesurer.

LA CORRECTION
-------------
A 40 m, la carte s'etend au-dela des candidats les plus proches :

    candidat a 25 m -> zone deja cartographiee -> gain faible
    candidat a 45 m -> zone inconnue           -> gain eleve

Le critere redevient discriminant, c'est-a-dire conforme a sa definition
en Active SLAM. Le LiDAR porte deja a 80 m : les donnees existent, seule
la grille d'occupation les tronquait a 25 m.

POURQUOI LES DEUX FICHIERS
--------------------------
slam_evaluation.launch.py (SLAM classique) et active_slam.launch.py
doivent partager exactement la meme configuration de cartographie. Sinon
un ecart d'ATE entre les deux ne serait plus attribuable a la methode de
decision : il pourrait venir du reglage de la carte. Le commentaire de
slam_evaluation.launch.py l'exige d'ailleurs deja -- "Grid/CellSize et
Grid/RangeMax doivent rester identiques".

COUT
----
La grille couvre 2,5 fois plus de cellules a resolution egale
(Grid/CellSize reste a 0,15 m). Surveiller les delais de RTAB-Map dans
le journal du run de verification : s'ils depassent nettement 0,3 s,
c'est que la charge devient trop lourde et il faudra soit relacher
CellSize, soit se contenter de 35 m.

Usage :
    python3 patch_range_max.py
"""

import io
import os
import shutil
import sys
import time

ACTIVE = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/launch/"
    "active_slam.launch.py"
)
CLASSIQUE = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/slam_evaluation/launch/"
    "slam_evaluation.launch.py"
)

ANCRE = "                'Grid/RangeMax': '25.0',\n"

REMPLACEMENT = (
    "                # --- Portee de la grille d'occupation ---\n"
    "                # DOIT rester STRICTEMENT SUPERIEURE au\n"
    "                # min_candidate_distance du candidate_generator (25 m).\n"
    "                # Quand les deux sont egales, tout candidat tombe au\n"
    "                # bord ou au-dela de la zone cartographiee et\n"
    "                # compute_information_gain renvoie la meme valeur pour\n"
    "                # tous : mesure sur la campagne complete, 0,92 a 0,96 de\n"
    "                # moyenne pour les quatre strategies. Le critere qui\n"
    "                # porte le poids le plus fort de la decision (0,30) ne\n"
    "                # classait donc plus rien.\n"
    "                #\n"
    "                # A 40 m, un candidat a 25 m tombe en zone connue (gain\n"
    "                # faible) et un candidat a 45 m en zone inconnue (gain\n"
    "                # eleve) : le critere redevient discriminant.\n"
    "                #\n"
    "                # Cette valeur doit rester IDENTIQUE entre\n"
    "                # active_slam.launch.py et slam_evaluation.launch.py,\n"
    "                # sinon un ecart d'ATE entre SLAM classique et Active\n"
    "                # SLAM ne serait plus attribuable a la methode de\n"
    "                # decision.\n"
    "                'Grid/RangeMax': '40.0',\n"
)


def appliquer(chemin):
    src = io.open(chemin, encoding="utf-8").read()
    n = src.count(ANCRE)
    if n != 1:
        raise SystemExit(
            "ANCRE TROUVEE %d FOIS (attendu 1) dans %s.\n"
            "Aucun fichier n'a ete modifie." % (n, os.path.basename(chemin))
        )
    return src.replace(ANCRE, REMPLACEMENT)


def main():
    for chemin in (ACTIVE, CLASSIQUE):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    if "'Grid/RangeMax': '40.0'," in io.open(ACTIVE, encoding="utf-8").read():
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    resultats = {
        ACTIVE: appliquer(ACTIVE),
        CLASSIQUE: appliquer(CLASSIQUE),
    }

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in resultats.items():
        sauvegarde = "%s.bak_rangemax_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("\nGrid/RangeMax : 25.0 -> 40.0 dans les 2 fichiers de lancement.")
    print("\nReconstruire :")
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    return 0


if __name__ == "__main__":
    sys.exit(main())
