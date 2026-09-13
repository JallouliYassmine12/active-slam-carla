#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dernier essai sur le gain d'information : aligner sensor_range sur
Grid/RangeMax. Et retour a min_candidate_distance = 25 m.

CE QUI A ETE MESURE
-------------------
Run valid_mincand12.log, budget atteint (604 m). Les deux extremes
apparaissent dans le MEME run :

    debut (7 decisions en 80 ms, 13 puis 3 candidats)
        info_gain [0.000 - 0.000] etendue=0.000
    fin (8 a 10 candidats)
        info_gain [1.000 - 1.000] etendue=0.000

Et l'instrumentation donne la geometrie exacte :

    DIAG grille : x=[-1.6,308.9] y=[-5.8,124.0] res=0.15 2070x865
                | pose_grille=(306.2,57.9)
                | candidats (309.7,45.4), (309.6,37.4), (309.4,29.4)

Les trois candidats sont a x = 309.4 a 309.7 pour un bord de grille a
x = 308.9 : 80 centimetres au-dela, tous les trois, tous du meme cote.

LE MECANISME
------------
A chaque decision, le generateur propose des candidats situes a peu
pres a la meme distance et dans la meme direction -- devant, le long de
la route. Ils tombent donc tous du meme cote de la frontiere de
cartographie : soit tous dans le disque deja cartographie (0.000), soit
tous au-dela (1.000).

Le critere n'est pas mal calcule. Le vivier de candidats n'a aucune
diversite d'exploration a classer.

LES DEUX MODIFICATIONS
----------------------
1. sensor_range : 20 -> 40 m.

   Ce n'est pas un reglage de plus, c'est une incoherence que le code
   signale lui-meme. information_gain.py :

       sensor_range : rayon de recherche (m). A aligner avec la portee
                      utile du LiDAR / le parametre Grid/RangeMax cote
                      RTAB-Map.

   sensor_range vaut 20 m, Grid/RangeMax vaut 40 m. Un disque de 40 m
   chevauche la frontiere la ou un disque de 20 m tombe entierement
   d'un cote -- c'est exactement ce qui manque pour obtenir des valeurs
   intermediaires.

   C'est la SEULE variable de ce test.

2. min_candidate_distance : 12 -> 25 m (retour).

   Ce n'est pas un reglage exploratoire, c'est un retour au meilleur
   comportement mesure. Comparaison a budget egal :

       25 m : 602 m parcourus, 13 destinations atteintes
       12 m : 604 m parcourus,  3 destinations atteintes

   Pour une campagne comparative, plus de destinations veut dire plus
   de decisions, donc plus d'influence reelle de la fonction de
   decision sur la trajectoire. 25 m est meilleur independamment du
   gain d'information, et le seuil de rejet de on_arrived (1,0 s) a ete
   dimensionne pour 25 m.

REGLE D'ARRET
-------------
Si l'etendue reste nulle apres ce run, on cesse de poursuivre ce
critere. L'instrumentation temporaire est retiree, la campagne est
lancee, et information_gain est documente comme sature dans ce
dispositif -- avec le mecanisme et les chiffres ci-dessus. Une limite
mesuree et expliquee est un resultat ; c'est le silence sur le sujet
qui n'en serait pas un.

VERIFICATION
------------
    bash ~/valider.sh capteur40
    grep "CRITERES" ~/valid_capteur40.log | tail -10

Usage :
    python3 patch_portee_capteur.py
"""

import io
import os
import shutil
import sys
import time

LAUNCH = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/launch/"
    "active_slam.launch.py"
)


# ---------------------------------------------------------------------
# 1. sensor_range : 20 -> 40 m
# ---------------------------------------------------------------------
ANCRE_CAPTEUR = "            'sensor_range': 20.0,\n"

REMPLACEMENT_CAPTEUR = (
    "            # --- Rayon d'evaluation du gain d'information ---\n"
    "            # A ALIGNER SUR Grid/RangeMax (40 m), comme le prescrit\n"
    "            # la documentation de information_gain.py. A 20 m contre\n"
    "            # 40, le disque d'evaluation d'un candidat tombait\n"
    "            # entierement d'un cote de la frontiere de cartographie.\n"
    "            #\n"
    "            # Mesure sur valid_mincand12.log : candidats a\n"
    "            # x = 309.4 a 309.7 pour un bord de grille a x = 308.9 --\n"
    "            # 80 cm au-dela, tous du meme cote. D'ou info_gain = 1.000\n"
    "            # pour tous en fin de run, et 0.000 pour tous au debut,\n"
    "            # sans jamais de valeur intermediaire.\n"
    "            #\n"
    "            # Un disque de 40 m chevauche la frontiere au lieu de la\n"
    "            # manquer.\n"
    "            'sensor_range': 40.0,\n"
)


# ---------------------------------------------------------------------
# 2. min_candidate_distance : retour a 25 m
# ---------------------------------------------------------------------
ANCRE_DISTANCE = (
    "                # --- Distance routiere minimale d'une destination ---\n"
    "                # DOIT rester nettement INFERIEURE au sensor_range du\n"
    "                # decision_maker (20 m), sinon aucun candidat ne tombe\n"
    "                # dans la zone deja cartographiee et le gain\n"
    "                # d'information vaut 1,0 pour tous.\n"
    "                #\n"
    "                # Mesure a 25 m, sur 12 decisions consecutives :\n"
    "                #   info_gain [1.000 - 1.000] etendue=0.000\n"
    "                # Le critere qui porte le poids le plus fort de la\n"
    "                # decision (0,30) ne classait donc rien.\n"
    "                #\n"
    "                # A 12 m, un candidat proche evalue majoritairement du\n"
    "                # terrain connu (gain faible) et un candidat lointain du\n"
    "                # terrain inconnu (gain eleve) : le critere retrouve une\n"
    "                # plage utile.\n"
    "                #\n"
    "                # 12 m reste au-dessus du tabu_radius (10 m), donc le\n"
    "                # garde-fou contre les sauts de quelques metres -- la\n"
    "                # raison pour laquelle ce seuil avait ete releve de 5 a\n"
    "                # 25 m -- reste en place.\n"
    "                'min_candidate_distance': 12.0,\n"
)

REMPLACEMENT_DISTANCE = (
    "                # --- Distance routiere minimale d'une destination ---\n"
    "                # 12 m a ete essaye pour rapprocher les candidats de la\n"
    "                # zone cartographiee. L'essai est concluant, mais dans\n"
    "                # l'autre sens que prevu -- a budget egal :\n"
    "                #\n"
    "                #   25 m : 602 m parcourus, 13 destinations atteintes\n"
    "                #   12 m : 604 m parcourus,  3 destinations atteintes\n"
    "                #\n"
    "                # Le gain d'information restait sature dans les deux\n"
    "                # cas, et 25 m produit quatre fois plus de decisions --\n"
    "                # donc quatre fois plus d'influence de la fonction de\n"
    "                # decision sur la trajectoire, ce qui est l'objet meme\n"
    "                # de la campagne comparative.\n"
    "                #\n"
    "                # 25 m est aussi la valeur pour laquelle le seuil de\n"
    "                # rejet de on_arrived (1,0 s) a ete dimensionne : a\n"
    "                # 12 m, une arrivee reelle a pleine vitesse prend 0,8 s\n"
    "                # et serait comptee comme un rejet.\n"
    "                'min_candidate_distance': 25.0,\n"
)


def main():
    if not os.path.exists(LAUNCH):
        raise SystemExit("Fichier introuvable : %s" % LAUNCH)

    src = io.open(LAUNCH, encoding="utf-8").read()

    if "'sensor_range': 40.0," in src:
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    for nom, ancre in (
        ("sensor_range", ANCRE_CAPTEUR),
        ("min_candidate_distance", ANCRE_DISTANCE),
    ):
        n = src.count(ancre)
        if n != 1:
            raise SystemExit(
                "ANCRE '%s' TROUVEE %d FOIS (attendu 1).\n"
                "Aucune modification effectuee." % (nom, n)
            )

    src = src.replace(ANCRE_CAPTEUR, REMPLACEMENT_CAPTEUR)
    src = src.replace(ANCRE_DISTANCE, REMPLACEMENT_DISTANCE)

    sauvegarde = "%s.bak_capteur_%s" % (LAUNCH, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(LAUNCH, sauvegarde)
    io.open(LAUNCH, "w", encoding="utf-8").write(src)

    print("sensor_range           : 20.0 -> 40.0   (aligne sur Grid/RangeMax)")
    print("min_candidate_distance : 12.0 -> 25.0   (retour au meilleur mesure)")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Reconstruire :")
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis valider :")
    print("  bash ~/valider.sh capteur40")
    print("  grep 'CRITERES' ~/valid_capteur40.log | tail -10")
    return 0


if __name__ == "__main__":
    sys.exit(main())
