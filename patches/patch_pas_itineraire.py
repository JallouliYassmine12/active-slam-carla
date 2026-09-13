#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Aligne le pas de parcours du controleur sur celui du generateur.

LE FAIT MESURE
--------------
Depuis le meme waypoint de depart (spawn 0 de Town03_Opt, road=1156,
dans une jonction) :

    candidate_generator, pas = 8.0 m  -> 23 destinations atteignables
    vehicle_controller,  pas = 4.0 m  -> 75 aretes explorees, aucune
                                         des 23 n'est routable

Releve du run valid_run6 :

    DIAG echec : aucun itineraire vers (108.21,-35.74) | aretes=75
    DIAG echec : aucun itineraire vers ( 99.79,-51.66) | aretes=75
    FIN DE RUN (no_reachable_goal) : 31 buts ecartes, 0 m parcouru

75 aretes pour un budget de 250 m a 4 m par saut, c'est 62 sauts en
chaine plus deux ou trois ramifications : le parcours suit une voie et ne
tourne pratiquement jamais. L'ajout de road_id a la cle de deduplication
n'a pas change ce nombre -- il etait de 75 avant, il est de 75 apres.

La seule difference restante entre les deux noeuds est le pas. A 8 m,
wp.next() enjambe l'interieur des carrefours et rend directement les
routes sortantes ; a 4 m, le parcours s'arrete dans le carrefour et n'en
suit qu'une seule.

Le commentaire de candidate_generator l'exigeait deja :

    # Pas du parcours du graphe routier. Doit rester coherent avec
    # route_step du controleur pour que les deux voient le meme reseau.

Il ne l'a jamais ete : 8.0 d'un cote, 4.0 de l'autre.

LA CORRECTION
-------------
route_step du controleur passe a 8.0, fixe explicitement dans le fichier
de lancement -- a cote de waypoint_spacing du generateur, pour que la
prochaine personne qui modifie l'un voie immediatement l'autre.

CONSEQUENCE A SURVEILLER
------------------------
Les waypoints de l'itineraire sont desormais espaces de 8 m au lieu de
4 m, alors que waypoint_reach_tolerance vaut 3 m : le vehicule doit
passer a moins de 3 m de chacun pour que route_index avance. Le
generateur echantillonne deja le reseau a 8 m sur les centres de voie
sans difficulte, et le vehicule roule sur ces centres de voie, mais si
un blocage a wp=N/M reapparaissait, c'est ce parametre qu'il faudrait
relever.

Usage :
    python3 patch_pas_itineraire.py
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

ANCRE = "                'lookahead_distance': 6.0,\n"

REMPLACEMENT = (
    "                # --- Pas du parcours du reseau routier ---\n"
    "                # DOIT valoir la meme chose que waypoint_spacing du\n"
    "                # candidate_generator (8.0 m, plus haut dans ce fichier).\n"
    "                # Les deux noeuds repondent a la meme question -- 'puis-je\n"
    "                # aller la ?' -- et doivent donc parcourir le meme graphe.\n"
    "                #\n"
    "                # Mesure avec 4.0 ici et 8.0 la-bas, depuis le meme\n"
    "                # waypoint de depart : le generateur trouvait 23\n"
    "                # destinations atteignables, le controleur n'en routait\n"
    "                # aucune, avec 75 aretes explorees seulement -- soit une\n"
    "                # chaine quasi droite sur 250 m. A 4 m, le parcours\n"
    "                # s'arrete a l'interieur des carrefours et n'en suit\n"
    "                # qu'une branche ; a 8 m, il les enjambe et retombe sur\n"
    "                # toutes les routes sortantes.\n"
    "                'route_step': 8.0,\n"
    "                'lookahead_distance': 6.0,\n"
)


def main():
    if not os.path.exists(LAUNCH):
        raise SystemExit("Fichier introuvable : %s" % LAUNCH)

    src = io.open(LAUNCH, encoding="utf-8").read()

    if "'route_step': 8.0," in src:
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    n = src.count(ANCRE)
    if n != 1:
        raise SystemExit(
            "ANCRE TROUVEE %d FOIS (attendu 1).\n"
            "Aucune modification effectuee." % n
        )

    src = src.replace(ANCRE, REMPLACEMENT)

    sauvegarde = "%s.bak_pas_%s" % (LAUNCH, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(LAUNCH, sauvegarde)
    io.open(LAUNCH, "w", encoding="utf-8").write(src)

    print("route_step fixe a 8.0 dans active_slam.launch.py")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Reconstruire :")
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    return 0


if __name__ == "__main__":
    sys.exit(main())
