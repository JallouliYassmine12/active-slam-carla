#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Elargit le champ de vision tactique du controleur.

LE FAIT MESURE
--------------
Run valid_granularite.log, 603 m parcourus :

    OBSTACLE DEVANT ....... 1
    OBSTACLE PROCHE ....... 1
    collisions ............ 4

Une seule detection pour quatre collisions. Et au moment de la
premiere :

    POURSUITE : dist_but=54.0m cap_err=0deg vitesse=5.10m/s steer=0.01
    EVENEMENT DE SECURITE : collision vehicule

Cap aligne, volant a 0.01 : le vehicule roulait tout droit et a percute
une voiture sans jamais l'avoir signalee.

LA CAUSE
--------
_closest_obstacle_ahead ne retient un obstacle que s'il satisfait DEUX
conditions, toutes deux trop serrees :

    obstacle_slow_distance  = 14 m    portee longitudinale
    obstacle_corridor_width =  3 m    soit 1,5 m de part et d'autre

Sur la portee : le vehicule roule jusqu'a 9 m/s. Quatorze metres, c'est
une seconde et demie de trajet -- moins que le temps de reaction du
freinage progressif, qui ne commence a agir qu'a partir de cette meme
distance. Un vehicule arrete au bout d'une file est ignore jusqu'a ce
qu'il soit trop tard.

Sur le couloir : une voie CARLA fait environ 3,5 m de large. Un couloir
de 1,5 m de demi-largeur suppose que l'ego est parfaitement centre sur
l'axe de sa voie. Le pure pursuit ne le garantit pas -- il vise un point
situe 6 m devant et s'ecarte naturellement de l'axe en approche de
virage. Il suffit d'un metre et demi d'ecart pour que le vehicule
pourtant juste devant sorte du couloir et devienne invisible.

LA CORRECTION
-------------
    obstacle_slow_distance  : 14 -> 25 m   (2,8 s a 9 m/s)
    obstacle_stop_distance  :  7 ->  9 m   (une longueur de vehicule)
    obstacle_corridor_width :  3 ->  4 m   (2 m de part et d'autre)

Aucune ligne de code n'est modifiee : ce sont trois parametres du
launch. Le risque est donc borne, et son sens est connu d'avance --
elargir le couloir peut faire freiner l'ego pour des vehicules d'une
voie adjacente, ce qui le ralentirait. Le commentaire du code signale
d'ailleurs qu'un filtre trop large avait deja produit cet effet
("l'ego freinait en permanence et n'avancait plus"), mais avec un CONE
angulaire, pas avec un couloir : a 25 m, un couloir de 4 m reste plus
selectif qu'un cone de meme portee.

Deux metres de demi-largeur restent en dessous des 3,5 m d'une voie :
un vehicule centre sur la voie d'a cote est a 3,5 m de l'axe, donc
toujours ignore.

VERIFICATION
------------
    bash ~/valider.sh vision

    grep -c "OBSTACLE DEVANT" ~/valid_vision.log
    grep -c "OBSTACLE PROCHE" ~/valid_vision.log
    grep -c "EVENEMENT DE SECURITE" ~/valid_vision.log
    grep "FIN DE RUN" ~/valid_vision.log

Deux grandeurs a lire ensemble : le nombre de collisions doit baisser,
et la duree du run ne doit pas exploser. Si le run met beaucoup plus
longtemps a parcourir ses 600 m, l'ego freine trop et il faut revenir a
un couloir de 3,5 m.

Usage :
    python3 patch_vision_obstacles.py
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

ANCRE = "                'route_step': 8.0,\n"

REMPLACEMENT = (
    "                # --- Vision tactique du controleur ---\n"
    "                # Mesure sur valid_granularite.log : 1 seule detection\n"
    "                # d'obstacle pour 4 collisions sur 603 m, et au moment\n"
    "                # de la premiere collision le vehicule roulait tout\n"
    "                # droit a 5,1 m/s, volant a 0,01, sans avoir rien\n"
    "                # signale.\n"
    "                #\n"
    "                # Portee : 14 m ne font qu'1,5 s de trajet a 9 m/s,\n"
    "                # moins que ce qu'il faut au freinage progressif pour\n"
    "                # agir. 25 m donnent 2,8 s.\n"
    "                #\n"
    "                # Couloir : une voie CARLA fait 3,5 m. Un demi-couloir\n"
    "                # de 1,5 m suppose l'ego parfaitement centre sur son\n"
    "                # axe, ce que le pure pursuit ne garantit pas -- il vise\n"
    "                # un point situe 6 m devant et s'ecarte en approche de\n"
    "                # virage. 2 m de demi-largeur restent sous les 3,5 m\n"
    "                # d'une voie : un vehicule centre sur la voie d'a cote\n"
    "                # est toujours ignore.\n"
    "                'obstacle_stop_distance': 9.0,\n"
    "                'obstacle_slow_distance': 25.0,\n"
    "                'obstacle_corridor_width': 4.0,\n"
    "                'route_step': 8.0,\n"
)


def main():
    if not os.path.exists(LAUNCH):
        raise SystemExit("Fichier introuvable : %s" % LAUNCH)

    src = io.open(LAUNCH, encoding="utf-8").read()

    for nom in ("obstacle_stop_distance", "obstacle_slow_distance",
                "obstacle_corridor_width"):
        if "'%s'" % nom in src:
            raise SystemExit(
                "'%s' est deja present dans le fichier de launch.\n"
                "Ajouter une seconde fois la meme cle produirait un\n"
                "doublon silencieux dans le dictionnaire de parametres.\n"
                "Aucune modification effectuee -- corriger la valeur "
                "existante." % nom
            )

    n = src.count(ANCRE)
    if n != 1:
        raise SystemExit(
            "ANCRE TROUVEE %d FOIS (attendu 1).\n"
            "Aucune modification effectuee." % n
        )

    src = src.replace(ANCRE, REMPLACEMENT)

    sauvegarde = "%s.bak_vision_%s" % (LAUNCH, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(LAUNCH, sauvegarde)
    io.open(LAUNCH, "w", encoding="utf-8").write(src)

    print("obstacle_stop_distance  :  7.0 ->  9.0 m")
    print("obstacle_slow_distance  : 14.0 -> 25.0 m")
    print("obstacle_corridor_width :  3.0 ->  4.0 m")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Reconstruire :")
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    return 0


if __name__ == "__main__":
    sys.exit(main())
