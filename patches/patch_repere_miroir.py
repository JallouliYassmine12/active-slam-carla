#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Corrige l'incoherence de repere entre decision_maker et le reste de la
chaine Active SLAM.

LE DEFAUT
---------
bridge_active_slam publie le nuage LiDAR dans la convention MAIN GAUCHE
de CARLA, sans inverser Y. Le repere que RTAB-Map construit a partir de
ces scans -- et dans lequel il publie /localization_pose, /grid_prob_map
et /mapGraph -- est donc l'image MIROIR, selon l'axe lateral, du repere
'map' employe par candidate_generator.py et obstacle_detector.py, qui
appliquent tous deux -(y - origin_y) puis la rotation par le cap de
spawn.

decision_maker melangeait les deux : il lisait la pose brute de RTAB-Map
et la comparait a des candidats et des obstacles exprimes dans l'autre
convention.

PREUVE
------
Releve du run info_gain repetition 1, a t = 1788012632.8 :

    pose annoncee par decision_maker (brute) : x=216.88  y=+16.07
    position vraie dans le repere candidats  : x=211.6   y=-11.3
    apres inversion du signe de Y            : x=216.88  y=-16.07

L'ecart sur x vaut 5 m (derive normale) ; sur y il valait 27 m avant
correction et 5 m apres. localization_evaluator.py applique deja cette
meme inversion (est_l['y'] = -est_l['y']) avant de calculer l'ATE, ce
qui est la raison pour laquelle l'erreur mesuree restait bonne alors que
la decision, elle, travaillait sur des coordonnees fausses.

Consequence mesurable, et test d'acceptation du correctif : la distance
euclidienne annoncee vers le candidat retenu depassait la longueur de
l'itineraire ROUTIER calcule par le controleur dans 26 decisions sur 29.
Un trajet par la route ne pouvant jamais etre plus court que la ligne
droite, ce test ne peut pas mentir.

PORTEE
------
Quatre des cinq criteres etaient affectes :

    distance                  pose (miroir)   vs candidat
    safety                    pose (miroir)   vs obstacles
    information_gain          grille (miroir) vs candidat
    loop_closure              graphe (miroir) vs candidat
    localization_uncertainty  scalaire global, non affecte

Les deux premiers se corrigent en inversant Y a la source, dans
on_pose(). Les deux suivants interrogent des donnees publiees par
RTAB-Map, donc exprimees dans SON repere : c'est la coordonnee du
candidat qu'il faut y transposer, et non l'inverse.

Le but publie vers le controleur reste inchange : il est exprime dans le
repere des candidats, que _map_to_carla() sait deja convertir.

Usage :
    python3 patch_repere_miroir.py
"""

import io
import os
import shutil
import sys
import time

CIBLE = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/"
    "active_slam_decision/decision_maker.py"
)


BLOCS = [
    # ------------------------------------------------------------------
    # 1. La pose, a la source
    # ------------------------------------------------------------------
    (
        "        self.current_x = msg.pose.pose.position.x\n"
        "        self.current_y = msg.pose.pose.position.y\n",

        "        self.current_x = msg.pose.pose.position.x\n"
        "        # --- Convention de repere : inversion de Y ---\n"
        "        # Le nuage LiDAR est publie dans la convention MAIN\n"
        "        # GAUCHE de CARLA. Le repere que RTAB-Map en deduit est\n"
        "        # donc le miroir lateral du repere 'map' de\n"
        "        # candidate_generator et obstacle_detector, qui\n"
        "        # appliquent -(y - origin_y). localization_evaluator\n"
        "        # compense deja ce miroir avant de calculer l'ATE\n"
        "        # (est_l['y'] = -est_l['y']) ; la compensation manquait\n"
        "        # ici, si bien que les criteres geometriques comparaient\n"
        "        # deux reperes differents.\n"
        "        #\n"
        "        # Effet mesure avant correction : la distance annoncee\n"
        "        # vers le candidat retenu depassait la longueur de\n"
        "        # l'itineraire routier reel dans 26 decisions sur 29, ce\n"
        "        # qui est geometriquement impossible. L'erreur valait le\n"
        "        # double de l'ecart lateral du vehicule a son axe de\n"
        "        # depart : nulle au demarrage, croissante ensuite,\n"
        "        # retombant a chaque retour vers cet axe.\n"
        "        self.current_y = -msg.pose.pose.position.y\n",
    ),

    # ------------------------------------------------------------------
    # 2. Le gain d'information : la grille vient de RTAB-Map
    # ------------------------------------------------------------------
    (
        "            info_gain = compute_information_gain(\n"
        "                self.current_grid,\n"
        "                c.x,\n"
        "                c.y,\n"
        "                sensor_range=self.sensor_range\n"
        "            )\n",

        "            # /grid_prob_map est publiee par RTAB-Map, donc dans\n"
        "            # SON repere (miroir lateral, cf. on_pose). C'est\n"
        "            # donc la coordonnee du candidat qu'on y transpose.\n"
        "            # Sans cela, la grille etait interrogee a une position\n"
        "            # miroir, presque toujours hors de la zone deja\n"
        "            # cartographiee, donc jugee inexploree : le critere\n"
        "            # saturait a 1.0 pour la quasi-totalite des candidats\n"
        "            # et ne discriminait plus rien.\n"
        "            info_gain = compute_information_gain(\n"
        "                self.current_grid,\n"
        "                c.x,\n"
        "                -c.y,\n"
        "                sensor_range=self.sensor_range\n"
        "            )\n",
    ),

    # ------------------------------------------------------------------
    # 3. Le potentiel de fermeture de boucle : le graphe aussi
    # ------------------------------------------------------------------
    (
        "            loop_closure = compute_loop_closure_potential(\n"
        "                self.current_map_graph,\n"
        "                c.x,\n"
        "                c.y,\n"
        "                self.current_node_id\n"
        "            )\n",

        "            # /mapGraph vient egalement de RTAB-Map : meme\n"
        "            # transposition que pour la grille. Le graphe etait\n"
        "            # jusqu'ici interroge loin de ses noeuds reels, d'ou\n"
        "            # un potentiel de fermeture de boucle bloque autour\n"
        "            # de 0.10 sur toute la campagne.\n"
        "            loop_closure = compute_loop_closure_potential(\n"
        "                self.current_map_graph,\n"
        "                c.x,\n"
        "                -c.y,\n"
        "                self.current_node_id\n"
        "            )\n",
    ),
]


def main():
    if not os.path.exists(CIBLE):
        raise SystemExit("Fichier introuvable : %s" % CIBLE)

    src = io.open(CIBLE, encoding="utf-8").read()

    if "self.current_y = -msg.pose.pose.position.y" in src:
        raise SystemExit(
            "Le correctif semble deja applique (on_pose inverse deja Y).\n"
            "Aucune modification effectuee."
        )

    for ancre, remplacement in BLOCS:
        n = src.count(ancre)
        if n != 1:
            raise SystemExit(
                "ANCRE TROUVEE %d FOIS (attendu 1) :\n---\n%s\n---\n"
                "Aucune modification effectuee." % (n, ancre[:200])
            )
        src = src.replace(ancre, remplacement)

    sauvegarde = "%s.bak_miroir_%s" % (CIBLE, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(CIBLE, sauvegarde)
    io.open(CIBLE, "w", encoding="utf-8").write(src)

    print("3 blocs corriges dans decision_maker.py")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Verifier, puis reconstruire :")
    print("  python3 -m py_compile %s" % CIBLE)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    return 0


if __name__ == "__main__":
    sys.exit(main())
