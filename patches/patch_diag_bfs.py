#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
INSTRUMENTATION TEMPORAIRE : pourquoi le controleur ne trouve pas un
itineraire que le generateur declare atteignable.

LA CONTRADICTION
----------------
Run valid_obstacles.log, meme instant, meme point de depart
wp=(227.30,-1.59) road=1156 lane=-1 jonction=True :

    [candidate_generator] 23 destinations candidates ATTEIGNABLES
                          publiees (distance routiere 32-248m)

    [vehicle_controller]  DIAG echec : aucun itineraire vers
                          (108.21,-35.74) | aretes=52

Les deux noeuds parcourent le graphe routier CARLA depuis le MEME
waypoint, avec le MEME pas de 8 m, la MEME cle de deduplication
(_wp_key : position arrondie a 2 m, road_id, lane_id) et une portee
comparable (250 m cote generateur, 300 m cote controleur).

Ils ne peuvent pas avoir raison tous les deux.

Consequence : les candidats les plus proches sont rejetes un a un,
entrent dans unreachable_points, et le decideur finit par ne plus
disposer que de buts lointains -- d'ou les 17 rejets et le run termine
a 0 m. Le choix du decideur n'est pas en cause ; ce sont les rejets
initiaux.

CE QUE LE PATCH AJOUTE
----------------------
Le controleur retient, pendant son parcours, le waypoint le plus proche
du but qu'il a REELLEMENT visite, et le journalise en cas d'echec :

    DIAG echec : aucun itineraire vers (108.21,-35.74)
               | aretes=52 noeuds=53 arete_max=8.94m
               | plus proche visite=(112.4,-33.1) a 3.9m du but

Deux lectures, et une seule sera vraie :

  - distance de l'ordre du metre
        Le waypoint est atteint, mais la comparaison echoue. La
        tolerance route_goal_tolerance (4 m) est alors trop juste, ou
        la conversion map -> CARLA introduit un decalage. Correctif :
        relever la tolerance, ou corriger la conversion.

  - distance de plusieurs dizaines de metres
        Le but n'est pas sur le graphe que le controleur explore, alors
        que le generateur l'y a trouve. C'est le parcours du controleur
        qui est en cause -- vraisemblablement une branche de carrefour
        perdue au depart, le vehicule spawnant DANS un carrefour
        (jonction=True). Correctif : dans le parcours, pas dans les
        seuils.

Le nombre de noeuds visites (len(parents)) est ajoute a cote du nombre
d'aretes : leur ecart dit si le graphe explore est une simple chaine
(noeuds ~ aretes) ou un vrai reseau ramifie (aretes > noeuds).

A RETIRER APRES DIAGNOSTIC
--------------------------
Comme les autres DIAG, par patch inverse et non par restauration de
sauvegarde -- restaurer effacerait les correctifs appliques depuis.

VERIFICATION
------------
    bash ~/valider.sh bfs
    grep "DIAG echec" ~/valid_bfs.log | tail -5

Si le run se deroule normalement (le defaut est intermittent), aucune
ligne DIAG echec n'apparaitra : relancer jusqu'a reproduire le
blocage, ou lire le resultat sur un run qui echoue.

Usage :
    python3 patch_diag_bfs.py
"""

import io
import os
import shutil
import sys
import time

CIBLE = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/"
    "active_slam_decision/vehicle_controller_active_slam.py"
)


# ---------------------------------------------------------------------
# 1. Suivi du waypoint visite le plus proche du but
# ---------------------------------------------------------------------
ANCRE_BOUCLE = (
    "        nb_aretes = 0\n"
    "        arete_max = 0.0\n"
    "\n"
    "        while queue:\n"
    "            wp, key, dist = queue.popleft()\n"
    "            loc = wp.transform.location\n"
    "\n"
    "            if math.hypot(goal_x - loc.x, goal_y - loc.y) <= self.route_goal_tolerance:\n"
)

REMPLACEMENT_BOUCLE = (
    "        nb_aretes = 0\n"
    "        arete_max = 0.0\n"
    "        # Waypoint le plus proche du but reellement visite. Sert a\n"
    "        # departager deux causes d'echec : le but est atteint mais la\n"
    "        # comparaison echoue (distance de l'ordre du metre), ou le but\n"
    "        # n'est pas sur le graphe explore (plusieurs dizaines de\n"
    "        # metres) alors que le generateur l'y a trouve.\n"
    "        meilleur_d = float('inf')\n"
    "        meilleur_pt = (0.0, 0.0)\n"
    "\n"
    "        while queue:\n"
    "            wp, key, dist = queue.popleft()\n"
    "            loc = wp.transform.location\n"
    "\n"
    "            d_but = math.hypot(goal_x - loc.x, goal_y - loc.y)\n"
    "            if d_but < meilleur_d:\n"
    "                meilleur_d = d_but\n"
    "                meilleur_pt = (loc.x, loc.y)\n"
    "\n"
    "            if d_but <= self.route_goal_tolerance:\n"
)


# ---------------------------------------------------------------------
# 2. Journal d'echec enrichi
# ---------------------------------------------------------------------
ANCRE_ECHEC = (
    "        self.get_logger().warn(\n"
    "            f\"DIAG echec : aucun itineraire vers \"\n"
    "            f\"({goal_x:.2f},{goal_y:.2f}) | \"\n"
    "            f\"aretes={nb_aretes} arete_max={arete_max:.2f}m\"\n"
    "        )\n"
)

REMPLACEMENT_ECHEC = (
    "        self.get_logger().warn(\n"
    "            f\"DIAG echec : aucun itineraire vers \"\n"
    "            f\"({goal_x:.2f},{goal_y:.2f}) | \"\n"
    "            f\"aretes={nb_aretes} noeuds={len(parents)} \"\n"
    "            f\"arete_max={arete_max:.2f}m | \"\n"
    "            f\"plus proche visite=\"\n"
    "            f\"({meilleur_pt[0]:.1f},{meilleur_pt[1]:.1f}) \"\n"
    "            f\"a {meilleur_d:.1f}m du but \"\n"
    "            f\"(tolerance={self.route_goal_tolerance:.1f}m)\"\n"
    "        )\n"
)


def main():
    if not os.path.exists(CIBLE):
        raise SystemExit("Fichier introuvable : %s" % CIBLE)

    src = io.open(CIBLE, encoding="utf-8").read()

    if "plus proche visite" in src:
        raise SystemExit(
            "L'instrumentation semble deja en place.\n"
            "Aucune modification effectuee."
        )

    for nom, ancre in (("boucle", ANCRE_BOUCLE), ("echec", ANCRE_ECHEC)):
        n = src.count(ancre)
        if n != 1:
            raise SystemExit(
                "ANCRE '%s' TROUVEE %d FOIS (attendu 1).\n"
                "Aucune modification effectuee." % (nom, n)
            )

    src = src.replace(ANCRE_BOUCLE, REMPLACEMENT_BOUCLE)
    src = src.replace(ANCRE_ECHEC, REMPLACEMENT_ECHEC)

    sauvegarde = "%s.bak_diagbfs_%s" % (CIBLE, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(CIBLE, sauvegarde)
    io.open(CIBLE, "w", encoding="utf-8").write(src)

    print("DIAG echec enrichi dans vehicle_controller_active_slam.py")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % CIBLE)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis :")
    print("  bash ~/valider.sh bfs")
    print("  grep 'DIAG echec' ~/valid_bfs.log | tail -5")
    return 0


if __name__ == "__main__":
    sys.exit(main())
