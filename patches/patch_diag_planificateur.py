#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Instrumentation TEMPORAIRE du planificateur d'itineraire.

Ce patch n'apporte aucune correction. Il ajoute uniquement des traces,
pour faire temoigner le programme la ou le raisonnement ne suffit plus.

POURQUOI
--------
Le journal du run annonce, pour la premiere destination :

    depart  : (227.26, -1.59)      (origine capturee par le controleur)
    but     : (108.20, -35.70)
    ligne droite                   : 123.85 m
    itineraire annonce             : 9 waypoints, 32.0 m

Un trajet routier ne peut pas etre plus court que la ligne droite, et
8 sauts de route_step=4.0 ne peuvent pas couvrir 124 m.

Rejoue hors ligne, avec le meme code, les memes parametres et le meme
point de depart, le meme BFS ne trouve AUCUN itineraire -- ni au cout du
projet, ni au cout geometrique reel. Les verifications suivantes ont
toutes ete faites :

  - route_step, route_goal_tolerance, route_max_distance et
    waypoint_reach_tolerance ne sont surcharges nulle part dans
    active_slam.launch.py : 4.0 / 4.0 / 250.0 / 3.0 ;
  - le fichier execute (build/) est identique au fichier source ;
  - wp.next(4.0) franchit 3.66 a 4.55 m, jamais davantage : aucune
    arete ne "saute" ;
  - le point de spawn hors ligne coincide a 0.00 m pres avec l'origine
    relevee dans le journal.

Une seule chose n'a pas pu etre observee : le waypoint de depart que le
controleur obtient reellement a l'execution. Hors ligne, la projection
tombe sur road=1156, lane=-1, DANS UNE JONCTION -- un waypoint dont
next() n'offre presque aucune ramification (75 aretes explorees en tout,
la ou une carte urbaine devrait en offrir des centaines).

CE QUE LE PATCH AJOUTE
----------------------
Deux traces par appel a _plan_route() :

  DIAG depart : coordonnees du waypoint de depart projete, son road_id,
                son lane_id, s'il est dans une jonction, la position
                brute du vehicule, le but et les parametres effectifs ;

  DIAG trouve : nombre de waypoints, longueur ANNONCEE (celle que le
                projet calcule) et longueur MESUREE (la geometrie
                reelle), premier et dernier waypoint, distance du
                dernier waypoint au but, nombre d'aretes explorees et
                longueur de la plus grande.

  DIAG echec  : idem en cas d'absence d'itineraire.

A RETIRER APRES DIAGNOSTIC
--------------------------
    cp <fichier>.bak_diag_<horodatage> <fichier>
    cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install

Usage :
    python3 patch_diag_planificateur.py
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

ANCRE = """        start_key = self._wp_key(start_wp)
        parents = {start_key: (start_wp, None)}
        queue = deque([(start_wp, start_key, 0.0)])

        while queue:
            wp, key, dist = queue.popleft()
            loc = wp.transform.location

            if math.hypot(goal_x - loc.x, goal_y - loc.y) <= self.route_goal_tolerance:
                return self._rebuild_route(parents, key), dist

            if dist + self.route_step > self.route_max_distance:
                continue

            for nxt in wp.next(self.route_step):
                nkey = self._wp_key(nxt)
                if nkey in parents:
                    continue
                parents[nkey] = (nxt, key)
                queue.append((nxt, nkey, dist + self.route_step))

        return None, float('inf')
"""

REMPLACEMENT = """        start_key = self._wp_key(start_wp)
        parents = {start_key: (start_wp, None)}
        queue = deque([(start_wp, start_key, 0.0)])

        # --- INSTRUMENTATION TEMPORAIRE (a retirer apres diagnostic) ---
        # Le journal annonce des itineraires de 32 m vers des buts situes
        # a 124 m. Rejoue hors ligne avec le meme code, les memes
        # parametres et le meme point de depart, ce BFS ne trouve aucun
        # itineraire. Le code execute a ete verifie identique au source.
        # Reste a savoir sur quel waypoint la projection tombe REELLEMENT
        # a l'execution, et ce que le chemin reconstruit vaut
        # geometriquement.
        swl = start_wp.transform.location
        self.get_logger().warn(
            f"DIAG depart : wp=({swl.x:.2f},{swl.y:.2f},{swl.z:.2f}) "
            f"road={start_wp.road_id} lane={start_wp.lane_id} "
            f"jonction={start_wp.is_junction} | "
            f"ego=({start_loc.x:.2f},{start_loc.y:.2f},{start_loc.z:.2f}) | "
            f"but=({goal_x:.2f},{goal_y:.2f}) "
            f"droite={math.hypot(goal_x - start_loc.x, goal_y - start_loc.y):.1f}m"
            f" | step={self.route_step} tol={self.route_goal_tolerance} "
            f"max={self.route_max_distance}"
        )
        nb_aretes = 0
        arete_max = 0.0

        while queue:
            wp, key, dist = queue.popleft()
            loc = wp.transform.location

            if math.hypot(goal_x - loc.x, goal_y - loc.y) <= self.route_goal_tolerance:
                route = self._rebuild_route(parents, key)
                pts = [
                    (w.transform.location.x, w.transform.location.y)
                    for w in route
                ]
                # Longueur GEOMETRIQUE, par opposition a 'dist' qui vaut
                # (nombre de sauts) x route_step sans jamais rien mesurer.
                mesuree = sum(
                    math.hypot(pts[i + 1][0] - pts[i][0],
                               pts[i + 1][1] - pts[i][1])
                    for i in range(len(pts) - 1)
                )
                ecart_max = max(
                    (
                        math.hypot(pts[i + 1][0] - pts[i][0],
                                   pts[i + 1][1] - pts[i][1])
                        for i in range(len(pts) - 1)
                    ),
                    default=0.0
                )
                self.get_logger().warn(
                    f"DIAG trouve : {len(route)} wp | "
                    f"annoncee={dist:.1f}m mesuree={mesuree:.1f}m "
                    f"ecart_max={ecart_max:.2f}m | "
                    f"debut=({pts[0][0]:.2f},{pts[0][1]:.2f}) "
                    f"fin=({pts[-1][0]:.2f},{pts[-1][1]:.2f}) "
                    f"fin_au_but="
                    f"{math.hypot(goal_x - pts[-1][0], goal_y - pts[-1][1]):.2f}m"
                    f" | aretes={nb_aretes} arete_max={arete_max:.2f}m"
                )
                return route, dist

            if dist + self.route_step > self.route_max_distance:
                continue

            for nxt in wp.next(self.route_step):
                nkey = self._wp_key(nxt)
                if nkey in parents:
                    continue
                nloc = nxt.transform.location
                saut = math.hypot(nloc.x - loc.x, nloc.y - loc.y)
                nb_aretes += 1
                if saut > arete_max:
                    arete_max = saut
                parents[nkey] = (nxt, key)
                queue.append((nxt, nkey, dist + self.route_step))

        self.get_logger().warn(
            f"DIAG echec : aucun itineraire vers "
            f"({goal_x:.2f},{goal_y:.2f}) | "
            f"aretes={nb_aretes} arete_max={arete_max:.2f}m"
        )
        return None, float('inf')
"""


def main():
    if not os.path.exists(CIBLE):
        raise SystemExit("Fichier introuvable : %s" % CIBLE)

    src = io.open(CIBLE, encoding="utf-8").read()

    if "DIAG depart" in src:
        raise SystemExit(
            "L'instrumentation est deja en place.\n"
            "Aucune modification effectuee."
        )

    n = src.count(ANCRE)
    if n != 1:
        raise SystemExit(
            "ANCRE TROUVEE %d FOIS (attendu 1).\n"
            "Le corps de _plan_route() ne correspond pas a celui attendu.\n"
            "Aucune modification effectuee." % n
        )

    src = src.replace(ANCRE, REMPLACEMENT)

    sauvegarde = "%s.bak_diag_%s" % (CIBLE, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(CIBLE, sauvegarde)
    io.open(CIBLE, "w", encoding="utf-8").write(src)

    print("Instrumentation inseree dans _plan_route().")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Verifier, reconstruire, puis lancer un run court :")
    print("  python3 -m py_compile %s" % CIBLE)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("  ros2 launch active_slam_decision active_slam.launch.py \\")
    print("      strategy:=weighted max_distance:=150.0 run_duration:=120.0 \\")
    print("      > ~/diag_run.log 2>&1")
    print("")
    print("  grep DIAG ~/diag_run.log")
    print("")
    print("Pour retirer l'instrumentation ensuite :")
    print("  cp %s %s" % (os.path.basename(sauvegarde), os.path.basename(CIBLE)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
