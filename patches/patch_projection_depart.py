#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Corrige la projection du point de depart du planificateur d'itineraire.

LE DEFAUT
---------
vehicle_controller_active_slam capture la carte dans son CONSTRUCTEUR :

    self.carla_map = self.world.get_map()

get_map() analyse et met en cache l'OpenDRIVE de l'episode courant a cet
instant. Or bridge_active_slam appelle load_world(town) quand la carte
chargee n'est pas la bonne -- ce qui est le cas au tout premier run apres
un demarrage de CARLA, la carte par defaut n'etant pas Town03_Opt. La
carte conservee par le controleur est donc celle de l'episode PRECEDENT,
et elle le reste pour tout le run.

get_actors(), lui, interroge la simulation en direct : le vehicule ego
est bien trouve, et rien ne signale l'incoherence.

LA MESURE
---------
Trace du run instrumente (30/08, t=1788096223) :

    ego     = (227.26, -1.59, -0.01)
    wp      = (109.88,  -2.39,  0.00)   road=566 lane=-2 jonction=True
    but     = (108.21, -35.74)          droite = 123.9 m
    itineraire : 9 wp, annoncee=32.0 m, mesuree=33.5 m, fin_au_but=0.00 m

Le waypoint de depart est a 117 m du vehicule. L'itineraire lui-meme est
correct -- il mesure bien 33,5 m et se termine exactement sur le but --
mais il commence ailleurs.

CE QUE CELA EXPLIQUAIT
----------------------
  - 32 m d'itineraire annonces vers un but a 123,9 m : les deux valeurs
    etaient justes, elles ne partaient pas du meme point ;
  - cap_err ~ 0 deg pendant 85 m : le vehicule visait route[0], situe a
    117 m devant lui, dans le meme axe ;
  - route_index bloque a 0 pendant 85 m : il devait parcourir ces 117 m
    avant de passer a moins de waypoint_reach_tolerance du premier
    waypoint ;
  - le virage a 48 deg puis le mur : arrive la-bas a 8,6 m/s, l'itineraire
    bifurquait brutalement ;
  - "c'est toujours le premier run" : c'est le seul ou la carte est
    rechargee pendant le demarrage des noeuds.

LE CORRECTIF
------------
1. Rafraichir self.world et self.carla_map a chaque tentative de
   recherche de l'ego, donc APRES le rechargement de carte et jamais
   avant.
2. Controler la projection a chaque planification : si le waypoint de
   depart est a plus de max_projection_error du vehicule, recharger la
   carte et reessayer ; si l'ecart persiste, rejeter le but plutot que de
   suivre un itineraire qui commence ailleurs.

Le point 2 n'est pas une redondance du point 1 : il transforme un defaut
silencieux en defaut visible. Si la situation se represente pour une
autre raison, elle apparaitra dans le journal au lieu d'envoyer le
vehicule dans un mur.

Ce patch s'applique par-dessus l'instrumentation DIAG, qu'il ne touche
pas.

Usage :
    python3 patch_projection_depart.py
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


BLOCS = [
    # ------------------------------------------------------------------
    # 1. Declaration du parametre
    # ------------------------------------------------------------------
    (
        "        # Planification d'itineraire\n"
        "        self.declare_parameter('route_step', 4.0)\n"
        "        self.declare_parameter('route_max_distance', 250.0)\n"
        "        self.declare_parameter('route_goal_tolerance', 4.0)\n",

        "        # Planification d'itineraire\n"
        "        self.declare_parameter('route_step', 4.0)\n"
        "        self.declare_parameter('route_max_distance', 250.0)\n"
        "        self.declare_parameter('route_goal_tolerance', 4.0)\n"
        "\n"
        "        # Ecart maximal admis entre le vehicule et le waypoint sur\n"
        "        # lequel get_waypoint() projette sa position. Une voie CARLA\n"
        "        # fait environ 3,5 m de large : au-dela de 10 m, la\n"
        "        # projection ne designe plus la chaussee sous le vehicule.\n"
        "        self.declare_parameter('max_projection_error', 10.0)\n",
    ),

    # ------------------------------------------------------------------
    # 2. Lecture du parametre
    # ------------------------------------------------------------------
    (
        "        self.route_step = self.get_parameter('route_step').value\n"
        "        self.route_max_distance = self.get_parameter('route_max_distance').value\n"
        "        self.route_goal_tolerance = self.get_parameter('route_goal_tolerance').value\n",

        "        self.route_step = self.get_parameter('route_step').value\n"
        "        self.route_max_distance = self.get_parameter('route_max_distance').value\n"
        "        self.route_goal_tolerance = self.get_parameter('route_goal_tolerance').value\n"
        "        self.max_projection_error = self.get_parameter(\n"
        "            'max_projection_error'\n"
        "        ).value\n",
    ),

    # ------------------------------------------------------------------
    # 3. Rafraichissement du monde et de la carte
    # ------------------------------------------------------------------
    (
        "    def _try_find_ego_vehicle(self):\n"
        "        self.search_attempts += 1\n"
        "        for actor in self.world.get_actors().filter('vehicle.*'):\n",

        "    def _try_find_ego_vehicle(self):\n"
        "        self.search_attempts += 1\n"
        "\n"
        "        # --- Rafraichissement du monde et de la carte ---\n"
        "        # self.carla_map est capture dans le constructeur, donc\n"
        "        # AVANT que bridge_active_slam n'ait eventuellement appele\n"
        "        # load_world(town). get_map() met en cache l'OpenDRIVE de\n"
        "        # l'episode courant a l'instant de l'appel : au premier run\n"
        "        # apres un demarrage de CARLA, la carte conservee est celle\n"
        "        # de l'episode precedent, et elle le reste tout le run.\n"
        "        #\n"
        "        # get_actors() interrogeant la simulation en direct, l'ego\n"
        "        # etait bien trouve et rien ne signalait l'incoherence. Seule\n"
        "        # la projection trahissait le probleme : ego=(227.26,-1.59)\n"
        "        # projete sur un waypoint situe a (109.88,-2.39), soit 117 m\n"
        "        # plus loin. L'itineraire calcule etait correct, mais il\n"
        "        # commencait a 117 m du vehicule.\n"
        "        #\n"
        "        # On rafraichit donc les deux tant que l'ego n'est pas\n"
        "        # trouve : la derniere lecture est necessairement posterieure\n"
        "        # au spawn, donc au rechargement de carte.\n"
        "        try:\n"
        "            self.world = self.client.get_world()\n"
        "            self.carla_map = self.world.get_map()\n"
        "        except RuntimeError as e:\n"
        "            self.get_logger().warn(\n"
        "                f\"Rafraichissement du monde impossible : {e}\"\n"
        "            )\n"
        "            return\n"
        "\n"
        "        for actor in self.world.get_actors().filter('vehicle.*'):\n",
    ),

    # ------------------------------------------------------------------
    # 4. Controle de la projection a chaque planification
    # ------------------------------------------------------------------
    (
        "        start_loc = self.ego_vehicle.get_location()\n"
        "        start_wp = self.carla_map.get_waypoint(start_loc, project_to_road=True)\n"
        "        if start_wp is None:\n"
        "            return None, float('inf')\n",

        "        start_loc = self.ego_vehicle.get_location()\n"
        "        start_wp = self.carla_map.get_waypoint(start_loc, project_to_road=True)\n"
        "        if start_wp is None:\n"
        "            return None, float('inf')\n"
        "\n"
        "        # --- Controle de la projection du point de depart ---\n"
        "        # get_waypoint() doit rendre un waypoint situe SOUS le\n"
        "        # vehicule. Quand la carte detenue par ce noeud n'est pas\n"
        "        # celle de l'episode courant, elle rend un point arbitraire.\n"
        "        # L'itineraire produit est alors geometriquement valide mais\n"
        "        # commence ailleurs : le vehicule ne peut pas le suivre,\n"
        "        # route_index reste bloque, et la visee anticipee finit par\n"
        "        # retomber sur le but EN LIGNE DROITE, a travers les murs.\n"
        "        #\n"
        "        # Ce controle est redondant avec le rafraichissement fait\n"
        "        # dans _try_find_ego_vehicle. C'est voulu : il transforme un\n"
        "        # defaut silencieux en defaut visible. Si la situation se\n"
        "        # represente pour une autre raison, elle apparaitra dans le\n"
        "        # journal au lieu d'envoyer le vehicule dans un mur.\n"
        "        ecart = math.hypot(\n"
        "            start_wp.transform.location.x - start_loc.x,\n"
        "            start_wp.transform.location.y - start_loc.y,\n"
        "        )\n"
        "        if ecart > self.max_projection_error:\n"
        "            self.get_logger().warn(\n"
        "                f\"PROJECTION ABERRANTE : waypoint de depart a \"\n"
        "                f\"{ecart:.1f} m du vehicule \"\n"
        "                f\"(seuil {self.max_projection_error:.1f} m). \"\n"
        "                f\"Rechargement de la carte et nouvel essai.\"\n"
        "            )\n"
        "            try:\n"
        "                self.world = self.client.get_world()\n"
        "                self.carla_map = self.world.get_map()\n"
        "            except RuntimeError as e:\n"
        "                self.get_logger().error(\n"
        "                    f\"Rechargement de la carte impossible : {e}\"\n"
        "                )\n"
        "                return None, float('inf')\n"
        "\n"
        "            start_wp = self.carla_map.get_waypoint(\n"
        "                start_loc, project_to_road=True\n"
        "            )\n"
        "            if start_wp is None:\n"
        "                return None, float('inf')\n"
        "\n"
        "            ecart = math.hypot(\n"
        "                start_wp.transform.location.x - start_loc.x,\n"
        "                start_wp.transform.location.y - start_loc.y,\n"
        "            )\n"
        "            if ecart > self.max_projection_error:\n"
        "                self.get_logger().error(\n"
        "                    f\"PROJECTION TOUJOURS ABERRANTE ({ecart:.1f} m) : \"\n"
        "                    f\"but rejete. Suivre un itineraire commencant \"\n"
        "                    f\"ailleurs enverrait le vehicule hors de la route.\"\n"
        "                )\n"
        "                return None, float('inf')\n"
        "\n"
        "            self.get_logger().info(\n"
        "                f\"Carte rechargee, projection revenue a {ecart:.1f} m.\"\n"
        "            )\n",
    ),
]


def main():
    if not os.path.exists(CIBLE):
        raise SystemExit("Fichier introuvable : %s" % CIBLE)

    src = io.open(CIBLE, encoding="utf-8").read()

    if "max_projection_error" in src:
        raise SystemExit(
            "Le correctif semble deja applique "
            "(max_projection_error est deja declare).\n"
            "Aucune modification effectuee."
        )

    for ancre, remplacement in BLOCS:
        n = src.count(ancre)
        if n != 1:
            raise SystemExit(
                "ANCRE TROUVEE %d FOIS (attendu 1) :\n---\n%s\n---\n"
                "Aucune modification effectuee." % (n, ancre[:300])
            )
        src = src.replace(ancre, remplacement)

    sauvegarde = "%s.bak_projection_%s" % (
        CIBLE, time.strftime("%Y%m%d_%H%M%S")
    )
    shutil.copy2(CIBLE, sauvegarde)
    io.open(CIBLE, "w", encoding="utf-8").write(src)

    print("4 blocs corriges dans vehicle_controller_active_slam.py")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Verifier, puis reconstruire :")
    print("  python3 -m py_compile %s" % CIBLE)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    return 0


if __name__ == "__main__":
    sys.exit(main())
