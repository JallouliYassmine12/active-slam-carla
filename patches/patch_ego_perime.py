#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Empeche les noeuds de piloter un vehicule ego DETRUIT.

LE DEFAUT
---------
Horodatages du run valid_grid40.log :

    1788209142.854  [pont]       CARTE ACTIVE : Town03_Opt
    1788209142.879  [controleur] ego trouve, origine (196.08, 5.95)
    1788209147.114  [pont]       Vehicule spawne au point 0 (227.3, -1.6)
    1788209147.310  [generateur] origine (227.26, -1.59)
    1788209147.471  [detecteur]  origine (227.26, -1.59)

Le controleur a capture son origine 4,2 SECONDES AVANT que le pont ne
spawne le vehicule. Il s'est donc accroche a un ego reste d'un run
precedent, a 31 m de la, que le pont a detruit juste apres en nettoyant
les acteurs orphelins.

Depuis, il pilote un acteur mort : get_location() renvoie eternellement
la meme position. Consequences mesurees sur ce run :

  - origine decalee de 31 m -> toutes les conversions de repere fausses
  - 12 candidats au lieu de 23, aucun routable
  - 0 m parcouru, run termine sur no_reachable_goal

Les trois noeuds concernes -- controleur, generateur de candidats,
detecteur d'obstacles -- prennent le premier acteur nomme 'ego' qu'ils
trouvent et ne verifient plus jamais qu'il existe encore. Le controleur
a ete le seul touche ce coup-ci parce qu'il demarre le plus vite ; les
deux autres ont regarde apres le spawn, par chance.

C'est le meme mecanisme que le defaut de la carte perimee : ces noeuds
demarrent avant que le pont n'ait fini son travail, et capturent un etat
qui n'est pas encore le bon.

LA CORRECTION
-------------
Un acteur CARLA detruit expose is_alive == False. Chaque noeud verifie
donc, a chaque cycle, que son ego est toujours vivant. S'il ne l'est
plus, il le relache, remet son origine a None, et en cherche un nouveau
au cycle suivant -- ce qui recapture l'origine sur le bon vehicule.

Le correctif est volontairement identique dans les trois fichiers : le
repere 'map' n'a de sens que si les trois s'accordent sur la meme
origine.

Usage :
    python3 patch_ego_perime.py
"""

import io
import os
import shutil
import sys
import time

BASE = os.path.expanduser("~/active_slam_carla/ros2_ws/src/active_slam_decision/"
                          "active_slam_decision/")
CONTROLEUR = BASE + "vehicle_controller_active_slam.py"
GENERATEUR = BASE + "candidate_generator.py"
DETECTEUR = BASE + "obstacle_detector.py"


CONTROLEUR_BLOCS = [
    (
        "    def control_loop(self):\n"
        "        if self.ego_vehicle is None:\n"
        "            self._try_find_ego_vehicle()\n"
        "            return\n"
        "\n"
        "        self._update_distance()\n",

        "    def control_loop(self):\n"
        "        # --- L'ego detenu est-il toujours vivant ? ---\n"
        "        # Ce noeud demarre plus vite que bridge_active_slam. Quand un\n"
        "        # ego d'un run precedent traine encore dans le simulateur, il\n"
        "        # s'y accroche, puis le pont le detruit en nettoyant les\n"
        "        # acteurs orphelins. Le noeud pilote alors un acteur mort :\n"
        "        # get_location() renvoie toujours la meme position, l'origine\n"
        "        # du repere 'map' est fausse, et le vehicule reel ne bouge\n"
        "        # jamais.\n"
        "        #\n"
        "        # Mesure : origine capturee a (196.08, 5.95) 4,2 s avant que\n"
        "        # le pont ne spawne le vrai vehicule a (227.3, -1.6).\n"
        "        # Resultat du run : 0 m parcouru.\n"
        "        if self.ego_vehicle is not None:\n"
        "            try:\n"
        "                vivant = self.ego_vehicle.is_alive\n"
        "            except RuntimeError:\n"
        "                vivant = False\n"
        "            if not vivant:\n"
        "                self.get_logger().warn(\n"
        "                    \"EGO PERIME : le vehicule suivi a ete detruit \"\n"
        "                    \"(probablement un acteur d'un run precedent, \"\n"
        "                    \"nettoye par le pont). Recherche d'un nouvel ego \"\n"
        "                    \"et recapture de l'origine.\"\n"
        "                )\n"
        "                self.ego_vehicle = None\n"
        "                self.origin_x = None\n"
        "                self.origin_y = None\n"
        "                self.origin_yaw = None\n"
        "                self.current_goal = None\n"
        "                self.current_goal_msg = None\n"
        "                self.route = []\n"
        "                self.last_steer = 0.0\n"
        "                # Sans cela, le premier pas d'integration apres le\n"
        "                # changement d'ego serait le saut entre les deux\n"
        "                # vehicules.\n"
        "                self.last_position = None\n"
        "                return\n"
        "\n"
        "        if self.ego_vehicle is None:\n"
        "            self._try_find_ego_vehicle()\n"
        "            return\n"
        "\n"
        "        self._update_distance()\n",
    ),
]


GENERATEUR_BLOCS = [
    (
        "    def publish_candidates(self):\n"
        "        if self.ego_vehicle is None:\n"
        "            self.ego_vehicle = self._find_ego_vehicle()\n"
        "            if self.ego_vehicle is None:\n"
        "                self.get_logger().warn(\"Vehicule ego introuvable dans CARLA, en attente...\")\n"
        "                return\n",

        "    def publish_candidates(self):\n"
        "        # --- L'ego detenu est-il toujours vivant ? ---\n"
        "        # Meme garde que dans vehicle_controller_active_slam : un ego\n"
        "        # d'un run precedent peut etre trouve avant que le pont ne\n"
        "        # l'ait detruit et remplace. L'origine capturee serait alors\n"
        "        # celle du mauvais vehicule, et TOUS les candidats publies\n"
        "        # vivraient dans un repere decale.\n"
        "        if self.ego_vehicle is not None:\n"
        "            try:\n"
        "                vivant = self.ego_vehicle.is_alive\n"
        "            except RuntimeError:\n"
        "                vivant = False\n"
        "            if not vivant:\n"
        "                self.get_logger().warn(\n"
        "                    \"EGO PERIME : vehicule suivi detruit. Recherche \"\n"
        "                    \"d'un nouvel ego et recapture de l'origine.\"\n"
        "                )\n"
        "                self.ego_vehicle = None\n"
        "                self.origin_x = None\n"
        "                self.origin_y = None\n"
        "                self.origin_yaw = None\n"
        "                return\n"
        "\n"
        "        if self.ego_vehicle is None:\n"
        "            self.ego_vehicle = self._find_ego_vehicle()\n"
        "            if self.ego_vehicle is None:\n"
        "                self.get_logger().warn(\"Vehicule ego introuvable dans CARLA, en attente...\")\n"
        "                return\n",
    ),
]


DETECTEUR_BLOCS = [
    (
        "    def publish_obstacles(self):\n"
        "        if self.ego_vehicle is None:\n"
        "            self.ego_vehicle = self._find_ego_vehicle()\n"
        "            if self.ego_vehicle is None:\n"
        "                self.get_logger().warn(\"Vehicule ego introuvable dans CARLA, en attente...\")\n"
        "                return\n",

        "    def publish_obstacles(self):\n"
        "        # --- L'ego detenu est-il toujours vivant ? ---\n"
        "        # Meme garde que dans les deux autres noeuds. Ici l'enjeu est\n"
        "        # le critere de securite : des obstacles publies dans un\n"
        "        # repere decale de 31 m rendraient la penalisation de securite\n"
        "        # arbitraire, sans qu'aucun message ne le signale.\n"
        "        if self.ego_vehicle is not None:\n"
        "            try:\n"
        "                vivant = self.ego_vehicle.is_alive\n"
        "            except RuntimeError:\n"
        "                vivant = False\n"
        "            if not vivant:\n"
        "                self.get_logger().warn(\n"
        "                    \"EGO PERIME : vehicule suivi detruit. Recherche \"\n"
        "                    \"d'un nouvel ego et recapture de l'origine.\"\n"
        "                )\n"
        "                self.ego_vehicle = None\n"
        "                self.origin_x = None\n"
        "                self.origin_y = None\n"
        "                self.origin_yaw = None\n"
        "                return\n"
        "\n"
        "        if self.ego_vehicle is None:\n"
        "            self.ego_vehicle = self._find_ego_vehicle()\n"
        "            if self.ego_vehicle is None:\n"
        "                self.get_logger().warn(\"Vehicule ego introuvable dans CARLA, en attente...\")\n"
        "                return\n",
    ),
]


def appliquer(chemin, blocs):
    src = io.open(chemin, encoding="utf-8").read()
    for ancre, remplacement in blocs:
        n = src.count(ancre)
        if n != 1:
            raise SystemExit(
                "ANCRE TROUVEE %d FOIS (attendu 1) dans %s :\n---\n%s\n---\n"
                "Aucun fichier n'a ete modifie."
                % (n, os.path.basename(chemin), ancre[:300])
            )
        src = src.replace(ancre, remplacement)
    return src


def main():
    for chemin in (CONTROLEUR, GENERATEUR, DETECTEUR):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    if "EGO PERIME" in io.open(CONTROLEUR, encoding="utf-8").read():
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    resultats = {
        CONTROLEUR: appliquer(CONTROLEUR, CONTROLEUR_BLOCS),
        GENERATEUR: appliquer(GENERATEUR, GENERATEUR_BLOCS),
        DETECTEUR: appliquer(DETECTEUR, DETECTEUR_BLOCS),
    }

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in resultats.items():
        sauvegarde = "%s.bak_ego_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("\n3 gardes inserees dans 3 fichiers.")
    print("\nReconstruire :")
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    return 0


if __name__ == "__main__":
    sys.exit(main())
