#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fait travailler les trois noeuds sur LA MEME carte CARLA, et ferme la
derniere voie de recapture de l'origine.

LE FAIT MESURE
--------------
Run valid_bfs.log. Meme instant, meme point de depart :

    [candidate_generator] 23 destinations candidates ATTEIGNABLES
                          publiees (distance routiere 32-248m)

    [vehicle_controller]  DIAG echec : aucun itineraire vers
                          (108.21,-35.74) | aretes=52 noeuds=53
                          | plus proche visite=(107.3,-3.4)
                          a 32.4m du but (tolerance=4.0m)
    [vehicle_controller]  ... plus proche visite=(99.3,-3.5) a 48.2m
    [vehicle_controller]  ... plus proche visite=(83.3,-3.7) a 59.3m

Tous les waypoints visites par le controleur sont a y ~ -3.5 : il
n'explore qu'une seule rue, en ligne droite, sans jamais tourner. Les
buts, eux, sont a y = -35, -51, -63 -- dans des rues perpendiculaires
que le generateur atteint en 32 a 248 m de route.

Les deux noeuds ne parcourent pas le meme reseau routier.

LA CAUSE
--------
Constructeur de candidate_generator :

    self.world = self.client.get_world()
    self.carla_map = self.world.get_map()      # dans __init__

get_map() met en cache l'OpenDRIVE de l'episode COURANT a l'instant de
l'appel. Ce noeud demarre avant que bridge_active_slam n'ait recharge
la ville : la carte conservee est celle de l'episode PRECEDENT, et elle
le reste tout le run.

Le controleur, lui, rafraichit deja monde et carte dans
_try_find_ego_vehicle -- c'est le correctif fait pour le decalage de
117 m (PROJECTION ABERRANTE). Le meme defaut n'avait jamais ete corrige
dans le generateur ni dans le detecteur.

C'est ce qui explique l'intermittence observee toute la journee : quand
CARLA redemarre deja sur Town03_Opt, les deux cartes coincident et le
run se deroule normalement ; quand l'episode precedent etait different,
le generateur propose des destinations qui n'existent pas dans la ville
courante, le controleur les rejette toutes, elles entrent une a une
dans unreachable_points, et le run meurt a 0 m.

Sept runs sur dix passaient, trois echouaient. Ce n'etait pas du
hasard : c'etait l'etat de CARLA au demarrage des noeuds.

CE QUE FAIT LE PATCH
--------------------
1. GENERATEUR et DETECTEUR : monde ET carte rafraichis juste avant
   chaque recherche d'ego. La derniere lecture est necessairement
   posterieure au spawn de l'ego, donc posterieure au rechargement de
   la ville.

2. LES TROIS NOEUDS journalisent 'Carte active : <nom>' au moment ou
   ils trouvent leur ego. Un simple grep prouve desormais qu'ils
   partagent la meme carte -- ou revele immediatement qu'ils ne la
   partagent pas. Ce defaut a coute une journee parce que rien ne le
   signalait.

3. CONTROLEUR : _try_find_ego_vehicle ecrasait l'origine SANS
   CONDITION a chaque fois qu'il trouvait un ego. C'etait une seconde
   voie de recapture, qui contournait le verrou pose dans la garde
   EGO PERIME : relacher l'ego suffisait a deplacer le repere 'map' en
   cours de run. La capture est desormais conditionnee a
   'if self.origin_x is None', comme dans les deux autres noeuds.

VERIFICATION
------------
    bash ~/valider.sh carte
    grep "Carte active" ~/valid_carte.log
    grep -c "DIAG echec" ~/valid_carte.log

Attendu : trois lignes 'Carte active', identiques, portant le nom de la
ville du run ; et zero ou tres peu de 'DIAG echec'.

Si les trois noms different encore, le rechargement de ville par le
pont survient APRES la premiere recherche d'ego, et il faudra faire
attendre les noeuds plutot que rafraichir.

Usage :
    python3 patch_carte_partagee.py
"""

import io
import os
import shutil
import sys
import time

BASE = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/"
    "active_slam_decision/"
)
CONTROLEUR = BASE + "vehicle_controller_active_slam.py"
GENERATEUR = BASE + "candidate_generator.py"
DETECTEUR = BASE + "obstacle_detector.py"


# ---------------------------------------------------------------------
# 1. Generateur et detecteur : rafraichir monde + carte avant recherche
# ---------------------------------------------------------------------
ANCRE_RECHERCHE = (
    "        if self.ego_vehicle is None:\n"
    "            self.ego_vehicle = self._find_ego_vehicle()\n"
    "            if self.ego_vehicle is None:\n"
    "                self.get_logger().warn(\"Vehicule ego introuvable dans CARLA, en attente...\")\n"
    "                return\n"
)

REMPLACEMENT_RECHERCHE = (
    "        if self.ego_vehicle is None:\n"
    "            # --- Rafraichir le monde ET la carte avant de chercher ---\n"
    "            # self.carla_map est capture dans le constructeur, donc\n"
    "            # AVANT que bridge_active_slam n'ait recharge la ville.\n"
    "            # get_map() met en cache l'OpenDRIVE de l'episode courant a\n"
    "            # l'instant de l'appel : la carte conservee est alors celle\n"
    "            # de l'episode PRECEDENT, et elle le reste tout le run.\n"
    "            #\n"
    "            # Mesure sur valid_bfs.log : ce noeud annoncait 23\n"
    "            # destinations atteignables en 32-248 m de route pendant que\n"
    "            # le controleur -- qui rafraichit deja sa carte dans\n"
    "            # _try_find_ego_vehicle -- n'explorait qu'une chaine de 53\n"
    "            # waypoints le long d'une seule rue, a y ~ -3.5, et ne\n"
    "            # trouvait aucun itineraire. Les deux noeuds parcouraient\n"
    "            # deux reseaux routiers differents.\n"
    "            #\n"
    "            # La derniere lecture faite ici est necessairement\n"
    "            # posterieure au spawn de l'ego, donc au rechargement de la\n"
    "            # ville.\n"
    "            try:\n"
    "                self.world = self.client.get_world()\n"
    "                self.carla_map = self.world.get_map()\n"
    "            except Exception as e:\n"
    "                self.get_logger().warn(\n"
    "                    f\"Rafraichissement du monde CARLA impossible : {e}\"\n"
    "                )\n"
    "            self.ego_vehicle = self._find_ego_vehicle()\n"
    "            if self.ego_vehicle is None:\n"
    "                self.get_logger().warn(\"Vehicule ego introuvable dans CARLA, en attente...\")\n"
    "                return\n"
    "            # Trace permanente : permet de verifier d'un grep que les\n"
    "            # trois noeuds partagent bien la meme carte.\n"
    "            self.get_logger().info(\n"
    "                f\"Carte active : {self.carla_map.name}\"\n"
    "            )\n"
)


# ---------------------------------------------------------------------
# 2. Controleur : capture d'origine conditionnelle + carte journalisee
# ---------------------------------------------------------------------
ANCRE_CTRL = (
    "        for actor in self.world.get_actors().filter('vehicle.*'):\n"
    "            if actor.attributes.get('role_name') == self.role_name:\n"
    "                self.ego_vehicle = actor\n"
    "                ego_transform = actor.get_transform()\n"
    "                loc = ego_transform.location\n"
    "                self.origin_x = loc.x\n"
    "                self.origin_y = loc.y\n"
    "                self.origin_yaw = math.radians(-ego_transform.rotation.yaw)\n"
    "                self.get_logger().info(\n"
    "                    f\"Vehicule ego trouve, origine CARLA->map capturee : \"\n"
    "                    f\"({self.origin_x:.2f}, {self.origin_y:.2f})\"\n"
    "                )\n"
    "                self.get_logger().info(\"Vehicule ego trouve, demarrage du controle\")\n"
    "                return\n"
)

REMPLACEMENT_CTRL = (
    "        for actor in self.world.get_actors().filter('vehicle.*'):\n"
    "            if actor.attributes.get('role_name') == self.role_name:\n"
    "                self.ego_vehicle = actor\n"
    "                ego_transform = actor.get_transform()\n"
    "                loc = ego_transform.location\n"
    "                # --- Capture CONDITIONNELLE de l'origine ---\n"
    "                # Cette affectation etait inconditionnelle : c'etait une\n"
    "                # seconde voie de recapture, qui contournait le verrou\n"
    "                # pose dans la garde EGO PERIME. Il suffisait que ce\n"
    "                # noeud relache son ego en cours de run pour que le\n"
    "                # repere 'map' se deplace ici, silencieusement.\n"
    "                #\n"
    "                # L'origine ne dit pas ou est le vehicule : elle DEFINIT\n"
    "                # le repere 'map', que RTAB-Map fige a son premier scan.\n"
    "                if self.origin_x is None:\n"
    "                    self.origin_x = loc.x\n"
    "                    self.origin_y = loc.y\n"
    "                    self.origin_yaw = math.radians(\n"
    "                        -ego_transform.rotation.yaw\n"
    "                    )\n"
    "                    self.get_logger().info(\n"
    "                        f\"Vehicule ego trouve, origine CARLA->map \"\n"
    "                        f\"capturee : ({self.origin_x:.2f}, \"\n"
    "                        f\"{self.origin_y:.2f})\"\n"
    "                    )\n"
    "                else:\n"
    "                    self.get_logger().info(\n"
    "                        f\"Nouvel ego trouve, origine du repere 'map' \"\n"
    "                        f\"conservee : ({self.origin_x:.2f}, \"\n"
    "                        f\"{self.origin_y:.2f})\"\n"
    "                    )\n"
    "                # Trace permanente : permet de verifier d'un grep que les\n"
    "                # trois noeuds partagent bien la meme carte.\n"
    "                self.get_logger().info(\n"
    "                    f\"Carte active : {self.carla_map.name}\"\n"
    "                )\n"
    "                self.get_logger().info(\"Vehicule ego trouve, demarrage du controle\")\n"
    "                return\n"
)


def remplacer(src, ancre, remplacement, nom, fichier):
    n = src.count(ancre)
    if n != 1:
        raise SystemExit(
            "ANCRE '%s' TROUVEE %d FOIS (attendu 1) dans %s.\n"
            "Aucun fichier n'a ete modifie."
            % (nom, n, os.path.basename(fichier))
        )
    return src.replace(ancre, remplacement)


def main():
    for chemin in (CONTROLEUR, GENERATEUR, DETECTEUR):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    if "Carte active" in io.open(GENERATEUR, encoding="utf-8").read():
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    resultats = {}

    src = io.open(CONTROLEUR, encoding="utf-8").read()
    resultats[CONTROLEUR] = remplacer(
        src, ANCRE_CTRL, REMPLACEMENT_CTRL, "origine", CONTROLEUR
    )

    for chemin in (GENERATEUR, DETECTEUR):
        src = io.open(chemin, encoding="utf-8").read()
        resultats[chemin] = remplacer(
            src, ANCRE_RECHERCHE, REMPLACEMENT_RECHERCHE, "recherche", chemin
        )

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in resultats.items():
        sauvegarde = "%s.bak_carte_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("")
    print("Monde et carte rafraichis dans le generateur et le detecteur.")
    print("Capture d'origine du controleur rendue conditionnelle.")
    print("Ligne 'Carte active' ajoutee dans les trois noeuds.")
    print("")
    print("Verifier puis reconstruire :")
    for chemin in (CONTROLEUR, GENERATEUR, DETECTEUR):
        print("  python3 -m py_compile %s" % chemin)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis valider :")
    print("  bash ~/valider.sh carte")
    print("  grep 'Carte active' ~/valid_carte.log")
    return 0


if __name__ == "__main__":
    sys.exit(main())
