#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fige l'origine du repere 'map' apres le demarrage.

CE QUI A ETE MESURE
-------------------
Run valid_portee500.log. Le vehicule roule 275 m, atteint sa
destination, puis 55 buts consecutifs sont rejetes et le run meurt.

    [candidate_generator] Origine CARLA->map capturee : (4.03, -70.27,
                          cap=97.5deg)
    [candidate_generator] 17 destinations candidates ATTEIGNABLES
                          publiees (distance routiere 72-248m)

Cette paire de lignes se repete a CHAQUE cycle, toutes les 6 secondes,
avec une origine differente a chaque fois -- celle de la position
courante du vehicule.

Les 17 candidats sont corrects : 72 a 248 m par la route, tous
atteignables. Mais ils sont exprimes dans un repere ancre en
(4.03, -70.27), alors que le controleur et RTAB-Map travaillent dans le
repere ancre au point de spawn (227.26, -1.59). Le decideur voit
pose=(221.36, -65.76) et des candidats venus d'un autre referentiel.
Reconvertis en coordonnees CARLA par le controleur, ils atterrissent
300 m plus loin, groupes autour de la zone de depart :

    but=(257.10,121.52) droite=317.5m -> DIAG echec, aretes=635

D'ou 55 rejets, 0 nouvelle destination, run termine sur
no_reachable_goal.

L'ORIGINE DU DEFAUT
-------------------
patch_ego_perime.py -- un correctif precedent, qui reglait un vrai
probleme de demarrage. Sa garde fait :

    self.ego_vehicle = None
    self.origin_x = None
    self.origin_y = None
    self.origin_yaw = None

Au demarrage c'est juste : si le noeud s'est accroche a un vehicule
fantome d'un run precedent, l'origine capturee est fausse et doit etre
reprise sur le bon vehicule.

En cours de run c'est faux. L'origine ne decrit pas ou est le vehicule :
elle DEFINIT le repere 'map', celui que RTAB-Map fige a son premier
scan et que les trois noeuds doivent partager pour toute la duree du
run. La redefinir en route deplace le referentiel sous le systeme.

AGGRAVANT : UN FAUX POSITIF
---------------------------
74 'EGO PERIME' en 120 secondes, dans candidate_generator et
obstacle_detector. Or le vehicule est bien vivant : le controleur le
pilote sur 275 m pendant ce temps. is_alive renvoie donc False a tort
dans ces deux noeuds.

Cause probable : ils interrogent un objet 'world' herite d'un episode
precedent. Un handle d'acteur issu d'un episode qui n'est plus celui en
cours rapporte is_alive == False alors que le vehicule existe. Le
controleur n'est pas touche parce qu'il rafraichit deja son monde dans
_try_find_ego_vehicle (patch_projection_depart).

LA CORRECTION
-------------
1. L'origine ne se recapture QUE pendant les 30 premieres secondes de
   vie du noeud. Passe ce delai, l'ego peut etre relache et retrouve,
   mais le repere reste fixe. 30 s couvrent largement la course au
   demarrage mesuree a l'epoque : 4,2 s entre la capture du controleur
   et le spawn par le pont.

2. 'world' et 'carla_map' sont rafraichis avant de rechercher un nouvel
   ego, ce qui tarit le faux positif a sa source.

Le journal distingue desormais les deux cas :

    EGO PERIME : ... , origine recapturee            (demarrage)
    EGO PERIME : ... , origine du repere 'map' CONSERVEE   (en run)

Le correctif est applique aux TROIS noeuds. Le controleur porte la meme
faille dans sa garde ; elle ne s'est pas declenchee sur ce run, mais
rien ne garantit qu'elle ne se declenchera pas sur un autre -- et elle y
serait bien plus destructrice, puisque c'est lui qui conduit.

VERIFICATION
------------
    bash ~/valider.sh origine

Attendu :
  - 'Origine CARLA->map capturee' n'apparait plus qu'une fois par noeud,
    avec la meme valeur pour les trois, proche du point de spawn ;
  - les 'EGO PERIME' se rarefient ou disparaissent ;
  - plusieurs 'DIAG trouve', plusieurs 'Destination atteinte', et une
    fin de run sur 'budget de distance atteint'.

Usage :
    python3 patch_origine_figee.py
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
# Prelude commun : age du noeud, calcule sans toucher a __init__.
# ---------------------------------------------------------------------
PRELUDE = (
    "        # --- Age du noeud ---\n"
    "        # Distingue la course au demarrage (ou l'origine DOIT\n"
    "        # pouvoir etre reprise sur le bon vehicule) de la perte\n"
    "        # d'ego en cours de run (ou elle ne doit surtout pas\n"
    "        # bouger). Initialise ici plutot que dans __init__ pour que\n"
    "        # le correctif reste local a cette methode.\n"
    "        if not hasattr(self, '_t_demarrage'):\n"
    "            self._t_demarrage = self.get_clock().now()\n"
    "        age_noeud = (\n"
    "            self.get_clock().now() - self._t_demarrage\n"
    "        ).nanoseconds / 1e9\n"
    "\n"
)

# ---------------------------------------------------------------------
# Rafraichissement du monde + liberation conditionnelle de l'origine.
# ---------------------------------------------------------------------
CORPS_COMMUN = (
    "                self.ego_vehicle = None\n"
    "                # Un handle d'acteur issu d'un episode CARLA\n"
    "                # precedent rapporte is_alive == False alors que le\n"
    "                # vehicule est bien vivant. Rafraichir le monde tarit\n"
    "                # ce faux positif : 74 declenchements en 120 s ont ete\n"
    "                # observes pendant que le controleur pilotait sans\n"
    "                # difficulte le meme vehicule sur 275 m.\n"
    "                try:\n"
    "                    if hasattr(self, 'client'):\n"
    "                        self.world = self.client.get_world()\n"
    "                        if hasattr(self, 'carla_map'):\n"
    "                            self.carla_map = self.world.get_map()\n"
    "                except Exception as e:\n"
    "                    self.get_logger().warn(\n"
    "                        f\"Rafraichissement du monde CARLA impossible : {e}\"\n"
    "                    )\n"
    "                # --- L'origine ne se recapture QU'AU DEMARRAGE ---\n"
    "                # Elle ne dit pas ou est le vehicule : elle DEFINIT le\n"
    "                # repere 'map', que RTAB-Map fige a son premier scan et\n"
    "                # que les trois noeuds doivent partager pour tout le\n"
    "                # run. La reprendre en cours de route deplace le\n"
    "                # referentiel sous le systeme : mesure sur\n"
    "                # valid_portee500.log, le generateur la recapturait a\n"
    "                # chaque cycle, ses 17 candidats pourtant corrects\n"
    "                # (72-248 m par la route) arrivaient dans un repere\n"
    "                # decale de 300 m, et les 55 buts suivants ont ete\n"
    "                # rejetes.\n"
    "                if age_noeud < 30.0:\n"
    "                    self.origin_x = None\n"
    "                    self.origin_y = None\n"
    "                    self.origin_yaw = None\n"
)


# ---------------------------------------------------------------------
# Generateur et detecteur : bloc identique.
# ---------------------------------------------------------------------
ANCRE_GEN_DET = (
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
)

REMPLACEMENT_GEN_DET = (
    PRELUDE
    + "        if self.ego_vehicle is not None:\n"
      "            try:\n"
      "                vivant = self.ego_vehicle.is_alive\n"
      "            except RuntimeError:\n"
      "                vivant = False\n"
      "            if not vivant:\n"
      "                self.get_logger().warn(\n"
      "                    \"EGO PERIME : vehicule suivi detruit, nouvel ego \"\n"
      "                    \"recherche (%s).\"\n"
      "                    % (\n"
      "                        \"origine recapturee\" if age_noeud < 30.0\n"
      "                        else \"origine du repere 'map' CONSERVEE\"\n"
      "                    )\n"
      "                )\n"
    + CORPS_COMMUN
    + "                return\n"
)


# ---------------------------------------------------------------------
# Controleur : meme faille, resets supplementaires a conserver.
# ---------------------------------------------------------------------
ANCRE_CTRL = (
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
)

REMPLACEMENT_CTRL = (
    PRELUDE
    + "        if self.ego_vehicle is not None:\n"
      "            try:\n"
      "                vivant = self.ego_vehicle.is_alive\n"
      "            except RuntimeError:\n"
      "                vivant = False\n"
      "            if not vivant:\n"
      "                self.get_logger().warn(\n"
      "                    \"EGO PERIME : le vehicule suivi a ete detruit. \"\n"
      "                    \"Nouvel ego recherche (%s).\"\n"
      "                    % (\n"
      "                        \"origine recapturee\" if age_noeud < 30.0\n"
      "                        else \"origine du repere 'map' CONSERVEE\"\n"
      "                    )\n"
      "                )\n"
    + CORPS_COMMUN
    + "                self.current_goal = None\n"
      "                self.current_goal_msg = None\n"
      "                self.route = []\n"
      "                self.last_steer = 0.0\n"
      "                # Sans cela, le premier pas d'integration apres le\n"
      "                # changement d'ego serait le saut entre les deux\n"
      "                # vehicules.\n"
      "                self.last_position = None\n"
      "                return\n"
)


def appliquer(chemin, ancre, remplacement):
    src = io.open(chemin, encoding="utf-8").read()
    n = src.count(ancre)
    if n != 1:
        raise SystemExit(
            "ANCRE TROUVEE %d FOIS (attendu 1) dans %s.\n"
            "Aucun fichier n'a ete modifie."
            % (n, os.path.basename(chemin))
        )
    return src.replace(ancre, remplacement)


def main():
    for chemin in (CONTROLEUR, GENERATEUR, DETECTEUR):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    if "_t_demarrage" in io.open(GENERATEUR, encoding="utf-8").read():
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    resultats = {
        CONTROLEUR: appliquer(CONTROLEUR, ANCRE_CTRL, REMPLACEMENT_CTRL),
        GENERATEUR: appliquer(GENERATEUR, ANCRE_GEN_DET, REMPLACEMENT_GEN_DET),
        DETECTEUR: appliquer(DETECTEUR, ANCRE_GEN_DET, REMPLACEMENT_GEN_DET),
    }

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in resultats.items():
        sauvegarde = "%s.bak_origine_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("\nOrigine du repere 'map' figee apres 30 s dans les 3 noeuds.")
    print("\nVerifier puis reconstruire :")
    for chemin in (CONTROLEUR, GENERATEUR, DETECTEUR):
        print("  python3 -m py_compile %s" % chemin)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("\nPuis valider :")
    print("  bash ~/valider.sh origine")
    return 0


if __name__ == "__main__":
    sys.exit(main())
