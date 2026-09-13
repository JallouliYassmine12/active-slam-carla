#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fige DEFINITIVEMENT l'origine du repere 'map', et repare le
rafraichissement du monde CARLA.

CE QUI A ETE MESURE
-------------------
Run valid_granularite.log. Les origines journalisees par
obstacle_detector, dans l'ordre :

    (227.26, -1.59)   <- correcte, au spawn
    (224.00, -1.63)
    (216.36, -1.75)
    (206.07, -1.90)
    (193.68, -2.09)
    (178.61, -2.31)
    (163.37, -2.54)
    (158.08, -2.62)

L'origine SUIT le vehicule. Elle recule de 227 a 158 m au fil du run.

Consequences mesurees sur le meme run :

    OBSTACLE DEVANT ....... 1
    OBSTACLE PROCHE ....... 1
    collisions ............ 4

Une seule detection tactique pour quatre collisions. Et au moment de la
premiere :

    POURSUITE : dist_but=54.0m cap_err=0deg vitesse=5.10m/s steer=0.01
    EVENEMENT DE SECURITE : collision vehicule

Cap aligne, volant a 0.01 : le vehicule roulait tout droit et a percute
une voiture qu'il ne voyait pas.

LA CHAINE DE CAUSES
-------------------
Trois defauts s'enchainent, et le premier est dans le constructeur :

    client = carla.Client(host, port)      # variable LOCALE
    self.world = client.get_world()

1. Le client n'est pas conserve. self.world est donc fige sur
   l'episode CARLA en cours a la construction du noeud -- donc AVANT le
   rechargement de carte fait par bridge_active_slam.

2. Les acteurs tires d'un episode perime rapportent is_alive == False
   alors que le vehicule est bien vivant. La garde EGO PERIME se
   declenche donc en permanence : 61 fois en 118 s. Le rafraichissement
   que j'avais ajoute pour tarir ce faux positif etait protege par
   hasattr(self, 'client') -- toujours faux ici, donc jamais execute.

3. A chaque declenchement, l'origine etait remise a None puis
   recapturee a la position COURANTE du vehicule.

Les obstacles etaient donc publies dans un repere glissant, tandis que
le controleur les reconvertissait avec sa propre origine restee fixe :
ils atterrissaient jusqu'a 70 m a cote de leur position reelle.

Ce n'est pas seulement un probleme de conduite. obstacle_detector
alimente aussi le CRITERE DE SECURITE du decision_maker, l'un des cinq
criteres evalues par le projet : il notait les candidats par rapport a
des obstacles fantomes, sans qu'aucun message ne le signale.

POURQUOI MON CORRECTIF PRECEDENT N'A PAS TENU
---------------------------------------------
patch_origine_figee.py autorisait la recapture pendant les 30 premieres
secondes de vie du noeud, en supposant le vehicule immobile pendant ce
delai. Les horodatages le dementent : la derive commence a t+10 s. La
fenetre reposait sur une hypothese non verifiee.

LA CORRECTION
-------------
1. Le client est conserve dans self.client, et self.world est rafraichi
   avant chaque recherche d'ego -- ce que le controleur fait depuis
   longtemps dans _try_find_ego_vehicle, et qui avait deja regle le
   defaut de la carte perimee.

2. L'origine n'est PLUS JAMAIS reprise. Aucune fenetre, aucune
   exception. Elle ne dit pas ou est le vehicule : elle definit le
   repere 'map', que RTAB-Map fige a son premier scan.

   La course au demarrage que la fenetre cherchait a couvrir est deja
   traitee ailleurs, par deux gardes permanentes et instrumentees : le
   rafraichissement de la carte dans _try_find_ego_vehicle, et le
   controle PROJECTION ABERRANTE dans _plan_route.

VERIFICATION
------------
    bash ~/valider.sh obstacles

    grep "Origine CARLA->map capturee" ~/valid_obstacles.log
    grep -c "EGO PERIME" ~/valid_obstacles.log
    grep -c "OBSTACLE DEVANT" ~/valid_obstacles.log
    grep -c "EVENEMENT DE SECURITE" ~/valid_obstacles.log

Attendu : UNE seule ligne d'origine par noeud, identique pour les
trois, proche de (227.26, -1.59) ; des EGO PERIME rarefies ou nuls ; et
un nombre de detections tactiques enfin coherent avec le trafic
rencontre.

Usage :
    python3 patch_origine_definitive.py
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
# 1. Constructeur : conserver le client (generateur et detecteur)
# ---------------------------------------------------------------------
ANCRE_CLIENT = (
    "        client = carla.Client(host, port)\n"
    "        client.set_timeout(10.0)\n"
    "        self.world = client.get_world()\n"
)

REMPLACEMENT_CLIENT = (
    "        # --- Le client est CONSERVE ---\n"
    "        # Sans lui, self.world reste fige sur l'episode CARLA en\n"
    "        # cours a la construction du noeud, donc AVANT le\n"
    "        # rechargement de carte fait par bridge_active_slam. Les\n"
    "        # acteurs tires de cet episode perime rapportent\n"
    "        # is_alive == False alors que le vehicule est bien vivant :\n"
    "        # 61 declenchements de la garde EGO PERIME mesures sur un\n"
    "        # run de 118 s.\n"
    "        self.client = carla.Client(host, port)\n"
    "        self.client.set_timeout(10.0)\n"
    "        self.world = self.client.get_world()\n"
)


# ---------------------------------------------------------------------
# 2. Garde EGO PERIME : generateur et detecteur (bloc identique)
# ---------------------------------------------------------------------
COMMENTAIRE_ORIGINE = (
    "        # --- L'origine n'est JAMAIS reprise ---\n"
    "        # Elle ne dit pas ou est le vehicule : elle DEFINIT le\n"
    "        # repere 'map', que RTAB-Map fige a son premier scan et que\n"
    "        # les trois noeuds doivent partager pour tout le run.\n"
    "        #\n"
    "        # La version precedente autorisait une recapture pendant les\n"
    "        # 30 premieres secondes, en supposant le vehicule immobile\n"
    "        # pendant ce delai. Mesure sur valid_granularite.log :\n"
    "        # l'origine du detecteur derivait des t+10 s, suivant le\n"
    "        # vehicule de (227.26,-1.59) a (158.08,-2.62). Les obstacles\n"
    "        # etaient publies dans un repere glissant, le controleur les\n"
    "        # reconvertissait avec sa propre origine restee fixe, et ils\n"
    "        # atterrissaient jusqu'a 70 m a cote : 1 seule detection\n"
    "        # tactique pour 4 collisions, et un critere de securite\n"
    "        # calcule sur des obstacles fantomes.\n"
    "        #\n"
    "        # La course au demarrage que cette fenetre couvrait est deja\n"
    "        # traitee par deux gardes permanentes : le rafraichissement\n"
    "        # de la carte dans _try_find_ego_vehicle, et le controle\n"
    "        # PROJECTION ABERRANTE dans _plan_route.\n"
)

RAFRAICHISSEMENT = (
    "                # Rafraichir le monde AVANT de rechercher : un\n"
    "                # acteur tire d'un episode perime rapporte\n"
    "                # is_alive == False alors que le vehicule est bien\n"
    "                # vivant. C'est la correction deja en service cote\n"
    "                # controleur (_try_find_ego_vehicle).\n"
    "                try:\n"
    "                    self.world = self.client.get_world()\n"
    "                except Exception as e:\n"
    "                    self.get_logger().warn(\n"
    "                        f\"Rafraichissement du monde CARLA \"\n"
    "                        f\"impossible : {e}\"\n"
    "                    )\n"
)


ANCRE_GARDE_GEN_DET = (
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
    "        if self.ego_vehicle is not None:\n"
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
    "                return\n"
)

REMPLACEMENT_GARDE_GEN_DET = (
    COMMENTAIRE_ORIGINE
    + "        if self.ego_vehicle is not None:\n"
      "            try:\n"
      "                vivant = self.ego_vehicle.is_alive\n"
      "            except RuntimeError:\n"
      "                vivant = False\n"
      "            if not vivant:\n"
      "                self.get_logger().warn(\n"
      "                    \"EGO PERIME : vehicule suivi detruit, nouvel ego \"\n"
      "                    \"recherche. Origine du repere 'map' conservee.\"\n"
      "                )\n"
      "                self.ego_vehicle = None\n"
    + RAFRAICHISSEMENT
    + "                return\n"
)


# ---------------------------------------------------------------------
# 3. Garde EGO PERIME : controleur (resets supplementaires a conserver)
# ---------------------------------------------------------------------
ANCRE_GARDE_CTRL = (
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
    "        if self.ego_vehicle is not None:\n"
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

REMPLACEMENT_GARDE_CTRL = (
    COMMENTAIRE_ORIGINE
    + "        if self.ego_vehicle is not None:\n"
      "            try:\n"
      "                vivant = self.ego_vehicle.is_alive\n"
      "            except RuntimeError:\n"
      "                vivant = False\n"
      "            if not vivant:\n"
      "                self.get_logger().warn(\n"
      "                    \"EGO PERIME : le vehicule suivi a ete detruit. \"\n"
      "                    \"Nouvel ego recherche, origine du repere 'map' \"\n"
      "                    \"conservee.\"\n"
      "                )\n"
      "                self.ego_vehicle = None\n"
      "                try:\n"
      "                    self.world = self.client.get_world()\n"
      "                    self.carla_map = self.world.get_map()\n"
      "                except Exception as e:\n"
      "                    self.get_logger().warn(\n"
      "                        f\"Rafraichissement du monde CARLA \"\n"
      "                        f\"impossible : {e}\"\n"
      "                    )\n"
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

    if "L'origine n'est JAMAIS reprise" in io.open(
        DETECTEUR, encoding="utf-8"
    ).read():
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    resultats = {}

    # Controleur : garde seule. Il conserve deja self.client et
    # rafraichit son monde dans _try_find_ego_vehicle.
    src = io.open(CONTROLEUR, encoding="utf-8").read()
    if "self.client = carla.Client" not in src:
        raise SystemExit(
            "vehicle_controller_active_slam.py ne conserve pas son client "
            "CARLA dans self.client.\nAucun fichier n'a ete modifie."
        )
    src = remplacer(src, ANCRE_GARDE_CTRL, REMPLACEMENT_GARDE_CTRL,
                    "garde", CONTROLEUR)
    resultats[CONTROLEUR] = src

    # Generateur : garde seule. Verification faite sur le source, il
    # conserve deja self.client des la construction :
    #     self.client = carla.Client(host, port)
    #     self.world  = self.client.get_world()
    # C'est ce qui distingue les deux noeuds, et ce qui explique
    # pourquoi seul le detecteur souffrait d'un monde perime.
    src = io.open(GENERATEUR, encoding="utf-8").read()
    if "self.client = carla.Client" not in src:
        raise SystemExit(
            "candidate_generator.py ne conserve pas son client CARLA "
            "dans self.client.\nAucun fichier n'a ete modifie."
        )
    src = remplacer(src, ANCRE_GARDE_GEN_DET,
                    REMPLACEMENT_GARDE_GEN_DET, "garde", GENERATEUR)
    resultats[GENERATEUR] = src

    # Detecteur : constructeur ET garde. C'est le seul des trois qui
    # construit son client en variable LOCALE, donc le seul dont
    # self.world reste fige sur l'episode CARLA d'avant le rechargement
    # de carte -- d'ou ses 61 EGO PERIME et son origine derivante.
    src = io.open(DETECTEUR, encoding="utf-8").read()
    src = remplacer(src, ANCRE_CLIENT, REMPLACEMENT_CLIENT,
                    "client", DETECTEUR)
    src = remplacer(src, ANCRE_GARDE_GEN_DET,
                    REMPLACEMENT_GARDE_GEN_DET, "garde", DETECTEUR)
    resultats[DETECTEUR] = src

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in resultats.items():
        sauvegarde = "%s.bak_origdef_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("")
    print("Origine du repere 'map' : figee definitivement dans les 3 noeuds.")
    print("Client CARLA conserve et monde rafraichi dans les 3 noeuds.")
    print("")
    print("Verifier puis reconstruire :")
    for chemin in (CONTROLEUR, GENERATEUR, DETECTEUR):
        print("  python3 -m py_compile %s" % chemin)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis valider :")
    print("  bash ~/valider.sh obstacles")
    return 0


if __name__ == "__main__":
    sys.exit(main())
