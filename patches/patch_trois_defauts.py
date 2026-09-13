#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Corrige les trois defauts restants de la boucle decision <-> controleur.

Les trois sont etablis par le code lui-meme ou par une trace d'execution,
aucun par inference.


DEFAUT 1 -- LE CHEMIN DE REJET EST MORT
---------------------------------------
decision_maker.on_goal_rejected() enregistrait le point inatteignable et
s'arretait la :

    def on_goal_rejected(self, msg):
        self.unreachable_points.append(...)

choose_next_goal() n'etant appele que depuis on_arrived() et depuis la
toute premiere reception de candidats, un but rejete figeait le vehicule
pour tout le reste du run : current_goal restait a None et le controleur
freinait indefiniment.

Observe en direct sur un run Town01_Opt :

    17:47:12  BUT REJETE (aucun itineraire routier) (total rejetes=1)
    ... plus aucune decision pendant tout le run, pose figee a (0.3, -0.05)

Le commentaire de vehicle_controller.on_goal() affirmait pourtant que
"decision_maker s'abonne desormais a /active_slam/goal_rejected et y
reagit en choisissant une nouvelle destination". Le commentaire decrivait
l'intention, pas le code.


DEFAUT 2 -- UN BUT ABANDONNE ETAIT COMPTE COMME UNE DESTINATION ATTEINTE
------------------------------------------------------------------------
A l'abandon, le controleur publiait /vehicle/arrived. Le filtre de
decision_maker.on_arrived() ne rejette que les republications survenant
MOINS D'UNE SECONDE apres la publication du but ; un abandon arrive apres
~70 s. Il passait donc le filtre et incrementait destinations_reached.

C'est le meme defaut que celui deja corrige pour les buts non routables,
reapparu sur le chemin des abandons. Consequence directe sur la campagne
comparative : une strategie qui s'enlise souvent affiche PLUS de
destinations atteintes qu'une strategie qui roule. La metrique s'inverse.

L'abandon publie desormais sur /active_slam/goal_rejected : le point
entre dans unreachable_points, une nouvelle destination est demandee, et
le compteur n'est pas incremente.


DEFAUT 3 -- LA MANOEUVRE DE DEGAGEMENT EST UNE OSCILLATION
----------------------------------------------------------
    self.recovery_steer = max(-1.0, min(1.0, -self.last_steer))

Le braquage du recul etait l'oppose du dernier braquage. Or quand le
vehicule pousse droit dans un mur, last_steer vaut ~0 : il reculait tout
droit, repartait tout droit, et retrouvait le meme obstacle. Trace du run
de chauffe, distance au but apres chaque recul :

    28.2 -> 25.0    28.1 -> 25.0    28.5 -> 25.0

Trois fois la meme valeur au decimetre. Ce n'est pas un vehicule qui
essaie et echoue, c'est un cycle deterministe. Il fallait attendre
no_progress_timeout (45 s) et 4 a 5 enlisements pour en sortir.

Deux corrections : le braquage ALTERNE pleine butee d'un cote puis de
l'autre, ce qui garantit une geometrie differente a chaque tentative ; et
le but est abandonne apres max_recoveries_per_goal tentatives (2 par
defaut) au lieu d'attendre l'expiration du chien de garde.


EFFET DE BORD A CONNAITRE
-------------------------
unreachable_exclusion_radius vaut 25 m et unreachable_points n'oublie
jamais. Chaque abandon exclut donc definitivement les candidats situes
dans un rayon de 25 m autour du point abandonne. C'est le comportement
voulu -- un point ou le vehicule s'enlise ne doit pas etre rechoisi --
mais sur un run tres long le vivier de candidats se reduit. Le nombre
d'abandons est journalise : c'est la grandeur a surveiller.


Usage :
    python3 patch_trois_defauts.py

Les deux fichiers sont calcules AVANT toute ecriture : si l'un n'est pas
patchable, l'autre n'est pas modifie, sinon le paquet se retrouverait
dans un etat ou le controleur publie un rejet que le noeud de decision ne
sait pas encore traiter.
"""

import io
import os
import shutil
import sys
import time

CONTROLEUR = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/"
    "active_slam_decision/vehicle_controller_active_slam.py"
)
DECISION = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/"
    "active_slam_decision/decision_maker.py"
)


CONTROLEUR_BLOCS = [
    # ------------------------------------------------------------------
    # 1. Parametre : nombre de desenlisements admis par but
    # ------------------------------------------------------------------
    (
        "        self.declare_parameter('recovery_stall_time', 6.0)\n"
        "        self.declare_parameter('recovery_duration', 2.5)\n"
        "        self.declare_parameter('recovery_speed_threshold', 0.3)\n"
        "        self.declare_parameter('recovery_throttle', 0.4)\n",

        "        self.declare_parameter('recovery_stall_time', 6.0)\n"
        "        self.declare_parameter('recovery_duration', 2.5)\n"
        "        self.declare_parameter('recovery_speed_threshold', 0.3)\n"
        "        self.declare_parameter('recovery_throttle', 0.4)\n"
        "        # Nombre de desenlisements tentes sur un MEME but avant de\n"
        "        # renoncer. Deux tentatives, l'une braquee a gauche l'autre\n"
        "        # a droite, suffisent a etablir que le but n'est pas\n"
        "        # atteignable depuis cette position. Au-dela on ne fait\n"
        "        # qu'user le budget de temps du run.\n"
        "        self.declare_parameter('max_recoveries_per_goal', 2)\n",
    ),

    # ------------------------------------------------------------------
    # 2. Lecture du parametre
    # ------------------------------------------------------------------
    (
        "        self.recovery_throttle = self.get_parameter('recovery_throttle').value\n",

        "        self.recovery_throttle = self.get_parameter('recovery_throttle').value\n"
        "        self.max_recoveries_per_goal = int(\n"
        "            self.get_parameter('max_recoveries_per_goal').value\n"
        "        )\n",
    ),

    # ------------------------------------------------------------------
    # 3. Etat : message du but courant + compteur par but
    # ------------------------------------------------------------------
    (
        "        # Etat du desenlisement\n"
        "        self.stall_start = None       # debut de l'immobilite en cours\n"
        "        self.recovery_until = 0.0     # instant de fin du recul en cours\n"
        "        self.recovery_steer = 0.0\n"
        "        self.recovery_count = 0\n",

        "        # Etat du desenlisement\n"
        "        self.stall_start = None       # debut de l'immobilite en cours\n"
        "        self.recovery_until = 0.0     # instant de fin du recul en cours\n"
        "        self.recovery_steer = 0.0\n"
        "        self.recovery_count = 0\n"
        "        # Desenlisements tentes sur le but COURANT, remis a zero a\n"
        "        # chaque nouvelle destination.\n"
        "        self.goal_recovery_count = 0\n"
        "        # Message d'origine du but courant, conserve pour pouvoir le\n"
        "        # republier sur /active_slam/goal_rejected en cas d'abandon.\n"
        "        # Il est exprime dans le repere 'map' des candidats, celui\n"
        "        # qu'attend decision_maker.\n"
        "        self.current_goal_msg = None\n",
    ),

    # ------------------------------------------------------------------
    # 4. Memorisation du but et remise a zero du compteur
    # ------------------------------------------------------------------
    (
        "        self.current_goal = (x_carla, y_carla)\n"
        "        self.route = route\n"
        "        self.route_index = 0\n"
        "        self.route_length = route_length\n"
        "        self.has_arrived_published = False\n"
        "        self.goal_start_time = time.time()\n"
        "        self.last_progress_time = time.time()\n"
        "        self.best_distance_to_goal = float('inf')\n"
        "        self.obstacle_wait_start = None\n"
        "        self.last_steer = 0.0\n",

        "        self.current_goal = (x_carla, y_carla)\n"
        "        self.current_goal_msg = msg\n"
        "        self.route = route\n"
        "        self.route_index = 0\n"
        "        self.route_length = route_length\n"
        "        self.has_arrived_published = False\n"
        "        self.goal_start_time = time.time()\n"
        "        self.last_progress_time = time.time()\n"
        "        self.best_distance_to_goal = float('inf')\n"
        "        self.obstacle_wait_start = None\n"
        "        self.last_steer = 0.0\n"
        "        self.goal_recovery_count = 0\n",
    ),

    # ------------------------------------------------------------------
    # 5. Abandon : publier un REJET, pas une arrivee
    # ------------------------------------------------------------------
    (
        "            self._apply_control(throttle=0.0, steer=0.0, brake=0.6)\n"
        "            self.current_goal = None\n"
        "            self.route = []\n"
        "            self.last_steer = 0.0\n"
        "            if not self.has_arrived_published:\n"
        "                self.arrived_pub.publish(Bool(data=True))\n"
        "                self.has_arrived_published = True\n"
        "            return\n",

        "            self._apply_control(throttle=0.0, steer=0.0, brake=0.6)\n"
        "            self.current_goal = None\n"
        "            self.route = []\n"
        "            self.last_steer = 0.0\n"
        "\n"
        "            # --- Un abandon est un REJET, pas une arrivee ---\n"
        "            # Cette branche publiait /vehicle/arrived. Le filtre de\n"
        "            # decision_maker.on_arrived ne rejette que les\n"
        "            # republications survenant moins d'une seconde apres la\n"
        "            # publication du but ; un abandon arrive apres ~70 s. Il\n"
        "            # passait donc le filtre et incrementait\n"
        "            # destinations_reached.\n"
        "            #\n"
        "            # Consequence sur la campagne comparative : une strategie\n"
        "            # qui s'enlise souvent affichait PLUS de destinations\n"
        "            # atteintes qu'une strategie qui roule. La metrique\n"
        "            # s'inversait.\n"
        "            #\n"
        "            # On publie donc sur /active_slam/goal_rejected, comme\n"
        "            # pour un but non routable : le point entre dans\n"
        "            # unreachable_points, une nouvelle destination est\n"
        "            # demandee, et le compteur n'est pas incremente.\n"
        "            if self.current_goal_msg is not None:\n"
        "                self.goal_rejected_pub.publish(self.current_goal_msg)\n"
        "            elif not self.has_arrived_published:\n"
        "                # Repli : sans message d'origine, on retombe sur\n"
        "                # l'ancien comportement plutot que de laisser le\n"
        "                # vehicule sans destination.\n"
        "                self.arrived_pub.publish(Bool(data=True))\n"
        "                self.has_arrived_published = True\n"
        "            return\n",
    ),

    # ------------------------------------------------------------------
    # 6. Desenlisement : braquage alterne + abandon apres N tentatives
    # ------------------------------------------------------------------
    (
        "        if speed < self.recovery_speed_threshold:\n"
        "            if self.stall_start is None:\n"
        "                self.stall_start = now\n"
        "            elif now - self.stall_start > self.recovery_stall_time:\n"
        "                self.recovery_count += 1\n"
        "                self.recovery_steer = max(-1.0, min(1.0, -self.last_steer))\n"
        "                self.recovery_until = now + self.recovery_duration\n"
        "                self.stall_start = None\n"
        "                self.get_logger().warn(\n"
        "                    f\"ENLISEMENT DETECTE : vitesse={speed:.2f}m/s pendant \"\n"
        "                    f\"{self.recovery_stall_time:.0f}s | recul de \"\n"
        "                    f\"{self.recovery_duration:.1f}s \"\n"
        "                    f\"(total desenlisements={self.recovery_count})\"\n"
        "                )\n"
        "                self._apply_control(\n"
        "                    throttle=self.recovery_throttle,\n"
        "                    steer=self.recovery_steer,\n"
        "                    brake=0.0,\n"
        "                    reverse=True\n"
        "                )\n"
        "                return\n"
        "        else:\n"
        "            self.stall_start = None\n",

        "        if speed < self.recovery_speed_threshold:\n"
        "            if self.stall_start is None:\n"
        "                self.stall_start = now\n"
        "            elif now - self.stall_start > self.recovery_stall_time:\n"
        "                self.recovery_count += 1\n"
        "                self.goal_recovery_count += 1\n"
        "\n"
        "                # --- Braquage ALTERNE ---\n"
        "                # La version precedente prenait l'oppose du dernier\n"
        "                # braquage : recovery_steer = -last_steer. Or quand\n"
        "                # le vehicule pousse droit dans un mur, last_steer\n"
        "                # vaut ~0 : il reculait tout droit, repartait tout\n"
        "                # droit, et retrouvait le meme obstacle. Trace du run\n"
        "                # de chauffe, distance au but apres chaque recul :\n"
        "                # 28.2 -> 25.0, 28.1 -> 25.0, 28.5 -> 25.0. Trois fois\n"
        "                # la meme valeur : un cycle deterministe, pas des\n"
        "                # tentatives.\n"
        "                #\n"
        "                # Alterner pleine butee d'un cote puis de l'autre est\n"
        "                # la seule facon de garantir que chaque tentative part\n"
        "                # d'une geometrie differente de la precedente.\n"
        "                self.recovery_steer = (\n"
        "                    0.9 if (self.recovery_count % 2 == 1) else -0.9\n"
        "                )\n"
        "                self.recovery_until = now + self.recovery_duration\n"
        "                self.stall_start = None\n"
        "                self.get_logger().warn(\n"
        "                    f\"ENLISEMENT DETECTE : vitesse={speed:.2f}m/s pendant \"\n"
        "                    f\"{self.recovery_stall_time:.0f}s | recul de \"\n"
        "                    f\"{self.recovery_duration:.1f}s, braquage=\"\n"
        "                    f\"{self.recovery_steer:+.1f} \"\n"
        "                    f\"(tentative {self.goal_recovery_count}/\"\n"
        "                    f\"{self.max_recoveries_per_goal} sur ce but, \"\n"
        "                    f\"total desenlisements={self.recovery_count})\"\n"
        "                )\n"
        "                self._apply_control(\n"
        "                    throttle=self.recovery_throttle,\n"
        "                    steer=self.recovery_steer,\n"
        "                    brake=0.0,\n"
        "                    reverse=True\n"
        "                )\n"
        "\n"
        "                # --- Abandon anticipe ---\n"
        "                # Sans ce test il faut attendre no_progress_timeout\n"
        "                # (45 s), soit 4 a 5 enlisements au meme endroit, pour\n"
        "                # renoncer. Le recul en cours se termine quand meme :\n"
        "                # control_loop traite le desenlisement AVANT le test\n"
        "                # de but.\n"
        "                if self.goal_recovery_count >= self.max_recoveries_per_goal:\n"
        "                    self.abandoned_goals += 1\n"
        "                    self.get_logger().warn(\n"
        "                        f\"BUT ABANDONNE (enlisements repetes) : \"\n"
        "                        f\"{self.goal_recovery_count} desenlisements \"\n"
        "                        f\"sans progres \"\n"
        "                        f\"(total abandonnes={self.abandoned_goals})\"\n"
        "                    )\n"
        "                    self.current_goal = None\n"
        "                    self.route = []\n"
        "                    self.last_steer = 0.0\n"
        "                    if self.current_goal_msg is not None:\n"
        "                        self.goal_rejected_pub.publish(\n"
        "                            self.current_goal_msg\n"
        "                        )\n"
        "                return\n"
        "        else:\n"
        "            self.stall_start = None\n",
    ),
]


DECISION_BLOCS = [
    # ------------------------------------------------------------------
    # 7. Le rejet doit relancer la decision
    # ------------------------------------------------------------------
    (
        "    def on_goal_rejected(self, msg):\n"
        "        self.unreachable_points.append(\n"
        "            (msg.pose.position.x, msg.pose.position.y)\n"
        "        )\n",

        "    def on_goal_rejected(self, msg):\n"
        "        self.unreachable_points.append(\n"
        "            (msg.pose.position.x, msg.pose.position.y)\n"
        "        )\n"
        "\n"
        "        # --- Relance de la decision ---\n"
        "        # Ce rappel se contentait d'enregistrer le point : il ne\n"
        "        # redemandait AUCUNE nouvelle destination. choose_next_goal()\n"
        "        # n'etant appele que depuis on_arrived() et depuis la toute\n"
        "        # premiere reception de candidats, un but rejete figeait le\n"
        "        # vehicule jusqu'a l'expiration du budget de duree :\n"
        "        # current_goal restait a None et le controleur freinait.\n"
        "        #\n"
        "        # Observe sur un run Town01_Opt : \"BUT REJETE (aucun\n"
        "        # itineraire routier)\" a t+2 s, puis plus une seule decision\n"
        "        # de tout le run, pose figee a (0.3, -0.05).\n"
        "        #\n"
        "        # Le compteur de destinations n'est PAS incremente : un but\n"
        "        # ecarte n'est pas une destination atteinte. C'est ce qui\n"
        "        # distingue ce chemin de on_arrived(), et ce qui garantit que\n"
        "        # le budget reste identique d'une strategie a l'autre.\n"
        "        if self.exploration_done:\n"
        "            return\n"
        "\n"
        "        self.consecutive_rejections += 1\n"
        "        self.get_logger().warn(\n"
        "            f\"BUT ECARTE : ({msg.pose.position.x:.1f}, \"\n"
        "            f\"{msg.pose.position.y:.1f}) marque inatteignable \"\n"
        "            f\"({len(self.unreachable_points)} au total, \"\n"
        "            f\"{self.consecutive_rejections} rejets consecutifs). \"\n"
        "            f\"Nouvelle destination demandee.\"\n"
        "        )\n"
        "\n"
        "        # Garde-fou : si plus aucun candidat n'est routable, mieux\n"
        "        # vaut terminer le run proprement que boucler jusqu'au budget.\n"
        "        if self.consecutive_rejections >= 20:\n"
        "            self._finish_run(\n"
        "                'no_reachable_goal',\n"
        "                \"20 buts consecutifs ecartes par le planificateur \"\n"
        "                \"routier\"\n"
        "            )\n"
        "            return\n"
        "\n"
        "        self.choose_next_goal()\n",
    ),
]


def appliquer(chemin, blocs):
    """Calcule le nouveau contenu sans rien ecrire."""
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
    for chemin in (CONTROLEUR, DECISION):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    src_ctrl = io.open(CONTROLEUR, encoding="utf-8").read()
    if "max_recoveries_per_goal" in src_ctrl:
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    # Les DEUX fichiers sont calcules avant toute ecriture : si le second
    # n'est pas patchable, le premier ne doit pas avoir ete modifie, sinon
    # le controleur publierait un rejet que le noeud de decision ne sait
    # pas encore traiter -- et le vehicule serait fige exactement comme
    # avant le correctif.
    resultats = {
        CONTROLEUR: appliquer(CONTROLEUR, CONTROLEUR_BLOCS),
        DECISION: appliquer(DECISION, DECISION_BLOCS),
    }

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in resultats.items():
        sauvegarde = "%s.bak_3defauts_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("\n7 blocs corriges dans 2 fichiers.")
    print("\nVerifier, puis reconstruire :")
    print("  python3 -m py_compile %s" % CONTROLEUR)
    print("  python3 -m py_compile %s" % DECISION)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    return 0


if __name__ == "__main__":
    sys.exit(main())
