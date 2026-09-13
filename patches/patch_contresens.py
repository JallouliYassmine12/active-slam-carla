#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Deux choses :
  A. le vehicule ne tourne plus indefiniment sur un itineraire devenu
     insuivable ;
  B. une mesure decisive sur le gain d'information.


A. ITINERAIRE INSUIVABLE APRES UN DEMI-TOUR
===========================================
Trace mesuree, valid_trajet.log, six cycles consecutifs :

    wp=0/5 cap_err=-138deg vitesse=0.00m/s steer=-0.15
    wp=0/5 cap_err=-123deg vitesse=0.76m/s steer=-1.00
    wp=0/5 cap_err= -87deg vitesse=0.81m/s steer=-1.00
    wp=0/5 cap_err= -63deg vitesse=0.48m/s steer=-1.00
    wp=0/5 cap_err= -44deg vitesse=0.50m/s steer=-0.73
    wp=0/5 cap_err=+156deg vitesse=0.72m/s steer=+1.00
    wp=0/5 cap_err=+124deg vitesse=1.24m/s steer=+1.00

'wp=0/5' sur toute la sequence : le premier waypoint de l'itineraire
n'est jamais atteint. L'erreur de cap traverse +/-180 deg, le volant
saute d'une butee a l'autre, et la distance au but reste bloquee autour
de 32 m. La capture d'ecran correspondante montre le vehicule sur les
voies en sens inverse -- double ligne jaune a sa droite.

LA CAUSE
--------
waypoint.next() ne parcourt le graphe routier que vers l'AVANT. Un
itineraire est donc calcule pour le cap qu'avait le vehicule au moment
ou le but a ete recu. S'il se retrouve ensuite a contresens, cet
itineraire commence derriere lui : il ne peut plus le rejoindre sans
faire demi-tour, ce que la loi de commande ne sait pas orchestrer.

Aucun garde-fou existant ne reagit :

  - le chien de garde de progression compare la distance au but, qui
    varie de quelques centimetres a chaque cycle -- il ne voit pas de
    stagnation franche ;
  - le desenlisement exige une vitesse inferieure a 0,3 m/s pendant 6 s,
    or le vehicule BOUGE : il tourne en rond entre 0,5 et 1,2 m/s.

Le vehicule peut donc tourner sur lui-meme jusqu'a l'expiration du
budget.

LA CORRECTION
-------------
Si route_index reste a 0 -- premier waypoint jamais atteint -- ET que
l'erreur de cap depasse 100 deg pendant plus de wrong_way_timeout
secondes, l'itineraire est declare insuivable : le but est publie sur
/active_slam/goal_rejected.

Le mecanisme existe deja et il est exactement adapte : le point entre
dans unreachable_points, le decideur choisit une autre destination, et
le controleur replanifie depuis la position ET LE CAP ACTUELS. Le
compteur de destinations n'est pas incremente, donc le budget reste
comparable d'une strategie a l'autre.

Les deux conditions sont necessaires. route_index > 0 signifie que le
vehicule suit reellement son chemin ; une erreur de cap passagere
au-dela de 100 deg est normale dans un virage serre.


B. OU LE GAIN D'INFORMATION SATURE
===================================
Apres le passage a l'evaluation le long du trajet, le critere reste a
1.000 pour tous les candidats, alors que la grille est bien recue
(94 releves grid=OK sur 95) et que la pose du vehicule est coherente.

Une mesure tranche : le gain d'information A LA POSITION DU VEHICULE.
Il roule la, donc c'est cartographie, donc il doit valoir ~0.

    gain au vehicule ~ 0
        La grille est interrogee dans le bon repere. Le critere sature
        parce que les trajets traversent reellement du terrain inconnu
        -- ce qui est alors la reponse correcte, et la limite est
        geometrique, pas logicielle.

    gain au vehicule ~ 1
        La grille est interrogee LOIN de la zone cartographiee. La
        conversion de repere est fausse, et elle l'etait depuis le
        debut : tous les releves info_gain de la journee seraient alors
        expliques d'un coup.

La ligne journalisee donne les trois valeurs d'un coup :

    DIAG gain : au vehicule=0.021 | candidat 0 : trajet=0.534
                arrivee=1.000 | grille x=[-1.6,308.9] y=[-5.8,124.0]

TEMPORAIRE : a retirer une fois la question tranchee.

Usage :
    python3 patch_contresens.py
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
DECIDEUR = BASE + "decision_maker.py"


# =====================================================================
# A. Controleur
# =====================================================================
ANCRE_PARAM = "        self.declare_parameter('max_recoveries_per_goal', 2)\n"

REMPLACEMENT_PARAM = (
    "        self.declare_parameter('max_recoveries_per_goal', 2)\n"
    "\n"
    "        # --- Itineraire insuivable (vehicule a contresens) ---\n"
    "        # Duree pendant laquelle le vehicule peut rester sur le premier\n"
    "        # waypoint de son itineraire avec un cap oppose avant que cet\n"
    "        # itineraire ne soit declare insuivable. Huit secondes laissent\n"
    "        # le temps d'une manoeuvre legitime tout en bornant les\n"
    "        # rotations sur place, mesurees jusqu'a l'expiration du budget.\n"
    "        self.declare_parameter('wrong_way_timeout', 8.0)\n"
)


ANCRE_LECTURE = (
    "        self.max_recoveries_per_goal = int(\n"
    "            self.get_parameter('max_recoveries_per_goal').value\n"
    "        )\n"
)

REMPLACEMENT_LECTURE = (
    "        self.max_recoveries_per_goal = int(\n"
    "            self.get_parameter('max_recoveries_per_goal').value\n"
    "        )\n"
    "        self.wrong_way_timeout = float(\n"
    "            self.get_parameter('wrong_way_timeout').value\n"
    "        )\n"
)


ANCRE_ETAT = "        self.last_light_log_time = 0.0\n"

REMPLACEMENT_ETAT = (
    "        self.last_light_log_time = 0.0\n"
    "        # Instant du debut de la situation 'premier waypoint jamais\n"
    "        # atteint, cap oppose'. None tant qu'elle ne dure pas.\n"
    "        self.wrong_way_start = None\n"
)


ANCRE_CAP = (
    "        dx, dy = target_x - vx, target_y - vy\n"
    "        target_heading = math.atan2(dy, dx)\n"
    "        heading_error = self._normalize_angle(target_heading - yaw_rad)\n"
)

REMPLACEMENT_CAP = (
    "        dx, dy = target_x - vx, target_y - vy\n"
    "        target_heading = math.atan2(dy, dx)\n"
    "        heading_error = self._normalize_angle(target_heading - yaw_rad)\n"
    "\n"
    "        # --- Itineraire devenu insuivable ---\n"
    "        # waypoint.next() ne parcourt le graphe routier que vers\n"
    "        # l'AVANT : l'itineraire a ete calcule pour le cap qu'avait le\n"
    "        # vehicule quand le but a ete recu. S'il s'est retrouve a\n"
    "        # contresens, ce chemin commence derriere lui et il ne peut plus\n"
    "        # le rejoindre.\n"
    "        #\n"
    "        # Trace mesuree sur six cycles consecutifs (valid_trajet.log) :\n"
    "        #   wp=0/5 cap_err=-138deg steer=-0.15\n"
    "        #   wp=0/5 cap_err=-123deg steer=-1.00\n"
    "        #   wp=0/5 cap_err= -87deg steer=-1.00\n"
    "        #   wp=0/5 cap_err=+156deg steer=+1.00\n"
    "        # Le vehicule tourne sur lui-meme, la distance au but reste a\n"
    "        # 32 m. Ni le chien de garde de progression (la distance varie\n"
    "        # de quelques centimetres) ni le desenlisement (le vehicule\n"
    "        # bouge, 0,5 a 1,2 m/s) ne reagissent.\n"
    "        #\n"
    "        # Les deux conditions sont necessaires : route_index > 0 signifie\n"
    "        # que le chemin est reellement suivi, et une erreur de cap\n"
    "        # passagere au-dela de 100 deg est normale dans un virage serre.\n"
    "        if self.route_index == 0 and abs(heading_error) > math.radians(100):\n"
    "            if self.wrong_way_start is None:\n"
    "                self.wrong_way_start = now\n"
    "            elif now - self.wrong_way_start > self.wrong_way_timeout:\n"
    "                self.abandoned_goals += 1\n"
    "                self.get_logger().warn(\n"
    "                    f\"ITINERAIRE INSUIVABLE : premier waypoint jamais \"\n"
    "                    f\"atteint et cap oppose \"\n"
    "                    f\"({math.degrees(heading_error):.0f}deg) depuis \"\n"
    "                    f\"{now - self.wrong_way_start:.0f}s. Le vehicule est \"\n"
    "                    f\"probablement a contresens : but rejete et \"\n"
    "                    f\"replanification depuis le cap actuel \"\n"
    "                    f\"(total abandonnes={self.abandoned_goals}).\"\n"
    "                )\n"
    "                self._apply_control(throttle=0.0, steer=0.0, brake=0.6)\n"
    "                self.current_goal = None\n"
    "                self.route = []\n"
    "                self.last_steer = 0.0\n"
    "                self.wrong_way_start = None\n"
    "                if self.current_goal_msg is not None:\n"
    "                    self.goal_rejected_pub.publish(self.current_goal_msg)\n"
    "                return\n"
    "        else:\n"
    "            self.wrong_way_start = None\n"
)


# =====================================================================
# B. Decideur : ou le gain sature
# =====================================================================
ANCRE_DIAG = (
    "        for c in candidates:\n"
    "            distance = cout_distance(c)\n"
)

REMPLACEMENT_DIAG = (
    "        # --- INSTRUMENTATION TEMPORAIRE (a retirer apres diagnostic) ---\n"
    "        # Le gain d'information reste a 1.000 pour tous les candidats,\n"
    "        # y compris depuis le passage a l'evaluation le long du trajet,\n"
    "        # alors que la grille est bien recue. Une mesure tranche : le\n"
    "        # gain A LA POSITION DU VEHICULE. Il roule la, donc c'est\n"
    "        # cartographie, donc il doit valoir ~0.\n"
    "        #\n"
    "        #   ~0 -> la grille est interrogee dans le bon repere, et la\n"
    "        #         saturation est geometrique : les trajets traversent\n"
    "        #         reellement du terrain inconnu.\n"
    "        #   ~1 -> la grille est interrogee loin de la zone cartographiee.\n"
    "        #         La conversion de repere est fausse, et l'etait depuis\n"
    "        #         le debut.\n"
    "        if self.current_grid is not None and candidates:\n"
    "            gi = self.current_grid.info\n"
    "            gain_vehicule = compute_information_gain(\n"
    "                self.current_grid,\n"
    "                self.current_x,\n"
    "                -self.current_y,\n"
    "                self.path_corridor_radius\n"
    "            )\n"
    "            c0 = candidates[0]\n"
    "            gain_arrivee = compute_information_gain(\n"
    "                self.current_grid, c0.x, -c0.y, self.path_corridor_radius\n"
    "            )\n"
    "            gain_trajet = compute_path_information_gain(\n"
    "                self.current_grid,\n"
    "                self.current_x,\n"
    "                -self.current_y,\n"
    "                c0.x,\n"
    "                -c0.y,\n"
    "                sample_step=self.path_sample_step,\n"
    "                corridor_radius=self.path_corridor_radius\n"
    "            )\n"
    "            self.get_logger().warn(\n"
    "                f\"DIAG gain : au vehicule={gain_vehicule:.3f} | \"\n"
    "                f\"candidat 0 : trajet={gain_trajet:.3f} \"\n"
    "                f\"arrivee={gain_arrivee:.3f} | \"\n"
    "                f\"pose=({self.current_x:.1f},{-self.current_y:.1f}) \"\n"
    "                f\"cand=({c0.x:.1f},{-c0.y:.1f}) | \"\n"
    "                f\"grille x=[{gi.origin.position.x:.1f},\"\n"
    "                f\"{gi.origin.position.x + gi.width * gi.resolution:.1f}] \"\n"
    "                f\"y=[{gi.origin.position.y:.1f},\"\n"
    "                f\"{gi.origin.position.y + gi.height * gi.resolution:.1f}]\"\n"
    "            )\n"
    "\n"
    "        for c in candidates:\n"
    "            distance = cout_distance(c)\n"
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
    for chemin in (CONTROLEUR, DECIDEUR):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    src_ctrl = io.open(CONTROLEUR, encoding="utf-8").read()
    src_dec = io.open(DECIDEUR, encoding="utf-8").read()

    if "ITINERAIRE INSUIVABLE" in src_ctrl:
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    src_ctrl = remplacer(src_ctrl, ANCRE_PARAM, REMPLACEMENT_PARAM,
                         "parametre", CONTROLEUR)
    src_ctrl = remplacer(src_ctrl, ANCRE_LECTURE, REMPLACEMENT_LECTURE,
                         "lecture", CONTROLEUR)
    src_ctrl = remplacer(src_ctrl, ANCRE_ETAT, REMPLACEMENT_ETAT,
                         "etat", CONTROLEUR)
    src_ctrl = remplacer(src_ctrl, ANCRE_CAP, REMPLACEMENT_CAP,
                         "cap", CONTROLEUR)

    src_dec = remplacer(src_dec, ANCRE_DIAG, REMPLACEMENT_DIAG,
                        "diag", DECIDEUR)

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in ((CONTROLEUR, src_ctrl), (DECIDEUR, src_dec)):
        sauvegarde = "%s.bak_contresens_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("")
    print("A. Itineraire insuivable -> but rejete apres 8 s de cap oppose.")
    print("B. DIAG gain : mesure du gain a la position du vehicule.")
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % CONTROLEUR)
    print("  python3 -m py_compile %s" % DECIDEUR)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis :")
    print("  bash ~/valider.sh gain")
    print("  grep 'DIAG gain' ~/valid_gain.log | tail -5")
    print("  grep -c 'ITINERAIRE INSUIVABLE' ~/valid_gain.log")
    return 0


if __name__ == "__main__":
    sys.exit(main())
