#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Retire les instrumentations temporaires avant la campagne, et conserve
ce qui merite de devenir permanent.

POURQUOI MAINTENANT
-------------------
Le dernier run de validation est concluant :

    Carte active : Carla/Maps/Town03_Opt   (les trois noeuds)
    FIN DE RUN (budget_reached) : 603m / 600m | destinations
    atteintes=12
    DIAG echec : 17 -> 5

La chaine fonctionne de bout en bout. Les DIAG ont fait leur travail :
ils ont permis d'identifier le repere glissant du detecteur, la portee
insuffisante du planificateur, la granularite des decisions et enfin la
carte perimee du generateur. Ils n'ont plus de raison d'etre, et ils
polluent des journaux qui vont servir de donnees brutes a la campagne.

CE QUI EST RETIRE
-----------------
  - DIAG depart      : waypoint de projection, road/lane/jonction,
                       parametres du parcours ;
  - DIAG trouve      : forme verbeuse, avec ecart_max et les extremites
                       du chemin ;
  - DIAG grille      : emprise de la grille d'occupation et position des
                       candidats, dans decision_maker ;
  - nb_aretes / arete_max : ne comptaient que les aretes de l'ARBRE de
                       parcours, donc toujours (noeuds - 1). La mesure
                       n'apprenait rien, contrairement a ce que j'en
                       avais deduit au depart.

CE QUI DEVIENT PERMANENT
------------------------
  - la LONGUEUR MESUREE de l'itineraire, sous forme d'une ligne INFO
    compacte. C'est la grandeur qui a tranche la question de la
    granularite des decisions (un itineraire de 495 m consommait 82 %
    d'un run) et elle a sa place dans le depouillement ;

  - en cas d'echec, le NOMBRE DE WAYPOINTS EXPLORES et la DISTANCE DU
    PLUS PROCHE au but. C'est ce couple qui distingue un but manque de
    peu d'un but hors du reseau explore -- la distinction qui a revele
    la carte perimee. Si le defaut reapparait pendant la campagne, il
    sera visible immediatement au lieu de couter une journee.

AUTRE CHANGEMENT
----------------
sensor_range : 40 -> 20 m.

Le passage a 40 m avait ete tente pour aligner le rayon d'evaluation du
gain d'information sur Grid/RangeMax. Mesure : aucun effet sur
l'etendue du critere, qui reste nulle, pour quatre fois le volume de
calcul (le disque parcouru croit comme le carre du rayon). On revient a
20 m, valeur sous laquelle tous les runs valides ont tourne.

L'essai est documente dans le commentaire : c'est une hypothese testee
et refutee, pas un reglage abandonne sans raison.

CE QUI RESTE APRES CE PATCH
---------------------------
Rien ne bloque plus la campagne. Les limites connues, a documenter dans
le rapport plutot qu'a corriger :

  - information_gain sature (0.000 ou 1.000, jamais de valeur
    intermediaire) : les candidats d'une meme decision tombent tous du
    meme cote de la frontiere de cartographie ;
  - le critere de distance pondere une distance euclidienne alors que le
    vehicule parcourt une distance routiere, jusqu'a 6 fois plus longue ;
  - le controleur n'a pas d'evitement lateral : il freine devant un
    obstacle mais ne cesse jamais de suivre son itineraire.

Usage :
    python3 patch_nettoyage.py
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
DECIDEUR = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/"
    "active_slam_decision/decision_maker.py"
)
LAUNCH = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/launch/"
    "active_slam.launch.py"
)


# ---------------------------------------------------------------------
# 1. Controleur : tout le bloc instrumente de _plan_route
# ---------------------------------------------------------------------
ANCRE_BFS = (
    "        # --- INSTRUMENTATION TEMPORAIRE (a retirer apres diagnostic) ---\n"
    "        # Le journal annonce des itineraires de 32 m vers des buts situes\n"
    "        # a 124 m. Rejoue hors ligne avec le meme code, les memes\n"
    "        # parametres et le meme point de depart, ce BFS ne trouve aucun\n"
    "        # itineraire. Le code execute a ete verifie identique au source.\n"
    "        # Reste a savoir sur quel waypoint la projection tombe REELLEMENT\n"
    "        # a l'execution, et ce que le chemin reconstruit vaut\n"
    "        # geometriquement.\n"
    "        swl = start_wp.transform.location\n"
    "        self.get_logger().warn(\n"
    "            f\"DIAG depart : wp=({swl.x:.2f},{swl.y:.2f},{swl.z:.2f}) \"\n"
    "            f\"road={start_wp.road_id} lane={start_wp.lane_id} \"\n"
    "            f\"jonction={start_wp.is_junction} | \"\n"
    "            f\"ego=({start_loc.x:.2f},{start_loc.y:.2f},{start_loc.z:.2f}) | \"\n"
    "            f\"but=({goal_x:.2f},{goal_y:.2f}) \"\n"
    "            f\"droite={math.hypot(goal_x - start_loc.x, goal_y - start_loc.y):.1f}m\"\n"
    "            f\" | step={self.route_step} tol={self.route_goal_tolerance} \"\n"
    "            f\"max={self.route_max_distance}\"\n"
    "        )\n"
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
    "                route = self._rebuild_route(parents, key)\n"
    "                pts = [\n"
    "                    (w.transform.location.x, w.transform.location.y)\n"
    "                    for w in route\n"
    "                ]\n"
    "                # Longueur GEOMETRIQUE, par opposition a 'dist' qui vaut\n"
    "                # (nombre de sauts) x route_step sans jamais rien mesurer.\n"
    "                mesuree = sum(\n"
    "                    math.hypot(pts[i + 1][0] - pts[i][0],\n"
    "                               pts[i + 1][1] - pts[i][1])\n"
    "                    for i in range(len(pts) - 1)\n"
    "                )\n"
    "                ecart_max = max(\n"
    "                    (\n"
    "                        math.hypot(pts[i + 1][0] - pts[i][0],\n"
    "                                   pts[i + 1][1] - pts[i][1])\n"
    "                        for i in range(len(pts) - 1)\n"
    "                    ),\n"
    "                    default=0.0\n"
    "                )\n"
    "                self.get_logger().warn(\n"
    "                    f\"DIAG trouve : {len(route)} wp | \"\n"
    "                    f\"annoncee={dist:.1f}m mesuree={mesuree:.1f}m \"\n"
    "                    f\"ecart_max={ecart_max:.2f}m | \"\n"
    "                    f\"debut=({pts[0][0]:.2f},{pts[0][1]:.2f}) \"\n"
    "                    f\"fin=({pts[-1][0]:.2f},{pts[-1][1]:.2f}) \"\n"
    "                    f\"fin_au_but=\"\n"
    "                    f\"{math.hypot(goal_x - pts[-1][0], goal_y - pts[-1][1]):.2f}m\"\n"
    "                    f\" | aretes={nb_aretes} arete_max={arete_max:.2f}m\"\n"
    "                )\n"
    "                return route, dist\n"
    "\n"
    "            if dist + self.route_step > self.route_max_distance:\n"
    "                continue\n"
    "\n"
    "            for nxt in wp.next(self.route_step):\n"
    "                nkey = self._wp_key(nxt)\n"
    "                if nkey in parents:\n"
    "                    continue\n"
    "                nloc = nxt.transform.location\n"
    "                saut = math.hypot(nloc.x - loc.x, nloc.y - loc.y)\n"
    "                nb_aretes += 1\n"
    "                if saut > arete_max:\n"
    "                    arete_max = saut\n"
    "                parents[nkey] = (nxt, key)\n"
    "                queue.append((nxt, nkey, dist + self.route_step))\n"
    "\n"
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
    "        return None, float('inf')\n"
)

REMPLACEMENT_BFS = (
    "        # Distance du waypoint le plus proche du but effectivement\n"
    "        # visite. Conservee : c'est elle qui distingue un but manque de\n"
    "        # peu (tolerance trop juste) d'un but situe hors du reseau\n"
    "        # explore. C'est cette distinction qui a revele que le\n"
    "        # generateur travaillait sur une carte perimee -- il annoncait\n"
    "        # des destinations atteignables que ce parcours ne pouvait pas\n"
    "        # atteindre, le plus proche waypoint visite restant a 32, 48 puis\n"
    "        # 59 m des buts proposes.\n"
    "        meilleur_d = float('inf')\n"
    "\n"
    "        while queue:\n"
    "            wp, key, dist = queue.popleft()\n"
    "            loc = wp.transform.location\n"
    "\n"
    "            d_but = math.hypot(goal_x - loc.x, goal_y - loc.y)\n"
    "            if d_but < meilleur_d:\n"
    "                meilleur_d = d_but\n"
    "\n"
    "            if d_but <= self.route_goal_tolerance:\n"
    "                route = self._rebuild_route(parents, key)\n"
    "                # Longueur GEOMETRIQUE, par opposition a 'dist' qui vaut\n"
    "                # (nombre de sauts) x route_step sans jamais rien mesurer.\n"
    "                # C'est la grandeur qui borne la granularite des\n"
    "                # decisions : un itineraire de 495 m consommait a lui\n"
    "                # seul 82 % d'un run de 600 m.\n"
    "                pts = [\n"
    "                    (w.transform.location.x, w.transform.location.y)\n"
    "                    for w in route\n"
    "                ]\n"
    "                mesuree = sum(\n"
    "                    math.hypot(pts[i + 1][0] - pts[i][0],\n"
    "                               pts[i + 1][1] - pts[i][1])\n"
    "                    for i in range(len(pts) - 1)\n"
    "                )\n"
    "                self.get_logger().info(\n"
    "                    f\"Itineraire trouve : {len(route)} waypoints, \"\n"
    "                    f\"longueur mesuree={mesuree:.1f}m\"\n"
    "                )\n"
    "                return route, dist\n"
    "\n"
    "            if dist + self.route_step > self.route_max_distance:\n"
    "                continue\n"
    "\n"
    "            for nxt in wp.next(self.route_step):\n"
    "                nkey = self._wp_key(nxt)\n"
    "                if nkey in parents:\n"
    "                    continue\n"
    "                parents[nkey] = (nxt, key)\n"
    "                queue.append((nxt, nkey, dist + self.route_step))\n"
    "\n"
    "        self.get_logger().warn(\n"
    "            f\"Aucun itineraire routier vers \"\n"
    "            f\"({goal_x:.2f},{goal_y:.2f}) en moins de \"\n"
    "            f\"{self.route_max_distance:.0f}m | \"\n"
    "            f\"{len(parents)} waypoints explores, le plus proche a \"\n"
    "            f\"{meilleur_d:.1f}m du but \"\n"
    "            f\"(tolerance={self.route_goal_tolerance:.1f}m)\"\n"
    "        )\n"
    "        return None, float('inf')\n"
)


# ---------------------------------------------------------------------
# 2. Decideur : DIAG grille
# ---------------------------------------------------------------------
ANCRE_GRILLE = (
    "        # --- INSTRUMENTATION TEMPORAIRE (a retirer apres diagnostic) ---\n"
    "        # Compare l'emprise reelle de la grille d'occupation a la\n"
    "        # position des candidats, dans le repere ou la grille est\n"
    "        # interrogee. Un gain d'information de 1,000 pour tous les\n"
    "        # candidats a deux causes possibles, que seule cette\n"
    "        # comparaison separe : candidats hors emprise (le critere\n"
    "        # repond correctement, c'est le generateur qui ne propose que\n"
    "        # du terrain jamais vu), ou candidats dans l'emprise mais\n"
    "        # conversion monde -> cellule decalee (defaut de calcul).\n"
    "        if self.current_grid is not None:\n"
    "            gi = self.current_grid.info\n"
    "            gx0 = gi.origin.position.x\n"
    "            gy0 = gi.origin.position.y\n"
    "            gx1 = gx0 + gi.width * gi.resolution\n"
    "            gy1 = gy0 + gi.height * gi.resolution\n"
    "            # Memes coordonnees que celles passees a\n"
    "            # compute_information_gain : (c.x, -c.y).\n"
    "            apercu = ', '.join(\n"
    "                '(%.1f,%.1f)' % (c.x, -c.y) for c in candidates[:3]\n"
    "            )\n"
    "            self.get_logger().warn(\n"
    "                f\"DIAG grille : x=[{gx0:.1f},{gx1:.1f}] \"\n"
    "                f\"y=[{gy0:.1f},{gy1:.1f}] res={gi.resolution:.2f} \"\n"
    "                f\"{gi.width}x{gi.height} | \"\n"
    "                f\"pose_grille=({self.current_x:.1f},\"\n"
    "                f\"{-self.current_y:.1f}) | \"\n"
    "                f\"candidats {apercu}\"\n"
    "            )\n"
    "\n"
    "        for c in candidates:\n"
)

REMPLACEMENT_GRILLE = "        for c in candidates:\n"


# ---------------------------------------------------------------------
# 3. Launch : sensor_range 40 -> 20
# ---------------------------------------------------------------------
ANCRE_CAPTEUR = (
    "            # --- Rayon d'evaluation du gain d'information ---\n"
    "            # A ALIGNER SUR Grid/RangeMax (40 m), comme le prescrit\n"
    "            # la documentation de information_gain.py. A 20 m contre\n"
    "            # 40, le disque d'evaluation d'un candidat tombait\n"
    "            # entierement d'un cote de la frontiere de cartographie.\n"
    "            #\n"
    "            # Mesure sur valid_mincand12.log : candidats a\n"
    "            # x = 309.4 a 309.7 pour un bord de grille a x = 308.9 --\n"
    "            # 80 cm au-dela, tous du meme cote. D'ou info_gain = 1.000\n"
    "            # pour tous en fin de run, et 0.000 pour tous au debut,\n"
    "            # sans jamais de valeur intermediaire.\n"
    "            #\n"
    "            # Un disque de 40 m chevauche la frontiere au lieu de la\n"
    "            # manquer.\n"
    "            'sensor_range': 40.0,\n"
)

REMPLACEMENT_CAPTEUR = (
    "            # --- Rayon d'evaluation du gain d'information ---\n"
    "            # Le passage a 40 m, pour aligner ce rayon sur\n"
    "            # Grid/RangeMax comme le prescrit la documentation de\n"
    "            # information_gain.py, a ete TENTE et n'a rien change :\n"
    "            # l'etendue du critere est restee nulle sur toutes les\n"
    "            # decisions du run valid_capteur40.log, pour quatre fois le\n"
    "            # volume de calcul (le disque croit comme le carre du\n"
    "            # rayon).\n"
    "            #\n"
    "            # Retour a 20 m, valeur sous laquelle tous les runs valides\n"
    "            # ont tourne. L'hypothese est testee et refutee, elle n'a\n"
    "            # pas ete abandonnee sans mesure.\n"
    "            'sensor_range': 20.0,\n"
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
    for chemin in (CONTROLEUR, DECIDEUR, LAUNCH):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    src_ctrl = io.open(CONTROLEUR, encoding="utf-8").read()
    if "DIAG depart" not in src_ctrl:
        raise SystemExit(
            "Les instrumentations DIAG semblent deja retirees.\n"
            "Aucune modification effectuee."
        )

    resultats = {
        CONTROLEUR: remplacer(
            src_ctrl, ANCRE_BFS, REMPLACEMENT_BFS, "bfs", CONTROLEUR
        ),
        DECIDEUR: remplacer(
            io.open(DECIDEUR, encoding="utf-8").read(),
            ANCRE_GRILLE, REMPLACEMENT_GRILLE, "grille", DECIDEUR
        ),
        LAUNCH: remplacer(
            io.open(LAUNCH, encoding="utf-8").read(),
            ANCRE_CAPTEUR, REMPLACEMENT_CAPTEUR, "sensor_range", LAUNCH
        ),
    }

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in resultats.items():
        sauvegarde = "%s.bak_nettoyage_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("")
    print("DIAG depart / trouve / echec / grille : retires.")
    print("Longueur mesuree de l'itineraire : conservee (INFO).")
    print("Echec de routage : conserve, avec waypoints explores et")
    print("distance du plus proche au but.")
    print("sensor_range : 40.0 -> 20.0")
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % CONTROLEUR)
    print("  python3 -m py_compile %s" % DECIDEUR)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis un dernier run de controle avant la campagne :")
    print("  bash ~/valider.sh propre")
    return 0


if __name__ == "__main__":
    sys.exit(main())
