#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Corrige le parcours du reseau routier et la boucle de rejets.

DEFAUT 1 -- LA CLE DE DEDUPLICATION EFFACE LES BRANCHES DES CARREFOURS
----------------------------------------------------------------------
Les deux noeuds qui parcourent le reseau routier -- _plan_route() du
controleur et _reachable_candidates() du generateur -- identifient un
waypoint par :

    (round(loc.x / 2.0), round(loc.y / 2.0), wp.lane_id)

Le road_id n'y figure pas. Or dans un carrefour CARLA, chaque branche est
une ROUTE DE LIAISON distincte : plusieurs branches passent par des
positions quasi identiques, avec le meme lane_id mais des road_id
differents. Elles s'ecrasent donc mutuellement dans 'parents' / 'visited',
et toutes les branches sauf une disparaissent du parcours.

Mesure, sur Town01_Opt puis Town03_Opt :

    aretes explorees : 62      = 62 sauts de 4 m, soit 248 m
    aretes > 8 m     : 0

62 aretes pour 62 sauts : une seule chaine, aucune ramification, sur une
ville en damier. Un parcours en largeur qui ne bifurque jamais n'est pas
un parcours en largeur, c'est un suivi de voie. Le controleur ne peut
alors atteindre que ce qui se trouve droit devant lui.

Le generateur de candidats, lui, parcourt avec un pas de 8 m : il enjambe
l'interieur des carrefours et retombe directement sur les routes
sortantes, dont les lane_id different. C'est pour cela qu'il annonce
20 destinations "ATTEIGNABLES" vers lesquelles le controleur ne trouve
aucun itineraire -- releve sur un run : 20 rejets, 0 destination
atteinte, 0 m parcouru.

Ajouter road_id a la cle suffit : deux waypoints de routes differentes
cessent de s'ecraser.


DEFAUT 2 -- LA LISTE NOIRE SE VIDE ET LA BOUCLE S'EMBALLE
---------------------------------------------------------
Dans choose_next_goal() :

    if topologically_reachable:
        reachable = topologically_reachable

Quand le filtre des points inatteignables exclut TOUS les candidats, le
code retombe silencieusement sur la liste complete -- donc sur les points
deja rejetes. Comme les echanges rejet -> nouvelle decision -> rejet se
font par messages ROS, la boucle tourne a pleine vitesse. Trace : les
memes 7 points rechoisis en cycle, 20 rejets en 180 millisecondes.

On attend desormais la publication de candidats suivante (toutes les
publish_period secondes, 3 s par defaut), qui tiendra compte de la
position mise a jour du vehicule, au lieu de rechoisir un point dont on
sait deja qu'il sera rejete.


DEFAUT 3 -- LE SEUIL D'ARRET COMPTE DES EVENEMENTS, PAS DU TEMPS
-----------------------------------------------------------------
"20 buts consecutifs ecartes" terminait le run apres 180 ms. Un critere
d'arret doit mesurer une duree pendant laquelle le systeme n'a pas
progresse, pas un nombre de messages echanges. Le seuil devient donc un
delai : le run s'arrete si aucune destination n'a ete acceptee pendant
rejection_timeout secondes consecutives.


VALIDATION
----------
L'instrumentation DIAG deja en place mesure le correctif elle-meme : la
ligne "DIAG trouve"/"DIAG echec" affiche 'aretes='. Si ce nombre passe de
62 a plusieurs centaines, le graphe bifurque enfin.

Usage :
    python3 patch_graphe_routier.py

Les trois fichiers sont calcules avant toute ecriture.
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
DECISION = BASE + "decision_maker.py"


COMMENTAIRE_CLE = (
    '        """Cle de deduplication : position arrondie a 2 m, route et voie.\n'
    "\n"
    "        Le road_id est INDISPENSABLE. Sans lui, les branches d'un\n"
    "        carrefour -- qui sont autant de routes de liaison distinctes,\n"
    "        passant par des positions quasi identiques avec le meme lane_id\n"
    "        -- s'ecrasent mutuellement, et toutes sauf une disparaissent du\n"
    "        parcours. Mesure avant correction : 62 aretes explorees pour\n"
    "        62 sauts, soit une seule chaine et aucune ramification, sur une\n"
    "        ville en damier.\n"
    '        """\n'
)


CONTROLEUR_BLOCS = [
    (
        "    @staticmethod\n"
        "    def _wp_key(wp):\n"
        '        """Cle de deduplication : position arrondie a 2 m + identifiant de voie."""\n'
        "        loc = wp.transform.location\n"
        "        return (round(loc.x / 2.0), round(loc.y / 2.0), wp.lane_id)\n",

        "    @staticmethod\n"
        "    def _wp_key(wp):\n"
        + COMMENTAIRE_CLE +
        "        loc = wp.transform.location\n"
        "        return (\n"
        "            round(loc.x / 2.0),\n"
        "            round(loc.y / 2.0),\n"
        "            wp.road_id,\n"
        "            wp.lane_id,\n"
        "        )\n",
    ),
]


GENERATEUR_BLOCS = [
    (
        "    @staticmethod\n"
        "    def _wp_key(wp):\n"
        '        """Cle de deduplication : position arrondie a 2 m + identifiant de\n'
        "        voie. Identique a celle du controleur, pour que les deux noeuds\n"
        '        aient exactement la meme vision du reseau."""\n'
        "        loc = wp.transform.location\n"
        "        return (round(loc.x / 2.0), round(loc.y / 2.0), wp.lane_id)\n",

        "    @staticmethod\n"
        "    def _wp_key(wp):\n"
        '        """Cle de deduplication : position arrondie a 2 m, route et voie.\n'
        "\n"
        "        DOIT rester identique a celle du controleur : c'est ce qui\n"
        "        garantit que les deux noeuds voient le meme reseau et donnent\n"
        "        la meme reponse a la question 'puis-je aller la ?'.\n"
        "\n"
        "        Le road_id est indispensable. Sans lui, les branches d'un\n"
        "        carrefour s'ecrasent mutuellement. Ce noeud y echappait\n"
        "        partiellement parce qu'il parcourt avec un pas de 8 m, qui\n"
        "        enjambe l'interieur des carrefours -- d'ou la contradiction\n"
        "        observee : 20 destinations annoncees atteignables ici, aucune\n"
        "        routable par le controleur, qui avance lui de 4 m.\n"
        '        """\n'
        "        loc = wp.transform.location\n"
        "        return (\n"
        "            round(loc.x / 2.0),\n"
        "            round(loc.y / 2.0),\n"
        "            wp.road_id,\n"
        "            wp.lane_id,\n"
        "        )\n",
    ),
]


DECISION_BLOCS = [
    # --- Etat : instant du premier rejet d'une serie ---
    (
        "        self.last_goal_time = None\n"
        "        self.consecutive_rejections = 0\n",

        "        self.last_goal_time = None\n"
        "        self.consecutive_rejections = 0\n"
        "        # Instant du premier rejet d'une serie ininterrompue. Sert au\n"
        "        # critere d'arret : c'est une DUREE sans progres qui doit\n"
        "        # terminer le run, pas un nombre de messages echanges.\n"
        "        self.premier_rejet_time = None\n",
    ),

    # --- Ne plus retomber sur la liste complete ---
    (
        "        if topologically_reachable:\n"
        "            reachable = topologically_reachable\n",

        "        if topologically_reachable:\n"
        "            reachable = topologically_reachable\n"
        "        else:\n"
        "            # Tous les candidats sont a moins de\n"
        "            # unreachable_exclusion_radius d'un point deja declare\n"
        "            # inatteignable. La version precedente retombait\n"
        "            # silencieusement sur la liste complete, donc rechoisissait\n"
        "            # un point deja rejete : la boucle\n"
        "            # rejet -> nouvelle decision -> rejet tournait a la vitesse\n"
        "            # des messages ROS. Trace : les memes 7 points en cycle,\n"
        "            # 20 rejets en 180 millisecondes.\n"
        "            #\n"
        "            # On attend plutot la publication de candidats suivante :\n"
        "            # elle sera calculee depuis la position courante du\n"
        "            # vehicule, donc differente. waiting_for_candidates fait\n"
        "            # que on_candidates relancera la decision.\n"
        "            self.get_logger().warn(\n"
        "                f\"AUCUN CANDIDAT EXPLOITABLE : les \"\n"
        "                f\"{len(reachable)} candidats sont tous a moins de \"\n"
        "                f\"{self.unreachable_exclusion_radius:.0f} m d'un point \"\n"
        "                f\"deja declare inatteignable \"\n"
        "                f\"({len(self.unreachable_points)} points). Attente de \"\n"
        "                f\"la prochaine liste de candidats.\"\n"
        "            )\n"
        "            self.waiting_for_candidates = True\n"
        "            return\n",
    ),

    # --- Critere d'arret : une duree, pas un compteur ---
    (
        "        # Garde-fou : si plus aucun candidat n'est routable, mieux\n"
        "        # vaut terminer le run proprement que boucler jusqu'au budget.\n"
        "        if self.consecutive_rejections >= 20:\n"
        "            self._finish_run(\n"
        "                'no_reachable_goal',\n"
        "                \"20 buts consecutifs ecartes par le planificateur \"\n"
        "                \"routier\"\n"
        "            )\n"
        "            return\n",

        "        # Garde-fou : si plus aucun candidat n'est routable, mieux\n"
        "        # vaut terminer le run proprement que boucler jusqu'au budget.\n"
        "        #\n"
        "        # Le critere est une DUREE, pas un nombre de rejets. Le seuil\n"
        "        # precedent -- 20 rejets consecutifs -- terminait le run apres\n"
        "        # 180 millisecondes, parce que ces echanges se font par\n"
        "        # messages ROS et non a la vitesse du vehicule. Un critere\n"
        "        # d'arret doit mesurer un temps pendant lequel le systeme n'a\n"
        "        # pas progresse.\n"
        "        if self.premier_rejet_time is None:\n"
        "            self.premier_rejet_time = self.get_clock().now()\n"
        "\n"
        "        sans_progres = (\n"
        "            self.get_clock().now() - self.premier_rejet_time\n"
        "        ).nanoseconds / 1e9\n"
        "\n"
        "        if sans_progres > 60.0:\n"
        "            self._finish_run(\n"
        "                'no_reachable_goal',\n"
        "                f\"aucune destination acceptee pendant \"\n"
        "                f\"{sans_progres:.0f}s \"\n"
        "                f\"({self.consecutive_rejections} buts ecartes)\"\n"
        "            )\n"
        "            return\n",
    ),

    # --- Remise a zero sur une arrivee reelle ---
    (
        "        self.consecutive_rejections = 0\n"
        "        self.destinations_reached += 1\n",

        "        self.consecutive_rejections = 0\n"
        "        self.premier_rejet_time = None\n"
        "        self.destinations_reached += 1\n",
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
    for chemin in (CONTROLEUR, GENERATEUR, DECISION):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    # Marqueur SPECIFIQUE au nouveau return de _wp_key : la ligne exacte,
    # indentation et virgule comprises. Un simple 'wp.road_id' serait vrai
    # a cause du 'road={start_wp.road_id}' de l'instrumentation DIAG deja
    # presente dans ce fichier -- le garde-fou refusait alors d'agir sur un
    # fichier non patche. Un test qui peut etre vrai pour la mauvaise
    # raison ne teste rien.
    if "            wp.road_id,\n" in io.open(CONTROLEUR, encoding="utf-8").read():
        raise SystemExit(
            "Le correctif semble deja applique (road_id present dans la "
            "cle de _wp_key).\nAucune modification effectuee."
        )

    resultats = {
        CONTROLEUR: appliquer(CONTROLEUR, CONTROLEUR_BLOCS),
        GENERATEUR: appliquer(GENERATEUR, GENERATEUR_BLOCS),
        DECISION: appliquer(DECISION, DECISION_BLOCS),
    }

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in resultats.items():
        sauvegarde = "%s.bak_graphe_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("\n6 blocs corriges dans 3 fichiers.")
    print("\nReconstruire :")
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    return 0


if __name__ == "__main__":
    sys.exit(main())
