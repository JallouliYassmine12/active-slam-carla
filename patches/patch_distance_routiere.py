#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Le critere de distance pondere enfin la distance REELLEMENT parcourue.

LE DEFAUT
---------
La fonction de decision penalise la distance a VOL D'OISEAU entre le
vehicule et le candidat :

    distance = math.hypot(c.x - self.current_x, c.y - self.current_y)

Or le vehicule parcourt une distance ROUTIERE : sens uniques, virages,
contournement des pates de maisons. Mesure sur valid_capteur40.log --
un candidat a 80 m a vol d'oiseau a produit un itineraire de 495 m,
soit un facteur 6,2.

Le critere de cout de deplacement, qui pese 0,15 dans la ponderation,
classait donc les candidats sur une grandeur qui pouvait etre six fois
eloignee de celle qu'elle pretend representer. Un candidat "proche"
pouvait en realite couter le budget entier du run.

C'est d'autant plus regrettable que le generateur CONNAIT la bonne
valeur : son parcours du graphe routier la calcule pour chaque candidat,
et il l'affiche meme dans son journal --

    23 destinations candidates ATTEIGNABLES publiees
    (distance routiere 32-248m)

-- mais ne la transmet pas. Le message PoseArray ne porte que des
positions.

LA CORRECTION
-------------
Le champ pose.position.z etait inutilise : le generateur le mettait a
0.0, les candidats etant tous au niveau du sol. Il transporte desormais
la distance routiere.

    generateur      : pose.position.z = distance routiere du parcours
    decision_maker  : Candidate.road_distance <- p.position.z
                      le cout de distance utilise cette valeur

Repli explicite : si z vaut 0 -- generateur non mis a jour, ou message
d'une autre source -- le calcul retombe sur la distance a vol d'oiseau.
Aucun risque de regression silencieuse.

CE QUE CELA CHANGE POUR LA CAMPAGNE
-----------------------------------
La strategie 'distance' (une des quatre references) selectionnait
jusqu'ici le candidat le plus proche A VOL D'OISEAU. Elle selectionne
desormais le plus proche PAR LA ROUTE, ce qui est le sens que le sujet
lui donne. La strategie 'weighted' voit son terme de cout corrige de la
meme facon.

C'est une correction de fond, pas un reglage : elle doit etre faite
AVANT la campagne, et signalee dans le rapport comme telle.

VERIFICATION
------------
    bash ~/valider.sh distance
    grep "CANDIDAT CHOISI" ~/valid_distance.log | tail -5

Le champ 'distance' des scores doit desormais correspondre a la
longueur des itineraires journalises ("Itineraire trouve : ...,
longueur mesuree=...m"), a la tolerance du planificateur pres, et non
plus a une valeur systematiquement plus petite.

Usage :
    python3 patch_distance_routiere.py
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
GENERATEUR = BASE + "candidate_generator.py"
DECIDEUR = BASE + "decision_maker.py"


# ---------------------------------------------------------------------
# 1. Generateur : transmettre la distance routiere
# ---------------------------------------------------------------------
ANCRE_GEN = (
    "        for wp, _dist in candidates:\n"
    "            loc = wp.transform.location\n"
    "            pose = Pose()\n"
    "            pose.position.x, pose.position.y = self._carla_to_map(loc.x, loc.y)\n"
    "            pose.position.z = 0.0\n"
    "            pose.orientation.w = 1.0\n"
    "            pose_array.poses.append(pose)\n"
)

REMPLACEMENT_GEN = (
    "        for wp, dist_routiere in candidates:\n"
    "            loc = wp.transform.location\n"
    "            pose = Pose()\n"
    "            pose.position.x, pose.position.y = self._carla_to_map(loc.x, loc.y)\n"
    "            # --- z transporte la DISTANCE ROUTIERE ---\n"
    "            # Ce champ etait inutilise (tous les candidats sont au\n"
    "            # niveau du sol). Il porte desormais la distance reellement\n"
    "            # a parcourir, calculee par le parcours du graphe ci-dessus.\n"
    "            #\n"
    "            # Sans elle, decision_maker penalisait la distance a vol\n"
    "            # d'oiseau : mesure sur valid_capteur40.log, un candidat a\n"
    "            # 80 m a vol d'oiseau demandait 495 m de route, soit un\n"
    "            # facteur 6,2. Le critere de cout de deplacement classait\n"
    "            # les candidats sur une grandeur sans rapport avec ce que le\n"
    "            # vehicule allait reellement parcourir.\n"
    "            pose.position.z = float(dist_routiere)\n"
    "            pose.orientation.w = 1.0\n"
    "            pose_array.poses.append(pose)\n"
)


# ---------------------------------------------------------------------
# 2. Decideur : porter la distance routiere dans Candidate
# ---------------------------------------------------------------------
ANCRE_DATACLASS = (
    "@dataclass\n"
    "class Candidate:\n"
    "    id: int\n"
    "    x: float\n"
    "    y: float\n"
)

REMPLACEMENT_DATACLASS = (
    "@dataclass\n"
    "class Candidate:\n"
    "    id: int\n"
    "    x: float\n"
    "    y: float\n"
    "    # Distance ROUTIERE transmise par candidate_generator dans\n"
    "    # position.z. Vaut 0.0 si le message vient d'une source qui ne la\n"
    "    # fournit pas : le cout de distance retombe alors sur la distance a\n"
    "    # vol d'oiseau, sans regression silencieuse.\n"
    "    road_distance: float = 0.0\n"
)


ANCRE_ON_CANDIDATES = (
    "        self.candidates = [\n"
    "            Candidate(\n"
    "                id=i,\n"
    "                x=p.position.x,\n"
    "                y=p.position.y\n"
    "            )\n"
    "            for i, p in enumerate(msg.poses)\n"
    "        ]\n"
)

REMPLACEMENT_ON_CANDIDATES = (
    "        self.candidates = [\n"
    "            Candidate(\n"
    "                id=i,\n"
    "                x=p.position.x,\n"
    "                y=p.position.y,\n"
    "                road_distance=p.position.z\n"
    "            )\n"
    "            for i, p in enumerate(msg.poses)\n"
    "        ]\n"
)


ANCRE_EVAL = (
    "        max_dist = max(\n"
    "            (\n"
    "                math.hypot(\n"
    "                    c.x - self.current_x,\n"
    "                    c.y - self.current_y\n"
    "                )\n"
    "                for c in candidates\n"
    "            ),\n"
    "            default=1.0\n"
    "        ) or 1.0\n"
    "\n"
    "        for c in candidates:\n"
    "            distance = math.hypot(\n"
    "                c.x - self.current_x,\n"
    "                c.y - self.current_y\n"
    "            )\n"
)

REMPLACEMENT_EVAL = (
    "        def cout_distance(c):\n"
    "            \"\"\"Distance REELLEMENT a parcourir jusqu'au candidat.\n"
    "\n"
    "            candidate_generator transmet la distance routiere dans\n"
    "            position.z, calculee par son parcours du graphe. C'est la\n"
    "            grandeur que le sujet demande pour le critere de cout de\n"
    "            deplacement.\n"
    "\n"
    "            La distance a vol d'oiseau, utilisee jusqu'ici, pouvait en\n"
    "            etre tres eloignee : mesure sur valid_capteur40.log, un\n"
    "            candidat a 80 m a vol d'oiseau demandait 495 m de route.\n"
    "\n"
    "            Repli sur la distance euclidienne si z n'est pas renseigne.\n"
    "            \"\"\"\n"
    "            if c.road_distance > 0.0:\n"
    "                return c.road_distance\n"
    "            return math.hypot(\n"
    "                c.x - self.current_x,\n"
    "                c.y - self.current_y\n"
    "            )\n"
    "\n"
    "        max_dist = max(\n"
    "            (cout_distance(c) for c in candidates),\n"
    "            default=1.0\n"
    "        ) or 1.0\n"
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
    for chemin in (GENERATEUR, DECIDEUR):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    src_gen = io.open(GENERATEUR, encoding="utf-8").read()
    src_dec = io.open(DECIDEUR, encoding="utf-8").read()

    if "road_distance" in src_dec:
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    src_gen = remplacer(src_gen, ANCRE_GEN, REMPLACEMENT_GEN,
                        "publication", GENERATEUR)

    src_dec = remplacer(src_dec, ANCRE_DATACLASS, REMPLACEMENT_DATACLASS,
                        "dataclass", DECIDEUR)
    src_dec = remplacer(src_dec, ANCRE_ON_CANDIDATES,
                        REMPLACEMENT_ON_CANDIDATES, "reception", DECIDEUR)
    src_dec = remplacer(src_dec, ANCRE_EVAL, REMPLACEMENT_EVAL,
                        "evaluation", DECIDEUR)

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in ((GENERATEUR, src_gen), (DECIDEUR, src_dec)):
        sauvegarde = "%s.bak_distroute_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("")
    print("La distance routiere transite par position.z et sert desormais")
    print("de cout de deplacement dans la fonction de decision.")
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % GENERATEUR)
    print("  python3 -m py_compile %s" % DECIDEUR)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    return 0


if __name__ == "__main__":
    sys.exit(main())
