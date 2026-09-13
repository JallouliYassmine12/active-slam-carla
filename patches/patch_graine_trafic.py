#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Aligne le regime de trafic de 'predefinie' sur celui des quatre
strategies Active SLAM.

L'ASYMETRIE MESUREE
-------------------
    slam_evaluation.launch.py:432 :   'seed': 42,
    active_slam.launch.py         :   (aucune graine)

traffic_spawner.py declare 'seed' a -1, ce qui signifie "aleatoire" :

    self.declare_parameter('seed', -1)          # -1 = aleatoire
    ...
    if seed is not None and seed >= 0:
        random.seed(seed)
    ...
    if self.seed is not None and self.seed >= 0:
        self.traffic_manager.set_random_device_seed(self.seed)

Consequence sur la campagne du 3 septembre : la reference SLAM
classique a affronte un trafic REPRODUCTIBLE -- memes modeles de
vehicules, memes points d'apparition, meme comportement du Traffic
Manager a chaque repetition -- pendant que 'random', 'distance',
'info_gain' et 'weighted' affrontaient un trafic tire au hasard.

La reference et les methodes comparees ne subissaient donc pas les
memes conditions experimentales. Les quatre strategies Active SLAM
restent comparables entre elles ; c'est la position de 'predefinie'
dans le classement qui n'est pas defendable.

LA CORRECTION
-------------
    'seed': 42   ->   'seed': -1

C'est-a-dire : trafic aleatoire pour les cinq conditions. Les trois
repetitions echantillonnent alors la variabilite du trafic urbain au
lieu d'en figer une realisation, et elles le font de la meme facon
pour toutes les strategies.

POURQUOI DANS CE SENS, ET NON L'INVERSE
---------------------------------------
On aurait pu ensemencer les quatre autres launches plutot que de
desensemencer celui-ci. Deux raisons de ne pas le faire.

D'abord le cout : ensemencer imposerait de refaire les 14 runs Active
SLAM, contre 3 runs 'predefinie' ici.

Ensuite et surtout, la mesure montre que cela ne servirait a rien.
Les trois 'predefinie' de la campagne v2 avaient DEJA une graine fixe,
une trajectoire identique (600 waypoints figes) et un simulateur
deterministe -- bridge_node.py fixe synchronous_mode=True et
fixed_delta_seconds=0.05. Ils ont pourtant produit des erreurs de
9,7 m, 77,7 m et 161,1 m, soit un facteur 16.

La variabilite residuelle ne vient donc ni du trafic ni du simulateur,
mais de la chaine de perception : icp_odometry et RTAB-Map consomment
les scans en temps souple et en perdent selon la charge machine
(delay=0.25 s releve sur RTAB-Map). Ensemencer le trafic ne toucherait
pas a cette source.

CE QU'IL FAUT CONSERVER
-----------------------
Les trois runs 'predefinie' actuels sont une PREUVE : trajectoire
identique + graine identique + simulateur deterministe, et malgre tout
un facteur 16 sur l'erreur. Ils etablissent le plancher de bruit du
dispositif. Les commandes ci-dessous les mettent de cote au lieu de
les ecraser.

EMPREINTE DU CODE
-----------------
Ce patch modifie un fichier de ros2_ws/src : l'empreinte change et
un_scenario.sh refusera de demarrer tant que la reference n'est pas
remise a zero. C'est voulu -- c'est exactement le garde-fou qui doit
se declencher.

Le fichier modifie ne sert QU'A 'predefinie'. Les 14 runs Active SLAM
deja archives ne sont pas affectes par ce changement, et il n'y a donc
aucune raison de les refaire.

VERIFICATION
------------
    grep -n "seed" ~/active_slam_carla/ros2_ws/src/slam_evaluation/\
launch/slam_evaluation.launch.py

Attendu : 'seed': -1, avec le commentaire qui l'explique.

Usage :
    python3 patch_graine_trafic.py
"""

import io
import os
import re
import shutil
import sys
import time

CIBLE = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/slam_evaluation/launch/"
    "slam_evaluation.launch.py"
)

# Contenu attendu de la ligne, espaces de gauche exclus. L'indentation
# est relevee sur le fichier plutot que codee en dur : elle depend de
# l'imbrication du dictionnaire de parametres.
LIGNE_CIBLE = "'seed': 42,"

COMMENTAIRES = [
    "# --- Graine du trafic : ALEATOIRE, comme les quatre autres ---",
    "#",
    "# Valait 42, alors qu'active_slam.launch.py n'en fixe aucune.",
    "# 'predefinie' affrontait donc un trafic reproductible pendant que",
    "# les quatre strategies Active SLAM en affrontaient un tire au",
    "# hasard : la reference et les methodes comparees ne subissaient pas",
    "# les memes conditions.",
    "#",
    "# On desensemence plutot que d'ensemencer les autres, car la mesure",
    "# montre que la graine ne gouverne pas la dispersion. Les trois",
    "# 'predefinie' de la campagne du 3 septembre avaient graine fixe,",
    "# trajectoire identique et simulateur deterministe (synchronous_mode",
    "# = True, fixed_delta_seconds = 0.05), et ont donne 9,7 m, 77,7 m et",
    "# 161,1 m d'erreur. La variabilite vient de la chaine de perception,",
    "# executee en temps souple, pas du scenario simule.",
    "#",
    "# -1 = aleatoire (voir traffic_spawner.declare_parameter('seed', -1))",
]


def main():
    if not os.path.exists(CIBLE):
        raise SystemExit("Fichier introuvable : %s" % CIBLE)

    lignes = io.open(CIBLE, encoding="utf-8").read().splitlines(True)

    indices = [
        i for i, l in enumerate(lignes)
        if l.strip() == LIGNE_CIBLE
    ]

    if len(indices) != 1:
        deja = [i for i, l in enumerate(lignes) if l.strip() == "'seed': -1,"]
        if deja:
            raise SystemExit(
                "Le correctif semble deja applique ('seed': -1 present "
                "ligne %d).\nAucune modification effectuee." % (deja[0] + 1)
            )
        raise SystemExit(
            "LIGNE \"%s\" TROUVEE %d FOIS (attendu 1).\n"
            "Aucune modification effectuee." % (LIGNE_CIBLE, len(indices))
        )

    i = indices[0]
    indentation = re.match(r"[ \t]*", lignes[i]).group(0)

    remplacement = [
        indentation + c + "\n" for c in COMMENTAIRES
    ] + [indentation + "'seed': -1,\n"]

    lignes[i:i + 1] = remplacement

    sauvegarde = "%s.bak_graine_%s" % (CIBLE, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(CIBLE, sauvegarde)
    io.open(CIBLE, "w", encoding="utf-8").write("".join(lignes))

    print("Graine du trafic de 'predefinie' : 42 -> -1 (aleatoire),")
    print("alignee sur les quatre strategies Active SLAM.")
    print("ligne %d, sauvegarde : %s" % (i + 1, os.path.basename(sauvegarde)))
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % CIBLE)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis mettre de cote les 3 runs 'predefinie' actuels (ils sont la")
    print("preuve du plancher de bruit, ils ne doivent pas etre ecrases),")
    print("remettre l'empreinte a zero, et refaire ces trois runs.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
