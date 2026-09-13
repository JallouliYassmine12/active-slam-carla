#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rend la portee du planificateur coherente avec la portee des candidats.

LE FAIT MESURE
--------------
Journal valid_retour25.log, trois premieres decisions du run :

    DIAG depart : ... but=(108.21,-35.74) droite=123.9m | step=8.0 max=250.0
    DIAG echec  : aucun itineraire vers (108.21,-35.74) | aretes=38
    DIAG depart : ... but=(99.79,-51.66)  droite=136.9m | step=8.0 max=250.0
    DIAG echec  : aucun itineraire vers (99.79,-51.66)  | aretes=38
    DIAG depart : ... but=(85.77,-62.97)  droite=154.2m | step=8.0 max=250.0

Puis 32 buts rejetes d'affilee, chacun mis en liste noire, et :

    FIN DE RUN (no_reachable_goal) : aucune destination acceptee pendant
    66s (32 buts ecartes) | destinations atteintes=0,
    distance parcourue=0m

LA CAUSE
--------
Deux parametres n'ont jamais ete confrontes :

    max_candidate_distance (decision_maker) = 150 m  a vol d'oiseau
    route_max_distance     (controleur)     = 250 m  par la route

Le decision_maker retient donc des buts jusqu'a 150 m a vol d'oiseau.
Le planificateur, lui, s'arrete a 250 m de parcours routier :

    if dist + self.route_step > self.route_max_distance:
        continue

Or un trajet routier fait 2 a 3 fois la distance a vol d'oiseau : sens
de circulation a respecter, virages, contournement des pates de
maisons. Un but a 124 m demande donc 250 a 400 m de route. Il est
physiquement atteignable, mais hors de portee du parcours.

La signature est nette dans le journal : aretes=38, soit 38 x 8 m =
304 m explores, c'est-a-dire exactement la limite. Le parcours ne bute
ni sur un cul-de-sac ni sur un defaut du graphe routier CARLA : il bute
sur son propre budget.

POURQUOI CE DEFAUT EST INTERMITTENT -- ET DONC DANGEREUX
--------------------------------------------------------
Si le premier but tire tombe a moins de 250 m par la route, le vehicule
part, s'eloigne, et les buts suivants restent proches : le run se
deroule normalement (run info_gain de 605 m). Sinon tout est rejete et
le run meurt a 0 m.

Le meme code, les memes parametres et la meme carte produisent donc
tantot un run valide, tantot un run vide. C'est tres probablement
l'origine d'une partie des runs invalides de la campagne precedente, et
cela devait etre corrige avant de relancer les 17 runs.

LA CORRECTION
-------------
route_max_distance : 250 -> 500 m.

Facteur 3,3 sur les 150 m admis par max_candidate_distance, au-dessus
du pire ratio routier realiste (3). Le parametre n'apparait pas dans le
fichier de launch -- il vient du defaut declare dans le noeud -- donc il
est ajoute explicitement a cote de route_step, comme cela avait ete fait
pour ce dernier. Les deux valeurs sont ainsi visibles au meme endroit,
ce qui est precisement ce qui manquait pour reperer l'incoherence.

COUT DE CALCUL
--------------
Horodatages du journal : le BFS complet a 250 m prend 3 ms
(1788271101.672 -> 1788271101.675). Doubler la portee le porte a
quelques dizaines de millisecondes, pour une decision toutes les 6
secondes. Negligeable.

VERIFICATION
------------
    bash ~/valider.sh portee500

Attendu : des lignes 'DIAG trouve' au lieu de 'DIAG echec', une distance
parcourue non nulle, et une fin de run sur 'budget de distance atteint'
et non sur 'no_reachable_goal'.

Si le vehicule roule, la grille d'occupation de RTAB-Map se construit et
'grid=' doit passer de NONE a OK -- c'est la condition pour que le
critere de gain d'information puisse enfin etre evalue.

Usage :
    python3 patch_portee_itineraire.py
"""

import io
import os
import shutil
import sys
import time

LAUNCH = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/launch/"
    "active_slam.launch.py"
)

ANCRE = "                'route_step': 8.0,\n"

REMPLACEMENT = (
    "                'route_step': 8.0,\n"
    "                # --- Portee du parcours routier ---\n"
    "                # DOIT valoir au moins 3 fois le\n"
    "                # max_candidate_distance du decision_maker (150 m) :\n"
    "                # un trajet routier fait 2 a 3 fois la distance a vol\n"
    "                # d'oiseau (sens de circulation, virages, contournement\n"
    "                # des pates de maisons).\n"
    "                #\n"
    "                # Le defaut du noeud, 250 m, etait incoherent avec les\n"
    "                # 150 m admis pour un candidat. Mesure sur\n"
    "                # valid_retour25.log : buts a 124, 137 et 154 m a vol\n"
    "                # d'oiseau, tous rejetes, aretes=38 soit 38 x 8 m =\n"
    "                # 304 m -- le parcours butait sur son propre budget, pas\n"
    "                # sur un cul-de-sac. Resultat : 32 buts ecartes,\n"
    "                # 0 destination atteinte, 0 m parcouru.\n"
    "                #\n"
    "                # Le defaut etait INTERMITTENT : si le premier but tire\n"
    "                # tombait a moins de 250 m par la route, le run se\n"
    "                # deroulait normalement. Meme code, memes parametres,\n"
    "                # meme carte, deux issues opposees selon le tirage.\n"
    "                'route_max_distance': 500.0,\n"
)


def main():
    if not os.path.exists(LAUNCH):
        raise SystemExit("Fichier introuvable : %s" % LAUNCH)

    src = io.open(LAUNCH, encoding="utf-8").read()

    if "'route_max_distance'" in src:
        raise SystemExit(
            "route_max_distance est deja present dans le fichier de launch.\n"
            "Aucune modification effectuee -- verifier sa valeur a la main."
        )

    n = src.count(ANCRE)
    if n != 1:
        raise SystemExit(
            "ANCRE TROUVEE %d FOIS (attendu 1).\n"
            "Aucune modification effectuee." % n
        )

    src = src.replace(ANCRE, REMPLACEMENT)

    sauvegarde = "%s.bak_portee_%s" % (LAUNCH, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(LAUNCH, sauvegarde)
    io.open(LAUNCH, "w", encoding="utf-8").write(src)

    print("route_max_distance : 250.0 (defaut du noeud) -> 500.0 (explicite)")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Reconstruire :")
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis valider :")
    print("  bash ~/valider.sh portee500")
    return 0


if __name__ == "__main__":
    sys.exit(main())
