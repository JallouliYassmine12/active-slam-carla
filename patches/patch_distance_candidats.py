#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rend le critere de gain d'information discriminant.

LE FAIT MESURE
--------------
Instrumentation de la dispersion, run de 600 m, 12 decisions :

    CRITERES sur 3 candidats : info_gain [1.000 - 1.000] etendue=0.000
    CRITERES sur 6 candidats : info_gain [1.000 - 1.000] etendue=0.000
    ... 12 fois sur 12

Le gain d'information vaut 1,000 pour TOUS les candidats, a toutes les
decisions. Un critere constant ne classe rien -- et c'est celui qui porte
le poids le plus fort de la fonction de decision : 0,30 sur 1,00.

Le meme run montre que les quatre autres criteres fonctionnent :
loop_closure monte a 0,600, l'incertitude a 0,87, la securite descend
sous 1,0, la distance varie. Le probleme est isole.

LA CAUSE
--------
Trois parametres se contredisent :

    min_candidate_distance = 25 m   (aucun candidat plus proche)
    sensor_range           = 20 m   (rayon d'evaluation autour du candidat)
    Grid/RangeMax          = 40 m   (portee de la carte)

compute_information_gain compte les cellules INCONNUES dans un rayon
sensor_range autour du candidat. Un candidat a 25 m evalue donc la zone
situee entre 5 et 45 m devant le vehicule, sur une route jamais
parcourue : il renvoie 1,0. Un candidat a 100 m aussi.

AUCUN candidat ne peut tomber dans le territoire deja cartographie, donc
aucun ne peut avoir un gain faible, donc il n'y a rien a classer. Le
relevement de Grid/RangeMax de 25 a 40 m n'y a rien change -- ce n'etait
pas le verrou.

LA CORRECTION
-------------
min_candidate_distance : 25 -> 12 m.

Un candidat a 12 m aura la majeure partie de son disque d'evaluation
dans la carte -> gain faible. Un candidat a 100 m restera a 1,0. Le
critere retrouve une plage utile.

LE RISQUE, ET POURQUOI IL EST ACCEPTABLE
----------------------------------------
Ce parametre avait ete releve de 5 a 25 m pour eviter que l'exploration
ne degenere en sauts de quelques metres, ce qui empechait toute
distinction entre strategies lors de la campagne comparative.

Deux garde-fous rendent aujourd'hui ce risque plus faible qu'alors :

  - la liste tabou (tabu_radius = 10 m, memoire de 5 buts) interdit de
    rechoisir un point tout juste visite ; 12 m reste au-dessus de ce
    rayon ;
  - le critere de distance est un COUT soustrait, pas un benefice : il
    pousse vers le proche, mais avec un poids de 0,15 seulement, contre
    0,30 au gain d'information qui, lui, poussera desormais vers
    l'inconnu. Les deux s'equilibrent au lieu de s'additionner.

Effet secondaire souhaitable : le run de verification ne proposait que 2
a 8 candidats par decision. Abaisser le seuil en produira nettement
plus, et une fonction multi-criteres n'a de sens qu'avec un vivier a
classer.

VERIFICATION
------------
Un run de 600 m suffit :

    grep "CRITERES" ~/valid_disp12.log | tail -10

'etendue' doit devenir franchement non nulle. Si elle reste a 0,000 meme
a 12 m, le critere est inutilisable dans cette configuration et il faut
le documenter comme une limite plutot que continuer a le poursuivre.

Usage :
    python3 patch_distance_candidats.py
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

ANCRE = "                'min_candidate_distance': 25.0,\n"

REMPLACEMENT = (
    "                # --- Distance routiere minimale d'une destination ---\n"
    "                # DOIT rester nettement INFERIEURE au sensor_range du\n"
    "                # decision_maker (20 m), sinon aucun candidat ne tombe\n"
    "                # dans la zone deja cartographiee et le gain\n"
    "                # d'information vaut 1,0 pour tous.\n"
    "                #\n"
    "                # Mesure a 25 m, sur 12 decisions consecutives :\n"
    "                #   info_gain [1.000 - 1.000] etendue=0.000\n"
    "                # Le critere qui porte le poids le plus fort de la\n"
    "                # decision (0,30) ne classait donc rien.\n"
    "                #\n"
    "                # A 12 m, un candidat proche evalue majoritairement du\n"
    "                # terrain connu (gain faible) et un candidat lointain du\n"
    "                # terrain inconnu (gain eleve) : le critere retrouve une\n"
    "                # plage utile.\n"
    "                #\n"
    "                # 12 m reste au-dessus du tabu_radius (10 m), donc le\n"
    "                # garde-fou contre les sauts de quelques metres -- la\n"
    "                # raison pour laquelle ce seuil avait ete releve de 5 a\n"
    "                # 25 m -- reste en place.\n"
    "                'min_candidate_distance': 12.0,\n"
)


def main():
    if not os.path.exists(LAUNCH):
        raise SystemExit("Fichier introuvable : %s" % LAUNCH)

    src = io.open(LAUNCH, encoding="utf-8").read()

    if "'min_candidate_distance': 12.0," in src:
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    n = src.count(ANCRE)
    if n != 1:
        raise SystemExit(
            "ANCRE TROUVEE %d FOIS (attendu 1).\n"
            "Aucune modification effectuee." % n
        )

    src = src.replace(ANCRE, REMPLACEMENT)

    sauvegarde = "%s.bak_mincand_%s" % (LAUNCH, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(LAUNCH, sauvegarde)
    io.open(LAUNCH, "w", encoding="utf-8").write(src)

    print("min_candidate_distance : 25.0 -> 12.0")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Reconstruire :")
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    return 0


if __name__ == "__main__":
    sys.exit(main())
