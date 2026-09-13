#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Borne la longueur d'une decision, pour que la campagne mesure bien une
politique de decision et non un tirage au sort.

LE FAIT MESURE
--------------
Quatre runs de validation consecutifs, meme budget de 600 m,
configuration quasi identique :

    origine     602 m   13 destinations
    grille      525 m   11 destinations
    mincand12   604 m    3 destinations
    capteur40   600 m    1 destination

Un facteur treize sur le nombre de decisions. Or c'est precisement ce
que la campagne pretend mesurer : l'effet d'une politique de choix. Un
run a une seule decision ne compare aucune strategie -- weighted,
random et info_gain y produiraient le meme resultat, faute d'occasion
de s'exprimer.

La cause est visible dans valid_capteur40.log :

    mesuree=495.2m
    mesuree=106.2m
    mesuree=65.9m

Le PREMIER itineraire faisait 495 m. Sur 600 m de budget, une seule
destination en a consomme 82 %.

LA CAUSE
--------
Deux mecanismes se combinent.

1. Le rapport entre distance a vol d'oiseau et distance routiere n'est
   pas borne. max_candidate_distance vaut 80 m, et pourtant un but a
   80 m a produit un itineraire de 495 m : facteur 6,2. Sens uniques,
   terre-plein central, contournement de pate de maisons. Ce n'est pas
   un reglage errone, c'est une propriete du reseau routier.

   route_max_distance, releve a 500 m pour debloquer le planificateur,
   accepte donc des trajets qui avalent le budget entier. En debloquant
   le planificateur j'ai autorise ce cas.

2. Le plafond de 80 m est contournable. Dans choose_next_goal :

       reachable = [c for c in self.candidates if hypot(...) <= 80]
       if not reachable:
           reachable = self.candidates      # la liste COMPLETE

   Quand le generateur ne publie que des candidats lointains, le repli
   reprend tout et le plafond ne s'applique plus. Mesure sur
   valid_retour25.log : buts retenus a 123.9, 136.9 et 154.2 m, tous
   au-dela de 80.

LES DEUX CORRECTIONS
--------------------
1. route_max_distance : 500 -> 300 m.

   Le planificateur refuse un itineraire de 495 m ; le decideur choisit
   alors un autre candidat. Avec des candidats plafonnes a 80 m a vol
   d'oiseau, 300 m de route laisse un facteur 3,75 -- large au regard
   des trajets observes (106 m et 66 m pour les deux autres buts du
   meme run).

   Effet : le pire cas est borne a 300 m par decision, donc au moins
   quatre decisions par run de 1200 m, et une dizaine en pratique.

2. Repli borne aux 5 candidats les plus proches.

   Le repli garde sa raison d'etre -- ne jamais figer le vehicule faute
   de candidat exploitable -- mais il ne peut plus ramener les buts les
   plus eloignes de tous, ceux-la memes que le plafond ecartait.

CE QUI N'EST PAS CORRIGE, ET DOIT ALLER DANS LE RAPPORT
-------------------------------------------------------
Le critere de distance de la fonction de decision pondere une distance
EUCLIDIENNE, alors que le vehicule parcourt une distance ROUTIERE qui
peut valoir six fois plus. Le generateur connait pourtant la vraie
valeur -- il journalise "distance routiere 72-248 m" -- mais ne la
transmet pas : le message PoseArray ne porte que des positions.

Corriger cela demanderait de changer le format des messages de
candidats et de retoucher trois noeuds. C'est une limite du dispositif,
identifiee et mesurable, pas un defaut a reparer a la veille de la
campagne.

VERIFICATION
------------
    bash ~/valider.sh granularite

Attendu : aucun 'mesuree=' au-dela de 300 m, et un nombre de
destinations qui remonte vers la dizaine. Verifier aussi que le run se
termine sur 'budget de distance atteint' et non sur 'no_reachable_goal'
-- si tous les candidats devenaient non routables sous 300 m, le remede
serait pire que le mal et il faudrait remonter le plafond a 400 m.

Usage :
    python3 patch_granularite.py
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
DECIDEUR = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/"
    "active_slam_decision/decision_maker.py"
)


# ---------------------------------------------------------------------
# 1. route_max_distance : 500 -> 300
# ---------------------------------------------------------------------
ANCRE_PORTEE = "                'route_max_distance': 500.0,\n"

REMPLACEMENT_PORTEE = (
    "                # Ramene de 500 a 300 m. 500 m avait ete choisi pour\n"
    "                # debloquer le planificateur, qui refusait des buts\n"
    "                # atteignables faute de portee. Mais en debloquant, on a\n"
    "                # autorise des trajets qui avalent le budget entier :\n"
    "                # mesure sur valid_capteur40.log, un but a 80 m a vol\n"
    "                # d'oiseau a produit un itineraire de 495 m, soit 82 %\n"
    "                # d'un run de 600 m pour UNE seule destination.\n"
    "                #\n"
    "                # Consequence sur la campagne : le nombre de decisions\n"
    "                # par run variait de 1 a 13 a configuration identique.\n"
    "                # Un run a une decision ne compare aucune strategie.\n"
    "                #\n"
    "                # 300 m laisse un facteur 3,75 sur les 80 m de\n"
    "                # max_candidate_distance, largement au-dessus des\n"
    "                # trajets normalement observes (106 m et 66 m pour les\n"
    "                # deux autres buts du meme run).\n"
    "                'route_max_distance': 300.0,\n"
)


# ---------------------------------------------------------------------
# 2. Repli borne aux 5 candidats les plus proches
# ---------------------------------------------------------------------
ANCRE_REPLI = (
    "        if not reachable:\n"
    "            reachable = self.candidates\n"
)

REMPLACEMENT_REPLI = (
    "        if not reachable:\n"
    "            # --- Repli BORNE ---\n"
    "            # Ce repli rendait max_candidate_distance inoperant :\n"
    "            # quand le generateur ne publie que des candidats\n"
    "            # lointains, on repartait sur la liste COMPLETE et le\n"
    "            # plafond de 80 m ne s'appliquait plus. Mesure sur\n"
    "            # valid_retour25.log : buts retenus a 123.9, 136.9 et\n"
    "            # 154.2 m, tous au-dela du plafond.\n"
    "            #\n"
    "            # Le repli garde sa raison d'etre -- ne jamais figer le\n"
    "            # vehicule faute de candidat exploitable -- mais il ne\n"
    "            # peut plus ramener les buts les plus eloignes de tous,\n"
    "            # ceux-la memes que le plafond ecartait.\n"
    "            reachable = sorted(\n"
    "                self.candidates,\n"
    "                key=lambda c: math.hypot(\n"
    "                    c.x - self.current_x,\n"
    "                    c.y - self.current_y\n"
    "                )\n"
    "            )[:5]\n"
)


def main():
    for chemin in (LAUNCH, DECIDEUR):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    src_launch = io.open(LAUNCH, encoding="utf-8").read()
    src_decideur = io.open(DECIDEUR, encoding="utf-8").read()

    if "'route_max_distance': 300.0," in src_launch:
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    n = src_launch.count(ANCRE_PORTEE)
    if n != 1:
        raise SystemExit(
            "ANCRE 'route_max_distance' TROUVEE %d FOIS (attendu 1).\n"
            "Aucune modification effectuee." % n
        )

    n = src_decideur.count(ANCRE_REPLI)
    if n != 1:
        raise SystemExit(
            "ANCRE 'repli' TROUVEE %d FOIS (attendu 1) dans "
            "decision_maker.py.\n"
            "Aucune modification effectuee." % n
        )

    src_launch = src_launch.replace(ANCRE_PORTEE, REMPLACEMENT_PORTEE)
    src_decideur = src_decideur.replace(ANCRE_REPLI, REMPLACEMENT_REPLI)

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in ((LAUNCH, src_launch), (DECIDEUR, src_decideur)):
        sauvegarde = "%s.bak_granularite_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("")
    print("route_max_distance : 500.0 -> 300.0")
    print("repli du decideur  : liste complete -> 5 candidats les plus proches")
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % DECIDEUR)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis valider :")
    print("  bash ~/valider.sh granularite")
    print("  grep -oE 'mesuree=[0-9.]+m' ~/valid_granularite.log")
    return 0


if __name__ == "__main__":
    sys.exit(main())
