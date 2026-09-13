#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fait produire a RTAB-Map des cellules LIBRES, sans quoi le gain
d'information ne peut rien mesurer.

L'ETAT DES MESURES
------------------
Run valid_grille.log, avec le choix automatique de grille :

    GRILLE ACTIVE : /grid_prob_map (connu=2.4%) -- /map ecartee (connu=0.0%)
    DIAG gain : au vehicule=1.000 | grille inconnu=100.0% libre=0.0%

Trois faits, dans l'ordre ou ils ont ete etablis :

  1. La grille n'est PAS vide : 2,4 % de cellules connues quand la carte
     est encore petite. Le topic /grid_prob_map est le bon -- /map, lui,
     reste a 0,0 %.

  2. Mais libre = 0,0 %. Les 2,4 % de cellules connues sont donc TOUTES
     occupees : des facades, des murs. Aucune cellule n'est marquee
     libre.

  3. Les parametres sont bien appliques -- le journal RTAB-Map le
     confirme, sans avertissement de base de donnees :

         Setting RTAB-Map parameter "Grid/3D"="false" (rosparam)
         Setting RTAB-Map parameter "Grid/RayTracing"="true" (rosparam)

Le lancer de rayons est donc actif et ne produit rien.

LA CAUSE
--------
Le lancer de rayons de RTAB-Map trace des rayons vers les points
classes SOL, en marquant libres les cellules traversees. S'il n'y a
aucun point sol, il n'a rien a tracer.

Or Grid/NormalsSegmentation est actif par defaut : le sol est
identifie par analyse des normales du nuage. Cette methode echoue
frequemment sur un LiDAR automobile, ou la chaussee est vue en
incidence tres rasante -- les normales y sont bruitees et les points
sont rejetes comme obstacles.

La consequence colle exactement aux mesures : seules les facades, vues
de face, sont retenues ; la chaussee ne l'est jamais. D'ou une carte
d'obstacles epars le long du trajet, sans aucune cellule libre.

LA CORRECTION
-------------
    'Grid/NormalsSegmentation': 'false'

RTAB-Map bascule alors sur un simple seuil de hauteur : tout point
situe sous Grid/MaxGroundHeight (deja regle a 1,0 m) est du sol, tout
ce qui est au-dessus est un obstacle. Sur une route plate c'est le mode
robuste, et il produit les points sol dont le lancer de rayons a
besoin.

Grid/MaxGroundHeight et Grid/MinGroundHeight sont deja renseignes
(1,0 et -2,0 m) : le seuil est donc pret a etre utilise.

CE QU'IL FAUT SURVEILLER
------------------------
Un seuil de hauteur ne distingue pas une chaussee d'un trottoir bas ou
d'une bordure. Quelques cellules seront classees libres a tort. Pour un
critere d'exploration, dont la question est "cette zone a-t-elle deja
ete vue ?", c'est sans consequence -- ce n'est pas une carte de
navigation.

REGLE D'ARRET
-------------
Derniere tentative sur ce critere. Si libre reste a 0 % apres ce run,
la campagne est lancee avec les quatre criteres operationnels
-- incertitude, fermeture de boucle, distance routiere, securite -- et
le gain d'information est documente comme limite du dispositif, avec la
chaine de mesures qui l'etablit : grille lue, composition, parametres
appliques, et les cinq formulations essayees.

VERIFICATION
------------
    bash ~/valider.sh sol
    grep "GRILLE ACTIVE" ~/valid_sol.log | head -3
    grep "DIAG gain"     ~/valid_sol.log | tail -5
    grep "CRITERES"      ~/valid_sol.log | tail -10

Attendu, dans cet ordre :
  - libre franchement superieur a 0 % ;
  - au vehicule proche de 0 -- il reconnait la route qu'il a parcourue ;
  - une etendue non nulle sur les decisions offrant plusieurs
    directions.

Usage :
    python3 patch_segmentation_sol.py
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

ANCRE = "                'Grid/MaxGroundHeight': '1.0',\n"

REMPLACEMENT = (
    "                # --- Segmentation du sol par SEUIL DE HAUTEUR ---\n"
    "                # Grid/NormalsSegmentation, actif par defaut, identifie\n"
    "                # le sol par analyse des normales du nuage. Sur un LiDAR\n"
    "                # automobile la chaussee est vue en incidence tres\n"
    "                # rasante : les normales y sont bruitees et les points\n"
    "                # sont rejetes comme obstacles.\n"
    "                #\n"
    "                # Sans points sol, le lancer de rayons n'a rien vers quoi\n"
    "                # tracer, donc aucune cellule n'est marquee LIBRE.\n"
    "                # Mesure sur valid_grille.log : 2,4 % de cellules connues\n"
    "                # et libre=0,0 % -- les seules cellules renseignees sont\n"
    "                # les facades, vues de face. Le gain d'information valait\n"
    "                # alors 1,000 partout, y compris a la position meme du\n"
    "                # vehicule.\n"
    "                #\n"
    "                # A 'false', RTAB-Map bascule sur un simple seuil :\n"
    "                # sous Grid/MaxGroundHeight (1,0 m) c'est du sol, au\n"
    "                # dessus c'est un obstacle. Sur une route plate c'est le\n"
    "                # mode robuste.\n"
    "                #\n"
    "                # Un seuil ne distingue pas une chaussee d'un trottoir\n"
    "                # bas : quelques cellules seront libres a tort. Sans\n"
    "                # consequence pour un critere d'exploration, dont la\n"
    "                # question est \"cette zone a-t-elle deja ete vue ?\" et\n"
    "                # non \"puis-je rouler ici ?\".\n"
    "                'Grid/NormalsSegmentation': 'false',\n"
    "                'Grid/MaxGroundHeight': '1.0',\n"
)


def main():
    if not os.path.exists(LAUNCH):
        raise SystemExit("Fichier introuvable : %s" % LAUNCH)

    src = io.open(LAUNCH, encoding="utf-8").read()

    if "Grid/NormalsSegmentation" in src:
        raise SystemExit(
            "Grid/NormalsSegmentation est deja present dans le launch.\n"
            "Aucune modification effectuee -- corriger sa valeur a la main."
        )

    n = src.count(ANCRE)
    if n != 1:
        raise SystemExit(
            "ANCRE 'Grid/MaxGroundHeight' TROUVEE %d FOIS (attendu 1).\n"
            "Aucune modification effectuee." % n
        )

    src = src.replace(ANCRE, REMPLACEMENT)

    sauvegarde = "%s.bak_sol_%s" % (LAUNCH, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(LAUNCH, sauvegarde)
    io.open(LAUNCH, "w", encoding="utf-8").write(src)

    print("Grid/NormalsSegmentation : 'false' (segmentation par seuil)")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Reconstruire :")
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis :")
    print("  bash ~/valider.sh sol")
    print("  grep 'DIAG gain' ~/valid_sol.log | tail -5")
    print("  grep 'CRITERES'  ~/valid_sol.log | tail -10")
    return 0


if __name__ == "__main__":
    sys.exit(main())
