#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fait de /grid_prob_map une carte de l'EXPLORE et non une carte des murs.

LE FAIT MESURE
--------------
Instrumentation DIAG gain, runs valid_frontiere2.log et
valid_raytracing.log, avec le calcul d'origine restaure :

    au vehicule=1.000 | pose=(237.2,116.4)
                        grille x=[-1.6,303.8] y=[-1.6,184.8]
    au vehicule=1.000 | pose=(217.5, 77.0)
                        grille x=[-1.6,221.2] y=[-2.0, 78.5]
    au vehicule=1.000 | pose=(228.6,180.3)
                        grille x=[-1.6,230.2] y=[-2.0,181.7]

Deux choses dans ces lignes.

D'abord, le vehicule roule sur du terrain que sa propre carte declare
inconnu -- y compris quand sa pose tombe franchement a l'interieur de
la grille (237 pour un bord a 304). Ce n'est donc ni un probleme de
repere, ni un probleme de frontiere, ni la distance des candidats.

Ensuite, la grille s'arrete a 1,4 m au-dela du vehicule, en x comme en
y, a chaque releve. La zone cartographiee n'est pas un couloir de 40 m
de large le long du trajet : c'est un ruban de la largeur du vehicule.
Avec un LiDAR portant a 40 m, c'est le signe que l'espace VIDE n'est
jamais marque.

LA CAUSE
--------
Configuration relevee dans active_slam.launch.py :

    'Grid/3D':         'true'
    'Grid/RayTracing': 'false'      <-- ligne 417

Une grille d'occupation a trois etats : libre (0), occupee (100),
inconnue (-1). RTAB-Map ne marque LIBRE l'espace entre le capteur et
les obstacles que si Grid/RayTracing est actif. Desactive, la grille ne
contient que les cellules occupees -- facades, murs, vehicules -- et
tout le reste, chaussee parcourue comprise, reste a -1.

C'est une carte des murs, pas une carte de l'explore. Le critere
mesurait la proportion d'inconnu dans une grille ou tout est inconnu :
le resultat ne pouvait etre que 1.000. A 25 m comme a 12 m de distance
minimale, avec un rayon d'evaluation de 20 m comme de 40, autour du
point d'arrivee comme le long du trajet. Toutes les pistes suivies
aujourd'hui portaient sur des parametres qui ne pouvaient rien y
changer.

Les quatre autres criteres ne dependent pas de cette grille :
l'incertitude vient de la covariance de pose, la fermeture de boucle du
graphe, la securite du detecteur d'obstacles, la distance du graphe
routier. C'est pourquoi eux fonctionnaient.

LES DEUX MODIFICATIONS
----------------------
    'Grid/RayTracing' : 'false' -> 'true'
    'Grid/3D'         : 'true'  -> 'false'

La seconde accompagne necessairement la premiere. Avec Grid/3D a true,
RTAB-Map fait le lancer de rayons EN VOLUME, ce qui est lent au point
de compromettre la cartographie temps reel. Or la grille que consomme
le decideur, /grid_prob_map, est de toute facon une projection 2D : la
construire nativement en 2D donne le meme resultat pour un cout bien
moindre.

Si la carte 3D est utilisee ailleurs -- visualisation, octomap --,
remettre Grid/3D a 'true' et surveiller la duree des runs.

TROISIEME AJOUT : LA PREUVE
---------------------------
La ligne DIAG gain affiche desormais la composition de la grille,
pourcentage d'inconnu et de libre. La cause est ainsi verifiee au lieu
d'etre supposee -- ce qui aurait evite plusieurs heures aujourd'hui.

VERIFICATION
------------
    bash ~/valider.sh raytracing2
    grep "DIAG gain" ~/valid_raytracing2.log | tail -5
    grep "CRITERES"  ~/valid_raytracing2.log | tail -10
    grep "FIN DE RUN" ~/valid_raytracing2.log

Attendu, dans cet ordre :
  - 'libre' franchement superieur a 0 % ;
  - 'au vehicule' proche de 0 ;
  - une etendue non nulle sur les decisions offrant plusieurs
    directions ;
  - une duree comparable aux 213 s de reference. Si elle explose, c'est
    le lancer de rayons qui coute trop cher et il faut reduire
    Grid/RangeMax.

Si 'libre' reste a 0 % malgre les deux changements, la cause est la
segmentation du sol : regarder Grid/MaxGroundHeight (1.0) et
Grid/MinGroundHeight (-2.0) plutot que le lancer de rayons.

Usage :
    python3 patch_grille_exploree.py
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


ANCRE_RAY = "                'Grid/RayTracing': 'false',\n"

REMPLACEMENT_RAY = (
    "                # --- Lancer de rayons : INDISPENSABLE ---\n"
    "                # Sans lui, RTAB-Map ne marque que les cellules\n"
    "                # OCCUPEES. La chaussee parcourue reste a -1 et la\n"
    "                # grille est une carte des murs, pas une carte de\n"
    "                # l'explore.\n"
    "                #\n"
    "                # Mesure : le gain d'information valait 1.000 A LA\n"
    "                # POSITION MEME du vehicule, pose=(237.2,116.4) dans une\n"
    "                # grille x=[-1.6,303.8] y=[-1.6,184.8]. Le critere le\n"
    "                # plus lourd de la decision (0,30) ne pouvait rien\n"
    "                # classer : il mesurait la proportion d'inconnu dans une\n"
    "                # grille ou tout est inconnu.\n"
    "                'Grid/RayTracing': 'true',\n"
)

ANCRE_3D = "                'Grid/3D': 'true',\n"

REMPLACEMENT_3D = (
    "                # 2D et non 3D : avec Grid/3D a 'true', le lancer de\n"
    "                # rayons se fait EN VOLUME, ce qui est lent au point de\n"
    "                # compromettre la cartographie temps reel. La grille\n"
    "                # consommee par le decideur, /grid_prob_map, est de\n"
    "                # toute facon une projection 2D : la construire\n"
    "                # nativement en 2D donne le meme resultat pour un cout\n"
    "                # bien moindre.\n"
    "                'Grid/3D': 'false',\n"
)


ANCRE_WARN = '            self.get_logger().warn(\n                f"DIAG gain : au vehicule={gain_vehicule:.3f} | "\n                f"candidat 0 : trajet={gain_trajet:.3f} "\n                f"arrivee={gain_arrivee:.3f} | "\n                f"pose=({self.current_x:.1f},{-self.current_y:.1f}) "\n                f"cand=({c0.x:.1f},{-c0.y:.1f}) | "\n                f"grille x=[{gi.origin.position.x:.1f},"\n                f"{gi.origin.position.x + gi.width * gi.resolution:.1f}] "\n                f"y=[{gi.origin.position.y:.1f},"\n                f"{gi.origin.position.y + gi.height * gi.resolution:.1f}]"\n            )\n'

REMPLACEMENT_WARN = '            # Composition de la grille. Un critere de gain\n            # d\'information mesure la proportion de cellules INCONNUES ;\n            # encore faut-il que la grille contienne des cellules LIBRES.\n            # RTAB-Map ne marque libre l\'espace entre le capteur et les\n            # obstacles que si Grid/RayTracing est actif. Sans lui la\n            # grille ne contient que les facades detectees, et tout le\n            # reste -- y compris la chaussee deja parcourue -- reste a\n            # -1. count() travaille en C, le cout reste negligeable.\n            cellules = self.current_grid.data\n            total_cellules = len(cellules) or 1\n            pct_inconnu = 100.0 * cellules.count(-1) / total_cellules\n            pct_libre = 100.0 * cellules.count(0) / total_cellules\n            self.get_logger().warn(\n                f"DIAG gain : au vehicule={gain_vehicule:.3f} | "\n                f"candidat 0 : trajet={gain_trajet:.3f} "\n                f"arrivee={gain_arrivee:.3f} | "\n                f"pose=({self.current_x:.1f},{-self.current_y:.1f}) "\n                f"cand=({c0.x:.1f},{-c0.y:.1f}) | "\n                f"grille inconnu={pct_inconnu:.1f}% "\n                f"libre={pct_libre:.1f}% | "\n                f"x=[{gi.origin.position.x:.1f},"\n                f"{gi.origin.position.x + gi.width * gi.resolution:.1f}] "\n                f"y=[{gi.origin.position.y:.1f},"\n                f"{gi.origin.position.y + gi.height * gi.resolution:.1f}]"\n            )\n'


def main():
    for chemin in (LAUNCH, DECIDEUR):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    src_launch = io.open(LAUNCH, encoding="utf-8").read()
    src_dec = io.open(DECIDEUR, encoding="utf-8").read()

    if "'Grid/RayTracing': 'true'," in src_launch:
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    for nom, ancre, remplacement in (
        ("Grid/RayTracing", ANCRE_RAY, REMPLACEMENT_RAY),
        ("Grid/3D", ANCRE_3D, REMPLACEMENT_3D),
    ):
        n = src_launch.count(ancre)
        if n != 1:
            raise SystemExit(
                "ANCRE '%s' TROUVEE %d FOIS (attendu 1) dans le launch.\n"
                "Aucune modification effectuee." % (nom, n)
            )
        src_launch = src_launch.replace(ancre, remplacement)

    n = src_dec.count(ANCRE_WARN)
    if n != 1:
        raise SystemExit(
            "ANCRE 'DIAG gain' TROUVEE %d FOIS (attendu 1) dans\n"
            "decision_maker.py. Aucune modification effectuee." % n
        )
    src_dec = src_dec.replace(ANCRE_WARN, REMPLACEMENT_WARN)

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in ((LAUNCH, src_launch), (DECIDEUR, src_dec)):
        sauvegarde = "%s.bak_grilleexp_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("")
    print("Grid/RayTracing : 'false' -> 'true'")
    print("Grid/3D         : 'true'  -> 'false'")
    print("DIAG gain affiche desormais inconnu%% et libre%% de la grille.")
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % DECIDEUR)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis :")
    print("  bash ~/valider.sh raytracing2")
    print("  grep 'DIAG gain' ~/valid_raytracing2.log | tail -5")
    return 0


if __name__ == "__main__":
    sys.exit(main())
