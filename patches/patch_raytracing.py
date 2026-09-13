#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Active le lancer de rayons de RTAB-Map, sans lequel la grille
d'occupation ne dit pas ce qui a ete explore.

LE FAIT MESURE
--------------
Instrumentation DIAG gain, run valid_frontiere2.log, avec le calcul
d'origine restaure :

    au vehicule=1.000 | pose=(237.2,116.4)
                        grille x=[-1.6,303.8] y=[-1.6,184.8]

Le vehicule est franchement A L'INTERIEUR de sa grille -- 237 pour un
bord a 304, 116 pour un bord a 185 -- et le critere lui attribue 1.000,
"totalement inconnu". Les cellules autour de lui valent donc -1, alors
qu'il vient de parcourir cette route.

Cela elimine toutes les hypotheses precedentes : ce n'est ni un
probleme de repere (la pose tombe dans la grille), ni un probleme de
frontiere (le comportement d'origine est restaure), ni un probleme de
distance des candidats (le point mesure est le vehicule lui-meme).

LA CAUSE
--------
Une grille d'occupation a trois etats : libre (0), occupee (100),
inconnue (-1). Un critere de gain d'information compte les cellules
inconnues -- encore faut-il que les cellules deja vues soient marquees
LIBRES.

RTAB-Map ne remplit l'espace entre le capteur et les obstacles que si
Grid/RayTracing est actif. Ce parametre est DESACTIVE par defaut. Sans
lui, la grille ne contient que les cellules occupees -- facades, murs,
vehicules -- et tout le reste, y compris la chaussee parcourue, reste a
-1.

C'est une carte d'obstacles, pas une carte de l'explore. Le critere
mesurait la proportion d'inconnu dans une grille ou tout est inconnu :
le resultat ne pouvait etre que 1.000, a 25 m comme a 12 m, avec un
rayon de 20 m comme de 40, autour du point d'arrivee comme le long du
trajet.

Les quatre autres criteres ne dependent pas de cette grille --
l'incertitude vient de la covariance de pose, la fermeture de boucle du
graphe, la securite du detecteur d'obstacles, la distance du graphe
routier. C'est pourquoi eux fonctionnaient.

CE QUE FAIT LE PATCH
--------------------
1. Grid/RayTracing passe a 'true' dans active_slam.launch.py. Les
   cellules traversees par un rayon LiDAR sans rencontrer d'obstacle
   sont marquees LIBRES. La grille devient une carte de l'explore.

2. La ligne DIAG gain affiche desormais la composition de la grille :
   pourcentage d'inconnu et de libre. La cause est ainsi prouvee au
   lieu d'etre supposee, avant et apres.

CE QU'IL FAUT SURVEILLER
------------------------
Le lancer de rayons coute du temps de calcul a chaque mise a jour de
carte. Si le run ralentit nettement par rapport aux 152 s de reference,
c'est le poste a regarder en premier.

VERIFICATION
------------
    bash ~/valider.sh raytracing
    grep "DIAG gain" ~/valid_raytracing.log | tail -5
    grep "CRITERES"  ~/valid_raytracing.log | tail -10

Attendu : 'libre' franchement superieur a 0 %, 'au vehicule' proche de
0, et une etendue non nulle sur les decisions offrant plusieurs
directions.

Si 'libre' reste a 0 % malgre le parametre, RTAB-Map ne segmente pas le
sol : il faudra alors regarder Grid/NormalsSegmentation et
Grid/MaxGroundHeight plutot que le lancer de rayons.

Usage :
    python3 patch_raytracing.py
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


ANCRE_WARN = '            self.get_logger().warn(\n                f"DIAG gain : au vehicule={gain_vehicule:.3f} | "\n                f"candidat 0 : trajet={gain_trajet:.3f} "\n                f"arrivee={gain_arrivee:.3f} | "\n                f"pose=({self.current_x:.1f},{-self.current_y:.1f}) "\n                f"cand=({c0.x:.1f},{-c0.y:.1f}) | "\n                f"grille x=[{gi.origin.position.x:.1f},"\n                f"{gi.origin.position.x + gi.width * gi.resolution:.1f}] "\n                f"y=[{gi.origin.position.y:.1f},"\n                f"{gi.origin.position.y + gi.height * gi.resolution:.1f}]"\n            )\n'

REMPLACEMENT_WARN = '            # Composition de la grille. Un critere de gain\n            # d\'information mesure la proportion de cellules INCONNUES ;\n            # encore faut-il que la grille contienne des cellules LIBRES.\n            # RTAB-Map ne marque libre l\'espace entre le capteur et les\n            # obstacles que si Grid/RayTracing est actif. Sans lui la\n            # grille ne contient que les facades detectees, et tout le\n            # reste -- y compris la chaussee deja parcourue -- reste a\n            # -1. count() travaille en C, le cout reste negligeable.\n            cellules = self.current_grid.data\n            total_cellules = len(cellules) or 1\n            pct_inconnu = 100.0 * cellules.count(-1) / total_cellules\n            pct_libre = 100.0 * cellules.count(0) / total_cellules\n            self.get_logger().warn(\n                f"DIAG gain : au vehicule={gain_vehicule:.3f} | "\n                f"candidat 0 : trajet={gain_trajet:.3f} "\n                f"arrivee={gain_arrivee:.3f} | "\n                f"pose=({self.current_x:.1f},{-self.current_y:.1f}) "\n                f"cand=({c0.x:.1f},{-c0.y:.1f}) | "\n                f"grille inconnu={pct_inconnu:.1f}% "\n                f"libre={pct_libre:.1f}% | "\n                f"x=[{gi.origin.position.x:.1f},"\n                f"{gi.origin.position.x + gi.width * gi.resolution:.1f}] "\n                f"y=[{gi.origin.position.y:.1f},"\n                f"{gi.origin.position.y + gi.height * gi.resolution:.1f}]"\n            )\n'


def main():
    for chemin in (LAUNCH, DECIDEUR):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    src_launch = io.open(LAUNCH, encoding="utf-8").read()
    src_dec = io.open(DECIDEUR, encoding="utf-8").read()

    if "Grid/RayTracing" in src_launch:
        raise SystemExit(
            "Grid/RayTracing est deja present dans le fichier de launch.\n"
            "Aucune modification effectuee -- verifier sa valeur a la main."
        )

    if "DIAG gain" not in src_dec:
        raise SystemExit(
            "L'instrumentation DIAG gain est absente : appliquer d'abord\n"
            "patch_contresens.py.\nAucune modification effectuee."
        )

    # --- Launch : inserer Grid/RayTracing juste apres Grid/RangeMax ---
    # Insertion par balayage de lignes plutot que par ancre exacte :
    # l'indentation de ce bloc de parametres RTAB-Map n'est pas connue a
    # l'avance, et elle doit etre reproduite fidelement.
    lignes = src_launch.splitlines(True)
    indices = [i for i, l in enumerate(lignes) if "'Grid/RangeMax'" in l]
    if len(indices) != 1:
        raise SystemExit(
            "'Grid/RangeMax' TROUVE %d FOIS (attendu 1) dans le launch.\n"
            "Aucune modification effectuee." % len(indices)
        )

    i = indices[0]
    indent = lignes[i][:len(lignes[i]) - len(lignes[i].lstrip())]
    ajout = (
        indent + "# --- Lancer de rayons ---\n"
        + indent + "# Sans lui, RTAB-Map ne marque que les cellules\n"
        + indent + "# OCCUPEES : la chaussee parcourue reste a -1, et la\n"
        + indent + "# grille est une carte d'obstacles, pas une carte de\n"
        + indent + "# l'explore. Mesure sur valid_frontiere2.log : le gain\n"
        + indent + "# d'information valait 1.000 A LA POSITION MEME du\n"
        + indent + "# vehicule, pourtant bien a l'interieur de la grille.\n"
        + indent + "# Le critere le plus lourd de la decision (0,30) ne\n"
        + indent + "# pouvait donc rien classer.\n"
        + indent + "'Grid/RayTracing': 'true',\n"
    )
    lignes.insert(i + 1, ajout)
    src_launch = "".join(lignes)

    # --- Decideur : composition de la grille dans DIAG gain ---
    n = src_dec.count(ANCRE_WARN)
    if n != 1:
        raise SystemExit(
            "ANCRE 'DIAG gain' TROUVEE %d FOIS (attendu 1).\n"
            "Aucune modification effectuee." % n
        )
    src_dec = src_dec.replace(ANCRE_WARN, REMPLACEMENT_WARN)

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in ((LAUNCH, src_launch), (DECIDEUR, src_dec)):
        sauvegarde = "%s.bak_raytracing_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("")
    print("Grid/RayTracing : 'true' (insere apres Grid/RangeMax)")
    print("DIAG gain affiche desormais inconnu%% et libre%% de la grille.")
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % DECIDEUR)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis :")
    print("  bash ~/valider.sh raytracing")
    print("  grep 'DIAG gain' ~/valid_raytracing.log | tail -5")
    return 0


if __name__ == "__main__":
    sys.exit(main())
