#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nettoyage final, et alignement des deux configurations RTAB-Map.

CE QUI EST ACQUIS
-----------------
Runs valid_sol.log et valid_final.log, apres desactivation de
Grid/NormalsSegmentation cote Active SLAM :

    grille inconnu=55.9% libre=43.6%
    au vehicule=0.000
    info_gain [0.045 - 0.840] etendue=0.795
    info_gain [0.000 - 0.865] etendue=0.865
    info_gain [0.000 - 0.634] etendue=0.634 | loop_closure max=0.870

La carte contient du terrain libre, le vehicule reconnait la route
qu'il vient de parcourir, et le gain d'information classe enfin les
candidats. La derniere ligne montre le gain d'information ET la
fermeture de boucle discriminant sur la MEME decision : c'est
exactement l'arbitrage que le sujet demande -- explorer du nouveau, ou
revenir se recaler.

Les cinq criteres de la fonction de decision sont operationnels.

A. RETRAIT DE L'INSTRUMENTATION DIAG gain
=========================================
Elle a etabli toute la chaine de diagnostic : quelle grille est lue, sa
composition, la position interrogee, le gain a la position du vehicule.
Sa mission est finie et les journaux de campagne doivent rester
lisibles.

Les lignes CRITERES restent : elles portent l'etendue de chaque critere
sur l'ensemble des candidats, decision par decision. C'est une donnee
du rapport -- c'est elle qui permettra d'ecrire quel critere a
reellement pese sur les choix.

B. ALIGNEMENT DU SLAM CLASSIQUE
===============================
slam_evaluation.launch.py porte exactement la configuration que
l'Active SLAM avait avant ce soir :

    'Grid/3D':         'true'     (ligne 352)
    'Grid/RayTracing': 'false'    (ligne 353)
    Grid/NormalsSegmentation absent, donc actif par defaut

Sa carte serait donc une carte de facades, sans aucune cellule libre --
le defaut qui vient d'etre corrige cote Active SLAM.

Le sujet demande de comparer la QUALITE DE LA CARTE entre les methodes.
Deux cartes construites avec des reglages RTAB-Map differents ne sont
pas comparables : l'ecart mesure viendrait de la configuration et non
de la strategie d'exploration. Les trois parametres sont donc alignes.

C'est une condition de validite de la campagne, pas une amelioration,
et elle doit figurer dans la partie protocole du rapport.

VERIFICATION
------------
    bash ~/valider.sh propre2
    grep -c "DIAG gain" ~/valid_propre2.log      # 0
    grep "CRITERES" ~/valid_propre2.log | tail -8

Puis la campagne :

    nohup bash ~/campagne_complete.sh > ~/campagne.log 2>&1 &
    bash ~/suivi.sh          # dans un second terminal

Usage :
    python3 patch_final2.py
"""

import io
import os
import shutil
import sys
import time

DECIDEUR = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/"
    "active_slam_decision/decision_maker.py"
)
LAUNCH_EVAL = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/slam_evaluation/launch/"
    "slam_evaluation.launch.py"
)


ANCRE_3D = "                'Grid/3D': 'true',\n"
REMPLACEMENT_3D = (
    "                # Aligne sur active_slam.launch.py. En 3D, le lancer de\n"
    "                # rayons se fait en volume : lent au point de\n"
    "                # compromettre la cartographie temps reel, alors que la\n"
    "                # grille exploitee est de toute facon une projection 2D.\n"
    "                'Grid/3D': 'false',\n"
)

ANCRE_RAY = "                'Grid/RayTracing': 'false',\n"
REMPLACEMENT_RAY = (
    "                # Aligne sur active_slam.launch.py. Sans lancer de\n"
    "                # rayons, RTAB-Map ne marque que les cellules OCCUPEES :\n"
    "                # la chaussee parcourue reste inconnue et la grille est\n"
    "                # une carte des murs. Mesure cote Active SLAM avant\n"
    "                # correction : 2,4 % de cellules connues, 0,0 % de\n"
    "                # libre ; apres correction, 43,6 % de libre.\n"
    "                'Grid/RayTracing': 'true',\n"
)

ANCRE_SOL = "                'Grid/MaxGroundHeight': '1.0',\n"
REMPLACEMENT_SOL = (
    "                # Aligne sur active_slam.launch.py. La segmentation par\n"
    "                # normales echoue sur un LiDAR automobile, ou la\n"
    "                # chaussee est vue en incidence rasante : les points du\n"
    "                # sol sont rejetes comme obstacles, le lancer de rayons\n"
    "                # n'a rien vers quoi tracer, et aucune cellule n'est\n"
    "                # marquee libre. A 'false', le sol est identifie par\n"
    "                # simple seuil de hauteur (Grid/MaxGroundHeight).\n"
    "                'Grid/NormalsSegmentation': 'false',\n"
    "                'Grid/MaxGroundHeight': '1.0',\n"
)


ANCRE_DIAG = '        # --- INSTRUMENTATION TEMPORAIRE (a retirer apres diagnostic) ---\n        # Le gain d\'information reste a 1.000 pour tous les candidats,\n        # y compris depuis le passage a l\'evaluation le long du trajet,\n        # alors que la grille est bien recue. Une mesure tranche : le\n        # gain A LA POSITION DU VEHICULE. Il roule la, donc c\'est\n        # cartographie, donc il doit valoir ~0.\n        #\n        #   ~0 -> la grille est interrogee dans le bon repere, et la\n        #         saturation est geometrique : les trajets traversent\n        #         reellement du terrain inconnu.\n        #   ~1 -> la grille est interrogee loin de la zone cartographiee.\n        #         La conversion de repere est fausse, et l\'etait depuis\n        #         le debut.\n        if self.current_grid is not None and candidates:\n            gi = self.current_grid.info\n            gain_vehicule = compute_information_gain(\n                self.current_grid,\n                self.current_x,\n                -self.current_y,\n                self.path_corridor_radius\n            )\n            c0 = candidates[0]\n            gain_arrivee = compute_information_gain(\n                self.current_grid, c0.x, -c0.y, self.path_corridor_radius\n            )\n            gain_trajet = compute_path_information_gain(\n                self.current_grid,\n                self.current_x,\n                -self.current_y,\n                c0.x,\n                -c0.y,\n                sample_step=self.path_sample_step,\n                corridor_radius=self.path_corridor_radius\n            )\n            # Composition de la grille. Un critere de gain\n            # d\'information mesure la proportion de cellules INCONNUES ;\n            # encore faut-il que la grille contienne des cellules LIBRES.\n            # RTAB-Map ne marque libre l\'espace entre le capteur et les\n            # obstacles que si Grid/RayTracing est actif. Sans lui la\n            # grille ne contient que les facades detectees, et tout le\n            # reste -- y compris la chaussee deja parcourue -- reste a\n            # -1. count() travaille en C, le cout reste negligeable.\n            cellules = self.current_grid.data\n            total_cellules = len(cellules) or 1\n            pct_inconnu = 100.0 * cellules.count(-1) / total_cellules\n            pct_libre = 100.0 * cellules.count(0) / total_cellules\n            self.get_logger().warn(\n                f"DIAG gain : au vehicule={gain_vehicule:.3f} | "\n                f"candidat 0 : trajet={gain_trajet:.3f} "\n                f"arrivee={gain_arrivee:.3f} | "\n                f"pose=({self.current_x:.1f},{-self.current_y:.1f}) "\n                f"cand=({c0.x:.1f},{-c0.y:.1f}) | "\n                f"grille inconnu={pct_inconnu:.1f}% "\n                f"libre={pct_libre:.1f}% | "\n                f"x=[{gi.origin.position.x:.1f},"\n                f"{gi.origin.position.x + gi.width * gi.resolution:.1f}] "\n                f"y=[{gi.origin.position.y:.1f},"\n                f"{gi.origin.position.y + gi.height * gi.resolution:.1f}]"\n            )\n\n        for c in candidates:\n            distance = cout_distance(c)\n'

REMPLACEMENT_DIAG = '        for c in candidates:\n            distance = cout_distance(c)\n'


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
    for chemin in (DECIDEUR, LAUNCH_EVAL):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    src_dec = io.open(DECIDEUR, encoding="utf-8").read()
    src_eval = io.open(LAUNCH_EVAL, encoding="utf-8").read()

    if "Grid/NormalsSegmentation" in src_eval:
        raise SystemExit(
            "slam_evaluation.launch.py contient deja\n"
            "Grid/NormalsSegmentation : correctif probablement deja\n"
            "applique. Aucun fichier n'a ete modifie."
        )

    src_dec = remplacer(src_dec, ANCRE_DIAG, REMPLACEMENT_DIAG,
                        "DIAG gain", DECIDEUR)

    src_eval = remplacer(src_eval, ANCRE_3D, REMPLACEMENT_3D,
                         "Grid/3D", LAUNCH_EVAL)
    src_eval = remplacer(src_eval, ANCRE_RAY, REMPLACEMENT_RAY,
                         "Grid/RayTracing", LAUNCH_EVAL)
    src_eval = remplacer(src_eval, ANCRE_SOL, REMPLACEMENT_SOL,
                         "Grid/MaxGroundHeight", LAUNCH_EVAL)

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in ((DECIDEUR, src_dec), (LAUNCH_EVAL, src_eval)):
        sauvegarde = "%s.bak_final2_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("")
    print("A. Instrumentation DIAG gain retiree du decideur.")
    print("B. slam_evaluation aligne :")
    print("     Grid/3D                  : 'true'  -> 'false'")
    print("     Grid/RayTracing          : 'false' -> 'true'")
    print("     Grid/NormalsSegmentation : ajoute a 'false'")
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % DECIDEUR)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Dernier controle, puis la campagne :")
    print("  bash ~/valider.sh propre2")
    print("  nohup bash ~/campagne_complete.sh > ~/campagne.log 2>&1 &")
    return 0


if __name__ == "__main__":
    sys.exit(main())
