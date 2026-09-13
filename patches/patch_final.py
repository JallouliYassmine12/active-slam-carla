#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nettoyage final et alignement des deux configurations RTAB-Map.

CE QUI EST ACQUIS
-----------------
Run valid_sol.log, apres desactivation de Grid/NormalsSegmentation :

    grille inconnu=55.9% libre=43.6%
    au vehicule=0.000
    info_gain [0.490 - 0.693] etendue=0.203
    info_gain [0.000 - 0.549] etendue=0.549
    info_gain [0.034 - 0.727] etendue=0.694
    FIN DE RUN (budget_reached) : 602m / 600m, 12 destinations, 131s

La carte contient enfin du terrain libre, le vehicule reconnait la
route qu'il vient de parcourir, et le gain d'information classe les
candidats. Les cinq criteres de la fonction de decision sont
operationnels.

A. RETRAIT DE L'INSTRUMENTATION DIAG gain
=========================================
Elle a servi a etablir toute la chaine : grille lue, composition,
position interrogee, gain au vehicule. Sa mission est finie et les
journaux de la campagne doivent rester lisibles.

Les lignes CRITERES restent, elles : elles portent l'etendue de chaque
critere sur l'ensemble des candidats, decision par decision, et
constituent une donnee du rapport -- c'est ce qui permettra d'ecrire
quel critere a reellement pese.

B. ALIGNEMENT DU SLAM CLASSIQUE
===============================
slam_evaluation.launch.py a son propre bloc de parametres RTAB-Map,
reste avec la segmentation par normales et sans lancer de rayons. Sa
carte serait donc construite autrement que celle de l'Active SLAM.

Le sujet demande de comparer la QUALITE DE LA CARTE entre les methodes.
Deux cartes produites par des reglages differents ne sont pas
comparables : l'ecart mesure viendrait de la configuration de RTAB-Map
et non de la strategie d'exploration. Les trois memes parametres sont
donc appliques des deux cotes :

    'Grid/3D':                  'false'
    'Grid/RayTracing':          'true'
    'Grid/NormalsSegmentation': 'false'

C'est une condition de validite de la campagne, pas une amelioration.

VERIFICATION
------------
    grep -c "DIAG gain" ~/valid_*.log      # 0 sur les prochains runs
    bash ~/valider.sh final

Puis, si ce dernier run passe, la campagne :

    nohup bash ~/campagne_complete.sh > ~/campagne.log 2>&1 &

Usage :
    python3 patch_final.py
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


ANCRE_DIAG = '        # --- INSTRUMENTATION TEMPORAIRE (a retirer apres diagnostic) ---\n        # Le gain d\'information reste a 1.000 pour tous les candidats,\n        # y compris depuis le passage a l\'evaluation le long du trajet,\n        # alors que la grille est bien recue. Une mesure tranche : le\n        # gain A LA POSITION DU VEHICULE. Il roule la, donc c\'est\n        # cartographie, donc il doit valoir ~0.\n        #\n        #   ~0 -> la grille est interrogee dans le bon repere, et la\n        #         saturation est geometrique : les trajets traversent\n        #         reellement du terrain inconnu.\n        #   ~1 -> la grille est interrogee loin de la zone cartographiee.\n        #         La conversion de repere est fausse, et l\'etait depuis\n        #         le debut.\n        if self.current_grid is not None and candidates:\n            gi = self.current_grid.info\n            gain_vehicule = compute_information_gain(\n                self.current_grid,\n                self.current_x,\n                -self.current_y,\n                self.path_corridor_radius\n            )\n            c0 = candidates[0]\n            gain_arrivee = compute_information_gain(\n                self.current_grid, c0.x, -c0.y, self.path_corridor_radius\n            )\n            gain_trajet = compute_path_information_gain(\n                self.current_grid,\n                self.current_x,\n                -self.current_y,\n                c0.x,\n                -c0.y,\n                sample_step=self.path_sample_step,\n                corridor_radius=self.path_corridor_radius\n            )\n            # Composition de la grille. Un critere de gain\n            # d\'information mesure la proportion de cellules INCONNUES ;\n            # encore faut-il que la grille contienne des cellules LIBRES.\n            # RTAB-Map ne marque libre l\'espace entre le capteur et les\n            # obstacles que si Grid/RayTracing est actif. Sans lui la\n            # grille ne contient que les facades detectees, et tout le\n            # reste -- y compris la chaussee deja parcourue -- reste a\n            # -1. count() travaille en C, le cout reste negligeable.\n            cellules = self.current_grid.data\n            total_cellules = len(cellules) or 1\n            pct_inconnu = 100.0 * cellules.count(-1) / total_cellules\n            pct_libre = 100.0 * cellules.count(0) / total_cellules\n            self.get_logger().warn(\n                f"DIAG gain : au vehicule={gain_vehicule:.3f} | "\n                f"candidat 0 : trajet={gain_trajet:.3f} "\n                f"arrivee={gain_arrivee:.3f} | "\n                f"pose=({self.current_x:.1f},{-self.current_y:.1f}) "\n                f"cand=({c0.x:.1f},{-c0.y:.1f}) | "\n                f"grille inconnu={pct_inconnu:.1f}% "\n                f"libre={pct_libre:.1f}% | "\n                f"x=[{gi.origin.position.x:.1f},"\n                f"{gi.origin.position.x + gi.width * gi.resolution:.1f}] "\n                f"y=[{gi.origin.position.y:.1f},"\n                f"{gi.origin.position.y + gi.height * gi.resolution:.1f}]"\n            )\n\n        for c in candidates:\n            distance = cout_distance(c)\n'

REMPLACEMENT_DIAG = '        for c in candidates:\n            distance = cout_distance(c)\n'


def main():
    for chemin in (DECIDEUR, LAUNCH_EVAL):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    src_dec = io.open(DECIDEUR, encoding="utf-8").read()
    src_eval = io.open(LAUNCH_EVAL, encoding="utf-8").read()

    # --- A. Retrait de DIAG gain ---
    n = src_dec.count(ANCRE_DIAG)
    if n != 1:
        raise SystemExit(
            "ANCRE 'DIAG gain' TROUVEE %d FOIS (attendu 1) dans\n"
            "decision_maker.py. Aucun fichier n'a ete modifie." % n
        )
    src_dec = src_dec.replace(ANCRE_DIAG, REMPLACEMENT_DIAG)

    # --- B. Alignement de slam_evaluation ---
    # Insertion par balayage de lignes : l'indentation du bloc RTAB-Map
    # de ce fichier n'est pas connue a l'avance et doit etre reproduite.
    deja = [p for p in ("Grid/RayTracing", "Grid/NormalsSegmentation")
            if p in src_eval]
    if deja:
        raise SystemExit(
            "Deja present dans slam_evaluation.launch.py : %s\n"
            "Ajouter une seconde fois la meme cle produirait un doublon\n"
            "silencieux. Aucun fichier n'a ete modifie -- corriger les\n"
            "valeurs a la main." % ", ".join(deja)
        )

    lignes = src_eval.splitlines(True)
    indices = [i for i, l in enumerate(lignes) if "'Grid/RangeMax'" in l]
    if len(indices) != 1:
        raise SystemExit(
            "'Grid/RangeMax' TROUVE %d FOIS (attendu 1) dans\n"
            "slam_evaluation.launch.py. Aucun fichier n'a ete modifie."
            % len(indices)
        )

    i = indices[0]
    indent = lignes[i][:len(lignes[i]) - len(lignes[i].lstrip())]
    ajout = (
        indent + "# --- Alignement sur la configuration Active SLAM ---\n"
        + indent + "# Le sujet demande de comparer la QUALITE DE LA CARTE\n"
        + indent + "# entre les methodes. Deux cartes construites avec des\n"
        + indent + "# reglages RTAB-Map differents ne sont pas comparables :\n"
        + indent + "# l'ecart mesure viendrait de la configuration et non de\n"
        + indent + "# la strategie d'exploration.\n"
        + indent + "#\n"
        + indent + "# Sans ces trois parametres, la grille ne contient que des\n"
        + indent + "# facades : mesure cote Active SLAM, 2,4 % de cellules\n"
        + indent + "# connues et 0,0 % de libre, contre 43,6 % de libre une\n"
        + indent + "# fois la segmentation du sol ramenee a un seuil de\n"
        + indent + "# hauteur.\n"
        + indent + "'Grid/3D': 'false',\n"
        + indent + "'Grid/RayTracing': 'true',\n"
        + indent + "'Grid/NormalsSegmentation': 'false',\n"
    )
    lignes.insert(i + 1, ajout)
    src_eval = "".join(lignes)

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in ((DECIDEUR, src_dec), (LAUNCH_EVAL, src_eval)):
        sauvegarde = "%s.bak_final_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("")
    print("A. Instrumentation DIAG gain retiree.")
    print("B. slam_evaluation aligne : Grid/3D=false, RayTracing=true,")
    print("   NormalsSegmentation=false.")
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % DECIDEUR)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Dernier run de controle, puis la campagne :")
    print("  bash ~/valider.sh final")
    print("  nohup bash ~/campagne_complete.sh > ~/campagne.log 2>&1 &")
    return 0


if __name__ == "__main__":
    sys.exit(main())
