#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
INSTRUMENTATION TEMPORAIRE : emprise de la grille vs position des
candidats.

CE QUI RESTE A EXPLIQUER
------------------------
Run valid_origine.log, premier run complet du projet :

    FIN DE RUN (budget_reached) : 602m / 600m | destinations
    atteintes=13
    grid=OK sur 39 releves sur 40
    loop_closure max = 0.571 a 0.600

Tout fonctionne, sauf un point :

    CRITERES sur 6 candidats : info_gain [1.000 - 1.000] etendue=0.000

Le correctif de frontiere EST applique (verifie dans le source). Un
gain de 1,000 signifie donc que TOUTES les cellules du disque
d'evaluation de CHAQUE candidat sont inconnues ou hors grille.

DEUX LECTURES POSSIBLES, QUE CE PATCH DEPARTAGE
-----------------------------------------------
1. Les candidats tombent hors de l'emprise de la grille. RTAB-Map ne
   cartographie qu'un couloir etroit le long du trajet parcouru ; des
   candidats situes 25 a 150 m devant le vehicule peuvent tous etre
   au-dela. Dans ce cas 1,000 est la reponse CORRECTE, et le vrai
   probleme est que le generateur ne propose jamais de destination en
   terrain deja vu. Ce n'est pas un bug du critere.

2. Les candidats tombent DANS l'emprise mais la grille y est vide, ou
   la conversion monde -> cellule est decalee. Dans ce cas c'est bien
   un defaut de calcul.

La difference se voit en comparant deux rectangles. Ce patch les
affiche.

CE QU'IL AJOUTE
---------------
Une ligne de journal par decision, avant l'evaluation des candidats :

    DIAG grille : x=[-12.3,208.7] y=[-91.2,14.5] res=0.15 1473x705
                | pose_grille=(221.4,65.8)
                | candidats (245.1,88.3), (251.0,95.7), (238.2,79.4)

Lecture directe : si les coordonnees des candidats sortent de
l'intervalle x=[...] ou y=[...], c'est la lecture 1. Si elles sont
dedans, c'est la lecture 2.

Les coordonnees des candidats sont affichees APRES la transposition
(c.x, -c.y) reellement passee a compute_information_gain, pas avant :
c'est cette valeur-la qui interroge la grille.

A RETIRER APRES DIAGNOSTIC
--------------------------
Comme l'instrumentation DIAG du planificateur. La sauvegarde creee par
ce patch permet de revenir en arriere, mais il vaut mieux passer par un
patch inverse : restaurer la sauvegarde effacerait les correctifs
appliques apres elle.

Usage :
    python3 patch_diag_grille.py
"""

import io
import os
import shutil
import sys
import time

CIBLE = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/"
    "active_slam_decision/decision_maker.py"
)

ANCRE = (
    "        for c in candidates:\n"
    "            distance = math.hypot(\n"
    "                c.x - self.current_x,\n"
    "                c.y - self.current_y\n"
    "            )\n"
)

REMPLACEMENT = (
    "        # --- INSTRUMENTATION TEMPORAIRE (a retirer apres diagnostic) ---\n"
    "        # Compare l'emprise reelle de la grille d'occupation a la\n"
    "        # position des candidats, dans le repere ou la grille est\n"
    "        # interrogee. Un gain d'information de 1,000 pour tous les\n"
    "        # candidats a deux causes possibles, que seule cette\n"
    "        # comparaison separe : candidats hors emprise (le critere\n"
    "        # repond correctement, c'est le generateur qui ne propose que\n"
    "        # du terrain jamais vu), ou candidats dans l'emprise mais\n"
    "        # conversion monde -> cellule decalee (defaut de calcul).\n"
    "        if self.current_grid is not None:\n"
    "            gi = self.current_grid.info\n"
    "            gx0 = gi.origin.position.x\n"
    "            gy0 = gi.origin.position.y\n"
    "            gx1 = gx0 + gi.width * gi.resolution\n"
    "            gy1 = gy0 + gi.height * gi.resolution\n"
    "            # Memes coordonnees que celles passees a\n"
    "            # compute_information_gain : (c.x, -c.y).\n"
    "            apercu = ', '.join(\n"
    "                '(%.1f,%.1f)' % (c.x, -c.y) for c in candidates[:3]\n"
    "            )\n"
    "            self.get_logger().warn(\n"
    "                f\"DIAG grille : x=[{gx0:.1f},{gx1:.1f}] \"\n"
    "                f\"y=[{gy0:.1f},{gy1:.1f}] res={gi.resolution:.2f} \"\n"
    "                f\"{gi.width}x{gi.height} | \"\n"
    "                f\"pose_grille=({self.current_x:.1f},\"\n"
    "                f\"{-self.current_y:.1f}) | \"\n"
    "                f\"candidats {apercu}\"\n"
    "            )\n"
    "\n"
    "        for c in candidates:\n"
    "            distance = math.hypot(\n"
    "                c.x - self.current_x,\n"
    "                c.y - self.current_y\n"
    "            )\n"
)


def main():
    if not os.path.exists(CIBLE):
        raise SystemExit("Fichier introuvable : %s" % CIBLE)

    src = io.open(CIBLE, encoding="utf-8").read()

    if "DIAG grille" in src:
        raise SystemExit(
            "L'instrumentation semble deja en place.\n"
            "Aucune modification effectuee."
        )

    n = src.count(ANCRE)
    if n != 1:
        raise SystemExit(
            "ANCRE TROUVEE %d FOIS (attendu 1).\n"
            "Aucune modification effectuee." % n
        )

    src = src.replace(ANCRE, REMPLACEMENT)

    sauvegarde = "%s.bak_diaggrille_%s" % (CIBLE, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(CIBLE, sauvegarde)
    io.open(CIBLE, "w", encoding="utf-8").write(src)

    print("DIAG grille insere dans decision_maker.py")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % CIBLE)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis :")
    print("  bash ~/valider.sh grille")
    print("  grep 'DIAG grille' ~/valid_grille.log | tail -5")
    return 0


if __name__ == "__main__":
    sys.exit(main())
