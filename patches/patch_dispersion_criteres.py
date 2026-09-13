#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Journalise la DISPERSION des criteres sur l'ensemble des candidats.

POURQUOI
--------
Le CSV de decisions n'enregistre que les scores du candidat RETENU. Or
la selection maximise un score ou le gain d'information pese 0,30 : le
gagnant a donc mecaniquement le gain le plus eleve, qu'il y ait
dispersion entre candidats ou non.

Mesure sur un run apres le passage de Grid/RangeMax a 40 m :

    13 decisions avec 'information_gain': 1.0
     1 decision  avec 'information_gain': 0.0

Ce releve est compatible avec un critere parfaitement discriminant comme
avec un critere totalement sature. Il ne tranche rien.

decision_maker fait deja la bonne chose pour la securite :

    safety_min          = min sur TOUS les candidats
    candidats_penalises = combien ont un score < 1.0

C'est ce raisonnement qui manquait pour les deux criteres exploratoires.

CE QUE LE PATCH AJOUTE
----------------------
Trois colonnes au CSV et une ligne de journal par decision :

    info_gain_min / info_gain_max   etendue du gain d'information
    loop_closure_max                meilleur potentiel de fermeture

Lecture :
  - info_gain_min ~ info_gain_max ~ 1.0  -> critere SATURE, il ne classe
    rien, et la portee de la grille est encore trop courte ;
  - un ecart franc entre min et max      -> le critere discrimine, et le
    choix du candidat retenu a un sens.

Ces colonnes ne sont pas seulement un outil de diagnostic : elles
documentent, run par run, si chaque critere de la fonction de decision a
reellement pese. C'est une donnee du rapport.

Usage :
    python3 patch_dispersion_criteres.py
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


BLOCS = [
    # ------------------------------------------------------------------
    # 1. En-tete du CSV
    # ------------------------------------------------------------------
    (
        "                    'nb_candidats',\n"
        "                    'safety_min',\n"
        "                    'candidats_penalises'\n"
        "                ])\n",

        "                    'nb_candidats',\n"
        "                    'safety_min',\n"
        "                    'candidats_penalises',\n"
        "                    # Etendue des criteres exploratoires sur TOUS\n"
        "                    # les candidats. Sans elles, on ne lit que le\n"
        "                    # score du candidat retenu -- or la selection\n"
        "                    # maximise ce score, donc le gagnant a\n"
        "                    # mecaniquement le gain le plus eleve, que le\n"
        "                    # critere discrimine ou qu'il soit sature.\n"
        "                    'info_gain_min',\n"
        "                    'info_gain_max',\n"
        "                    'loop_closure_max'\n"
        "                ])\n",
    ),

    # ------------------------------------------------------------------
    # 2. Calcul des statistiques
    # ------------------------------------------------------------------
    (
        "        safety_values = [s['safety'] for s in scores.values()]\n"
        "        nb_candidats = len(safety_values)\n"
        "        safety_min = min(safety_values) if safety_values else 1.0\n"
        "        candidats_penalises = sum(1 for v in safety_values if v < 1.0)\n",

        "        safety_values = [s['safety'] for s in scores.values()]\n"
        "        nb_candidats = len(safety_values)\n"
        "        safety_min = min(safety_values) if safety_values else 1.0\n"
        "        candidats_penalises = sum(1 for v in safety_values if v < 1.0)\n"
        "\n"
        "        # Meme raisonnement que pour la securite, applique aux deux\n"
        "        # criteres exploratoires : c'est l'ETENDUE sur l'ensemble des\n"
        "        # candidats qui dit si un critere classe quelque chose. Le\n"
        "        # score du seul candidat retenu ne le dit pas.\n"
        "        ig_values = [s['information_gain'] for s in scores.values()]\n"
        "        lc_values = [s['loop_closure'] for s in scores.values()]\n"
        "        info_gain_min = min(ig_values) if ig_values else 0.0\n"
        "        info_gain_max = max(ig_values) if ig_values else 0.0\n"
        "        loop_closure_max = max(lc_values) if lc_values else 0.0\n",
    ),

    # ------------------------------------------------------------------
    # 3. Ligne de journal
    # ------------------------------------------------------------------
    (
        "        self.get_logger().info(\n"
        "            f\"SECURITE : {candidats_penalises}/{nb_candidats} candidats \"\n"
        "            f\"penalises | safety_min={safety_min:.4f} | \"\n"
        "            f\"safety_retenu={score['safety']:.4f}\"\n"
        "        )\n",

        "        self.get_logger().info(\n"
        "            f\"SECURITE : {candidats_penalises}/{nb_candidats} candidats \"\n"
        "            f\"penalises | safety_min={safety_min:.4f} | \"\n"
        "            f\"safety_retenu={score['safety']:.4f}\"\n"
        "        )\n"
        "\n"
        "        self.get_logger().info(\n"
        "            f\"CRITERES sur {nb_candidats} candidats : \"\n"
        "            f\"info_gain [{info_gain_min:.3f} - {info_gain_max:.3f}] \"\n"
        "            f\"etendue={info_gain_max - info_gain_min:.3f} | \"\n"
        "            f\"loop_closure max={loop_closure_max:.3f}\"\n"
        "        )\n",
    ),

    # ------------------------------------------------------------------
    # 4. Ecriture de la ligne CSV
    # ------------------------------------------------------------------
    (
        "            nb_candidats,\n"
        "            safety_min,\n"
        "            candidats_penalises,\n"
        "        ])\n",

        "            nb_candidats,\n"
        "            safety_min,\n"
        "            candidats_penalises,\n"
        "            info_gain_min,\n"
        "            info_gain_max,\n"
        "            loop_closure_max,\n"
        "        ])\n",
    ),
]


def main():
    if not os.path.exists(CIBLE):
        raise SystemExit("Fichier introuvable : %s" % CIBLE)

    src = io.open(CIBLE, encoding="utf-8").read()

    if "info_gain_min" in src:
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    for ancre, remplacement in BLOCS:
        n = src.count(ancre)
        if n != 1:
            raise SystemExit(
                "ANCRE TROUVEE %d FOIS (attendu 1) :\n---\n%s\n---\n"
                "Aucune modification effectuee." % (n, ancre[:300])
            )
        src = src.replace(ancre, remplacement)

    sauvegarde = "%s.bak_dispersion_%s" % (CIBLE, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(CIBLE, sauvegarde)
    io.open(CIBLE, "w", encoding="utf-8").write(src)

    print("4 blocs inseres dans decision_maker.py")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Reconstruire :")
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    return 0


if __name__ == "__main__":
    sys.exit(main())
