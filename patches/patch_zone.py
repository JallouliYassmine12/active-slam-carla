#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Patch : restreint les destinations candidates a une ZONE BORNEE.

Voie 1 du plan de correction. La tache passe de "parcourir 1200 m" a
"cartographier completement une zone", ce qui est la seule formulation
ou une politique de decision peut battre une trajectoire predefinie :
sur un budget de distance en reseau ouvert, la ligne droite atteint
89 % du maximum geometrique et ne laisse aucune marge.

La zone est un carre centre sur le point d'apparition, exprime dans le
repere 'map'. Son demi-cote vient de la variable d'environnement
ZONE_HALF_SIZE (0 = desactive, comportement strictement inchange), donc
AUCUN fichier de lancement n'est modifie.

Sauvegarde horodatee, ancres verifiees uniques, py_compile.
"""

import os
import re
import shutil
import subprocess
import sys
import time

CIBLE = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/"
    "active_slam_decision/candidate_generator.py")

# (description, ancre, remplacement)
REMPLACEMENTS = [
    (
        "import os",
        "import math\nfrom collections import deque\n",
        "import math\nimport os\nfrom collections import deque\n",
    ),
    (
        "parametre zone_half_size",
        "        # Garde-fou sur la taille du message publie.\n"
        "        self.declare_parameter('max_candidates', 400)\n",
        "        # Garde-fou sur la taille du message publie.\n"
        "        self.declare_parameter('max_candidates', 400)\n"
        "\n"
        "        # --- ZONE BORNEE ---\n"
        "        # Demi-cote du carre centre sur le point d'apparition,\n"
        "        # dans le repere 'map'. Passe par l'environnement plutot\n"
        "        # que par le fichier de lancement : un seul fichier du\n"
        "        # workspace est ainsi modifie, comme pour la resolution\n"
        "        # camera de la variante F.\n"
        "        # 0 = desactive : comportement strictement inchange.\n"
        "        self.declare_parameter(\n"
        "            'zone_half_size',\n"
        "            float(os.environ.get('ZONE_HALF_SIZE', 0.0)))\n",
    ),
    (
        "lecture du parametre",
        "        self.max_candidates = int(self.get_parameter('max_candidates').value)\n",
        "        self.max_candidates = int(self.get_parameter('max_candidates').value)\n"
        "        self.zone_half_size = float(\n"
        "            self.get_parameter('zone_half_size').value)\n",
    ),
    (
        "trace de la zone",
        "        self.timer = self.create_timer(publish_period, self.publish_candidates)\n",
        "        if self.zone_half_size > 0.0:\n"
        "            self.get_logger().info(\n"
        "                f\"ZONE BORNEE : carre de \"\n"
        "                f\"{2 * self.zone_half_size:.0f} m de cote, centre sur \"\n"
        "                f\"le point d'apparition (repere map).\")\n"
        "\n"
        "        self.timer = self.create_timer(publish_period, self.publish_candidates)\n",
    ),
    (
        "appel du filtre",
        "        return candidates\n",
        "        return self._filtrer_zone(candidates)\n",
    ),
    (
        "methode _filtrer_zone",
        "    # ------------------------------------------------------------------\n"
        "    # Publication\n"
        "    # ------------------------------------------------------------------\n",
        "    def _filtrer_zone(self, candidates):\n"
        "        \"\"\"Ne garde que les candidats situes dans la zone bornee.\n"
        "\n"
        "        La zone materialise la tache : cartographier completement\n"
        "        une region donnee, et non parcourir une distance. C'est la\n"
        "        seule asymetrie introduite par rapport a la reference --\n"
        "        et elle est honnete, une trajectoire predefinie ne pouvant\n"
        "        par construction tenir compte d'une carte en cours de\n"
        "        construction.\n"
        "\n"
        "        REPLI INDISPENSABLE. Si le vehicule est sorti de la zone,\n"
        "        aucun candidat interieur n'est necessairement atteignable\n"
        "        dans la portee de parcours. Retourner une liste vide\n"
        "        priverait le decideur de tout but et immobiliserait le\n"
        "        vehicule : le run serait perdu sans qu'aucune erreur ne\n"
        "        soit levee. On retourne alors les candidats les plus\n"
        "        PROCHES DU CENTRE, ce qui ramene le vehicule vers la zone.\n"
        "        \"\"\"\n"
        "        if self.zone_half_size <= 0.0 or self.origin_x is None:\n"
        "            return candidates\n"
        "\n"
        "        dedans, dehors = [], []\n"
        "        for wp, dist in candidates:\n"
        "            loc = wp.transform.location\n"
        "            mx, my = self._carla_to_map(loc.x, loc.y)\n"
        "            if (abs(mx) <= self.zone_half_size\n"
        "                    and abs(my) <= self.zone_half_size):\n"
        "                dedans.append((wp, dist))\n"
        "            else:\n"
        "                dehors.append((wp, dist, math.hypot(mx, my)))\n"
        "\n"
        "        if dedans:\n"
        "            return dedans\n"
        "        if not dehors:\n"
        "            return []\n"
        "\n"
        "        dehors.sort(key=lambda t: t[2])\n"
        "        garde = dehors[:8]\n"
        "        self.get_logger().warn(\n"
        "            f\"HORS ZONE : aucun candidat interieur atteignable, \"\n"
        "            f\"repli sur les {len(garde)} plus proches du centre \"\n"
        "            f\"(le plus proche a {garde[0][2]:.0f} m).\")\n"
        "        return [(wp, dist) for wp, dist, _ in garde]\n"
        "\n"
        "    # ------------------------------------------------------------------\n"
        "    # Publication\n"
        "    # ------------------------------------------------------------------\n",
    ),
]


def echec(message):
    print("ECHEC : " + message)
    print("Aucune modification n'a ete appliquee.")
    sys.exit(1)


def main():
    if not os.path.isfile(CIBLE):
        echec("fichier introuvable : " + CIBLE)

    with open(CIBLE, encoding="utf-8") as f:
        src = f.read()

    if "zone_half_size" in src:
        print("Le patch est deja applique (zone_half_size present).")
        sys.exit(0)

    # --- Verification prealable de TOUTES les ancres ---
    for description, ancre, _ in REMPLACEMENTS:
        n = src.count(ancre)
        if n != 1:
            echec("ancre '%s' trouvee %d fois (attendu 1)." % (description, n))
    print("  %d ancres verifiees, toutes uniques." % len(REMPLACEMENTS))

    # --- Sauvegarde ---
    sauvegarde = CIBLE + ".bak_" + time.strftime("%Y%m%d_%H%M%S")
    shutil.copy2(CIBLE, sauvegarde)
    print("  sauvegarde : " + os.path.basename(sauvegarde))

    # --- Application ---
    for description, ancre, remplacement in REMPLACEMENTS:
        src = src.replace(ancre, remplacement, 1)
        print("  applique : " + description)

    with open(CIBLE, "w", encoding="utf-8") as f:
        f.write(src)

    # --- Verification syntaxique ---
    r = subprocess.run([sys.executable, "-m", "py_compile", CIBLE],
                       capture_output=True, text=True)
    if r.returncode != 0:
        shutil.copy2(sauvegarde, CIBLE)
        print(r.stderr)
        echec("py_compile a echoue ; le fichier d'origine a ete restaure.")
    print("  py_compile : OK")

    print("")
    print("Patch applique. Reconstruis le workspace :")
    print("    cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")


if __name__ == "__main__":
    main()
