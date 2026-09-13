#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Le decideur retient automatiquement la grille d'occupation qui contient
reellement des cellules connues.

LE FAIT MESURE
--------------
Instrumentation DIAG gain, run valid_raytracing2.log, apres activation
du lancer de rayons :

    au vehicule=1.000 | grille inconnu=100.0% libre=0.0%
                      | x=[-1.6,221.5] y=[-1.7,32.8]
    au vehicule=1.000 | grille inconnu=100.0% libre=0.0%
                      | x=[-1.6,236.5] y=[-1.7,187.7]

CENT POUR CENT des cellules valent -1. La grille recue sur
/grid_prob_map est integralement vide : sa boite englobante grandit
avec la trajectoire, mais aucune cellule n'est jamais renseignee, ni
libre ni occupee.

Cela disqualifie toutes les causes envisagees jusqu'ici -- repere,
frontiere, distance des candidats, rayon d'evaluation, lancer de rayons.
Aucune ne peut produire du contenu dans une carte qui n'en a pas.

LA CAUSE
--------
/grid_prob_map est un topic OPTIONNEL de RTAB-Map : la grille
probabiliste n'est remplie que dans certaines configurations. La grille
d'occupation standard, elle, est publiee sur /map.

Le decideur ecoutait donc, depuis le debut du projet, un topic qui lui
repondait une carte allouee mais jamais renseignee.

LA CORRECTION
-------------
Plutot que de remplacer un nom de topic par un autre -- ce qui
demanderait de le verifier a nouveau, et de recommencer si la
configuration RTAB-Map change --, le decideur s'abonne aux DEUX et
retient celle qui contient effectivement des cellules connues.

Le choix est journalise, une seule fois par changement :

    GRILLE ACTIVE : /map (connu=34.2%) --
    /grid_prob_map ecartee (connu=0.0%)

C'est aussi une protection durable : si la configuration evolue, le
critere suivra la carte qui a du contenu au lieu de saturer en silence.

COUT
----
La proportion de cellules connues se mesure par data.count(-1), qui
s'execute en C. Le calcul n'est refait qu'au plus une fois toutes les
deux secondes, alors que les cartes arrivent plus souvent.

VERIFICATION
------------
    bash ~/valider.sh grille
    grep "GRILLE ACTIVE" ~/valid_grille.log | head -3
    grep "DIAG gain"     ~/valid_grille.log | tail -5
    grep "CRITERES"      ~/valid_grille.log | tail -10

Attendu :
  - une ligne GRILLE ACTIVE nommant le topic retenu et sa proportion de
    connu ;
  - 'libre' franchement superieur a 0 % ;
  - 'au vehicule' proche de 0 -- le vehicule reconnait la route qu'il
    vient de parcourir ;
  - une etendue non nulle sur les decisions offrant plusieurs
    directions.

Si les DEUX grilles restent a 0 % de connu, le probleme est dans la
production de la carte cote RTAB-Map et non dans sa consommation : il
faudra alors regarder la segmentation du sol (Grid/MaxGroundHeight,
Grid/MinGroundHeight, Grid/NormalsSegmentation) et verifier que le
nuage LiDAR arrive bien au noeud.

Usage :
    python3 patch_grille_active.py
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


ANCRE_ETAT = "        self.current_grid = None\n"

REMPLACEMENT_ETAT = (
    "        # --- Grille d'occupation ---\n"
    "        # Deux sources possibles. /grid_prob_map est un topic OPTIONNEL\n"
    "        # de RTAB-Map : sa grille probabiliste n'est remplie que dans\n"
    "        # certaines configurations. Mesure sur valid_raytracing2.log :\n"
    "        # 100,0 % de cellules a -1, boite englobante qui grandit avec la\n"
    "        # trajectoire mais aucun contenu. /map porte la grille\n"
    "        # d'occupation standard.\n"
    "        #\n"
    "        # On s'abonne aux deux et on retient celle qui a du contenu,\n"
    "        # plutot que de figer un nom de topic qu'il faudrait revalider a\n"
    "        # chaque changement de configuration.\n"
    "        self.current_grid = None       # celle effectivement utilisee\n"
    "        self.grille_prob = None        # /grid_prob_map\n"
    "        self.grille_std = None         # /map\n"
    "        self.grille_nom = None         # topic retenu, pour le journal\n"
    "        self.grille_dernier_choix = 0.0\n"
)


ANCRE_SUB = (
    "        self.create_subscription(\n"
    "            OccupancyGrid,\n"
    "            '/grid_prob_map',\n"
    "            self.on_grid,\n"
    "            10\n"
    "        )\n"
)

REMPLACEMENT_SUB = (
    "        self.create_subscription(\n"
    "            OccupancyGrid,\n"
    "            '/grid_prob_map',\n"
    "            self.on_grid,\n"
    "            10\n"
    "        )\n"
    "\n"
    "        # Grille d'occupation standard de RTAB-Map. Voir _choisir_grille.\n"
    "        self.create_subscription(\n"
    "            OccupancyGrid,\n"
    "            '/map',\n"
    "            self.on_grid_std,\n"
    "            10\n"
    "        )\n"
)


ANCRE_CB = (
    "    def on_grid(self, msg):\n"
    "        self.current_grid = msg\n"
)

REMPLACEMENT_CB = (
    "    def on_grid(self, msg):\n"
    "        self.grille_prob = msg\n"
    "        self._choisir_grille()\n"
    "\n"
    "    def on_grid_std(self, msg):\n"
    "        self.grille_std = msg\n"
    "        self._choisir_grille()\n"
    "\n"
    "    @staticmethod\n"
    "    def _proportion_connue(grille):\n"
    "        \"\"\"Fraction de cellules NON inconnues, entre 0 et 1.\n"
    "\n"
    "        count() s'execute en C : le cout reste negligeable meme sur une\n"
    "        grille de plusieurs millions de cellules.\n"
    "        \"\"\"\n"
    "        if grille is None or grille.info.width == 0:\n"
    "            return -1.0\n"
    "        donnees = grille.data\n"
    "        total = len(donnees)\n"
    "        if total == 0:\n"
    "            return -1.0\n"
    "        return 1.0 - donnees.count(-1) / float(total)\n"
    "\n"
    "    def _choisir_grille(self):\n"
    "        \"\"\"Retient la grille qui contient reellement des cellules connues.\n"
    "\n"
    "        /grid_prob_map est un topic optionnel de RTAB-Map : dans cette\n"
    "        configuration il publiait une carte integralement a -1 (mesure :\n"
    "        100,0 % d'inconnu, boite englobante grandissant avec la\n"
    "        trajectoire). Le critere de gain d'information mesurait donc la\n"
    "        proportion d'inconnu dans une grille ou tout est inconnu, et\n"
    "        saturait a 1,000 quels que soient les reglages.\n"
    "\n"
    "        Choisir automatiquement evite de figer un nom de topic qu'il\n"
    "        faudrait revalider a chaque changement de configuration.\n"
    "        \"\"\"\n"
    "        maintenant = self.get_clock().now().nanoseconds / 1e9\n"
    "        if maintenant - self.grille_dernier_choix < 2.0:\n"
    "            # Le choix ne peut pas changer plusieurs fois par seconde ;\n"
    "            # on met simplement a jour la grille deja retenue.\n"
    "            if self.grille_nom == '/map':\n"
    "                self.current_grid = self.grille_std\n"
    "            elif self.grille_nom == '/grid_prob_map':\n"
    "                self.current_grid = self.grille_prob\n"
    "            else:\n"
    "                self.current_grid = self.grille_std or self.grille_prob\n"
    "            return\n"
    "\n"
    "        self.grille_dernier_choix = maintenant\n"
    "\n"
    "        connu_prob = self._proportion_connue(self.grille_prob)\n"
    "        connu_std = self._proportion_connue(self.grille_std)\n"
    "\n"
    "        if connu_std >= connu_prob:\n"
    "            retenue, nom = self.grille_std, '/map'\n"
    "            ecartee, nom_ecartee, connu_ecartee = (\n"
    "                self.grille_prob, '/grid_prob_map', connu_prob\n"
    "            )\n"
    "            connu_retenu = connu_std\n"
    "        else:\n"
    "            retenue, nom = self.grille_prob, '/grid_prob_map'\n"
    "            ecartee, nom_ecartee, connu_ecartee = (\n"
    "                self.grille_std, '/map', connu_std\n"
    "            )\n"
    "            connu_retenu = connu_prob\n"
    "\n"
    "        self.current_grid = retenue\n"
    "\n"
    "        if nom != self.grille_nom and retenue is not None:\n"
    "            self.grille_nom = nom\n"
    "            self.get_logger().warn(\n"
    "                f\"GRILLE ACTIVE : {nom} \"\n"
    "                f\"(connu={100.0 * connu_retenu:.1f}%) -- \"\n"
    "                f\"{nom_ecartee} \"\n"
    "                f\"{'ecartee' if ecartee is not None else 'absente'} \"\n"
    "                f\"(connu={100.0 * max(connu_ecartee, 0.0):.1f}%)\"\n"
    "            )\n"
)


def main():
    if not os.path.exists(CIBLE):
        raise SystemExit("Fichier introuvable : %s" % CIBLE)

    src = io.open(CIBLE, encoding="utf-8").read()

    if "_choisir_grille" in src:
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    for nom, ancre in (("etat", ANCRE_ETAT),
                       ("abonnement", ANCRE_SUB),
                       ("callback", ANCRE_CB)):
        n = src.count(ancre)
        if n != 1:
            raise SystemExit(
                "ANCRE '%s' TROUVEE %d FOIS (attendu 1).\n"
                "Aucune modification effectuee." % (nom, n)
            )

    src = src.replace(ANCRE_ETAT, REMPLACEMENT_ETAT)
    src = src.replace(ANCRE_SUB, REMPLACEMENT_SUB)
    src = src.replace(ANCRE_CB, REMPLACEMENT_CB)

    sauvegarde = "%s.bak_grilleactive_%s" % (
        CIBLE, time.strftime("%Y%m%d_%H%M%S")
    )
    shutil.copy2(CIBLE, sauvegarde)
    io.open(CIBLE, "w", encoding="utf-8").write(src)

    print("Le decideur s'abonne a /grid_prob_map ET a /map,")
    print("et retient celle qui contient des cellules connues.")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % CIBLE)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis :")
    print("  bash ~/valider.sh grille")
    print("  grep 'GRILLE ACTIVE' ~/valid_grille.log | head -3")
    print("  grep 'DIAG gain'     ~/valid_grille.log | tail -5")
    return 0


if __name__ == "__main__":
    sys.exit(main())
