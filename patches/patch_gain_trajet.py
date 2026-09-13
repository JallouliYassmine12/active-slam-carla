#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Le gain d'information est evalue LE LONG DU TRAJET, et non plus
seulement autour du point d'arrivee.

LE DEFAUT, ET POURQUOI IL EST DE FORMULATION
--------------------------------------------
compute_information_gain compte les cellules inconnues dans un disque
de rayon sensor_range autour du CANDIDAT. Le score ne depend donc que
de ce qu'on trouvera une fois arrive, jamais de ce qu'on decouvrira en
y allant.

Consequence mesuree, sur tous les runs de la journee :

    CRITERES sur 23 candidats : info_gain [0.000 - 0.000] etendue=0.000
    CRITERES sur 18 candidats : info_gain [1.000 - 1.000] etendue=0.000

Jamais de valeur intermediaire, jamais d'ecart entre candidats. La
raison est geometrique : a une decision donnee, les candidats sont tous
a peu pres a la meme distance du vehicule, donc tous du meme cote de la
frontiere de cartographie. Soit tous dans le disque deja cartographie
(0.000), soit tous au-dela (1.000).

Instrumentation DIAG grille, run valid_mincand12.log :

    grille    : x=[-1.6, 308.9]  y=[-5.8, 124.0]
    candidats : (309.7,45.4), (309.6,37.4), (309.4,29.4)

Les trois candidats sont a 80 centimetres au-dela du bord de la grille,
tous les trois, tous du meme cote.

Le critere qui porte le poids le plus fort de la fonction de decision
-- 0,30 sur 1,00 -- ne classait donc rien. Ce n'est pas un bug de
calcul : le calcul est juste, c'est la QUESTION POSEE qui ne discrimine
pas.

LA CORRECTION
-------------
La formulation usuelle en Active SLAM evalue le gain d'information
ATTENDU SUR LE TRAJET : ce qu'on decouvrira en s'y rendant, pas
seulement une fois arrive. Deux candidats situes a 40 m dans deux rues
differentes -- l'une deja cartographiee, l'autre inconnue -- obtiennent
alors des scores franchement differents. C'est exactement la
discrimination qui manque.

compute_path_information_gain echantillonne le segment vehicule ->
candidat tous les path_sample_step metres, mesure autour de chaque
point la proportion de cellules inconnues dans un rayon
path_corridor_radius, et renvoie la moyenne.

APPROXIMATION ASSUMEE
---------------------
Le segment echantillonne est une LIGNE DROITE, alors que le vehicule
suivra un itineraire routier qui peut etre bien plus long (facteur
mesure jusqu'a 6,2). Le decision_maker ne connait pas cet itineraire :
seul le controleur le calcule, et seulement apres avoir recu le but.

La ligne droite reste un proxy raisonnable de la DIRECTION
d'exploration, qui est ce que le critere doit trancher. C'est une
approximation a signaler dans le rapport, pas a masquer.

CHOIX DES DEUX PARAMETRES
-------------------------
  path_sample_step   = 8 m   pas d'echantillonnage, aligne sur le pas
                             du parcours routier (waypoint_spacing et
                             route_step valent tous deux 8 m)

  path_corridor_radius = 6 m environ une voie et demie. Assez large
                             pour ne pas dependre d'une cellule isolee,
                             assez etroit pour que deux rues voisines ne
                             se confondent pas -- et pour que le calcul
                             reste rapide : le disque parcouru croit
                             comme le carre du rayon.

Les deux sont des parametres ROS, donc modifiables sans recompiler.

CE QUI EST CONSERVE
-------------------
compute_information_gain n'est pas supprimee. Elle reste utilisee pour
le point d'arrivee lui-meme et, surtout, elle documente la premiere
formulation. Le rapport doit presenter les DEUX et expliquer pourquoi
la premiere ne discriminait pas : c'est un resultat de la demarche, pas
un brouillon a effacer.

VERIFICATION
------------
    bash ~/valider.sh trajet
    grep "CRITERES" ~/valid_trajet.log | tail -10

Attendu : une 'etendue' franchement non nulle sur les decisions ou le
vehicule a le choix entre plusieurs directions. Elle restera nulle
quand tous les candidats partagent le meme debut de trajet -- c'est
normal et correct.

Surveiller aussi la duree du run : si elle augmente nettement par
rapport aux 210 s de reference, l'echantillonnage coute trop cher et il
faut relever path_sample_step.

Usage :
    python3 patch_gain_trajet.py
"""

import io
import os
import shutil
import sys
import time

BASE = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/"
    "active_slam_decision/"
)
MODULE = BASE + "information_gain.py"
DECIDEUR = BASE + "decision_maker.py"


# ---------------------------------------------------------------------
# 1. Nouvelle fonction dans information_gain.py
# ---------------------------------------------------------------------
ANCRE_MODULE = (
    "    if total_checked == 0:\n"
    "        # N'est plus atteint que si sensor_range est inferieur a la\n"
    "        # resolution de la grille. Conserve par prudence : une zone dont\n"
    "        # aucune cellule n'a ete examinee est une zone jamais vue.\n"
    "        return 1.0\n"
    "\n"
    "    return unknown_count / float(total_checked)\n"
)

REMPLACEMENT_MODULE = (
    "    if total_checked == 0:\n"
    "        # N'est plus atteint que si sensor_range est inferieur a la\n"
    "        # resolution de la grille. Conserve par prudence : une zone dont\n"
    "        # aucune cellule n'a ete examinee est une zone jamais vue.\n"
    "        return 1.0\n"
    "\n"
    "    return unknown_count / float(total_checked)\n"
    "\n"
    "\n"
    "def compute_path_information_gain(occupancy_grid, start_x, start_y,\n"
    "                                  goal_x, goal_y, sample_step=8.0,\n"
    "                                  corridor_radius=6.0):\n"
    "    \"\"\"Gain d'information attendu LE LONG DU TRAJET vers le candidat.\n"
    "\n"
    "    POURQUOI CETTE FONCTION EXISTE\n"
    "    ------------------------------\n"
    "    compute_information_gain ci-dessus n'evalue que le voisinage du\n"
    "    POINT D'ARRIVEE. A une decision donnee, les candidats sont tous a\n"
    "    peu pres a la meme distance du vehicule, donc tous du meme cote de\n"
    "    la frontiere de cartographie : le critere renvoyait 0.000 pour tous\n"
    "    ou 1.000 pour tous, jamais de valeur intermediaire.\n"
    "\n"
    "    Releve sur les runs de campagne :\n"
    "        CRITERES sur 23 candidats : info_gain [0.000 - 0.000]\n"
    "        CRITERES sur 18 candidats : info_gain [1.000 - 1.000]\n"
    "\n"
    "    Le critere qui porte le poids le plus fort de la decision (0,30) ne\n"
    "    classait donc rien. Le calcul etait juste ; c'est la question posee\n"
    "    qui ne discriminait pas.\n"
    "\n"
    "    CE QU'ELLE MESURE\n"
    "    -----------------\n"
    "    La formulation usuelle en Active SLAM evalue ce qu'on DECOUVRIRA EN\n"
    "    Y ALLANT. Le segment vehicule -> candidat est echantillonne tous les\n"
    "    sample_step metres ; autour de chaque point on mesure la proportion\n"
    "    de cellules inconnues dans un rayon corridor_radius ; on renvoie la\n"
    "    moyenne.\n"
    "\n"
    "    Deux candidats a 40 m dans deux rues differentes -- l'une deja\n"
    "    cartographiee, l'autre inconnue -- obtiennent alors des scores\n"
    "    franchement differents.\n"
    "\n"
    "    APPROXIMATION ASSUMEE\n"
    "    ---------------------\n"
    "    Le segment est une LIGNE DROITE, alors que le vehicule suivra un\n"
    "    itineraire routier qui peut etre bien plus long (facteur mesure\n"
    "    jusqu'a 6,2). Le decideur ne connait pas cet itineraire : seul le\n"
    "    controleur le calcule, et seulement apres avoir recu le but. La\n"
    "    ligne droite reste un proxy raisonnable de la DIRECTION\n"
    "    d'exploration, qui est ce que le critere doit trancher.\n"
    "\n"
    "    Retourne un score entre 0 et 1.\n"
    "    \"\"\"\n"
    "    if occupancy_grid is None or occupancy_grid.info.width == 0:\n"
    "        return 0.0\n"
    "\n"
    "    longueur = math.hypot(goal_x - start_x, goal_y - start_y)\n"
    "\n"
    "    # Candidat confondu avec la position courante : il n'y a pas de\n"
    "    # trajet a evaluer, on retombe sur le voisinage du point.\n"
    "    if longueur < 1e-6:\n"
    "        return compute_information_gain(\n"
    "            occupancy_grid, goal_x, goal_y, corridor_radius\n"
    "        )\n"
    "\n"
    "    nb_points = max(1, int(longueur / sample_step))\n"
    "\n"
    "    total = 0.0\n"
    "    for i in range(1, nb_points + 1):\n"
    "        t = i / float(nb_points)\n"
    "        px = start_x + (goal_x - start_x) * t\n"
    "        py = start_y + (goal_y - start_y) * t\n"
    "        total += compute_information_gain(\n"
    "            occupancy_grid, px, py, corridor_radius\n"
    "        )\n"
    "\n"
    "    return total / nb_points\n"
)


# ---------------------------------------------------------------------
# 2. Decideur : import, parametres, appel
# ---------------------------------------------------------------------
ANCRE_IMPORT = (
    "from active_slam_decision.information_gain import compute_information_gain\n"
)

REMPLACEMENT_IMPORT = (
    "from active_slam_decision.information_gain import (\n"
    "    compute_information_gain,\n"
    "    compute_path_information_gain,\n"
    ")\n"
)


ANCRE_DECLARE = "        self.declare_parameter('sensor_range', 20.0)\n"

REMPLACEMENT_DECLARE = (
    "        self.declare_parameter('sensor_range', 20.0)\n"
    "\n"
    "        # --- Gain d'information evalue le long du trajet ---\n"
    "        # Pas d'echantillonnage du segment vehicule -> candidat, aligne\n"
    "        # sur le pas du parcours routier (waypoint_spacing et route_step\n"
    "        # valent tous deux 8 m).\n"
    "        self.declare_parameter('path_sample_step', 8.0)\n"
    "        # Rayon mesure autour de chaque point echantillonne : environ\n"
    "        # une voie et demie. Assez large pour ne pas dependre d'une\n"
    "        # cellule isolee, assez etroit pour que deux rues voisines ne se\n"
    "        # confondent pas -- et pour que le calcul reste rapide, le\n"
    "        # disque parcouru croissant comme le carre du rayon.\n"
    "        self.declare_parameter('path_corridor_radius', 6.0)\n"
)


ANCRE_LECTURE = "        self.sensor_range = self.get_parameter('sensor_range').value\n"

REMPLACEMENT_LECTURE = (
    "        self.sensor_range = self.get_parameter('sensor_range').value\n"
    "        self.path_sample_step = float(\n"
    "            self.get_parameter('path_sample_step').value\n"
    "        )\n"
    "        self.path_corridor_radius = float(\n"
    "            self.get_parameter('path_corridor_radius').value\n"
    "        )\n"
)


ANCRE_APPEL = (
    "            info_gain = compute_information_gain(\n"
    "                self.current_grid,\n"
    "                c.x,\n"
    "                -c.y,\n"
    "                sensor_range=self.sensor_range\n"
    "            )\n"
)

REMPLACEMENT_APPEL = (
    "            # --- Gain d'information LE LONG DU TRAJET ---\n"
    "            # L'ancienne formulation n'evaluait que le voisinage du\n"
    "            # point d'arrivee. Les candidats d'une meme decision etant\n"
    "            # tous a peu pres a la meme distance, ils tombaient tous du\n"
    "            # meme cote de la frontiere de cartographie : le critere\n"
    "            # valait 0.000 pour tous ou 1.000 pour tous, jamais de\n"
    "            # valeur intermediaire. Le critere qui pese le plus lourd\n"
    "            # dans la decision (0,30) ne classait rien.\n"
    "            #\n"
    "            # On mesure desormais ce que le vehicule decouvrira EN S'Y\n"
    "            # RENDANT, ce qui est la formulation usuelle en Active SLAM.\n"
    "            #\n"
    "            # La transposition en -c.y reste necessaire : la grille vient\n"
    "            # de RTAB-Map, donc de son repere miroir (cf. on_pose). La\n"
    "            # position courante subit la meme transposition.\n"
    "            info_gain = compute_path_information_gain(\n"
    "                self.current_grid,\n"
    "                self.current_x,\n"
    "                -self.current_y,\n"
    "                c.x,\n"
    "                -c.y,\n"
    "                sample_step=self.path_sample_step,\n"
    "                corridor_radius=self.path_corridor_radius\n"
    "            )\n"
)


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
    for chemin in (MODULE, DECIDEUR):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    src_mod = io.open(MODULE, encoding="utf-8").read()
    src_dec = io.open(DECIDEUR, encoding="utf-8").read()

    if "compute_path_information_gain" in src_mod:
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    if "import math" not in src_mod:
        raise SystemExit(
            "information_gain.py n'importe pas math : appliquer d'abord\n"
            "patch_gain_frontiere.py.\nAucune modification effectuee."
        )

    src_mod = remplacer(src_mod, ANCRE_MODULE, REMPLACEMENT_MODULE,
                        "fonction", MODULE)

    src_dec = remplacer(src_dec, ANCRE_IMPORT, REMPLACEMENT_IMPORT,
                        "import", DECIDEUR)
    src_dec = remplacer(src_dec, ANCRE_DECLARE, REMPLACEMENT_DECLARE,
                        "declaration", DECIDEUR)
    src_dec = remplacer(src_dec, ANCRE_LECTURE, REMPLACEMENT_LECTURE,
                        "lecture", DECIDEUR)
    src_dec = remplacer(src_dec, ANCRE_APPEL, REMPLACEMENT_APPEL,
                        "appel", DECIDEUR)

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in ((MODULE, src_mod), (DECIDEUR, src_dec)):
        sauvegarde = "%s.bak_gaintrajet_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("")
    print("Le gain d'information est desormais evalue le long du trajet.")
    print("Parametres : path_sample_step=8.0 m, path_corridor_radius=6.0 m")
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % MODULE)
    print("  python3 -m py_compile %s" % DECIDEUR)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis valider :")
    print("  bash ~/valider.sh trajet")
    print("  grep 'CRITERES' ~/valid_trajet.log | tail -10")
    return 0


if __name__ == "__main__":
    sys.exit(main())
