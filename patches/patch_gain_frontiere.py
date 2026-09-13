#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Corrige le calcul du gain d'information a la FRONTIERE de la carte.

LE DEFAUT
---------
compute_information_gain compte la proportion de cellules INCONNUES dans
un disque de rayon sensor_range autour du candidat. Le disque est borne
aux limites de la grille :

    row_min = max(0, row_c - radius_cells)
    row_max = min(height, row_c + radius_cells)
    col_min = max(0, col_c - radius_cells)
    col_max = min(width, col_c + radius_cells)

Les cellules situees HORS de la grille sont donc simplement ecartees du
decompte : elles ne comptent ni comme connues, ni comme inconnues.

Consequence sur les trois positions possibles d'un candidat :

  - entierement dans la zone cartographiee
        disque complet, cellules majoritairement connues -> gain ~ 0
        (correct)

  - a cheval sur la frontiere de la carte
        la moitie du disque qui regarde vers l'inconnu SORT de la grille
        et disparait du decompte. Il ne reste que la moitie deja
        cartographiee, donc unknown_count ~ 0 sur total_checked > 0
        -> gain ~ 0   (FAUX : c'est le meilleur candidat d'exploration)

  - entierement hors de la grille
        total_checked == 0, le garde-fou renvoie 1.0
        -> gain = 1.0

Le critere est donc INVERSE exactement la ou il devrait trancher : le
candidat qui longe la frontiere -- celui qui fera progresser la carte --
est note plus bas que le candidat lointain dont on ne sait rien. Et il
est discontinu : il saute de 0,05 a 1,00 d'un metre au suivant.

C'est aussi ce qui explique que le relevement de Grid/RangeMax de 25 a
40 m n'ait rien change : agrandir la grille deplace la frontiere, elle
ne change pas la facon dont la frontiere est traitee.

LA CORRECTION
-------------
Une cellule hors de la grille d'occupation n'est pas une cellule
"absente" : c'est une cellule que le SLAM n'a jamais observee. Elle est
INCONNUE, au meme titre qu'une cellule a -1 a l'interieur de la grille.

Le disque est donc parcouru en entier, sans bornage, et chaque cellule
hors limites est comptee comme inconnue.

Le critere devient monotone et continu :

    terrain entierement cartographie ......... 0,0
    candidat a cheval sur la frontiere ....... valeur intermediaire
    terrain jamais vu ........................ 1,0

Les deux extremes sont IDENTIQUES a l'ancien comportement : un candidat
entierement hors grille vaut toujours 1,0 (toutes ses cellules sont hors
limites), un candidat en terrain connu vaut toujours ~0. Seul le cas
intermediaire -- celui qui etait faux -- change. Rien de ce qui
fonctionnait n'est modifie.

Le garde-fou total_checked == 0 est conserve par prudence, mais il n'est
desormais atteint que si sensor_range est plus petit que la resolution.

COUT DE CALCUL
--------------
Le parcours n'est plus borne, donc plus de cellules sont visitees. Deux
mesures compensent largement :

  - les bornes de colonne de chaque ligne sont calculees par l'equation
    du cercle au lieu d'etre testees cellule par cellule : on parcourt
    pi*r^2 cellules au lieu de (2r)^2, soit 21 % de moins ;
  - le decompte des -1 a l'interieur de la grille se fait par
    list.count() sur une tranche, donc en C et non en Python.

Le calcul est plus rapide qu'avant pour un candidat en terrain connu.

VERIFICATION
------------
Run de 600 m, puis :

    grep "CRITERES" ~/valid_frontiere.log | tail -10

Attendu : des valeurs intermediaires, et une 'etendue' franchement non
nulle. Verifier AUSSI que la ligne "CANDIDATS RECUS" affiche grid=OK :
si elle affiche grid=NONE, RTAB-Map n'a rien publie et le run ne mesure
rien -- ni ce correctif, ni aucun autre.

Usage :
    python3 patch_gain_frontiere.py
"""

import io
import os
import shutil
import sys
import time

CIBLE = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/"
    "active_slam_decision/information_gain.py"
)


ANCRE_IMPORT = (
    '"""\n'
    '\n'
    '\n'
    'def _world_to_grid(x, y, grid_info):\n'
)

REMPLACEMENT_IMPORT = (
    '"""\n'
    '\n'
    'import math\n'
    '\n'
    '\n'
    'def _world_to_grid(x, y, grid_info):\n'
)


ANCRE_CORPS = (
    "    col_c, row_c = _world_to_grid(candidate_x, candidate_y, info)\n"
    "    radius_cells = max(1, int(sensor_range / resolution))\n"
    "\n"
    "    row_min = max(0, row_c - radius_cells)\n"
    "    row_max = min(height, row_c + radius_cells)\n"
    "    col_min = max(0, col_c - radius_cells)\n"
    "    col_max = min(width, col_c + radius_cells)\n"
    "\n"
    "    unknown_count = 0\n"
    "    total_checked = 0\n"
    "    sensor_range_sq = sensor_range * sensor_range\n"
    "\n"
    "    for row in range(row_min, row_max):\n"
    "        base = row * width\n"
    "        dy_cells = (row - row_c) * resolution\n"
    "        for col in range(col_min, col_max):\n"
    "            dx_cells = (col - col_c) * resolution\n"
    "            if dx_cells * dx_cells + dy_cells * dy_cells > sensor_range_sq:\n"
    "                continue\n"
    "            total_checked += 1\n"
    "            if data[base + col] == -1:\n"
    "                unknown_count += 1\n"
    "\n"
    "    if total_checked == 0:\n"
    "        # Aucune cellule de la grille ne couvre ce candidat : soit il est\n"
    "        # hors des limites actuelles de la carte, soit la grille est trop\n"
    "        # petite pour l'englober. Dans les deux cas c'est une zone jamais\n"
    "        # vue -> on la traite comme un candidat a fort potentiel plutot\n"
    "        # que de la penaliser comme si elle etait deja entierement\n"
    "        # cartographiee (ce qui faussait la detection de stagnation\n"
    "        # cote decision_maker.py).\n"
    "        return 1.0\n"
    "\n"
    "    return unknown_count / float(total_checked)\n"
)


REMPLACEMENT_CORPS = (
    "    col_c, row_c = _world_to_grid(candidate_x, candidate_y, info)\n"
    "    radius_cells = max(1, int(sensor_range / resolution))\n"
    "\n"
    "    unknown_count = 0\n"
    "    total_checked = 0\n"
    "    sensor_range_sq = sensor_range * sensor_range\n"
    "\n"
    "    # --- Les cellules hors grille sont INCONNUES, pas absentes ---\n"
    "    #\n"
    "    # La version precedente bornait le disque aux limites de la grille\n"
    "    # (max(0, ...) / min(width, ...)). Les cellules situees au-dela\n"
    "    # n'etaient alors comptees ni comme connues ni comme inconnues :\n"
    "    # elles disparaissaient du denominateur.\n"
    "    #\n"
    "    # Pour un candidat a cheval sur la frontiere de la carte, la moitie\n"
    "    # du disque qui regarde vers l'inconnu sortait donc du calcul, et il\n"
    "    # ne restait que la moitie deja cartographiee : gain ~ 0, alors que\n"
    "    # c'est precisement le meilleur candidat d'exploration. Un candidat\n"
    "    # entierement hors grille, lui, tombait sur total_checked == 0 et\n"
    "    # recoltait 1.0. Le critere etait inverse a la frontiere, et\n"
    "    # discontinu : il sautait de 0,05 a 1,00 d'un metre au suivant.\n"
    "    #\n"
    "    # Une cellule hors de la grille est une cellule que le SLAM n'a\n"
    "    # jamais observee. On la compte donc comme inconnue, exactement\n"
    "    # comme une cellule a -1 a l'interieur de la grille. Le critere\n"
    "    # redevient monotone :\n"
    "    #     terrain cartographie ...... 0,0\n"
    "    #     frontiere ................. valeur intermediaire\n"
    "    #     terrain jamais vu ......... 1,0\n"
    "    # Les deux extremes sont inchanges par rapport a l'ancien code.\n"
    "    #\n"
    "    # Les bornes de colonne sont deduites de l'equation du cercle au\n"
    "    # lieu d'etre testees cellule par cellule : on parcourt pi*r^2\n"
    "    # cellules au lieu de (2r)^2, et le decompte des -1 se fait par\n"
    "    # count() sur une tranche, donc en C. Le calcul est plus rapide\n"
    "    # qu'avant malgre le disque desormais complet.\n"
    "    for row in range(row_c - radius_cells, row_c + radius_cells + 1):\n"
    "        dy = (row - row_c) * resolution\n"
    "        reste = sensor_range_sq - dy * dy\n"
    "        if reste < 0.0:\n"
    "            continue\n"
    "\n"
    "        demi = int(math.sqrt(reste) / resolution)\n"
    "        col_deb = col_c - demi\n"
    "        col_fin = col_c + demi          # borne INCLUSE\n"
    "        nb_cellules = col_fin - col_deb + 1\n"
    "        total_checked += nb_cellules\n"
    "\n"
    "        # Ligne entierement au-dessus ou au-dessous de la grille.\n"
    "        if row < 0 or row >= height:\n"
    "            unknown_count += nb_cellules\n"
    "            continue\n"
    "\n"
    "        col_deb_in = max(0, col_deb)\n"
    "        col_fin_in = min(width - 1, col_fin)\n"
    "\n"
    "        # Ligne entierement a gauche ou a droite de la grille.\n"
    "        if col_deb_in > col_fin_in:\n"
    "            unknown_count += nb_cellules\n"
    "            continue\n"
    "\n"
    "        # Debords lateraux : hors grille, donc inconnus.\n"
    "        unknown_count += (col_deb_in - col_deb) + (col_fin - col_fin_in)\n"
    "\n"
    "        # Portion effectivement couverte par la grille.\n"
    "        base = row * width\n"
    "        unknown_count += data[\n"
    "            base + col_deb_in:base + col_fin_in + 1\n"
    "        ].count(-1)\n"
    "\n"
    "    if total_checked == 0:\n"
    "        # N'est plus atteint que si sensor_range est inferieur a la\n"
    "        # resolution de la grille. Conserve par prudence : une zone dont\n"
    "        # aucune cellule n'a ete examinee est une zone jamais vue.\n"
    "        return 1.0\n"
    "\n"
    "    return unknown_count / float(total_checked)\n"
)


def main():
    if not os.path.exists(CIBLE):
        raise SystemExit("Fichier introuvable : %s" % CIBLE)

    src = io.open(CIBLE, encoding="utf-8").read()

    if "hors grille sont INCONNUES" in src:
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    for nom, ancre in (("import", ANCRE_IMPORT), ("corps", ANCRE_CORPS)):
        n = src.count(ancre)
        if n != 1:
            raise SystemExit(
                "ANCRE '%s' TROUVEE %d FOIS (attendu 1).\n"
                "Aucune modification effectuee." % (nom, n)
            )

    src = src.replace(ANCRE_IMPORT, REMPLACEMENT_IMPORT)
    src = src.replace(ANCRE_CORPS, REMPLACEMENT_CORPS)

    sauvegarde = "%s.bak_frontiere_%s" % (CIBLE, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(CIBLE, sauvegarde)
    io.open(CIBLE, "w", encoding="utf-8").write(src)

    print("information_gain.py : cellules hors grille comptees comme inconnues")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % CIBLE)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    return 0


if __name__ == "__main__":
    sys.exit(main())
