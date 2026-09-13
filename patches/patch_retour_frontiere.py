#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Retire le correctif de frontiere : les cellules hors grille redeviennent
IGNOREES, comme dans le code d'origine.

POURQUOI JE REVIENS EN ARRIERE
------------------------------
patch_gain_frontiere.py partait d'un raisonnement qui semblait solide :
une cellule situee hors de la grille d'occupation n'a jamais ete
observee, donc elle est INCONNUE, donc elle doit compter comme telle.

C'etait faux dans ce dispositif, et la mesure le prouve.
valid_gain.log, instrumentation DIAG gain :

    au vehicule=1.000 | pose=(221.0, 98.7)
                        grille x=[-1.6,223.0] y=[-1.7,100.1]
    au vehicule=1.000 | pose=(220.6,133.5)
                        grille x=[-1.6,223.0] y=[-1.7,135.1]
    au vehicule=1.000 | pose=(220.4,157.4)
                        grille x=[-1.6,223.0] y=[-1.7,158.9]

Le vehicule roule sur du terrain qu'il vient lui-meme de cartographier,
et le critere lui attribue 1.000 -- "totalement inconnu".

La raison tient a la nature de la grille : celle de RTAB-Map n'est pas
une fenetre de perception centree sur le vehicule, c'est la BOITE
ENGLOBANTE des cellules deja observees. Le vehicule se trouve donc
toujours sur son bord, a un ou deux metres. Son disque d'evaluation
deborde massivement, le debord est compte comme inconnu, et le score
sature a 1.000. Si le vehicule lui-meme vaut 1.000, tout vaut 1.000 --
ce qui explique tous les releves info_gain de la journee, y compris
apres le passage a l'evaluation le long du trajet.

Le code d'origine ignorait ces cellules et ne mesurait que la portion
reellement couverte par la carte. C'est le comportement correct : la
grille ne dit rien de ce qui est hors d'elle, et le garde-fou
total_checked == 0 traite deja le cas ou le candidat est entierement
au-dela.

MESURE COMPARATIVE
------------------
Grille reproduisant le releve -- boite [0,223]x[0,100], couloir
cartographie de 6 m le long d'une trajectoire en L, vehicule dans le
coin a (221.0, 98.7) :

                                          origine   patch frontiere
    gain a la position du vehicule          0.032        0.588
    candidat 40 m devant, hors boite        1.000        1.000
    candidat 40 m en arriere, cartographie  0.001        0.329
    candidat 80 m en arriere                0.797        0.825
    ETENDUE                                 0.999        0.671

Le code d'origine reconnait le terrain deja parcouru et donne une
etendue quasi maximale. Le correctif de frontiere ecrase tout vers le
haut.

CE QUI EST CONSERVE
-------------------
compute_path_information_gain -- l'evaluation le long du trajet -- est
conservee. Elle n'etait pas en cause : elle echantillonnait fidelement
une fonction ponctuelle devenue fausse. C'est la conjonction des deux
qui doit maintenant produire des scores discriminants, les premiers
echantillons tombant en terrain connu et les derniers en terrain
inconnu.

'import math' est conserve : compute_path_information_gain l'utilise.

VERIFICATION
------------
    bash ~/valider.sh frontiere2
    grep "DIAG gain" ~/valid_frontiere2.log | tail -5
    grep "CRITERES"  ~/valid_frontiere2.log | tail -10

Attendu : 'au vehicule' proche de 0, et une etendue franchement non
nulle sur les decisions offrant plusieurs directions.

Usage :
    python3 patch_retour_frontiere.py
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


ANCRE = "    col_c, row_c = _world_to_grid(candidate_x, candidate_y, info)\n    radius_cells = max(1, int(sensor_range / resolution))\n\n    unknown_count = 0\n    total_checked = 0\n    sensor_range_sq = sensor_range * sensor_range\n\n    # --- Les cellules hors grille sont INCONNUES, pas absentes ---\n    #\n    # La version precedente bornait le disque aux limites de la grille\n    # (max(0, ...) / min(width, ...)). Les cellules situees au-dela\n    # n'etaient alors comptees ni comme connues ni comme inconnues :\n    # elles disparaissaient du denominateur.\n    #\n    # Pour un candidat a cheval sur la frontiere de la carte, la moitie\n    # du disque qui regarde vers l'inconnu sortait donc du calcul, et il\n    # ne restait que la moitie deja cartographiee : gain ~ 0, alors que\n    # c'est precisement le meilleur candidat d'exploration. Un candidat\n    # entierement hors grille, lui, tombait sur total_checked == 0 et\n    # recoltait 1.0. Le critere etait inverse a la frontiere, et\n    # discontinu : il sautait de 0,05 a 1,00 d'un metre au suivant.\n    #\n    # Une cellule hors de la grille est une cellule que le SLAM n'a\n    # jamais observee. On la compte donc comme inconnue, exactement\n    # comme une cellule a -1 a l'interieur de la grille. Le critere\n    # redevient monotone :\n    #     terrain cartographie ...... 0,0\n    #     frontiere ................. valeur intermediaire\n    #     terrain jamais vu ......... 1,0\n    # Les deux extremes sont inchanges par rapport a l'ancien code.\n    #\n    # Les bornes de colonne sont deduites de l'equation du cercle au\n    # lieu d'etre testees cellule par cellule : on parcourt pi*r^2\n    # cellules au lieu de (2r)^2, et le decompte des -1 se fait par\n    # count() sur une tranche, donc en C. Le calcul est plus rapide\n    # qu'avant malgre le disque desormais complet.\n    for row in range(row_c - radius_cells, row_c + radius_cells + 1):\n        dy = (row - row_c) * resolution\n        reste = sensor_range_sq - dy * dy\n        if reste < 0.0:\n            continue\n\n        demi = int(math.sqrt(reste) / resolution)\n        col_deb = col_c - demi\n        col_fin = col_c + demi          # borne INCLUSE\n        nb_cellules = col_fin - col_deb + 1\n        total_checked += nb_cellules\n\n        # Ligne entierement au-dessus ou au-dessous de la grille.\n        if row < 0 or row >= height:\n            unknown_count += nb_cellules\n            continue\n\n        col_deb_in = max(0, col_deb)\n        col_fin_in = min(width - 1, col_fin)\n\n        # Ligne entierement a gauche ou a droite de la grille.\n        if col_deb_in > col_fin_in:\n            unknown_count += nb_cellules\n            continue\n\n        # Debords lateraux : hors grille, donc inconnus.\n        unknown_count += (col_deb_in - col_deb) + (col_fin - col_fin_in)\n\n        # Portion effectivement couverte par la grille.\n        base = row * width\n        unknown_count += data[\n            base + col_deb_in:base + col_fin_in + 1\n        ].count(-1)\n\n    if total_checked == 0:\n        # N'est plus atteint que si sensor_range est inferieur a la\n        # resolution de la grille. Conserve par prudence : une zone dont\n        # aucune cellule n'a ete examinee est une zone jamais vue.\n        return 1.0\n\n    return unknown_count / float(total_checked)\n"

REMPLACEMENT = "    col_c, row_c = _world_to_grid(candidate_x, candidate_y, info)\n    radius_cells = max(1, int(sensor_range / resolution))\n\n    row_min = max(0, row_c - radius_cells)\n    row_max = min(height, row_c + radius_cells)\n    col_min = max(0, col_c - radius_cells)\n    col_max = min(width, col_c + radius_cells)\n\n    unknown_count = 0\n    total_checked = 0\n    sensor_range_sq = sensor_range * sensor_range\n\n    for row in range(row_min, row_max):\n        base = row * width\n        dy_cells = (row - row_c) * resolution\n        for col in range(col_min, col_max):\n            dx_cells = (col - col_c) * resolution\n            if dx_cells * dx_cells + dy_cells * dy_cells > sensor_range_sq:\n                continue\n            total_checked += 1\n            if data[base + col] == -1:\n                unknown_count += 1\n\n    if total_checked == 0:\n        # Aucune cellule de la grille ne couvre ce candidat : soit il est\n        # hors des limites actuelles de la carte, soit la grille est trop\n        # petite pour l'englober. Dans les deux cas c'est une zone jamais\n        # vue -> on la traite comme un candidat a fort potentiel plutot\n        # que de la penaliser comme si elle etait deja entierement\n        # cartographiee (ce qui faussait la detection de stagnation\n        # cote decision_maker.py).\n        return 1.0\n\n    return unknown_count / float(total_checked)\n"


def main():
    if not os.path.exists(CIBLE):
        raise SystemExit("Fichier introuvable : %s" % CIBLE)

    src = io.open(CIBLE, encoding="utf-8").read()

    if "hors grille sont INCONNUES" not in src:
        raise SystemExit(
            "Le correctif de frontiere n'est pas present (deja retire ?).\n"
            "Aucune modification effectuee."
        )

    if "compute_path_information_gain" not in src:
        raise SystemExit(
            "compute_path_information_gain est absente : appliquer d'abord\n"
            "patch_gain_trajet.py.\nAucune modification effectuee."
        )

    n = src.count(ANCRE)
    if n != 1:
        raise SystemExit(
            "ANCRE TROUVEE %d FOIS (attendu 1).\n"
            "Aucune modification effectuee." % n
        )

    src = src.replace(ANCRE, REMPLACEMENT)

    sauvegarde = "%s.bak_retourfront_%s" % (
        CIBLE, time.strftime("%Y%m%d_%H%M%S")
    )
    shutil.copy2(CIBLE, sauvegarde)
    io.open(CIBLE, "w", encoding="utf-8").write(src)

    print("Cellules hors grille : de nouveau IGNOREES (comportement d'origine).")
    print("compute_path_information_gain conservee.")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % CIBLE)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis :")
    print("  bash ~/valider.sh frontiere2")
    print("  grep 'DIAG gain' ~/valid_frontiere2.log | tail -5")
    print("  grep 'CRITERES'  ~/valid_frontiere2.log | tail -10")
    return 0


if __name__ == "__main__":
    sys.exit(main())
