"""
Calcul du gain d'information attendu pour une destination candidate, a
partir de la grille d'occupation probabiliste publiee par RTAB-Map sur
/grid_prob_map (nav_msgs/msg/OccupancyGrid).

Principe (approche frontiere / next-best-view simplifiee) :
- Chaque cellule de la grille vaut entre 0 et 100 (probabilite d'occupation)
  ou -1 si elle est inconnue.
- Le gain d'information d'une destination candidate est estime en comptant
  la proportion de cellules INCONNUES dans un rayon "sensor_range" autour
  de cette destination : plus il y a de cellules inconnues a proximite,
  plus on s'attend a decouvrir de nouvelle information en s'y rendant.

Cette approximation ne necessite pas de raytracing complet (couteux en
temps reel) ; elle est suffisante pour comparer des candidats entre eux.
Si besoin d'une estimation plus fine plus tard (entropie de Shannon par
cellule, raycasting simule), le point d'entree a modifier est
compute_information_gain.
"""

import math


def _world_to_grid(x, y, grid_info):
    """Convertit une coordonnee monde (m, repere de la grille) en indice de cellule."""
    col = int((x - grid_info.origin.position.x) / grid_info.resolution)
    row = int((y - grid_info.origin.position.y) / grid_info.resolution)
    return col, row


def compute_information_gain(occupancy_grid, candidate_x, candidate_y, sensor_range=20.0):
    """
    Retourne un score entre 0 et 1 : proportion de cellules inconnues
    dans un disque de rayon `sensor_range` autour du candidat.

    occupancy_grid : nav_msgs/msg/OccupancyGrid recu de /grid_prob_map
    candidate_x, candidate_y : position candidate dans le repere de la grille (m)
    sensor_range : rayon de recherche (m). A aligner avec la portee utile
                   du LiDAR / le parametre Grid/RangeMax cote RTAB-Map.
    """
    if occupancy_grid is None or occupancy_grid.info.width == 0:
        return 0.0

    info = occupancy_grid.info
    resolution = info.resolution
    width = info.width
    height = info.height
    data = occupancy_grid.data

    col_c, row_c = _world_to_grid(candidate_x, candidate_y, info)
    radius_cells = max(1, int(sensor_range / resolution))

    row_min = max(0, row_c - radius_cells)
    row_max = min(height, row_c + radius_cells)
    col_min = max(0, col_c - radius_cells)
    col_max = min(width, col_c + radius_cells)

    unknown_count = 0
    total_checked = 0
    sensor_range_sq = sensor_range * sensor_range

    for row in range(row_min, row_max):
        base = row * width
        dy_cells = (row - row_c) * resolution
        for col in range(col_min, col_max):
            dx_cells = (col - col_c) * resolution
            if dx_cells * dx_cells + dy_cells * dy_cells > sensor_range_sq:
                continue
            total_checked += 1
            if data[base + col] == -1:
                unknown_count += 1

    if total_checked == 0:
        # Aucune cellule de la grille ne couvre ce candidat : soit il est
        # hors des limites actuelles de la carte, soit la grille est trop
        # petite pour l'englober. Dans les deux cas c'est une zone jamais
        # vue -> on la traite comme un candidat a fort potentiel plutot
        # que de la penaliser comme si elle etait deja entierement
        # cartographiee (ce qui faussait la detection de stagnation
        # cote decision_maker.py).
        return 1.0

    return unknown_count / float(total_checked)


def compute_path_information_gain(occupancy_grid, start_x, start_y,
                                  goal_x, goal_y, sample_step=8.0,
                                  corridor_radius=6.0):
    """Gain d'information attendu LE LONG DU TRAJET vers le candidat.

    POURQUOI CETTE FONCTION EXISTE
    ------------------------------
    compute_information_gain ci-dessus n'evalue que le voisinage du
    POINT D'ARRIVEE. A une decision donnee, les candidats sont tous a
    peu pres a la meme distance du vehicule, donc tous du meme cote de
    la frontiere de cartographie : le critere renvoyait 0.000 pour tous
    ou 1.000 pour tous, jamais de valeur intermediaire.

    Releve sur les runs de campagne :
        CRITERES sur 23 candidats : info_gain [0.000 - 0.000]
        CRITERES sur 18 candidats : info_gain [1.000 - 1.000]

    Le critere qui porte le poids le plus fort de la decision (0,30) ne
    classait donc rien. Le calcul etait juste ; c'est la question posee
    qui ne discriminait pas.

    CE QU'ELLE MESURE
    -----------------
    La formulation usuelle en Active SLAM evalue ce qu'on DECOUVRIRA EN
    Y ALLANT. Le segment vehicule -> candidat est echantillonne tous les
    sample_step metres ; autour de chaque point on mesure la proportion
    de cellules inconnues dans un rayon corridor_radius ; on renvoie la
    moyenne.

    Deux candidats a 40 m dans deux rues differentes -- l'une deja
    cartographiee, l'autre inconnue -- obtiennent alors des scores
    franchement differents.

    APPROXIMATION ASSUMEE
    ---------------------
    Le segment est une LIGNE DROITE, alors que le vehicule suivra un
    itineraire routier qui peut etre bien plus long (facteur mesure
    jusqu'a 6,2). Le decideur ne connait pas cet itineraire : seul le
    controleur le calcule, et seulement apres avoir recu le but. La
    ligne droite reste un proxy raisonnable de la DIRECTION
    d'exploration, qui est ce que le critere doit trancher.

    Retourne un score entre 0 et 1.
    """
    if occupancy_grid is None or occupancy_grid.info.width == 0:
        return 0.0

    longueur = math.hypot(goal_x - start_x, goal_y - start_y)

    # Candidat confondu avec la position courante : il n'y a pas de
    # trajet a evaluer, on retombe sur le voisinage du point.
    if longueur < 1e-6:
        return compute_information_gain(
            occupancy_grid, goal_x, goal_y, corridor_radius
        )

    nb_points = max(1, int(longueur / sample_step))

    total = 0.0
    for i in range(1, nb_points + 1):
        t = i / float(nb_points)
        px = start_x + (goal_x - start_x) * t
        py = start_y + (goal_y - start_y) * t
        total += compute_information_gain(
            occupancy_grid, px, py, corridor_radius
        )

    return total / nb_points
