"""
Evaluation du risque de securite d'une destination candidate, a partir
des obstacles/pietons/vehicules deja detectes par obstacle_detector.py.

PRINCIPE : le score ne mesure pas le risque AU POINT D'ARRIVEE mais le
risque DU DEPLACEMENT. On evalue la distance de chaque obstacle au
segment [position courante du vehicule -> candidat]. Un obstacle situe
sur le trajet fait donc chuter le score, meme si le point d'arrivee est
degage. La version precedente, qui ne regardait que le voisinage du
point d'arrivee, retournait 1.0 sur la quasi-totalite des decisions :
les candidats sont a 20-400 m alors que les obstacles detectes sont
necessairement proches du vehicule, donc les deux ensembles ne se
recoupaient jamais. Un critere constant contribue le meme terme a tous
les candidats et ne modifie aucun classement : il etait neutralise.

REPERE : obstacle_detector publie ses obstacles avec frame_id = 'map'
et un rayon de detection de 100 m ; les candidats sont exprimes dans le
meme repere 'map'. Les coordonnees sont donc directement comparables,
sans transformation TF.

HORIZON : seuls les horizon premiers metres du trajet sont evalues.
obstacle_detector publie les positions INSTANTANEES des obstacles ; un
vehicule NPC detecte a 20 m aura change de place dans cinq secondes.
Evaluer un couloir de 250 m avec ces positions n'a donc pas de sens
physique, et rendait le critere quasi binaire : sur une route droite,
tout NPC circulant devant est aligne avec n'importe quelle destination
situee plus loin sur cette meme route, ce qui ecrasait le score a zero
des qu'un vehicule etait vaguement dans l'axe. Borner l'horizon ramene
le critere a ce qu'il peut reellement mesurer : le risque du DEBUT du
trajet, seul intervalle ou les positions mesurees restent valables.

MARGE EGO : les ego_margin premiers metres du segment sont exclus du
calcul. L'environnement immediat du vehicule releve de la couche
REACTIVE (obstacle_detector + vehicle_controller, freinage a 7 m) ;
cette fonction implemente la couche STRATEGIQUE, qui doit comparer des
directions entre elles. Sans cette marge, un obstacle colle au vehicule
penaliserait identiquement tous les candidats et neutraliserait a
nouveau le critere.
"""

import math


def _distance_point_segment(px, py, ax, ay, bx, by):
    """
    Distance euclidienne du point (px, py) au segment [A, B].

    On projette le point sur la droite (AB), puis on borne le parametre
    de projection a [0, 1] : si la projection tombe hors du segment, la
    distance retenue est celle a l'extremite la plus proche. C'est la
    formule standard, exacte, sans cas particulier hormis le segment
    degenere (A confondu avec B).
    """
    dx = bx - ax
    dy = by - ay
    seg_len2 = dx * dx + dy * dy

    if seg_len2 < 1e-9:
        return math.hypot(px - ax, py - ay)

    t = ((px - ax) * dx + (py - ay) * dy) / seg_len2
    t = max(0.0, min(1.0, t))

    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def compute_safety_score(obstacles, candidate_x, candidate_y,
                         vehicle_x, vehicle_y,
                         danger_radius=8.0, ego_margin=5.0, horizon=50.0):
    """
    Retourne un score de securite entre 0 (trajet dangereux, obstacle
    sur le couloir de passage) et 1 (sur, aucun obstacle a proximite du
    trajet).

    obstacles     : liste de tuples (x, y) en repere map
    candidate_x/y : destination evaluee, en repere map
    vehicle_x/y   : position courante du vehicule, en repere map
    danger_radius : demi-largeur (m) du couloir en-deca de laquelle le
                    score commence a baisser
    ego_margin    : longueur (m) exclue en debut de segment, laissee a
                    la couche reactive
    horizon       : longueur (m) de trajet reellement evaluee au-dela
                    de la marge ego
    """
    if not obstacles:
        return 1.0

    dx = candidate_x - vehicle_x
    dy = candidate_y - vehicle_y
    seg_len = math.hypot(dx, dy)

    if seg_len <= ego_margin:
        # Candidat quasiment sur place : le segment utile disparait,
        # on retombe sur une evaluation ponctuelle du point d'arrivee.
        start_x, start_y = candidate_x, candidate_y
        end_x, end_y = candidate_x, candidate_y
    else:
        ratio_debut = ego_margin / seg_len
        start_x = vehicle_x + ratio_debut * dx
        start_y = vehicle_y + ratio_debut * dy
        # Fin du couloir evalue : le point d'arrivee s'il est proche,
        # sinon le point situe a 'horizon' metres devant le vehicule.
        ratio_fin = min(1.0, horizon / seg_len)
        end_x = vehicle_x + ratio_fin * dx
        end_y = vehicle_y + ratio_fin * dy

    min_dist = float('inf')
    for obs_x, obs_y in obstacles:
        dist = _distance_point_segment(
            obs_x, obs_y,
            start_x, start_y,
            end_x, end_y
        )
        if dist < min_dist:
            min_dist = dist

    if min_dist >= danger_radius:
        return 1.0
    if min_dist <= 0.0:
        return 0.0
    return min_dist / danger_radius
