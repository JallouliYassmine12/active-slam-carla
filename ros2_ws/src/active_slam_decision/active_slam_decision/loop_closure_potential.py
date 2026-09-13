"""
Estimation du potentiel de fermeture de boucle pour une destination
candidate, a partir de /mapGraph (rtabmap_msgs/msg/MapGraph).

Champs utilises (ROS 2 / rtabmap_msgs, verifie sur la doc Humble) :
    header, map_to_odom, poses_id (int32[]), poses (geometry_msgs/Pose[]), links

Principe simplifie :
- Le graphe contient les poses deja visitees (poses_id / poses) au fil
  de la trajectoire, avec un identifiant de noeud croissant dans le temps.
- Un candidat a un fort potentiel de fermeture de boucle s'il est PROCHE
  (distance euclidienne) d'un noeud deja visite, mais ELOIGNE dans
  l'indice du graphe (donc probablement une zone deja cartographiee mais
  quittee depuis longtemps) : y retourner permet de corriger la derive
  accumulee depuis.
"""

import math


def compute_loop_closure_potential(map_graph, candidate_x, candidate_y,
                                    current_node_id, min_id_gap=15,
                                    proximity_radius=20.0):
    """
    Retourne un score entre 0 (aucun potentiel) et 1 (tres proche d'un
    noeud ancien du graphe).
    """
    if map_graph is None or len(map_graph.poses) == 0:
        return 0.0

    best_score = 0.0
    for node_id, pose in zip(map_graph.poses_id, map_graph.poses):
        if current_node_id is not None and (current_node_id - node_id) < min_id_gap:
            # Noeud trop recent : ce n'est pas une "vraie" boucle, on ignore
            continue

        dx = pose.position.x - candidate_x
        dy = pose.position.y - candidate_y
        dist = math.hypot(dx, dy)

        if dist <= proximity_radius:
            score = 1.0 - (dist / proximity_radius)
            if score > best_score:
                best_score = score

    return best_score
