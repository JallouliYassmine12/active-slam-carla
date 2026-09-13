"""
Generateur de destinations candidates sur le reseau routier.

Role dans le projet (fidele au sujet) :
  "Plusieurs destinations candidates seront ensuite generees sur le
  reseau routier." -- Ce noeud fait EXACTEMENT cette etape, et rien
  d'autre : il ne score pas, ne decide pas, ne connait pas
  l'incertitude ni le gain d'information. Il fournit seulement une
  liste de positions valides et atteignables en conduisant.

Cette liste est ensuite consommee par n'importe quelle strategie de
decision (aleatoire, distance seule, gain d'info seul, Active SLAM
complet) -- toutes partent du meme jeu de candidats, ce qui rend la
comparaison equitable, comme demande dans le sujet.
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import carla


CARLA_IP = 'localhost'
CARLA_PORT = 2000

REGULAR_SPACING = 25.0        # distance entre candidats "tout droit" (m)
JUNCTION_BRANCH_DEPTH = 15.0   # distance a l'interieur d'une branche d'intersection (m)
MAX_SEARCH_RANGE = 80.0        # portee totale de recherche depuis le vehicule (m)
GENERATION_PERIOD = 3.0        # frequence de regeneration de la liste (s)
DEDUP_RADIUS = 5.0             # deux candidats a moins de 5m sont consideres identiques


class DestinationGenerator(Node):
    def __init__(self):
        super().__init__('destination_generator')

        self.get_logger().info('Connexion a CARLA...')
        self.client = carla.Client(CARLA_IP, CARLA_PORT)
        self.client.set_timeout(15.0)
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()

        self.vehicle = self.find_ego_vehicle()
        if self.vehicle is None:
            self.get_logger().error('Aucun vehicule ego trouve.')
            return

        self.pub_candidates = self.create_publisher(
            String, '/active_slam/candidate_destinations', 10
        )

        self.timer = self.create_timer(GENERATION_PERIOD, self.generate_and_publish)

        self.get_logger().info(
            'Generateur de destinations actif (portee=' + str(MAX_SEARCH_RANGE) + 'm).'
        )

    def find_ego_vehicle(self):
        actors = self.world.get_actors().filter('vehicle.*')
        for a in actors:
            if a.attributes.get('role_name') == 'ego':
                return a
        if len(actors) > 0:
            return actors[0]
        return None

    def generate_and_publish(self):
        vehicle_location = self.vehicle.get_location()
        start_waypoint = self.carla_map.get_waypoint(vehicle_location)

        if start_waypoint is None:
            self.get_logger().warn('Vehicule hors de la route connue, pas de candidats generes.')
            return

        candidates = self.explore_candidates(start_waypoint, vehicle_location)
        candidates = self.deduplicate(candidates)

        msg = String()
        msg.data = json.dumps(candidates)
        self.pub_candidates.publish(msg)

        self.get_logger().info(str(len(candidates)) + ' destinations candidates generees.')

    def explore_candidates(self, start_waypoint, vehicle_location):
        """Parcourt le reseau routier depuis le vehicule et genere :
        - un candidat regulier tous les REGULAR_SPACING m en ligne "normale"
        - un candidat par branche a chaque intersection rencontree
        Retourne une liste de dicts (pas encore dedupliquee)."""
        candidates = []

        # File d'attente de parcours : (waypoint, distance_parcourue_depuis_vehicule)
        frontier = [(start_waypoint, 0.0)]
        visited_ids = set()

        while frontier:
            waypoint, dist_so_far = frontier.pop(0)

            if dist_so_far >= MAX_SEARCH_RANGE:
                continue

            wp_id = waypoint.id
            if wp_id in visited_ids:
                continue
            visited_ids.add(wp_id)

            next_wps = waypoint.next(REGULAR_SPACING)

            if not next_wps:
                continue

            if len(next_wps) == 1:
                # Route simple, pas d'intersection : un candidat "continuer"
                nxt = next_wps[0]
                new_dist = dist_so_far + REGULAR_SPACING
                candidates.append(self.make_candidate(nxt, new_dist, vehicle_location, 'continue'))
                frontier.append((nxt, new_dist))
            else:
                # Intersection : un candidat par branche possible
                for branch_wp in next_wps:
                    # On avance encore un peu dans la branche pour avoir
                    # un point de destination clair (pas juste au bord de l'intersection)
                    branch_target = branch_wp.next(JUNCTION_BRANCH_DEPTH)
                    target_wp = branch_target[0] if branch_target else branch_wp
                    new_dist = dist_so_far + REGULAR_SPACING + JUNCTION_BRANCH_DEPTH
                    candidates.append(
                        self.make_candidate(target_wp, new_dist, vehicle_location, 'branch')
                    )
                    frontier.append((target_wp, new_dist))

        return candidates

    def make_candidate(self, waypoint, distance_from_vehicle, vehicle_location, origin_type):
        loc = waypoint.transform.location
        return {
            'x': round(loc.x, 2),
            'y': round(loc.y, 2),
            'distance_from_vehicle': round(distance_from_vehicle, 2),
            'is_junction_branch': origin_type == 'branch',
        }

    def deduplicate(self, candidates):
        """Fusionne les candidats trop proches les uns des autres (evite
        les doublons quand plusieurs chemins de parcours convergent vers
        la meme zone)."""
        unique = []
        for cand in candidates:
            is_duplicate = False
            for existing in unique:
                dx = cand['x'] - existing['x']
                dy = cand['y'] - existing['y']
                if math.sqrt(dx * dx + dy * dy) < DEDUP_RADIUS:
                    is_duplicate = True
                    break
            if not is_duplicate:
                unique.append(cand)
        return unique


def main(args=None):
    rclpy.init(args=args)
    node = DestinationGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
