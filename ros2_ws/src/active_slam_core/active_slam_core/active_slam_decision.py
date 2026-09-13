"""
Fonction de decision Active SLAM (coeur du sujet).

Recoit la liste de destinations candidates (generees sur le reseau
routier) et evalue chacune selon plusieurs criteres, puis publie la
meilleure comme prochaine destination.

Version 1 : criteres implementes = distance + risque/obstacle.
Les criteres suivants (gain d'information, incertitude de
localisation, potentiel de fermeture de boucle) seront ajoutes en
gardant la meme structure de score, sans casser cette base.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


MAX_SEARCH_RANGE = 80.0     # doit correspondre a destination_generator
CRITICAL_DISTANCE = 5.0     # meme seuil que decision_maker (obstacle proche)

WEIGHT_DISTANCE = 0.4
WEIGHT_SECURITE = 0.6


class ActiveSlamDecisionNode(Node):
    def __init__(self):
        super().__init__('active_slam_decision')

        self.last_obstacle_detected = False
        self.last_obstacle_distance = None

        self.sub_candidates = self.create_subscription(
            String, '/active_slam/candidate_destinations',
            self.candidates_callback, 10
        )
        self.sub_obstacle = self.create_subscription(
            String, '/obstacle_detection/result',
            self.obstacle_callback, 10
        )

        self.pub_goal = self.create_publisher(
            PoseStamped, '/active_slam/next_goal', 10
        )

        self.get_logger().info('Fonction de decision Active SLAM active (v1: distance + securite).')

    def obstacle_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError):
            return

        self.last_obstacle_detected = data.get('detected', False)
        self.last_obstacle_distance = data.get('distance', None)

    def candidates_callback(self, msg):
        try:
            candidates = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError):
            return

        if not candidates:
            return

        scored = []
        for cand in candidates:
            score, details = self.score_candidate(cand)
            scored.append((score, details, cand))

        scored.sort(key=lambda item: item[0], reverse=True)
        best_score, best_details, best_candidate = scored[0]

        self.get_logger().info(
            'Meilleur candidat: x=' + str(best_candidate['x'])
            + ', y=' + str(best_candidate['y'])
            + ', score=' + str(round(best_score, 3))
            + ' (' + best_details + ')'
        )

        self.publish_goal(best_candidate)

    def score_candidate(self, candidate):
        distance = candidate['distance_from_vehicle']
        is_branch = candidate['is_junction_branch']

        # --- Critere 1 : distance (plus proche = mieux, normalise 0..1) ---
        score_distance = 1.0 - min(distance / MAX_SEARCH_RANGE, 1.0)

        # --- Critere 2 : securite / obstacle ---
        obstacle_proche = (
            self.last_obstacle_detected
            and self.last_obstacle_distance is not None
            and self.last_obstacle_distance < CRITICAL_DISTANCE
        )

        if obstacle_proche and not is_branch:
            # Danger immediat devant, et ce candidat continue tout droit
            score_securite = 0.0
        else:
            score_securite = 1.0

        total = WEIGHT_DISTANCE * score_distance + WEIGHT_SECURITE * score_securite

        details = (
            'dist=' + str(round(score_distance, 2))
            + ', secu=' + str(round(score_securite, 2))
        )
        return total, details

    def publish_goal(self, candidate):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = candidate['x']
        msg.pose.position.y = candidate['y']
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.pub_goal.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActiveSlamDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
