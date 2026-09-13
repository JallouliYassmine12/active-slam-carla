#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
import csv
import os
import math
import glob
import datetime


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class LocalizationEvaluator(Node):

    def __init__(self):

        super().__init__('localization_evaluator')

        # Paramètres
        self.declare_parameter('ground_truth_topic', '/odom')
        self.declare_parameter('estimated_topic', '/localization_pose')
        self.declare_parameter(
            'output_file',
            '~/active_slam_carla/metrics/localization_error.csv'
        )

        self.gt_topic = self.get_parameter(
            'ground_truth_topic'
        ).value

        self.est_topic = self.get_parameter(
            'estimated_topic'
        ).value


        self.output_file = os.path.expanduser(
            self.get_parameter('output_file').value
        )


        # ==============================
        # Gestion des dossiers résultats
        # ==============================

        self.metrics_dir = os.path.dirname(
            self.output_file
        )

        self.all_runs_dir = os.path.join(
            self.metrics_dir,
            "all_runs"
        )

        self.best_results_dir = os.path.join(
            self.metrics_dir,
            "best_results"
        )


        os.makedirs(
            self.all_runs_dir,
            exist_ok=True
        )

        os.makedirs(
            self.best_results_dir,
            exist_ok=True
        )


        # Buffers

        self.gt_buffer = {}
        self.est_buffer = {}

        self.trajectory_data = []

        self.total_error = 0.0
        self.count = 0


        # Alignement

        self.gt_origin = None
        self.est_origin = None

        self.first_gt_seen = None

        self.align_min_distance = 2.0


        # Logging

        self.logging_active = True


        # Subscribers

        self.gt_sub = self.create_subscription(
            Odometry,
            self.gt_topic,
            self.gt_callback,
            10
        )


        self.est_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.est_topic,
            self.est_callback,
            10
        )


        self.stop_sub = self.create_subscription(
            Bool,
            '/slam_evaluation/stop_logging',
            self.stop_callback,
            10
        )


        # Timer

        self.create_timer(
            1.0,
            self.evaluate
        )


        self.get_logger().info(
            '📊 Evaluateur démarré (ATE aligné)'
        )


    def gt_callback(self,msg):

        t = (
            msg.header.stamp.sec +
            msg.header.stamp.nanosec / 1e9
        )


        p = msg.pose.pose.position


        self.gt_buffer[t] = {

            'x':p.x,
            'y':p.y,
            'z':p.z,
            'yaw':yaw_from_quat(
                msg.pose.pose.orientation
            )
        }



    def est_callback(self,msg):

        t = (
            msg.header.stamp.sec +
            msg.header.stamp.nanosec / 1e9
        )


        p = msg.pose.pose.position


        # Filtre pose nulle

        if abs(p.x)<1e-6 and abs(p.y)<1e-6:

            self.rejected_zero = getattr(
                self,
                'rejected_zero',
                0
            ) + 1

            return



        # Filtre saut ICP

        if hasattr(self,'last_est'):

            jump = math.sqrt(
                (p.x-self.last_est['x'])**2 +
                (p.y-self.last_est['y'])**2
            )


            # --- Seuil exprime en VITESSE et non en distance ---
            # Le seuil fixe de 5 m rejetait tout deplacement normal
            # au-dela de 5 m/s. A 14 m/s la quasi-totalite des poses
            # etait jetee, et l'ATE n'etait calcule que sur les phases
            # lentes ou a l'arret : il etait optimiste par construction.
            # 20 m/s borne la vitesse physiquement plausible ; les 15 m
            # de marge couvrent les corrections de graphe apres une
            # fermeture de boucle, qui sont legitimes.
            dt = t - getattr(self, 'last_est_t', t - 1.0)
            self.last_est_t = t
            if dt <= 0.0 or dt > 5.0:
                dt = 1.0
            seuil = 20.0 * dt + 15.0

            if jump > seuil:

                self.rejected_jump = getattr(
                    self,
                    'rejected_jump',
                    0
                ) + 1

                self.get_logger().warn(
                    f'Saut de pose rejete : {jump:.1f}m en {dt:.2f}s '
                    f'({jump/dt:.1f} m/s > seuil {seuil:.1f}m)'
                )
                self.last_est={
                    'x':p.x,
                    'y':p.y
                }

                return



        self.last_est={
            'x':p.x,
            'y':p.y
        }



        self.est_buffer[t]={

            'x':p.x,
            'y':p.y,
            'z':p.z,

            'yaw':yaw_from_quat(
                msg.pose.pose.orientation
            )

        }



    def stop_callback(self,msg):

        if msg.data and self.logging_active:

            self.logging_active=False

            self.get_logger().info(
                "🛑 Arrêt reçu, sauvegarde finale"
            )

            self.save_data()



    @staticmethod
    def to_local(pose,origin):

        dx = pose['x']-origin['x']
        dy = pose['y']-origin['y']


        c = math.cos(-origin['yaw'])
        s = math.sin(-origin['yaw'])


        return {

            'x':c*dx-s*dy,
            'y':s*dx+c*dy,
            'z':pose['z']-origin['z']

        }




    def evaluate(self):

        if not self.logging_active:
            return


        TOLERANCE=0.1


        matched=[]


        for t_est in sorted(
            self.est_buffer.keys()
        ):

            if not self.gt_buffer:
                continue


            closest_t_gt=min(
                self.gt_buffer.keys(),
                key=lambda t:
                abs(t-t_est)
            )


            if abs(
                closest_t_gt-t_est
            ) <= TOLERANCE:

                matched.append(
                    (
                        closest_t_gt,
                        t_est
                    )
                )



        for t_gt,t_est in matched:


            gt=self.gt_buffer.get(t_gt)
            est=self.est_buffer.get(t_est)



            if gt is None or est is None:
                continue



            if self.gt_origin is None:


                if self.first_gt_seen is None:

                    self.first_gt_seen=(
                        gt['x'],
                        gt['y']
                    )

                    continue



                moved=math.hypot(
                    gt['x']-self.first_gt_seen[0],
                    gt['y']-self.first_gt_seen[1]
                )


                if moved < self.align_min_distance:
                    continue



                self.gt_origin=dict(gt)
                self.est_origin=dict(est)



            gt_l=self.to_local(
                gt,
                self.gt_origin
            )


            est_l=self.to_local(
                est,
                self.est_origin
            )

            # Le nuage LiDAR est publie dans la convention main gauche de
            # CARLA alors que la verite terrain est convertie en convention
            # ROS. La trajectoire estimee est donc l'image miroir de la
            # trajectoire reelle : on retablit l'axe lateral avant de
            # comparer. Une reflexion etant une isometrie, aucune autre
            # metrique n'est affectee.
            est_l['y'] = -est_l['y']


            error=math.sqrt(

                (gt_l['x']-est_l['x'])**2 +
                (gt_l['y']-est_l['y'])**2

            )


            self.total_error+=error
            self.count+=1



            self.trajectory_data.append({

                'timestamp':t_est,

                'gt_x':gt_l['x'],
                'gt_y':gt_l['y'],

                'est_x':est_l['x'],
                'est_y':est_l['y'],

                'error':error,

                'z_drift':
                est_l['z']-gt_l['z']

            })



            self.gt_buffer.pop(
                t_gt,
                None
            )

            self.est_buffer.pop(
                t_est,
                None
            )



        if (
            len(self.trajectory_data)>0
            and
            len(self.trajectory_data)%20==0
        ):

            self.save_data()




    # ===================================
    # Nouvelle sauvegarde automatique
    # ===================================

    def save_data(self):

        if not self.trajectory_data:
            return



        timestamp=datetime.datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )



        run_file=os.path.join(

            self.all_runs_dir,

            f"localization_error_{timestamp}.csv"

        )



        with open(
            run_file,
            'w',
            newline=''
        ) as f:


            writer=csv.DictWriter(

                f,

                fieldnames=
                self.trajectory_data[0].keys()

            )


            writer.writeheader()
            writer.writerows(
                self.trajectory_data
            )



        ate=self.total_error/self.count



        self.get_logger().info(
            f"RESULTAT DU RUN | ATE moyen={ate:.3f} m | "
            f"echantillons={self.count} | "
            f"poses rejetees (saut)={getattr(self, 'rejected_jump', 0)} | "
            f"fichier={run_file}"
        )

        self.check_best_result(
            ate
        )





    def check_best_result(self,ate):


        best_files=glob.glob(

            os.path.join(

                self.best_results_dir,

                "best_ATE_*.csv"

            )

        )



        save_best=False



        if len(best_files)==0:

            save_best=True



        else:


            best_values=[]


            for f in best_files:

                name=os.path.basename(f)

                value=float(

                    name.split("_")[2]
                    .replace("m.csv","")

                )

                best_values.append(value)



            if ate < min(best_values):

                save_best=True




        if save_best:


            best_file=os.path.join(

                self.best_results_dir,

                f"best_ATE_{ate:.3f}m.csv"

            )



            with open(
                best_file,
                'w',
                newline=''
            ) as f:


                writer=csv.DictWriter(

                    f,

                    fieldnames=
                    self.trajectory_data[0].keys()

                )


                writer.writeheader()
                writer.writerows(
                    self.trajectory_data
                )



            self.get_logger().info(
                f"🏆 Nouveau meilleur ATE {ate:.3f}m"
            )




def main(args=None):

    rclpy.init(args=args)

    node=LocalizationEvaluator()


    try:

        rclpy.spin(node)


    except KeyboardInterrupt:

        pass


    finally:

        node.save_data()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()



if __name__=="__main__":

    main()
