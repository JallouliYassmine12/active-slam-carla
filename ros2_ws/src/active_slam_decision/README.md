# active_slam_decision

Nouveau package ROS 2, distinct de l'ancien package `active_slam` (buggé) et
du package `slam_evaluation` (SLAM classique par waypoints prédéfinis).
Il porte le cœur du sujet de stage : la sélection automatique de la
prochaine destination du véhicule.

## Ce qui est réutilisé (pas recréé)

- **`vehicle_controller_active_slam`** (package `vehicle_controller`, déjà
  développé) : s'abonne à `/goal` et `/decision_maker/maneuver`, publie
  `/vehicle/arrived`. C'est le "contrôle véhicule pour l'exploration" —
  il existe déjà, le nouveau launch file le réutilise tel quel au lieu
  d'en recréer un.
- **Bridge CARLA, transforms statiques, filtre IMU, odométrie ICP,
  RTAB-Map, rtabmap_viz, localization_evaluator** : repris tels quels de
  `slam_evaluation.launch.py`.

## Ce qui est nouveau

| Fichier | Rôle |
|---|---|
| `candidate_generator.py` | Génère les destinations candidates sur le réseau routier CARLA (`generate_waypoints`) autour du véhicule, publie `/active_slam/candidate_goals` |
| `information_gain.py` | Gain d'information attendu par candidat, via cellules inconnues de `/rtabmap/grid_prob_map` |
| `localization_uncertainty.py` | Incertitude de localisation courante, via covariance de `/rtabmap/localization_pose` |
| `loop_closure_potential.py` | Potentiel de fermeture de boucle par candidat, via `/rtabmap/mapGraph` |
| `safety_evaluator.py` | Score de sécurité par candidat, via un topic d'obstacles (paramétrable) |
| `strategies.py` | Stratégie pondérée (méthode proposée) + 3 stratégies de comparaison (random, distance, info_gain) |
| `decision_maker.py` | Noeud principal : combine les 5 critères, publie `/goal` |
| `launch/active_slam.launch.py` | Lance tout le pipeline Active SLAM |

## Architecture des topics

```
candidate_generator ──/active_slam/candidate_goals──▶ decision_maker
RTAB-Map ──/rtabmap/grid_prob_map──────────────────▶ decision_maker
RTAB-Map ──/rtabmap/localization_pose──────────────▶ decision_maker
RTAB-Map ──/rtabmap/mapGraph────────────────────────▶ decision_maker
(detection obstacles) ──/obstacle_detection/obstacles──▶ decision_maker
decision_maker ──/goal────────────────────────────▶ vehicle_controller_active_slam
vehicle_controller_active_slam ──/vehicle/arrived──▶ decision_maker (déclenche le choix suivant)
```

## Installation

```bash
# Copier ce dossier dans ton workspace
cp -r active_slam_decision ~/active_slam_carla/ros2_ws/src/

cd ~/active_slam_carla/ros2_ws
colcon build --packages-select active_slam_decision --symlink-install
source install/setup.bash

ros2 launch active_slam_decision active_slam.launch.py
```

Pour comparer les stratégies (objectif du sujet : comparaison à une
sélection aléatoire, une trajectoire par distance, ou par gain
d'information seul) :

```bash
ros2 launch active_slam_decision active_slam.launch.py strategy:=random
ros2 launch active_slam_decision active_slam.launch.py strategy:=distance
ros2 launch active_slam_decision active_slam.launch.py strategy:=info_gain
ros2 launch active_slam_decision active_slam.launch.py strategy:=weighted
```

Chaque run écrit un CSV (`metrics/decisions_<timestamp>.csv`) avec le
détail des scores de la décision choisie à chaque étape, en plus du CSV
déjà produit par `localization_evaluator`.

## Points à vérifier / adapter sur ton système (hypothèses faites)

1. **Type et nom exact du topic d'obstacles.** J'ai supposé
   `/obstacle_detection/obstacles` en `geometry_msgs/msg/PoseArray`.
   Si ton noeud de détection existant publie autre chose, ajuste
   `obstacles_topic` (paramètre) et le parsing dans
   `decision_maker.on_obstacles`.
2. **Alignement des repères CARLA ↔ RTAB-Map ('map').**
   `candidate_generator.py` applique l'inversion Y standard
   (repère main gauche CARLA → main droite ROS). Vérifie que
   `carla_sensors_bridge` applique exactement la même convention pour
   les données capteurs, sinon les candidats seront décalés par rapport
   à la carte construite par RTAB-Map.
3. **Poids par défaut** (`information_gain=0.30`,
   `localization_uncertainty=0.20`, `loop_closure=0.20`,
   `distance=0.15`, `safety=0.15`) sont un point de départ raisonnable
   mais arbitraire — à ajuster une fois que tu observes le comportement
   réel du véhicule.
4. **Pas d'arrêt automatique** dans `active_slam.launch.py`
   (contrairement à `slam_evaluation.launch.py`, dont le contrôleur par
   waypoints a une distance fixe). À ajouter plus tard si tu veux
   automatiser des runs de comparaison (ex: nombre de destinations
   atteintes, durée max).
5. **`information_gain.py`** utilise une approximation simple (comptage
   de cellules inconnues dans un rayon). C'est volontairement léger pour
   tourner en temps réel ; si besoin d'un calcul plus fidèle (entropie de
   Shannon, raytracing), c'est le seul fichier à modifier.
