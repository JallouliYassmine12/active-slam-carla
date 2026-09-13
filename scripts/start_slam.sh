#!/bin/bash

# Nettoyer et configurer
rm -f ~/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Sourcer
source /opt/ros/humble/setup.bash
source ~/active_slam_carla/ros2_ws/install/setup.bash

# Vérifier
echo "✅ RMW_IMPLEMENTATION = $RMW_IMPLEMENTATION"

# Lancer
echo "🚗 Lancement de la démo SLAM..."
echo "📋 Assure-toi que CARLA tourne sur Windows !"
echo "📋 Appuie sur Entrée pour continuer..."
read

ros2 launch slam_evaluation slam_evaluation.launch.py
