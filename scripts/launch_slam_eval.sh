#!/bin/bash

echo "======================================"
echo "🚗 LANCEMENT SLAM + ÉVALUATION"
echo "======================================"

# Configuration
export CARLA_HOST=127.0.0.1
export CARLA_PORT=2000
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Sourcer ROS2
source /opt/ros/humble/setup.bash
source ~/active_slam_carla/ros2_ws/install/setup.bash

echo ""
echo "📋 Assure-toi que CARLA est lancé sur Windows !"
echo "📋 Appuie sur Entrée pour continuer..."
read

echo "🚀 Lancement de la démo..."
ros2 launch slam_evaluation slam_evaluation.launch.py
