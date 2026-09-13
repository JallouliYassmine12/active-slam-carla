#!/usr/bin/env bash
# Usage : bash ~/demo_video.sh <strategie> <graine>
#   bash ~/demo_video.sh weighted 8
#   bash ~/demo_video.sh predefinie 8
S="${1:-weighted}"; G="${2:-8}"
LOG=~/logs_campagne2
source /opt/ros/humble/setup.bash
source ~/active_slam_carla/ros2_ws/install/setup.bash

rm -f $LOG/${S}G_${G}_essai*.log
VARIANTE=G bash ~/un_scenario.sh "$S" "$G" &
RUN_PID=$!

F=""
while [ -z "$F" ]; do
  sleep 2
  kill -0 $RUN_PID 2>/dev/null || { echo "run termine avant detection"; exit 1; }
  F=$(ls -t $LOG/${S}G_${G}_essai*.log 2>/dev/null | head -1)
done

echo ""; echo "=================================================="
echo ">>> RUN MESURE DEMARRE — LANCE L'ENREGISTREMENT <<<"
echo "=================================================="; echo ""
sleep 8

ros2 run rtabmap_viz rtabmap_viz --ros-args \
  -r __ns:=/rtabmap \
  -p subscribe_depth:=true -p subscribe_scan_cloud:=true \
  -p approx_sync:=true -p frame_id:=base_link \
  -r rgb/image:=/carla/ego/camera/image_raw \
  -r depth/image:=/carla/ego/camera/depth \
  -r rgb/camera_info:=/carla/ego/camera/camera_info \
  -r scan_cloud:=/carla/ego/lidar/points &

rviz2 -d ~/demo.rviz &

wait $RUN_PID
echo ""; echo ">>> RUN TERMINE — arrete l'enregistrement"
