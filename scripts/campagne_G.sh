#!/usr/bin/env bash
# Campagne G (zone bornee) : 3 bras x 5 graines, appariees.
# Les runs deja archives sont sautes SANS pause : inutile de
# laisser refroidir une machine qui n'a rien calcule.
for g in 1 2 3 4 5; do
  for s in weighted random predefinie; do
    if [ -f "$HOME/active_slam_carla/metrics/ate_${s}G_${g}.csv" ]; then
      echo "[$(date +%H:%M:%S)] ${s}G_${g} deja archive, saute"
      continue
    fi
    echo "[$(date +%H:%M:%S)] === ${s}G_${g} ==="
    VARIANTE=G bash ~/un_scenario.sh "$s" "$g"
    sleep 600
  done
done
echo "[$(date +%H:%M:%S)] CAMPAGNE G TERMINEE"
