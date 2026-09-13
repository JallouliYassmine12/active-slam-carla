#!/usr/bin/env bash
# =====================================================================
# Run de validation court (600 m), strategie weighted
# =====================================================================
#
# Sert a verifier un correctif AVANT de lancer les 3 h de campagne. Il
# reprend exactement la sequence de campagne_complete.sh -- nettoyage
# des noeuds, redemarrage de CARLA, meme ligne 'ros2 launch' avec les
# memes noms de parametres -- mais avec un budget de 600 m au lieu de
# 1200 m.
#
# Ce n'est PAS un run de campagne : son resultat ne doit jamais entrer
# dans les moyennes. Il ne repond qu'a une question : le correctif
# produit-il l'effet attendu dans les journaux ?
#
# Usage :
#     bash ~/valider.sh                      -> ~/valid_frontiere.log
#     bash ~/valider.sh mon_nom              -> ~/valid_mon_nom.log
#
# Suivre pendant le run, dans un second terminal :
#     tail -f ~/valid_frontiere.log | grep --line-buffered "CRITERES"
# =====================================================================

set -u

NOM="${1:-frontiere}"
LOG="$HOME/valid_${NOM}.log"

CARLA_EXE="/mnt/c/carla/CarlaUE4.exe"
BUDGET_DISTANCE=600.0
BUDGET_DUREE=600.0

TASKKILL="/mnt/c/Windows/System32/taskkill.exe"
[ -x "$TASKKILL" ] || TASKKILL="taskkill.exe"

# --- Refus de tourner pendant une campagne ---------------------------
# Deux 'ros2 launch' sur le meme domaine ROS, c'est deux vehicules et
# deux decision_maker ecrivant dans les memes fichiers de metrics/.
# Rien ne le signalerait dans les resultats.
if pgrep -f "[c]ampagne_complete.sh" >/dev/null 2>&1; then
    echo "ERREUR : une campagne est en cours. Run de validation refuse."
    ps -ef | grep "[c]ampagne_complete.sh"
    exit 1
fi

# --- Nettoyage -------------------------------------------------------
# Liste identique a celle de campagne_complete.sh. vehicle_controller_eval
# est le plus important : un exemplaire orphelin piloterait le vehicule
# en concurrence avec le controleur du run.
nettoyer_ros2() {
    for motif in "ros2 launch" rtabmap icp_odometry \
                 bridge_node bridge_active_slam \
                 decision_maker candidate_generator obstacle_detector \
                 traffic_spawner localization_evaluator \
                 vehicle_controller_eval vehicle_controller_active_slam \
                 static_transform_publisher imu_filter_madgwick ; do
        pkill -9 -f "$motif" >/dev/null 2>&1
    done
    sleep 3
}

carla_repond() {
    python3 - <<'EOF' >/dev/null 2>&1
import sys
try:
    import carla
    c = carla.Client('127.0.0.1', 2000)
    c.set_timeout(5.0)
    c.get_world().get_map()
    sys.exit(0)
except Exception:
    sys.exit(1)
EOF
}

echo "=== Run de validation : ${NOM} (${BUDGET_DISTANCE%.*} m) ==="
echo "journal : $LOG"
echo ""

nettoyer_ros2

echo "[CARLA] redemarrage"
"$TASKKILL" /F /IM CarlaUE4.exe                >/dev/null 2>&1
"$TASKKILL" /F /IM CarlaUE4-Win64-Shipping.exe >/dev/null 2>&1
"$TASKKILL" /F /IM CrashReportClient.exe       >/dev/null 2>&1
sleep 5

"$CARLA_EXE" -quality-level=Low -carla-rpc-port=2000 >/dev/null 2>&1 &

pret=0
for essai in $(seq 1 40); do
    sleep 5
    if carla_repond; then
        echo "[CARLA] pret apres $((essai * 5)) s"
        sleep 10   # laisse la carte finir de se charger
        pret=1
        break
    fi
done

if [ "$pret" -ne 1 ]; then
    echo "[CARLA] ne repond pas apres 200 s. Abandon."
    exit 1
fi

echo "[RUN] demarrage"
timeout --signal=INT --kill-after=30 900 \
    ros2 launch active_slam_decision active_slam.launch.py \
    strategy:=weighted \
    run_duration:="$BUDGET_DUREE" \
    max_distance:="$BUDGET_DISTANCE" \
    weather:=clear \
    > "$LOG" 2>&1

nettoyer_ros2

echo ""
echo "=== Fin du run ==="
echo ""

echo "--- Donnees RTAB-Map recues ? ---"
grep -oE "grid=[A-Z]+ \| pose_cov=[A-Z]+ \| graph=[A-Z]+" "$LOG" \
    | sort | uniq -c
echo ""

echo "--- Etendue des criteres (10 dernieres decisions) ---"
grep "CRITERES" "$LOG" | tail -10 | sed 's/^.*CRITERES/CRITERES/'
echo ""

echo "--- Fin de run ---"
grep "FIN DE RUN" "$LOG" || echo "  (aucune ligne FIN DE RUN : run interrompu)"
echo ""
echo "Journal complet : $LOG"
