#!/usr/bin/env bash
# =====================================================================
# Reprise des trois runs 'predefinie' (SLAM classique)
# =====================================================================
#
# POURQUOI CE SCRIPT
# ------------------
# La campagne a declare 'predefinie' repetitions 2 et 3 VALIDES, mais
# l'analyse a du en exclure deux :
#
#     predefinie rep 1 : n'atteint que 106 m  -> exclu
#     predefinie rep 3 : n'atteint que 44 m   -> exclu
#
# Les deux affirmations se contredisent. La cause est dans archiver() de
# campagne_complete.sh :
#
#     ate=$(ls -t "$METRICS"/all_runs/localization_error_*.csv | head -1)
#
# Le fichier le plus RECENT du dossier n'est pas forcement celui du run
# qui vient de se terminer : un run de chauffe, une tentative
# interrompue ou un run manuel lance entre-temps peuvent l'avoir
# supplante. Resultat : un fichier de 14 echantillons couvrant 44 m
# archive sous le nom d'un run de 1200 m.
#
# Consequence : la comparaison SLAM classique / Active SLAM, qui est la
# comparaison centrale du sujet, ne repose plus que sur UN run.
#
# CE QUE CE SCRIPT CORRIGE
# ------------------------
# 1. Il ne retient que les fichiers ATE creees APRES le debut du run
#    (find -newermt), pas simplement le plus recent du dossier.
#
# 2. Il verifie le CONTENU du fichier archive : un run de 1200 m produit
#    entre 180 et 450 echantillons ; les deux runs fautifs en avaient 20
#    et 14. Un fichier de moins de MIN_ECHANTILLONS lignes invalide le
#    run, quoi qu'en dise le journal.
#
# Ces deux controles sont independants : le premier evite de prendre le
# mauvais fichier, le second detecte le cas ou on l'aurait quand meme
# pris.
#
# PROTOCOLE
# ---------
# Identique a celui de la campagne : run de chauffe jete apres chaque
# demarrage de CARLA, run valide seulement s'il atteint le budget de
# distance, nombre de tentatives journalise.
#
# Usage :
#     nohup bash ~/refaire_predefinie.sh > ~/predefinie.log 2>&1 &
#
# Duree : environ 25 minutes. Ne PAS lancer pendant qu'une campagne
# tourne (le verrou l'interdit de toute facon).
# =====================================================================

set -u

VERROU="$HOME/.campagne_complete.lock"
exec 9>"$VERROU"
if ! flock -n 9; then
    echo "ERREUR : une campagne est deja en cours d'execution."
    ps -ef | grep "[c]ampagne_complete.sh"
    exit 1
fi

CARLA_EXE="/mnt/c/carla/CarlaUE4.exe"
METRICS="$HOME/active_slam_carla/metrics"
LOGS="$HOME/logs_campagne2"
SUIVI="$LOGS/tentatives_predefinie.csv"

BUDGET_DISTANCE=1200.0
MAX_TENTATIVES=4

# Nombre minimal d'echantillons dans le fichier ATE d'un run de 1200 m.
# Releve sur les runs valides de la campagne : 182 a 451 echantillons.
# Les deux runs fautifs en avaient 20 et 14.
MIN_ECHANTILLONS=120

mkdir -p "$LOGS"
echo "repetition,tentatives,resultat,cause,echantillons" > "$SUIVI"

TASKKILL="/mnt/c/Windows/System32/taskkill.exe"
[ -x "$TASKKILL" ] || TASKKILL="taskkill.exe"


arreter_carla() {
    "$TASKKILL" /F /IM CarlaUE4.exe                >/dev/null 2>&1
    "$TASKKILL" /F /IM CarlaUE4-Win64-Shipping.exe >/dev/null 2>&1
    "$TASKKILL" /F /IM CrashReportClient.exe       >/dev/null 2>&1
    sleep 5
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

demarrer_carla() {
    echo "    [CARLA] redemarrage"
    arreter_carla
    "$CARLA_EXE" -quality-level=Low -carla-rpc-port=2000 >/dev/null 2>&1 &
    for essai in $(seq 1 40); do
        sleep 5
        if carla_repond; then
            echo "    [CARLA] pret apres $((essai * 5)) s"
            sleep 10
            return 0
        fi
    done
    echo "    [CARLA] ne repond pas apres 200 s"
    return 1
}

# Tue tout ce qui pourrait subsister d'un run precedent. La liste est
# volontairement exhaustive : un static_transform_publisher ou un
# vehicle_controller_eval orphelin survit aux pkill partiels et pilote
# le vehicule en concurrence avec le run suivant.
nettoyer_ros2() {
    for motif in "ros2 launch" rtabmap icp_odometry bridge_node \
                 bridge_active_slam decision_maker candidate_generator \
                 obstacle_detector traffic_spawner localization_evaluator \
                 vehicle_controller_eval static_transform_publisher \
                 imu_filter_madgwick ; do
        pkill -9 -f "$motif" >/dev/null 2>&1
    done
    sleep 3
}

lancer_run() {
    local log="$1"
    nettoyer_ros2
    timeout --signal=INT --kill-after=30 1500 \
        ros2 launch slam_evaluation slam_evaluation.launch.py \
        distance:="$BUDGET_DISTANCE" weather:=clear \
        > "$log" 2>&1
    nettoyer_ros2
}

# Renvoie le fichier ATE produit APRES l'instant passe en argument, ou
# une chaine vide. C'est le correctif principal : on ne prend plus le
# plus recent du dossier, on prend le plus recent PARMI CEUX CREES
# PENDANT CE RUN.
fichier_ate_du_run() {
    local debut="$1"
    find "$METRICS/all_runs" -name 'localization_error_*.csv' \
         -newermt "@$debut" -printf '%T@ %p\n' 2>/dev/null \
        | sort -rn | head -1 | cut -d' ' -f2-
}

main() {
    echo "=== REPRISE DES RUNS PREDEFINIE : debut $(date +%H:%M:%S) ==="

    demarrer_carla || exit 1
    echo "  run de chauffe initial (jete)"
    lancer_run "$LOGS/chauffe_predefinie.log"

    for repetition in 1 2 3; do
        echo ""
        echo "=== PREDEFINIE REPETITION $repetition ==="

        tentative=1
        while [ "$tentative" -le "$MAX_TENTATIVES" ]; do
            log="$LOGS/predefinie_${repetition}_essai${tentative}.log"
            echo "  -> tentative $tentative ($(date +%H:%M:%S))"

            debut=$(date +%s)
            lancer_run "$log"

            cause=""
            echantillons=0

            if [ ! -s "$log" ]; then
                cause="log vide"
            elif ! grep -q "budget de distance atteint" "$log"; then
                cause="budget non atteint (plantage ou arret premature)"
            else
                ate=$(fichier_ate_du_run "$debut")
                if [ -z "$ate" ]; then
                    # Le run dit avoir fini, mais aucun fichier ATE n'a
                    # ete produit pendant son execution. C'est le cas que
                    # l'ancien archiver() masquait en recuperant un
                    # fichier d'un run precedent.
                    cause="aucun fichier ATE produit pendant ce run"
                else
                    echantillons=$(( $(wc -l < "$ate") - 1 ))
                    if [ "$echantillons" -lt "$MIN_ECHANTILLONS" ]; then
                        cause="fichier ATE trop court ($echantillons echantillons, minimum $MIN_ECHANTILLONS)"
                    fi
                fi
            fi

            if [ -z "$cause" ]; then
                cp "$ate" "$METRICS/ate_predefinie_${repetition}.csv"
                cp "$log" "$LOGS/predefinie_${repetition}.log"
                echo "     valide, archive ($echantillons echantillons)"
                echo "     source : $(basename "$ate")"
                echo "$repetition,$tentative,valide,,$echantillons" >> "$SUIVI"
                break
            fi

            echo "     INVALIDE : $cause"
            echo "$repetition,$tentative,invalide,$cause,$echantillons" >> "$SUIVI"

            if ! demarrer_carla; then
                echo "     abandon : CARLA ne redemarre pas"
                break
            fi
            echo "     run de chauffe (jete)"
            lancer_run "$LOGS/chauffe_$(date +%H%M%S).log"

            tentative=$((tentative + 1))
            sleep 10
        done
    done

    echo ""
    echo "=== TERMINE : $(date +%H:%M:%S) ==="
    echo ""
    column -s, -t < "$SUIVI"
    echo ""
    echo "Fichiers archives :"
    for r in 1 2 3; do
        f="$METRICS/ate_predefinie_${r}.csv"
        if [ -f "$f" ]; then
            echo "  ate_predefinie_${r}.csv : $(( $(wc -l < "$f") - 1 )) echantillons"
        else
            echo "  ate_predefinie_${r}.csv : ABSENT"
        fi
    done
    echo ""
    echo "Analyse :"
    echo "  python3 ~/analyse_campagne.py"
}

main
