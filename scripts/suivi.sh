#!/usr/bin/env bash
# =====================================================================
# Suivi en direct de la campagne, sans fenetre CARLA
# =====================================================================
#
# Affiche toutes les 10 secondes l'etat du run en cours, lu dans son
# journal. Remplace avantageusement la fenetre du simulateur : la
# distance parcourue et la vitesse disent en un coup d'oeil si le
# vehicule avance, ce qu'une image ne dit pas.
#
# Usage :
#     bash ~/suivi.sh
#
# Ctrl-C pour arreter le suivi. Cela n'interrompt PAS la campagne.
# =====================================================================

set -u

LOGS="$HOME/logs_campagne2"
INTERVALLE=10

precedente=""
inchange=0

printf "Suivi de la campagne - Ctrl-C pour quitter (n'arrete pas la campagne)\n\n"

while true; do
    log=$(ls -t "$LOGS"/*.log 2>/dev/null | head -1)

    if [ -z "$log" ]; then
        printf "%s  aucun journal de run pour l'instant\n" "$(date +%H:%M:%S)"
        sleep "$INTERVALLE"
        continue
    fi

    run=$(basename "$log" .log)

    # Derniere distance annoncee par le controleur, en metres.
    distance=$(grep -oE "distance parcourue=[0-9]+" "$log" 2>/dev/null \
               | tail -1 | grep -oE "[0-9]+")
    [ -z "${distance:-}" ] && distance=0

    destinations=$(grep -c "Destination atteinte" "$log" 2>/dev/null)
    vitesse=$(grep -oE "vitesse=[0-9.]+" "$log" 2>/dev/null | tail -1 \
              | cut -d= -f2)
    [ -z "${vitesse:-}" ] && vitesse="-"

    enlisements=$(grep -c "ENLISEMENT DETECTE" "$log" 2>/dev/null)
    rejets=$(grep -c "BUT REJETE" "$log" 2>/dev/null)

    # Etendue du gain d'information a la derniere decision : dit si le
    # critere le plus lourd de la fonction de decision classe encore
    # quelque chose.
    etendue=$(grep -oE "etendue=[0-9.]+" "$log" 2>/dev/null | tail -1 \
              | cut -d= -f2)
    [ -z "${etendue:-}" ] && etendue="-"

    # Detection de stagnation : la distance n'a pas bouge depuis
    # plusieurs releves alors qu'un run est en cours.
    if [ "$distance" = "$precedente" ]; then
        inchange=$((inchange + 1))
    else
        inchange=0
    fi
    precedente="$distance"

    alerte=""
    if [ "$inchange" -ge 6 ]; then
        alerte="  <-- immobile depuis $((inchange * INTERVALLE))s"
    fi

    printf "%s  %-28s %5s m  %2s dest  v=%-5s  enlis=%-2s rejets=%-2s  etendue=%s%s\n" \
        "$(date +%H:%M:%S)" "$run" "$distance" "$destinations" \
        "$vitesse" "$enlisements" "$rejets" "$etendue" "$alerte"

    sleep "$INTERVALLE"
done
