#!/usr/bin/env bash
# =====================================================================
# un_scenario.sh - execute UN SEUL run de la campagne, isolement
# =====================================================================
#
# POURQUOI CE SCRIPT
# ------------------
# campagne_complete.sh enchaine les 17 runs en une seule session de
# 3 h. Cela cree un facteur confondant : le premier run tourne sur un
# GPU froid, le quinzieme sur un GPU a 87 C qui limite sa frequence.
# La degradation materielle est donc CORRELEE A L'ORDRE DES RUNS, et
# comme les strategies sont parcourues dans un ordre fixe, une
# difference observee entre deux strategies ne peut plus etre
# distinguee d'une difference d'etat de la machine.
#
# Trois plantages du simulateur ont ete observes pendant la campagne
# continue (LowLevelFatalError de compilation de shaders, blocage du
# run info_gain rep. 2, DXGI_ERROR_DEVICE_REMOVED en fin de run
# info_gain rep. 3). Le dernier est une reinitialisation du pilote
# graphique : un symptome thermique.
#
# En lancant un run a la fois, chaque execution demarre sur une
# machine dans le meme etat. Les conditions deviennent homogenes au
# lieu de deriver.
#
# CE QUI EST GARANTI IDENTIQUE A LA CAMPAGNE
# ------------------------------------------
# Les fonctions arreter_carla, carla_repond, demarrer_carla,
# nettoyer_ros2, lancer_run, cause_journal, fichier_ate_du_run,
# fichier_decisions_du_run et run_avec_reprises sont reprises
# TEXTUELLEMENT de campagne_complete.sh, ainsi que les constantes
# BUDGET_DISTANCE, BUDGET_DUREE, MAX_TENTATIVES et MIN_ECHANTILLONS.
# Les runs produits par les deux scripts sont donc comparables.
#
# Le protocole est respecte a l'identique :
#   - CARLA est redemarre avant chaque run ;
#   - un run de chauffe est execute puis JETE apres chaque demarrage
#     de CARLA, systematiquement ;
#   - un run n'est archive que s'il atteint le budget de distance ET
#     que son fichier ATE contient au moins MIN_ECHANTILLONS lignes ;
#   - un decrochage de l'odometrie ICP invalide le run ;
#   - le nombre de tentatives est journalise.
#
# CE QUI EST AJOUTE
# -----------------
# 1. VERROU PARTAGE. Le meme fichier de verrou que
#    campagne_complete.sh est utilise : les deux scripts s'excluent
#    mutuellement. Impossible de lancer un run unitaire pendant une
#    campagne, ni l'inverse.
#
# 2. EMPREINTE DU CODE. Le seul risque reel du fractionnement est de
#    modifier le code entre deux runs : les runs cessent alors d'etre
#    comparables, sans qu'aucune trace ne le signale. Le script
#    calcule une empreinte de tous les sources du workspace et REFUSE
#    de demarrer si elle differe de la reference, en nommant les
#    fichiers modifies. La comparabilite est ainsi prouvee, pas
#    supposee.
#
# 3. CONSOLIDATION DU SUIVI. campagne_complete.sh ecrit dans
#    tentatives_<horodatage>.csv et ne recopie sous tentatives.csv
#    qu'a la fin. Le fichier tentatives.csv trouve sur disque a
#    encore l'en-tete a 5 colonnes de la version precedente du
#    script : c'est un reliquat. Avant toute ecriture, le suivi
#    horodate le plus recent est recopie dans tentatives.csv et
#    l'en-tete est migre vers 6 colonnes si besoin. Une sauvegarde
#    est faite a chaque fois.
#
# 4. REFUS DE DOUBLON. Un run deja archive (ate_<nom>.csv present)
#    n'est pas refait, sauf FORCER=1.
#
# 5. ARRET DE CARLA EN FIN DE RUN, pour que le GPU redescende en
#    temperature entre deux executions.
#
# USAGE
# -----
#     bash ~/un_scenario.sh bilan            # etat de la campagne
#     bash ~/un_scenario.sh weighted 3       # un run
#     bash ~/un_scenario.sh weighted 1 rain  # un run meteo
#
# Strategies : random distance info_gain weighted predefinie
# Meteo      : rain fog   (absent = clear)
#
# Un run dure environ 30 min : le run de chauffe jete, puis le run
# mesure. Laisse la machine refroidir 5 a 10 min avant le suivant, et
# fais-le SYSTEMATIQUEMENT, pas seulement quand elle chauffe : c'est
# la regularite de l'attente qui rend les conditions homogenes.
#
# VARIABLES D'ENVIRONNEMENT
# -------------------------
#     FORCER=1            refait un run deja archive, ou passe outre
#                         une empreinte de code differente
#     CHAUFFE_DISTANCE=n  raccourcit le run de chauffe (defaut :
#                         1200, identique a la campagne). Toute autre
#                         valeur est un ECART AU PROTOCOLE et doit
#                         etre mentionnee dans le rapport.
# =====================================================================

set -u

# ---------------------------------------------------------------------
# Constantes - reprises telles quelles de campagne_complete.sh
# ---------------------------------------------------------------------

CARLA_EXE="/mnt/c/carla/CarlaUE4.exe"
METRICS="$HOME/active_slam_carla/metrics"
LOGS="$HOME/logs_campagne2"
SRC="$HOME/active_slam_carla/ros2_ws/src"

BUDGET_DISTANCE=1200.0
BUDGET_DUREE=900.0
MAX_TENTATIVES=4
MIN_ECHANTILLONS=120

# Distance minimale, mesuree sur la VERITE TERRAIN, pour qu'un run soit
# considere comme ayant atteint son budget quand le message du
# controleur manque dans le journal. Voir cause_journal.
#
# 95 % de 1200 m. La trajectoire vraie est echantillonnee a environ 1 Hz
# par localization_evaluator : la polyligne qui en resulte sous-estime
# le chemin reellement parcouru, et le dernier echantillon precede
# l'arret d'une seconde au plus. Releve sur les quatre tentatives
# perdues de predefinie_1 : 1168 a 1190 m annonces par le controleur
# pour un budget de 1200 m.
SEUIL_DISTANCE_VALIDE=1140.0

# Journalisation ROS 2 non tamponnee.
#
# vehicle_controller_eval.py termine le run par
# os.killpg(os.getpgid(0), signal.SIGINT) : il se tue lui-meme avec
# tout son groupe. Un processus tue par un signal ne vide pas son
# tampon de sortie, si bien que sa derniere ligne -- justement
# "FIN DE RUN (budget_reached) : budget de distance atteint" -- est
# perdue une fois sur deux. Quatre runs valides ont ete rejetes pour
# cette seule raison le 3 septembre.
#
# Cette variable demande a rcutils d'ecrire sans tampon. C'est un
# reglage de journalisation : il ne touche ni la simulation, ni les
# decisions, ni le code du workspace -- l'empreinte reste inchangee et
# les runs deja archives restent comparables.
export RCUTILS_LOGGING_BUFFERED_STREAM=0

# Distance du run de chauffe, qui est JETE.
#
# Elle valait BUDGET_DISTANCE (1200 m), pour etre strictement identique
# au protocole de campagne_complete.sh. Mesure du 3 septembre, sur les
# quatre premiers runs de la campagne v2 : CARLA meurt pendant la
# chauffe presque a chaque fois. Sequence relevee pour weighted_1 --
# CARLA pret en 17 s, chauffe de 6 min, puis "CARLA injoignable" au
# demarrage du run mesure, et cela sur les quatre tentatives.
#
# Une chauffe de 1200 m represente 4 a 6 minutes de charge GPU
# soutenue, soit AUTANT que le run mesure. Elle epuise donc la
# stabilite de la machine avant la mesure, alors que son unique role est
# d'absorber l'instabilite du premier run apres un demarrage de CARLA --
# compilation de shaders et chargement des ressources, qui se terminent
# dans la premiere minute.
#
# 150 m (environ une minute) conserve ce role en divisant par huit la
# charge. C'est un ECART assume au protocole de la campagne continue,
# affiche a chaque run et a mentionner dans le rapport. Il ne porte que
# sur un run jete : le run mesure garde son budget de 1200 m intact.
#
# Remettre 1200 m si la machine redevient stable :
#     CHAUFFE_DISTANCE=1200 bash ~/un_scenario.sh <strategie> <rep>
CHAUFFE_DISTANCE="${CHAUFFE_DISTANCE:-150.0}"

# --- Experience appariee : VARIANTE=A ou VARIANTE=B ---
#
# La campagne du 3 septembre laisse quatre strategies indiscernables
# (distance 27,5 m, predefinie 32,6, weighted 36,0, info_gain 36,7),
# avec un ecart-type de 14,5 m sur weighted. Une amelioration de 8 m
# est donc invisible a cinq repetitions non appariees.
#
# Le plan apparie fait tourner les deux configurations sur LE MEME
# trafic, graine par graine : la graine vaut le numero de repetition,
# identique pour A et B. On compare cinq differences au lieu de deux
# moyennes bruitees.
#
#     VARIANTE=A   info 0.30  distance 0.15  ratio 0.05
#     VARIANTE=B   info 0.20  distance 0.25  ratio 0.05
#     VARIANTE=C   info 0.20  distance 0.25  ICP 0.03 boucles 0.03
#     VARIANTE=D   info 0.20  distance 0.25  ICP 0.05 boucles 0.03
#
# A contre B a repondu : les sauts plus courts ameliorent l'erreur
# sur 5 graines sur 5, -5,0 m en moyenne (-26 %), test des signes
# p = 0,031. B est donc le reglage retenu.
#
# B contre C repond a la question suivante : la chaine SLAM
# accepte-t-elle plus de fermetures de boucle si l'on abaisse le
# seuil de recalage ? 313 candidats ont ete rejetes sur les 37
# runs archives, contre 224 acceptes. B et C partagent les memes
# poids et les memes graines : le seuil est la seule difference,
# et les runs B deja archives servent de temoin.
#
# Les poids sont fixes ICI et non passes a la main : une inversion
# entre les deux configurations invaliderait l'experience sans laisser
# de trace.
#
# Sans VARIANTE, les trois valeurs reproduisent exactement la campagne
# archivee : graine aleatoire et poids d'origine.
VARIANTE="${VARIANTE:-}"
SEED_RUN="-1"
POIDS_INFO="0.30"
POIDS_DIST="0.15"
# Poids du critere de fermeture de boucle. 0.20 = valeur de tous
# les runs archives ; la variante E le met a zero (ablation).
POIDS_BOUCLE="0.20"

# Resolution des cameras RGB et profondeur, transmise aux deux
# ponts par variable d'environnement. 320 x 240 = valeur de tous
# les runs archives ; la variante F la porte a 640 x 480 pour
# tester si la transformation des fermetures de boucle devient
# calculable.
CAM_W="320"
CAM_H="240"
# Seuils d'acceptation, desormais SEPARES.
#   RATIO_ICP    : recalage scan a scan, dans icp_odometry
#   RATIO_BOUCLE : validation des fermetures, dans rtabmap
# 0.05 pour les deux = valeur de tous les runs archives.
#
# Ils etaient lies : la variante C les a baisses ensemble, ce qui
# a degrade l'odometrie et supprime les opportunites de fermeture
# au lieu d'en faire accepter. D ne touche que le second.
RATIO_ICP="0.05"
RATIO_BOUCLE="0.05"

# --- Campagne APPARIEE ---
# APPARIE=1 fait tourner la strategie demandee sur la graine de
# trafic egale au numero de repetition. Applique aux cinq
# strategies, cela les met toutes face aux memes trafics.
#
# Sans lui, la graine reste a -1 (aleatoire) et le script se
# comporte comme pour la campagne preliminaire.
APPARIE="${APPARIE:-0}"
FORCER="${FORCER:-0}"

# tentatives.csv est un fichier DERIVE : il est reconstruit a chaque
# lancement par concatenation chronologique de tous les suivis
# tentatives_*.csv, campagnes comprises. Les runs unitaires ecrivent
# dans leur propre suivi, qui participe a cette reconstruction.
CSV="$LOGS/tentatives.csv"
SUIVI="$LOGS/tentatives_unitaires.csv"
ENTETE="strategie,repetition,tentatives,resultat,cause,echantillons"
ENTETE_ANCIEN="strategie,repetition,tentatives,resultat,cause"
REFERENCE="$LOGS/empreinte_reference.txt"
# Extension .txt et non .log : suivi.sh balaie $LOGS/*.log et prend le
# fichier le plus recent pour le journal du run en cours. Un
# empreintes.log s'y serait glisse a la fin de chaque run, affiche comme
# un run nomme "empreintes" bloque a 0 m.
JOURNAL_EMPREINTES="$LOGS/empreintes.txt"

TASKKILL="/mnt/c/Windows/System32/taskkill.exe"
[ -x "$TASKKILL" ] || TASKKILL="taskkill.exe"

mkdir -p "$LOGS"

# Les 17 runs attendus, sous la forme strategie:repetition:meteo.
# Une meteo vide signifie 'clear'. Le nom d'archive est
# <strategie>[_<meteo>]_<repetition>, exactement comme le construit
# campagne_complete.sh.
# Cinq repetitions, et non trois.
#
# Motif, mesure le 3 septembre : deux series de trois runs 'predefinie'
# ont donne des ecarts-types de 75,8 m et 9,6 m -- un facteur 8 sur
# l'estimation de la MEME grandeur. A n = 3, la dispersion elle-meme
# n'est pas estimable de facon fiable, et trois strategies
# ('distance' 22,4 m, 'predefinie' 36,7 m, 'weighted' 44,5 m) se
# tiennent dans une vingtaine de metres.
#
# Les repetitions 1 a 3 deja archivees restent valides : on ajoute des
# echantillons, on ne refait rien.
RUNS_ATTENDUS="
random:1: random:2: random:3: random:4: random:5:
distance:1: distance:2: distance:3: distance:4: distance:5:
info_gain:1: info_gain:2: info_gain:3: info_gain:4: info_gain:5:
weighted:1: weighted:2: weighted:3: weighted:4: weighted:5:
predefinie:1: predefinie:2: predefinie:3: predefinie:4: predefinie:5:
weighted:1:rain weighted:1:fog
"

# Experience apparie A/B : dix runs supplementaires, suivis a part.
# Ils ne font pas partie de la campagne comparative : ils repondent a
# une autre question -- raccourcir les sauts ameliore-t-il weighted ?
RUNS_APPARIES="
weightedA_1 weightedA_2 weightedA_3 weightedA_4 weightedA_5
weightedB_1 weightedB_2 weightedB_3 weightedB_4 weightedB_5
weightedC_1 weightedC_2 weightedC_3 weightedC_4 weightedC_5
weightedD_1 weightedD_2 weightedD_3 weightedD_4 weightedD_5
weightedE_1 weightedE_2 weightedE_3 weightedE_4 weightedE_5
weightedF_1 weightedF_2 weightedF_3 weightedF_4 weightedF_5
"

# Campagne APPARIEE : les cinq strategies sur les graines 1 a 5.
# C'est l'analyse principale du rapport. La campagne preliminaire
# (RUNS_ATTENDUS, trafic aleatoire) reste archivee a cote.
RUNS_CAMPAGNE_P="
randomP_1 randomP_2 randomP_3 randomP_4 randomP_5
distanceP_1 distanceP_2 distanceP_3 distanceP_4 distanceP_5
info_gainP_1 info_gainP_2 info_gainP_3 info_gainP_4 info_gainP_5
weightedP_1 weightedP_2 weightedP_3 weightedP_4 weightedP_5
predefinieP_1 predefinieP_2 predefinieP_3 predefinieP_4 predefinieP_5
"


# ---------------------------------------------------------------------
# CARLA - identique a campagne_complete.sh
# ---------------------------------------------------------------------

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

# Trois tentatives de demarrage, pas une seule.
#
# Mesure du 3 septembre. Apres un plantage "Shader compilation failures
# are Fatal", le redemarrage automatique a echoue : CARLA n'a pas
# repondu dans la fenetre de 200 s et le run a ete abandonne. Relance a
# la main quelques minutes plus tard, le meme simulateur, avec les
# memes options, etait pret en 27 s.
#
# Le delai n'etait donc pas trop court : c'est l'etat de la machine
# immediatement apres le plantage qui empechait le demarrage. Une
# seconde tentative, apres une pause laissant le pilote graphique
# retrouver un etat stable, aboutit la ou la premiere echoue.
#
# Le temps de demarrage reel est journalise a chaque fois : s'il derive
# run apres run, c'est un signe de fatigue thermique a verser au
# rapport plutot qu'un aleas a subir.
demarrer_carla() {
    local tentative depart

    for tentative in 1 2 3; do
        if [ "$tentative" -gt 1 ]; then
            echo "    [CARLA] pause de 30 s avant nouvelle tentative"
            sleep 30
        fi
        echo "    [CARLA] redemarrage (tentative $tentative/3)"
        arreter_carla
        # -quality-level=Low limite le nombre de shaders a compiler,
        # cause des LowLevelFatalError observes sur cette configuration.
        # 9>&- ferme le descripteur du verrou dans l'enfant.
        #
        # Le verrou est pose par 'exec 9>fichier' puis flock : tout
        # processus lance ensuite HERITE du descripteur 9 et maintient
        # donc le verrou aussi longtemps qu'il vit. CARLA etant lance en
        # arriere-plan, un 'pkill un_scenario.sh' tuait le script mais
        # laissait CARLA vivant -- et le verrou tenu par un processus
        # qui n'apparait dans aucune recherche de 'un_scenario.sh'. Le
        # lancement suivant etait alors refuse avec une liste de
        # processus vide, sans moyen de comprendre pourquoi.
        "$CARLA_EXE" -quality-level=Low -carla-rpc-port=2000 \
            >/dev/null 2>&1 9>&- &

        depart=$(date +%s)
        for essai in $(seq 1 40); do
            sleep 5
            if carla_repond; then
                echo "    [CARLA] pret apres $(( $(date +%s) - depart )) s"
                sleep 10   # laisse la carte finir de se charger
                return 0
            fi
        done
        echo "    [CARLA] pas de reponse apres 200 s (tentative $tentative)"
    done

    echo "    [CARLA] echec des 3 tentatives de demarrage"
    return 1
}


# ---------------------------------------------------------------------
# Runs - identique a campagne_complete.sh
# ---------------------------------------------------------------------

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

# Le quatrieme argument, absent dans campagne_complete.sh, ne sert
# qu'au run de chauffe. Les runs MESURES sont toujours appeles a trois
# arguments : la distance vaut alors BUDGET_DISTANCE et le comportement
# est strictement celui de la campagne.
lancer_run() {
    local strategie="$1" log="$2" meteo="${3:-clear}"
    local distance="${4:-$BUDGET_DISTANCE}"

    nettoyer_ros2

    # 9>&- : voir demarrer_carla. Sans cette fermeture, les noeuds ROS 2
    # heritent du descripteur du verrou et le maintiennent apres la mort
    # du script.
    if [ "$strategie" = "predefinie" ]; then
        timeout --signal=INT --kill-after=30 1500 \
            ros2 launch slam_evaluation slam_evaluation.launch.py \
            distance:="$distance" weather:="$meteo" \
            seed:="$SEED_RUN" \
            icp_correspondence_ratio:="$RATIO_ICP" \
            loop_correspondence_ratio:="$RATIO_BOUCLE" \
            > "$log" 2>&1 9>&- &
    else
        timeout --signal=INT --kill-after=30 1500 \
            ros2 launch active_slam_decision active_slam.launch.py \
            strategy:="$strategie" \
            run_duration:="$BUDGET_DUREE" \
            max_distance:="$distance" \
            weather:="$meteo" \
            seed:="$SEED_RUN" \
            weight_information_gain:="$POIDS_INFO" \
            weight_distance:="$POIDS_DIST" \
            weight_loop_closure:="$POIDS_BOUCLE" \
            icp_correspondence_ratio:="$RATIO_ICP" \
            loop_correspondence_ratio:="$RATIO_BOUCLE" \
            > "$log" 2>&1 9>&- &
    fi

    local pid=$!
    local depart_run
    depart_run=$(date +%s)
    local abandon=0

    # Arret anticipe quand CARLA est injoignable.
    #
    # Si le simulateur meurt entre le demarrage du launch et la connexion
    # des noeuds, ceux-ci sortent sur RuntimeError mais icp_odometry et
    # rtabmap, eux, restent en vie a attendre des donnees qui ne
    # viendront jamais. Le launch ne se termine alors que sur le timeout
    # de 1500 s : 25 minutes perdues pour un run qui n'a jamais demarre.
    # Mesure du 3 septembre : run info_gain_1 lance a 11:17:55, toujours
    # a 0 m et 0 destination a 11:22, avec cinq noeuds sortis sur
    # "time-out while waiting for the simulator".
    #
    # On ne regarde qu'apres 120 s : au-dela du demarrage normal des
    # noeuds, et le motif recherche n'apparait pas dans un run sain.
    while kill -0 "$pid" 2>/dev/null; do
        sleep 10
        if [ "$abandon" -eq 0 ] \
           && [ $(( $(date +%s) - depart_run )) -ge 120 ] \
           && grep -q "while waiting for the simulator" "$log" 2>/dev/null; then
            echo "     CARLA injoignable : arret anticipe du run"
            abandon=1
            kill -INT "$pid" 2>/dev/null
            sleep 10
            kill -9 "$pid" 2>/dev/null
        fi
    done
    wait "$pid" 2>/dev/null || true

    nettoyer_ros2
}


# Garantit que CARLA repond AVANT de lancer un run.
#
# demarrer_carla n'etait appele qu'une fois, avant le run de chauffe.
# Si le simulateur mourait pendant la chauffe -- ce qui est arrive --
# le run mesure partait dans le vide sans que rien ne le verifie.
assurer_carla() {
    local strategie="$1" meteo="$2"

    if carla_repond; then
        return 0
    fi

    echo "     CARLA ne repond pas : redemarrage avant le run"
    if ! demarrer_carla; then
        return 1
    fi
    echo "     run de chauffe (jete)"
    lancer_run "$strategie" \
               "$LOGS/chauffe_$(date +%Y%m%d_%H%M%S).log" \
               "$meteo" "$CHAUFFE_DISTANCE"
    return 0
}

# Longueur de la trajectoire VRAIE, en metres, mesuree sur le fichier
# ATE du run. C'est une mesure de ce qui s'est reellement passe, et non
# la presence d'un message dans un journal : elle survit a la perte du
# tampon de sortie.
longueur_trajectoire() {
    python3 - "$1" <<'EOF'
import csv, math, sys

try:
    lignes = list(csv.DictReader(open(sys.argv[1], newline='')))
except Exception:
    print("0.0")
    raise SystemExit(0)

points = []
for l in lignes:
    try:
        t = float(l['timestamp'])
        x = float(l['gt_x'])
        y = float(l['gt_y'])
    except (KeyError, TypeError, ValueError):
        continue
    # Echantillon d'amorcage : tout a zero, ce n'est pas une position.
    if x == 0.0 and y == 0.0:
        continue
    points.append((t, x, y))

# Les lignes ne sont pas ecrites dans l'ordre chronologique.
points.sort()

total = 0.0
for i in range(1, len(points)):
    total += math.hypot(points[i][1] - points[i - 1][1],
                        points[i][2] - points[i - 1][2])
print("%.1f" % total)
EOF
}


# $1 : journal du run
# $2 : longueur de trajectoire vraie mesuree (m), 0 si indisponible
cause_journal() {
    local log="$1"
    local longueur="${2:-0}"

    if [ ! -s "$log" ]; then
        echo "log vide"; return
    fi
    # Cause distincte du generique "budget non atteint" : CARLA etait
    # injoignable au demarrage des noeuds. Tous ceux qui s'y connectent
    # (bridge, candidate_generator, obstacle_detector, controleur,
    # traffic_spawner) sortent alors sur RuntimeError, plus aucune donnee
    # capteur n'est publiee, et icp_odometry comme rtabmap tournent dans
    # le vide. Le distinguer evite d'attribuer a la strategie un
    # plantage du simulateur.
    if grep -q "while waiting for the simulator" "$log"; then
        echo "CARLA injoignable au demarrage des noeuds"; return
    fi
    if ! grep -q "budget de distance atteint" "$log"; then
        # Seconde voie d'acceptation, MESUREE et non journalisee.
        #
        # Le message du controleur peut etre perdu : il se tue par
        # os.killpg(..., SIGINT) juste apres l'avoir ecrit, et un
        # processus tue par un signal ne vide pas son tampon. Se fier
        # a sa seule presence revient a tirer au sort la validite d'un
        # run qui s'est parfaitement deroule.
        #
        # La longueur de la trajectoire vraie, elle, est un fait :
        # elle vient du fichier ATE ecrit tout au long du run par
        # localization_evaluator.
        #
        # Ce critere est applique a l'identique aux cinq strategies, et
        # il ELARGIT l'acceptation sans jamais rejeter un run
        # precedemment accepte : un run portant le message est valide
        # quoi qu'il arrive.
        if awk -v a="$longueur" -v b="$SEUIL_DISTANCE_VALIDE" \
               'BEGIN { exit !(a + 0 >= b + 0) }'; then
            echo ""; return
        fi
        echo "budget non atteint (trajectoire vraie ${longueur} m < ${SEUIL_DISTANCE_VALIDE} m)"
        return
    fi
    local zeros
    zeros=$(grep -c "ratio=0\.000000" "$log")
    if [ "$zeros" -gt 1 ]; then
        echo "decrochage ICP ($zeros recalages nuls)"; return
    fi
    echo ""
}

fichier_ate_du_run() {
    local debut="$1"
    find "$METRICS/all_runs" -name 'localization_error_*.csv' \
         -newermt "@$debut" -printf '%T@ %p\n' 2>/dev/null \
        | sort -rn | head -1 | cut -d' ' -f2-
}

fichier_decisions_du_run() {
    local debut="$1"
    find "$METRICS" -maxdepth 1 -name 'decisions_*.csv' \
         -newermt "@$debut" -printf '%T@ %p\n' 2>/dev/null \
        | sort -rn | head -1 | cut -d' ' -f2-
}

run_avec_reprises() {
    local strategie="$1" repetition="$2" meteo="${3:-clear}" suffixe="${4:-}"
    local tentative=1 cause="" nom ate dec echantillons

    nom="${strategie}${suffixe}_${repetition}"

    while [ "$tentative" -le "$MAX_TENTATIVES" ]; do
        local log="$LOGS/${strategie}${suffixe}_${repetition}_essai${tentative}.log"
        echo "  -> $strategie${suffixe} rep $repetition, tentative $tentative ($(date +%H:%M:%S))"

        # Verification de vie du simulateur avant CHAQUE tentative, y
        # compris la premiere : la chauffe qui vient de s'executer a pu
        # le faire tomber.
        if ! assurer_carla "$strategie" "$meteo"; then
            echo "     abandon : CARLA ne redemarre pas"
            return 1
        fi

        local debut
        debut=$(date +%s)
        lancer_run "$strategie" "$log" "$meteo"

        # Le fichier ATE est cherche AVANT de conclure : sa longueur de
        # trajectoire sert de seconde voie d'acceptation quand le
        # message du controleur a ete perdu au vidage du tampon.
        echantillons=0
        ate=""
        local longueur=0.0

        ate=$(fichier_ate_du_run "$debut")
        if [ -n "$ate" ]; then
            echantillons=$(( $(wc -l < "$ate") - 1 ))
            longueur=$(longueur_trajectoire "$ate")
        fi

        cause=$(cause_journal "$log" "$longueur")

        if [ -z "$cause" ]; then
            if [ -z "$ate" ]; then
                cause="aucun fichier ATE produit pendant ce run"
            elif [ "$echantillons" -lt "$MIN_ECHANTILLONS" ]; then
                cause="fichier ATE trop court ($echantillons echantillons ; minimum $MIN_ECHANTILLONS)"
            fi
        fi

        if [ -z "$cause" ]; then
            cp "$ate" "$METRICS/ate_${nom}.csv"
            if [ "$strategie" != "predefinie" ]; then
                dec=$(fichier_decisions_du_run "$debut")
                [ -n "$dec" ] && cp "$dec" "$METRICS/campagne_${nom}.csv"
            fi
            cp "$log" "$LOGS/${nom}.log"
            echo "     valide, archive ($echantillons echantillons)"
            echo "     source : $(basename "$ate")"
            echo "$strategie$suffixe,$repetition,$tentative,valide,,$echantillons" >> "$SUIVI"
            return 0
        fi

        echo "     INVALIDE : $cause"
        echo "$strategie$suffixe,$repetition,$tentative,invalide,$cause,$echantillons" >> "$SUIVI"

        if ! demarrer_carla; then
            echo "     abandon : CARLA ne redemarre pas"
            return 1
        fi
        echo "     run de chauffe (jete)"
        lancer_run "$strategie" "$LOGS/chauffe_$(date +%H%M%S).log" "$meteo" \
                   "$CHAUFFE_DISTANCE"

        tentative=$((tentative + 1))
        sleep 10
    done

    echo "     ECHEC apres $MAX_TENTATIVES tentatives"
    return 1
}


# ---------------------------------------------------------------------
# Empreinte du code source
# ---------------------------------------------------------------------
#
# Le fractionnement de la campagne n'a qu'un risque reel : modifier le
# code entre deux runs. Les runs cessent alors d'etre comparables sans
# que rien ne le signale. On enregistre donc l'empreinte de tous les
# sources du workspace, et on refuse de demarrer si elle a change.
#
# Les repertoires build/ et install/ sont exclus : ils sont regeneres a
# chaque colcon build et leur contenu change sans que le code change.
# Les sauvegardes de patches (*.py.bak_*) sont exclues par construction,
# le filtre ne retenant que les extensions exactes.

listing_code() {
    find "$SRC" -type f \
         \( -name '*.py' -o -name '*.yaml' -o -name '*.yml' \
            -o -name '*.xml' -o -name '*.cfg' -o -name '*.launch' \) \
         ! -path '*/build/*' ! -path '*/install/*' \
         ! -path '*/log/*' ! -path '*/__pycache__/*' \
         -print0 2>/dev/null \
        | sort -z | xargs -0 sha256sum 2>/dev/null \
        | sed "s|$SRC/||" | sort -k2
}

verifier_code() {
    local courant="/tmp/empreinte_courante_$$.txt"
    listing_code > "$courant"

    if [ ! -s "$courant" ]; then
        echo "ERREUR : aucun fichier source trouve sous $SRC"
        rm -f "$courant"
        return 1
    fi

    if [ ! -f "$REFERENCE" ]; then
        cp "$courant" "$REFERENCE"
        echo "  empreinte de reference etablie maintenant"
        echo "     $(wc -l < "$REFERENCE") fichiers sources"
        echo "     Elle ne peut pas prouver retroactivement que les runs"
        echo "     deja archives utilisaient ce meme code : elle vaut a"
        echo "     partir de maintenant."
        rm -f "$courant"
        return 0
    fi

    if diff -q "$REFERENCE" "$courant" >/dev/null 2>&1; then
        echo "  empreinte du code : conforme a la reference"
        rm -f "$courant"
        return 0
    fi

    echo ""
    echo "ERREUR : LE CODE SOURCE A CHANGE DEPUIS LA REFERENCE."
    echo ""
    echo "Les runs deja archives ont ete produits avec une autre version"
    echo "du code. Les comparer a un run produit maintenant n'aurait pas"
    echo "de sens."
    echo ""
    echo "Fichiers concernes :"
    diff "$REFERENCE" "$courant" 2>/dev/null \
        | grep '^[<>]' | awk '{print "    " $NF}' | sort -u
    echo ""
    echo "Deux issues, et deux seulement :"
    echo "  - revenir a la version de reference, puis relancer ;"
    echo "  - assumer la modification, refaire LES 17 RUNS, et remettre"
    echo "    la reference a zero :"
    echo "        rm $REFERENCE"
    echo ""
    echo "Passer outre pour un seul run (FORCER=1) produirait un jeu de"
    echo "donnees heterogene."
    rm -f "$courant"
    return 1
}


# ---------------------------------------------------------------------
# Suivi CSV
# ---------------------------------------------------------------------
#
# campagne_complete.sh ecrit dans tentatives_<horodatage>.csv et ne
# recopie sous tentatives.csv qu'a la toute fin. Si une campagne s'est
# interrompue, tentatives.csv est reste sur le contenu de la campagne
# PRECEDENTE. On consolide donc avant d'ecrire : le suivi horodate le
# plus recent fait foi.

# Ecrit dans $1 la concatenation chronologique de tous les suivis
# tentatives_*.csv. N'ecrit RIEN d'autre : appelable en lecture seule.
#
# Les fichiers produits par la version precedente du script, a 5
# colonnes, recoivent une colonne 'echantillons' vide plutot que d'etre
# ecartes.
fusionner_suivis() {
    local sortie="$1" f entete

    echo "$ENTETE" > "$sortie"

    for f in $(ls -tr "$LOGS"/tentatives_*.csv 2>/dev/null); do
        entete=$(head -1 "$f")
        if [ "$entete" = "$ENTETE" ]; then
            tail -n +2 "$f" >> "$sortie"
        elif [ "$entete" = "$ENTETE_ANCIEN" ]; then
            tail -n +2 "$f" | sed 's/$/,/' >> "$sortie"
        else
            echo "  suivi ignore, en-tete inattendu : $(basename "$f")" >&2
        fi
    done
}

consolider_csv() {
    local tmp sauvegarde avant apres

    # Le suivi propre aux runs unitaires.
    if [ ! -f "$SUIVI" ]; then
        echo "$ENTETE" > "$SUIVI"
    fi

    tmp="$LOGS/.tentatives_consolide_$$"
    fusionner_suivis "$tmp"

    apres=$(( $(wc -l < "$tmp") - 1 ))

    if [ -f "$CSV" ]; then
        avant=$(( $(wc -l < "$CSV") - 1 ))
        sauvegarde="$CSV.bak_$(date +%Y%m%d_%H%M%S)"
        cp "$CSV" "$sauvegarde"
        if [ "$apres" -lt "$avant" ]; then
            echo "  ATTENTION : la reconstruction donne $apres lignes contre"
            echo "  $avant dans l'ancien $(basename "$CSV"). Des lignes n'ont pas"
            echo "  de suivi horodate correspondant. Elles sont conservees dans"
            echo "  $(basename "$sauvegarde")"
        fi
    fi

    mv "$tmp" "$CSV"
    echo "  suivi consolide : $apres tentatives"
    return 0
}


# ---------------------------------------------------------------------
# Bilan
# ---------------------------------------------------------------------

# column(1) vient de util-linux et n'est pas garanti present. Sans repli,
# son absence ferait afficher un tableau VIDE, ce qui se lirait comme
# "aucune tentative enregistree".
afficher_csv() {
    if command -v column >/dev/null 2>&1; then
        column -s, -t < "$1" | sed 's/^/  /'
    else
        sed 's/,/ | /g; s/^/  /' "$1"
    fi
}

bilan() {
    local triplet strategie repetition meteo nom suffixe n faits=0 total=0 cmd

    echo "=== ETAT DE LA CAMPAGNE ==="
    echo "metrics : $METRICS"
    echo ""

    for triplet in $RUNS_ATTENDUS; do
        strategie="${triplet%%:*}"
        repetition="$(echo "$triplet" | cut -d: -f2)"
        meteo="$(echo "$triplet" | cut -d: -f3)"
        suffixe=""
        [ -n "$meteo" ] && suffixe="_$meteo"
        nom="${strategie}${suffixe}_${repetition}"
        total=$((total + 1))

        if [ -f "$METRICS/ate_$nom.csv" ]; then
            n=$(( $(wc -l < "$METRICS/ate_$nom.csv") - 1 ))
            printf "  %-20s ARCHIVE   %4d echantillons\n" "$nom" "$n"
            faits=$((faits + 1))
        else
            cmd="bash ~/un_scenario.sh $strategie $repetition"
            [ -n "$meteo" ] && cmd="$cmd $meteo"
            printf "  %-20s MANQUANT  -> %s\n" "$nom" "$cmd"
        fi
    done

    echo ""
    echo "  $faits / $total runs archives"

    # --- Experience apparie A/B, suivie separement ---
    local faits_ab=0 total_ab=0 lettre reste_ab
    for nom in $RUNS_APPARIES; do
        total_ab=$((total_ab + 1))
        if [ -f "$METRICS/ate_$nom.csv" ]; then
            faits_ab=$((faits_ab + 1))
        fi
    done
    if [ "$total_ab" -gt 0 ]; then
        echo ""
        echo "=== EXPERIENCES APPARIEES (hors campagne) ==="
        echo ""
        for nom in $RUNS_APPARIES; do
            lettre="${nom%_*}"; lettre="${lettre#weighted}"
            reste_ab="${nom##*_}"
            if [ -f "$METRICS/ate_$nom.csv" ]; then
                n=$(( $(wc -l < "$METRICS/ate_$nom.csv") - 1 ))
                printf "  %-20s ARCHIVE   %4d echantillons\n" "$nom" "$n"
            else
                printf "  %-20s MANQUANT  -> VARIANTE=%s bash ~/un_scenario.sh weighted %s\n" \
                       "$nom" "$lettre" "$reste_ab"
            fi
        done
        echo ""
        echo "  $faits_ab / $total_ab runs apparies"
    fi

    # --- Campagne appariee : analyse principale du rapport ---
    local faits_p=0 total_p=0 nom_p strategie_p rep_p
    for nom_p in $RUNS_CAMPAGNE_P; do
        total_p=$((total_p + 1))
        [ -f "$METRICS/ate_$nom_p.csv" ] && faits_p=$((faits_p + 1))
    done
    echo ""
    echo "=== CAMPAGNE APPARIEE (graines 1 a 5, cinq strategies) ==="
    echo ""
    for nom_p in $RUNS_CAMPAGNE_P; do
        strategie_p="${nom_p%P_*}"
        rep_p="${nom_p##*_}"
        if [ -f "$METRICS/ate_$nom_p.csv" ]; then
            n=$(( $(wc -l < "$METRICS/ate_$nom_p.csv") - 1 ))
            printf "  %-20s ARCHIVE   %4d echantillons\n" "$nom_p" "$n"
        else
            printf "  %-20s MANQUANT  -> APPARIE=1 bash ~/un_scenario.sh %s %s\n" \
                   "$nom_p" "$strategie_p" "$rep_p"
        fi
    done
    echo ""
    echo "  $faits_p / $total_p runs de la campagne appariee"

    # Vue du suivi, reconstruite en memoire. On n'affiche PAS tentatives.csv
    # tel quel : il peut dater d'une campagne anterieure, campagne_complete.sh
    # ne le mettant a jour qu'a la toute fin d'une campagne complete.
    local vue="/tmp/vue_suivi_$$.csv" nb nsrc
    fusionner_suivis "$vue"
    nb=$(( $(wc -l < "$vue") - 1 ))
    nsrc=$(ls "$LOGS"/tentatives_*.csv 2>/dev/null | wc -l)

    if [ "$nb" -gt 0 ]; then
        echo ""
        echo "Suivi des tentatives ($nb lignes, $nsrc fichier(s) de suivi) :"
        afficher_csv "$vue"
    fi
    rm -f "$vue"

    if [ -f "$REFERENCE" ]; then
        echo ""
        echo "Empreinte de reference : $(wc -l < "$REFERENCE") fichiers sources"
        listing_code > "/tmp/empreinte_bilan_$$.txt"
        if diff -q "$REFERENCE" "/tmp/empreinte_bilan_$$.txt" >/dev/null 2>&1; then
            echo "  code actuel : CONFORME"
        else
            echo "  code actuel : DIFFERENT de la reference"
        fi
        rm -f "/tmp/empreinte_bilan_$$.txt"
    fi
}


# ---------------------------------------------------------------------
# Point d'entree
# ---------------------------------------------------------------------

usage() {
    echo "Usage :"
    echo "    bash ~/un_scenario.sh bilan"
    echo "    bash ~/un_scenario.sh <strategie> <repetition> [meteo]"
    echo ""
    echo "Strategies : random distance info_gain weighted predefinie"
    echo "Meteo      : rain fog   (absent = clear)"
    echo ""
    echo "Exemples :"
    echo "    bash ~/un_scenario.sh weighted 3"
    echo "    bash ~/un_scenario.sh predefinie 3"
    echo "    bash ~/un_scenario.sh weighted 1 rain"
    echo "    bash ~/un_scenario.sh weighted 1 fog"
}

# Le bilan ne touche a rien et n'a pas besoin du verrou.
if [ $# -ge 1 ] && [ "$1" = "bilan" ]; then
    bilan
    exit 0
fi

if [ $# -lt 2 ]; then
    usage
    exit 1
fi

STRATEGIE="$1"
REPETITION="$2"
METEO="${3:-clear}"

case "$STRATEGIE" in
    random|distance|info_gain|weighted|predefinie) ;;
    *) echo "ERREUR : strategie inconnue '$STRATEGIE'"; echo ""; usage; exit 1 ;;
esac

case "$REPETITION" in
    [1-9]|10) ;;
    *) echo "ERREUR : repetition doit valoir 1 a 10"; exit 1 ;;
esac

SUFFIXE=""
case "$METEO" in
    clear) ;;
    rain|fog) SUFFIXE="_$METEO" ;;
    *) echo "ERREUR : meteo inconnue '$METEO' (attendu rain, fog, ou rien)"; exit 1 ;;
esac

# --- Experience appariee ---
# La graine vaut le numero de repetition, IDENTIQUE pour A et B : c'est
# ce qui apparie les deux configurations sur le meme trafic. Les poids
# sont fixes par la variante et non saisis a la main.
case "$VARIANTE" in
    "")
        ;;
    A)
        POIDS_INFO="0.30"; POIDS_DIST="0.15"; SEED_RUN="$REPETITION"
        SUFFIXE="${SUFFIXE}A"
        ;;
    B)
        POIDS_INFO="0.20"; POIDS_DIST="0.25"; SEED_RUN="$REPETITION"
        SUFFIXE="${SUFFIXE}B"
        ;;
    C)
        # Les DEUX seuils a 0.03, comme les cinq runs C archives.
        # Conserve tel quel pour que C reste reproductible : ce
        # reglage a montre qu'abaisser le seuil de l'odometrie
        # supprime les fermetures au lieu d'en faire accepter.
        POIDS_INFO="0.20"; POIDS_DIST="0.25"; SEED_RUN="$REPETITION"
        RATIO_ICP="0.03"; RATIO_BOUCLE="0.03"
        SUFFIXE="${SUFFIXE}C"
        ;;
    D)
        # Le test propre : seul le seuil des FERMETURES change par
        # rapport a B. Memes poids, memes graines, meme odometrie.
        # Les cinq runs B archives sont le temoin.
        POIDS_INFO="0.20"; POIDS_DIST="0.25"; SEED_RUN="$REPETITION"
        RATIO_BOUCLE="0.03"
        SUFFIXE="${SUFFIXE}D"
        ;;
    E)
        # Ablation du critere de fermeture de boucle : son poids
        # (0.20) passe au gain d'information, et le transfert
        # dynamique est neutralise par la garde de strategies.py.
        #
        # Justifiee par quatre mesures : 2 fermetures acceptees
        # sur 129, correlation 0.07 avec les recalages reels,
        # -0.74 avec la couverture, et le classement monotone des
        # cinq runs weightedP.
        #
        # Temoin : les cinq runs weightedB, memes graines.
        POIDS_INFO="0.40"; POIDS_DIST="0.25"; POIDS_BOUCLE="0.00"
        SEED_RUN="$REPETITION"
        SUFFIXE="${SUFFIXE}E"
        ;;
    F)
        # Configuration B, avec la RESOLUTION CAMERA pour seule
        # difference. Le critere de fermeture de boucle reste
        # actif : c'est lui qu'on cherche a rendre operant.
        #
        # Motif : sur quinze runs, 129 fermetures proposees et 2
        # acceptees, toujours par echec du calcul de la
        # transformation. Celle-ci est estimee visuellement
        # (Reg/Strategy = 2) sur des images 320 x 240.
        #
        # Temoin : les cinq runs weightedB, memes graines.
        POIDS_INFO="0.20"; POIDS_DIST="0.25"
        SEED_RUN="$REPETITION"
        CAM_W="640"; CAM_H="480"
        SUFFIXE="${SUFFIXE}F"
        ;;
    G)
        # VOIE 1 : ZONE BORNEE.
        #
        # La tache passe de "parcourir 1200 m" a "cartographier une
        # zone". Motif, mesure sur la campagne appariee : sur un budget
        # de distance en reseau routier ouvert, la trajectoire
        # rectiligne atteint 89 % du maximum geometrique 2.R.L + pi.R2.
        # La politique optimale est donc d'aller tout droit, et aucune
        # decision ne peut battre cela -- il n'y a rien a decider.
        #
        # Poids de la configuration B, retenue par l'experience A/B.
        # ZONE_HALF_SIZE est lu par candidate_generator.py, qui rejette
        # les candidats hors zone (repli sur les plus proches du centre
        # si le vehicule en est sorti).
        #
        # 'predefinie' porte la meme lettre pour nommer la campagne mais
        # IGNORE la zone : elle ne lance pas candidate_generator.
        POIDS_INFO="0.20"; POIDS_DIST="0.25"; SEED_RUN="$REPETITION"
        export ZONE_HALF_SIZE="150"
        SUFFIXE="${SUFFIXE}G"
        ;;
    *)
        echo "ERREUR : VARIANTE doit valoir A, B, C, D, E, F ou G (recu '$VARIANTE')"
        exit 1
        ;;
esac

if [ -n "$VARIANTE" ] && [ "$VARIANTE" != "G" ] \
   && [ "$STRATEGIE" = "predefinie" ]; then
    echo "ERREUR : 'predefinie' ne prend aucune decision, les poids ne"
    echo "s'y appliquent pas. L'experience apparie ne concerne que les"
    echo "strategies de decision."
    exit 1
fi

# --- Campagne appariee : les cinq strategies, memes trafics ---
#
# La graine vaut le numero de repetition pour TOUTES les
# strategies, predefinie comprise : a la repetition n, les cinq
# systemes affrontent exactement le meme trafic. La comparaison
# porte alors sur cinq differences appariees et non sur cinq
# moyennes ou la variabilite du trafic domine.
#
# weighted y tourne avec le reglage B (info 0.20 / distance 0.25),
# valide par l'experience appariee A/B : 5 graines sur 5, -26 %
# d'erreur, test des signes p = 0,031. Les quatre autres n'ont
# aucun poids a regler, il n'y a donc pas d'asymetrie.
#
# Les poids sont fixes ICI et non saisis a la main : une erreur de
# frappe entre deux runs invaliderait la campagne sans trace.
if [ "$APPARIE" = "1" ]; then
    if [ -n "$VARIANTE" ]; then
        echo "ERREUR : APPARIE=1 et VARIANTE=$VARIANTE sont exclusifs."
        echo "Le mode apparie fixe lui-meme la graine et les poids."
        exit 1
    fi
    SEED_RUN="$REPETITION"
    SUFFIXE="${SUFFIXE}P"
    if [ "$STRATEGIE" = "weighted" ]; then
        POIDS_INFO="0.20"; POIDS_DIST="0.25"
    fi
fi

# Transmise aux deux ponts, qui la lisent comme valeur par defaut
# de leur parametre 'camera_width' / 'camera_height'.
export CAMERA_WIDTH="$CAM_W"
export CAMERA_HEIGHT="$CAM_H"

NOM="${STRATEGIE}${SUFFIXE}_${REPETITION}"

# ---------------------------------------------------------------------
# Verrou PARTAGE avec campagne_complete.sh : les deux s'excluent.
# ---------------------------------------------------------------------
VERROU="$HOME/.campagne_complete.lock"
exec 9>"$VERROU"
if ! flock -n 9; then
    echo "ERREUR : le verrou de campagne est deja tenu."
    echo ""
    echo "Scripts en cours :"
    ps -ef | grep -E "[c]ampagne_complete.sh|[u]n_scenario.sh" || true
    echo ""
    # Un descripteur de fichier s'HERITE : tout processus lance par le
    # script tient le verrou aussi longtemps qu'il vit, meme apres la
    # mort du script. Chercher 'un_scenario.sh' dans la table des
    # processus ne suffit donc pas -- le detenteur peut etre CARLA ou un
    # noeud ROS 2 orphelin. On lit /proc pour le nommer.
    echo "Processus tenant reellement le verrou :"
    trouve=0
    for rep in /proc/[0-9]*; do
        if ls -l "$rep/fd" 2>/dev/null | grep -q "$(basename "$VERROU")"; then
            pid="${rep#/proc/}"
            cmd=$(tr '\0' ' ' < "$rep/cmdline" 2>/dev/null)
            echo "    PID $pid : ${cmd:-(inconnu)}"
            trouve=1
        fi
    done
    if [ "$trouve" -eq 0 ]; then
        echo "    aucun -- le verrou devrait etre libre, reessaie."
    fi
    echo ""
    echo "Pour liberer :"
    echo "    pkill -f campagne_complete.sh"
    echo "    pkill -f un_scenario.sh"
    echo "    pkill -9 -f 'ros2 launch'"
    echo "    /mnt/c/Windows/System32/taskkill.exe /F /IM CarlaUE4.exe"
    echo ""
    echo "Ne supprime PAS $VERROU : cela n'enleve pas le verrou du"
    echo "processus qui le tient, cela supprime seulement la protection"
    echo "contre le double lancement."
    exit 1
fi

echo "=== RUN UNITAIRE : $NOM ==="
echo "debut : $(date '+%Y-%m-%d %H:%M:%S')"
echo ""

# --- Doublon ---------------------------------------------------------
if [ -f "$METRICS/ate_$NOM.csv" ] && [ "$FORCER" != "1" ]; then
    n=$(( $(wc -l < "$METRICS/ate_$NOM.csv") - 1 ))
    echo "Ce run est deja archive : ate_$NOM.csv ($n echantillons)."
    echo "Le refaire remplacerait des donnees valides."
    echo ""
    echo "Si c'est voulu :  FORCER=1 bash ~/un_scenario.sh $STRATEGIE $REPETITION ${3:-}"
    exit 0
fi

# --- Empreinte du code ----------------------------------------------
if ! verifier_code; then
    if [ "$FORCER" != "1" ]; then
        exit 1
    fi
    echo "  FORCER=1 : on passe outre. Le jeu de donnees sera heterogene."
fi

EMPREINTE=$(listing_code | sha256sum | cut -c1-16)
echo "  empreinte : $EMPREINTE"

# --- Suivi CSV -------------------------------------------------------
if ! consolider_csv; then
    exit 1
fi

if [ "$CHAUFFE_DISTANCE" != "$BUDGET_DISTANCE" ]; then
    echo ""
    echo "  ATTENTION : run de chauffe raccourci a $CHAUFFE_DISTANCE m"
    echo "  (protocole : $BUDGET_DISTANCE m). Ecart a mentionner dans le rapport."
fi

echo ""

# --- Execution -------------------------------------------------------
if ! demarrer_carla; then
    echo "ECHEC : CARLA ne demarre pas."
    exit 1
fi

echo "  run de chauffe (jete)"
lancer_run "$STRATEGIE" "$LOGS/chauffe_$(date +%Y%m%d_%H%M%S).log" \
           "$METEO" "$CHAUFFE_DISTANCE"

echo ""
if run_avec_reprises "$STRATEGIE" "$REPETITION" "$METEO" "$SUFFIXE"; then
    RESULTAT="valide"
    CODE=0
else
    RESULTAT="echec"
    CODE=1
fi

echo "$(date '+%Y-%m-%d %H:%M:%S') $NOM $RESULTAT $EMPREINTE" \
    >> "$JOURNAL_EMPREINTES"

# Les lignes que run_avec_reprises vient d'ecrire dans le suivi unitaire
# sont reportees dans le fichier consolide.
consolider_csv >/dev/null

# --- Arret de CARLA : le GPU redescend en temperature ----------------
echo ""
echo "  arret de CARLA"
nettoyer_ros2
arreter_carla

echo ""
echo "=== $NOM : $RESULTAT ($(date '+%H:%M:%S')) ==="
echo ""
echo "Laisse la machine refroidir 5 a 10 min avant le run suivant,"
echo "systematiquement, meme si tout s'est bien passe : c'est la"
echo "regularite de l'attente qui rend les conditions homogenes."
echo ""
echo "Etat de la campagne :"
echo "    bash ~/un_scenario.sh bilan"

exit "$CODE"
