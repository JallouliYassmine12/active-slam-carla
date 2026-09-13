#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Expose en arguments de lancement la graine du trafic et les deux poids
de decision a comparer, SANS changer le comportement par defaut.

POURQUOI
--------
La campagne du 3 septembre (27 runs, cinq repetitions par strategie)
laisse quatre strategies indiscernables :

    distance    27,5 m   (10,7 - 38,8)
    predefinie  32,6 m   (19,7 - 47,3)
    weighted    36,0 m   (19,3 - 56,5)
    info_gain   36,7 m   ( 8,7 - 122,1)

Avec un ecart-type de 14,5 m sur 'weighted', une amelioration de 8 m
est INVISIBLE a cinq repetitions. Modifier un poids et relancer cinq
runs ne prouverait rien : on lirait du bruit.

La reponse est un plan APPARIE. Les deux configurations tournent sur le
MEME trafic, graine par graine, et l'on compare cinq differences plutot
que deux moyennes bruitees. La variabilite du trafic sort alors de la
comparaison, puisqu'elle est identique dans chaque paire.

Pour que cela ait un sens, deux conditions :

  1. La graine du trafic doit etre pilotable. traffic_spawner declare
     deja 'seed' (-1 = aleatoire) et appelle random.seed() ainsi que
     traffic_manager.set_random_device_seed() -- mais ce launch ne lui
     passe rien, d'ou le trafic aleatoire de la campagne.

  2. Les poids testes doivent etre des ARGUMENTS, pas des valeurs
     codees en dur. Sinon changer la configuration change l'empreinte
     du code, et les deux moities de l'experience ne sont plus le meme
     systeme.

CE QUE FAIT CE PATCH
--------------------
Trois arguments de lancement, avec les valeurs ACTUELLES en defaut :

    seed                      -1     (aleatoire, comme aujourd'hui)
    weight_information_gain   0.30   (valeur actuelle)
    weight_distance           0.15   (valeur actuelle)

Lance sans argument, le systeme se comporte donc exactement comme
avant. Les 27 runs archives restent la reference.

L'HYPOTHESE A TESTER
--------------------
La distance moyenne au candidat retenu est la variable la mieux
correlee a l'erreur de localisation (+0,38 sur 25 runs, contre -0,34
pour le nombre de decisions et -0,05 pour la couverture). Et
'distance', qui vise a 42 m, n'a jamais diverge, tandis que 'weighted'
vise a 88 m.

    config A   info 0.30   distance 0.15    (actuel)
    config B   info 0.20   distance 0.25    (sauts plus courts)

Contraste volontairement modere : doubler le poids de la distance
transformerait 'weighted' en 'distance' et lui ferait perdre ce qui la
distingue -- zero divergence sur cinq runs, meilleur score de securite
(0,903) et deux fois moins de collisions que 'distance' (9 contre 22).
On veut la decaler, pas la remplacer.

VERIFICATION
------------
    python3 -m py_compile ~/active_slam_carla/ros2_ws/src/\
active_slam_decision/launch/active_slam.launch.py
    cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install

    ros2 launch active_slam_decision active_slam.launch.py --show-args \
        | grep -E "seed|weight_"

Les trois arguments doivent apparaitre avec leurs defauts.

Usage :
    python3 patch_arguments_lancement.py
"""

import io
import os
import shutil
import sys
import time

CIBLE = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/launch/"
    "active_slam.launch.py"
)


# --- 1. Les deux poids testes deviennent des arguments ----------------

ANCRE_POIDS = (
    "            'weight_information_gain': 0.30,\n"
    "            'weight_localization_uncertainty': 0.20,\n"
    "            'weight_loop_closure': 0.20,\n"
    "            'weight_distance': 0.15,\n"
    "            'weight_safety': 0.15,\n"
)

REMPLACEMENT_POIDS = (
    "            # --- Poids exposes en arguments de lancement ---\n"
    "            # Le gain d'information et le cout de distance sont les\n"
    "            # deux poids de l'experience apparie decrite dans\n"
    "            # patch_arguments_lancement.py. Les defauts reproduisent\n"
    "            # exactement les valeurs de la campagne du 3 septembre :\n"
    "            # lance sans argument, le systeme est inchange.\n"
    "            #\n"
    "            # Les rendre pilotables permet de comparer deux\n"
    "            # configurations SOUS LA MEME EMPREINTE DE CODE, ce qui\n"
    "            # est la condition pour que la comparaison porte sur le\n"
    "            # reglage et non sur deux versions differentes.\n"
    "            'weight_information_gain': ParameterValue(\n"
    "                LaunchConfiguration('weight_information_gain'),\n"
    "                value_type=float\n"
    "            ),\n"
    "            'weight_localization_uncertainty': 0.20,\n"
    "            'weight_loop_closure': 0.20,\n"
    "            'weight_distance': ParameterValue(\n"
    "                LaunchConfiguration('weight_distance'),\n"
    "                value_type=float\n"
    "            ),\n"
    "            'weight_safety': 0.15,\n"
)


# --- 2. La graine du trafic est transmise -----------------------------

ANCRE_SEED = "                'min_distance_from_ego': 10.0,\n"

REMPLACEMENT_SEED = (
    "                # --- Graine du trafic ---\n"
    "                # traffic_spawner declare 'seed' a -1 (aleatoire) et,\n"
    "                # des qu'elle est >= 0, appelle random.seed() puis\n"
    "                # traffic_manager.set_random_device_seed(). Ce launch\n"
    "                # ne la lui passait pas : les quatre strategies Active\n"
    "                # SLAM de la campagne ont donc affronte un trafic tire\n"
    "                # au hasard, alors que slam_evaluation.launch.py fixait\n"
    "                # la sienne a 42.\n"
    "                #\n"
    "                # Defaut -1 : comportement inchange. Fixee, elle rend\n"
    "                # possible la comparaison APPARIEE de deux reglages sur\n"
    "                # le meme trafic.\n"
    "                'seed': ParameterValue(\n"
    "                    LaunchConfiguration('seed'),\n"
    "                    value_type=int\n"
    "                ),\n"
    "                'min_distance_from_ego': 10.0,\n"
)


# --- 3. Declaration des trois arguments -------------------------------

ANCRE_DECLARE = (
    "        DeclareLaunchArgument('max_distance', default_value='0.0'),\n"
)

REMPLACEMENT_DECLARE = (
    "        DeclareLaunchArgument('max_distance', default_value='0.0'),\n"
    "        # --- Graine du generateur de trafic ---\n"
    "        # -1 = aleatoire, comme la campagne du 3 septembre. Une valeur\n"
    "        # >= 0 rend le trafic reproductible et permet de comparer deux\n"
    "        # reglages sur des conditions identiques :\n"
    "        #   ros2 launch ... seed:=3\n"
    "        DeclareLaunchArgument('seed', default_value='-1'),\n"
    "        # --- Poids de la fonction de decision ---\n"
    "        # Defauts identiques a la campagne du 3 septembre. La\n"
    "        # configuration B testee vaut 0.20 / 0.25 :\n"
    "        #   ros2 launch ... weight_information_gain:=0.20 \\\n"
    "        #                   weight_distance:=0.25\n"
    "        DeclareLaunchArgument(\n"
    "            'weight_information_gain', default_value='0.30'\n"
    "        ),\n"
    "        DeclareLaunchArgument('weight_distance', default_value='0.15'),\n"
)


def main():
    if not os.path.exists(CIBLE):
        raise SystemExit("Fichier introuvable : %s" % CIBLE)

    src = io.open(CIBLE, encoding="utf-8").read()

    if "DeclareLaunchArgument('seed'" in src:
        raise SystemExit(
            "Le correctif semble deja applique (argument 'seed' present).\n"
            "Aucune modification effectuee."
        )

    if "ParameterValue" not in src:
        raise SystemExit(
            "ParameterValue n'est pas importe dans ce fichier.\n"
            "Aucune modification effectuee."
        )

    for nom, ancre in (("poids", ANCRE_POIDS),
                       ("seed", ANCRE_SEED),
                       ("declarations", ANCRE_DECLARE)):
        n = src.count(ancre)
        if n != 1:
            raise SystemExit(
                "ANCRE '%s' TROUVEE %d FOIS (attendu 1).\n"
                "Aucune modification effectuee." % (nom, n)
            )

    src = src.replace(ANCRE_POIDS, REMPLACEMENT_POIDS)
    src = src.replace(ANCRE_SEED, REMPLACEMENT_SEED)
    src = src.replace(ANCRE_DECLARE, REMPLACEMENT_DECLARE)

    sauvegarde = "%s.bak_args_%s" % (CIBLE, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(CIBLE, sauvegarde)
    io.open(CIBLE, "w", encoding="utf-8").write(src)

    print("Trois arguments de lancement ajoutes, defauts inchanges :")
    print("    seed                     -1")
    print("    weight_information_gain  0.30")
    print("    weight_distance          0.15")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % CIBLE)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Controler que les arguments sont bien exposes :")
    print("  ros2 launch active_slam_decision active_slam.launch.py"
          " --show-args | grep -E 'seed|weight_'")
    return 0


if __name__ == "__main__":
    sys.exit(main())
