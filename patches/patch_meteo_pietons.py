#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ajoute les scenarios METEO et PIETONS a la pipeline Active SLAM.

Le sujet impose une validation "comprenant notamment des routes urbaines,
des intersections, du trafic, des pietons, des obstacles ainsi que
differentes conditions meteorologiques". Deux de ces items manquaient :

  - METEO : bridge_active_slam.py appliquait ClearNoon EN DUR. Ajouter
    l'argument au seul fichier de lancement n'aurait servi a rien, le
    noeud ne declarant aucun parametre 'weather' : il aurait ete
    silencieusement ignore, et on aurait cru avoir mesure sous la pluie.

  - PIETONS : le launch forcait num_walkers a 0, a cause d'un segfault de
    world.get_random_location_from_navigation() sur Town03_Opt, dont le
    maillage de navigation pietonne n'est pas charge. Le nombre devient
    un argument, ce qui permet le scenario pietons sur Town03 (variante
    non _Opt) sans toucher au code.

PRINCIPE : les deux nouveaux arguments ont une valeur par defaut qui
REPRODUIT EXACTEMENT le comportement actuel (weather='clear' donne
ClearNoon, num_pedestrians=0). La campagne comparative de 15 runs deja
realisee reste donc valide et comparable aux runs a venir.

Usage :
    python3 patch_meteo_pietons.py

Le script sauvegarde chaque fichier avant modification, refuse d'agir si
une ancre n'est pas trouvee exactement une fois, et ne modifie rien tant
que les deux fichiers ne sont pas patchables.
"""

import io
import os
import shutil
import sys
import time

BRIDGE = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/carla_sensors_bridge/"
    "carla_sensors_bridge/bridge_active_slam.py"
)
LAUNCH = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/launch/"
    "active_slam.launch.py"
)

# ====================================================================
# bridge_active_slam.py
# ====================================================================

BRIDGE_BLOCS = [
    # --- 1. Import des prereglages meteo ---------------------------
    (
        "from std_msgs.msg import Header, String\n",
        # Les prereglages sont IMPORTES de bridge_node plutot que
        # recopies. RTAB-Map se sert de la camera pour detecter les
        # fermetures de boucle : si 'rain' ne designait pas exactement la
        # meme chose dans le pont classique et dans le pont Active SLAM,
        # la comparaison entre les deux systemes serait faussee par
        # l'eclairage. Deux copies d'un dictionnaire finissent toujours
        # par diverger ; une source unique l'interdit.
        # bridge_node protege son main() par if __name__ == '__main__' et
        # ne fait aucun appel a CARLA au niveau module : l'import est sans
        # effet de bord.
        "from carla_sensors_bridge.bridge_node import WEATHER_PRESETS\n",
        "after",
    ),

    # --- 2. Declaration du parametre -------------------------------
    (
        "        self.declare_parameter('spawn_index', 0)\n",
        "        self.declare_parameter('weather', 'clear')\n",
        "after",
    ),

    # --- 3. Lecture du parametre -----------------------------------
    (
        "        self.spawn_index = self.get_parameter('spawn_index').value\n",
        "        self.weather_name = str(\n"
        "            self.get_parameter('weather').value\n"
        "        ).lower()\n",
        "after",
    ),

    # --- 4. Application de la meteo --------------------------------
    (
        "        # Meteo fixee, identique au pont du SLAM classique "
        "(preset 'clear').\n"
        "        # RTAB-Map utilise la camera pour detecter les fermetures "
        "de boucle :\n"
        "        # un eclairage different d'un systeme a l'autre fausserait "
        "la\n"
        "        # comparaison. ClearNoon est le meme preset des deux "
        "cotes.\n"
        "        self.world.set_weather(carla.WeatherParameters.ClearNoon)\n"
        "        self.get_logger().info(\"METEO : clear (ClearNoon)\")\n",

        "        # --- Meteo ---\n"
        "        # Prereglages partages avec le pont du SLAM classique (voir\n"
        "        # l'import de WEATHER_PRESETS en tete de fichier) : RTAB-Map\n"
        "        # detecte ses fermetures de boucle sur la camera, donc un\n"
        "        # eclairage different d'un systeme a l'autre fausserait la\n"
        "        # comparaison entre eux.\n"
        "        #\n"
        "        # La valeur par defaut 'clear' vaut carla.WeatherParameters.\n"
        "        # ClearNoon, exactement ce qui etait applique en dur ici\n"
        "        # jusqu'a present : les runs de la campagne comparative deja\n"
        "        # realises restent donc reproductibles a l'identique.\n"
        "        #\n"
        "        #   ros2 launch ... weather:=rain\n"
        "        if self.weather_name not in WEATHER_PRESETS:\n"
        "            self.get_logger().warn(\n"
        "                f\"Prereglage meteo inconnu : '{self.weather_name}'. \"\n"
        "                f\"Valeurs possibles : \"\n"
        "                f\"{', '.join(sorted(WEATHER_PRESETS))}. \"\n"
        "                f\"Repli sur 'clear'.\"\n"
        "            )\n"
        "            self.weather_name = 'clear'\n"
        "\n"
        "        weather = WEATHER_PRESETS[self.weather_name]()\n"
        "        self.world.set_weather(weather)\n"
        "        self.get_logger().info(\n"
        "            f\"METEO : {self.weather_name} \"\n"
        "            f\"(pluie={weather.precipitation:.0f}, \"\n"
        "            f\"brouillard={weather.fog_density:.0f}, \"\n"
        "            f\"soleil={weather.sun_altitude_angle:.0f} deg)\"\n"
        "        )\n",
        "replace",
    ),
]

# ====================================================================
# active_slam.launch.py
# ====================================================================

LAUNCH_BLOCS = [
    # --- 5. Declaration des deux nouveaux arguments ----------------
    (
        "        DeclareLaunchArgument('town', default_value='Town03_Opt'),\n",

        "        # Conditions meteorologiques, exigees par le sujet parmi les\n"
        "        # scenarios de validation. 'clear' reproduit exactement le\n"
        "        # ClearNoon applique en dur auparavant : la valeur par defaut\n"
        "        # ne change donc rien aux runs deja realises.\n"
        "        # Valeurs : clear, cloudy, wet, light_rain, rain, sunset,\n"
        "        #           fog, night\n"
        "        #   ros2 launch ... weather:=rain\n"
        "        DeclareLaunchArgument('weather', default_value='clear'),\n"
        "        # Nombre de pietons NPC. Defaut 0, comme jusqu'a present.\n"
        "        # ATTENTION : ne fonctionne PAS sur Town03_Opt. La variante\n"
        "        # _Opt ne charge pas le maillage de navigation pietonne, et\n"
        "        # world.get_random_location_from_navigation() y provoque un\n"
        "        # segfault (exit code -11) apres ~13 s, qui tue le noeud de\n"
        "        # trafic avant meme le message TRAFIC ACTIF - donc supprime\n"
        "        # aussi les vehicules. Le scenario pietons doit se lancer sur\n"
        "        # la carte pleine, qui embarque ce maillage :\n"
        "        #   ros2 launch ... town:=Town03 num_pedestrians:=15\n"
        "        DeclareLaunchArgument('num_pedestrians', default_value='0'),\n",
        "after",
    ),

    # --- 6. Transmission de la meteo au pont -----------------------
    (
        "                'town': LaunchConfiguration('town'),\n"
        "                'spawn_index': LaunchConfiguration('spawn_index'),\n",

        "                'town': LaunchConfiguration('town'),\n"
        "                'spawn_index': LaunchConfiguration('spawn_index'),\n"
        "                'weather': LaunchConfiguration('weather'),\n",
        "replace",
    ),

    # --- 7. Nombre de pietons pilotable ----------------------------
    (
        "                # Pietons desactives : la boucle d'appels a\n"
        "                # world.get_random_location_from_navigation() du\n"
        "                # traffic_spawner provoque un segfault (exit code -11) apres\n"
        "                # ~13 s sur Town03_Opt, dont le maillage de navigation\n"
        "                # pietonne n'est pas charge par la variante _Opt. Le noeud\n"
        "                # meurt avant meme d'avoir affiche \"TRAFIC ACTIF\", d'ou\n"
        "                # l'absence totale de trafic dans le dernier run.\n"
        "                'num_walkers': 0,\n",

        "                # Pietons : 0 par defaut, car la boucle d'appels a\n"
        "                # world.get_random_location_from_navigation() du\n"
        "                # traffic_spawner provoque un segfault (exit code -11)\n"
        "                # apres ~13 s sur Town03_Opt, dont le maillage de\n"
        "                # navigation pietonne n'est pas charge par la variante\n"
        "                # _Opt. Le noeud meurt avant meme d'avoir affiche\n"
        "                # \"TRAFIC ACTIF\", d'ou l'absence totale de trafic.\n"
        "                # Le nombre est desormais pilotable pour permettre le\n"
        "                # scenario pietons demande par le sujet, qui doit se\n"
        "                # lancer sur la carte pleine :\n"
        "                #   ros2 launch ... town:=Town03 num_pedestrians:=15\n"
        "                'num_walkers': ParameterValue(\n"
        "                    LaunchConfiguration('num_pedestrians'),\n"
        "                    value_type=int\n"
        "                ),\n",
        "replace",
    ),
]


def appliquer(chemin, blocs):
    """Applique les blocs et renvoie le nouveau contenu, sans ecrire."""
    src = io.open(chemin, encoding="utf-8").read()
    for ancre, texte, mode in blocs:
        n = src.count(ancre)
        if n != 1:
            raise SystemExit(
                "ANCRE TROUVEE %d FOIS (attendu 1) dans %s :\n---\n%s\n---\n"
                "Aucun fichier n'a ete modifie."
                % (n, os.path.basename(chemin), ancre[:200])
            )
        if mode == "after":
            src = src.replace(ancre, ancre + texte)
        elif mode == "before":
            src = src.replace(ancre, texte + ancre)
        else:
            src = src.replace(ancre, texte)
    return src


def main():
    for chemin in (BRIDGE, LAUNCH):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    # On calcule les DEUX resultats avant d'ecrire quoi que ce soit : si le
    # second fichier n'est pas patchable, le premier ne doit pas avoir ete
    # modifie, sinon le paquet se retrouve dans un etat intermediaire ou la
    # meteo est declaree cote noeud mais pas transmise cote lancement.
    resultats = {
        BRIDGE: appliquer(BRIDGE, BRIDGE_BLOCS),
        LAUNCH: appliquer(LAUNCH, LAUNCH_BLOCS),
    }

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in resultats.items():
        sauvegarde = "%s.bak_meteo_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("\n7 blocs inseres dans 2 fichiers.")
    print("\nVerifier, puis reconstruire :")
    print("  python3 -m py_compile %s" % BRIDGE)
    print("  python3 -m py_compile %s" % LAUNCH)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    return 0


if __name__ == "__main__":
    sys.exit(main())
