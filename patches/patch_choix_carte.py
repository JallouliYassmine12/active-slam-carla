#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rend le choix de la carte fiable et verifiable.

LE DEFAUT
---------
bridge_active_slam.py decide s'il faut recharger la carte avec ce test :

    current_map = self.client.get_world().get_map().name.split('/')[-1]
    if town and current_map != town and current_map != f"{town}_Opt":

La derniere condition accepte la variante _Opt comme equivalente a la
carte demandee. Lancer

    ros2 launch ... town:=Town03

alors que Town03_Opt est chargee ne declenche donc AUCUN rechargement :
le message "Carte 'Town03_Opt' deja chargee." s'affiche, et le run se
deroule sur Town03_Opt en croyant mesurer sur Town03.

Ce n'est pas un detail. Les deux cartes ne portent pas le meme maillage
de navigation pietonne -- c'est exactement ce qui distingue le scenario
pietons exige par le sujet, et la raison pour laquelle num_walkers est
force a 0 sur la variante _Opt. La substitution etait silencieuse : rien
dans le journal ne permettait de savoir sur quelle carte un run avait
reellement tourne.

CE QUE LE PATCH FAIT
--------------------
1. Comparaison STRICTE : 'Town03' n'est plus satisfait par 'Town03_Opt'.

2. Avertissement sur les cartes pleines. Releve sur cette configuration :
   LowLevelFatalError "Shader compilation failures are Fatal" au
   chargement de Town01 comme de Town03. Les variantes _Opt chargent
   leurs decors par couches et passent. Le chargement est quand meme
   effectue -- c'est un avertissement, pas un refus.

3. Trace de la carte REELLEMENT active, relue apres le chargement, et
   message d'erreur explicite si elle differe de celle demandee. Le
   journal de chaque run porte desormais la preuve du scenario mesure.

4. Valeur par defaut du parametre alignee sur celle du fichier de
   lancement ('Town03_Opt' au lieu de 'Town03'). Le noeud lance seul
   aurait sinon tente de charger la carte pleine, donc de faire planter
   CARLA.

CE QUE LE PATCH NE CHANGE PAS
-----------------------------
La campagne tourne sur Town03_Opt, valeur par defaut du launch, qui est
bien la carte chargee. Les mesures deja realisees ne sont pas affectees.
Ce correctif ne devient indispensable que pour le scenario pietons, qui
doit tourner sur Town03 pleine.

Usage :
    python3 patch_choix_carte.py
"""

import io
import os
import shutil
import sys
import time

CIBLE = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/carla_sensors_bridge/"
    "carla_sensors_bridge/bridge_active_slam.py"
)


BLOCS = [
    # ------------------------------------------------------------------
    # 1. Valeur par defaut alignee sur le fichier de lancement
    # ------------------------------------------------------------------
    (
        "        self.declare_parameter('town', 'Town03')\n",

        "        # Aligne sur la valeur par defaut du fichier de lancement.\n"
        "        # 'Town03' (carte pleine) provoquerait un rechargement vers\n"
        "        # une carte qui fait tomber cette configuration GPU des que\n"
        "        # le noeud est lance seul, sans le launch.\n"
        "        self.declare_parameter('town', 'Town03_Opt')\n",
    ),

    # ------------------------------------------------------------------
    # 2. Comparaison stricte + avertissement + verification effective
    # ------------------------------------------------------------------
    (
        "        current_map = self.client.get_world().get_map().name.split('/')[-1]\n"
        "        if town and current_map != town and current_map != f\"{town}_Opt\":\n"
        "            self.get_logger().info(\n"
        "                f\"Carte actuelle '{current_map}', chargement de '{town}'...\"\n"
        "            )\n"
        "            self.world = self.client.load_world(town)\n"
        "            self.get_logger().info(f\"Carte '{town}' chargee.\")\n"
        "        else:\n"
        "            self.get_logger().info(f\"Carte '{current_map}' deja chargee.\")\n"
        "            self.world = self.client.get_world()\n",

        "        current_map = self.client.get_world().get_map().name.split('/')[-1]\n"
        "\n"
        "        # --- Comparaison STRICTE ---\n"
        "        # La condition precedente etait :\n"
        "        #   if town and current_map != town \\\n"
        "        #           and current_map != f\"{town}_Opt\":\n"
        "        # Elle acceptait la variante _Opt comme equivalente a la carte\n"
        "        # demandee. Lancer town:=Town03 alors que Town03_Opt etait\n"
        "        # chargee ne declenchait donc aucun rechargement : le run se\n"
        "        # deroulait sur Town03_Opt en croyant mesurer sur Town03.\n"
        "        #\n"
        "        # Les deux cartes ne portent pas le meme maillage de navigation\n"
        "        # pietonne -- c'est precisement ce qui distingue le scenario\n"
        "        # pietons exige par le sujet, et la raison pour laquelle\n"
        "        # num_walkers est force a 0 sur la variante _Opt. La\n"
        "        # substitution n'etait donc pas anodine, et elle etait\n"
        "        # silencieuse.\n"
        "        if town and current_map != town:\n"
        "            # Les cartes PLEINES chargent tous leurs decors d'un coup,\n"
        "            # donc des milliers de shaders a compiler. Releve sur cette\n"
        "            # configuration : LowLevelFatalError \"Shader compilation\n"
        "            # failures are Fatal\" au chargement de Town01 comme de\n"
        "            # Town03. Les variantes _Opt chargent par couches et\n"
        "            # passent. On avertit, mais on charge quand meme : c'est un\n"
        "            # choix de scenario, pas une erreur.\n"
        "            if not town.endswith('_Opt'):\n"
        "                self.get_logger().warn(\n"
        "                    f\"'{town}' est une carte PLEINE (non _Opt). Sur \"\n"
        "                    f\"cette configuration GPU, ces cartes provoquent \"\n"
        "                    f\"un LowLevelFatalError de compilation de shaders. \"\n"
        "                    f\"La variante '{town}_Opt' charge par couches et \"\n"
        "                    f\"passe. Chargement demande malgre tout.\"\n"
        "                )\n"
        "            self.get_logger().info(\n"
        "                f\"Carte actuelle '{current_map}', chargement de \"\n"
        "                f\"'{town}'...\"\n"
        "            )\n"
        "            self.world = self.client.load_world(town)\n"
        "        else:\n"
        "            self.get_logger().info(f\"Carte '{current_map}' deja chargee.\")\n"
        "            self.world = self.client.get_world()\n"
        "\n"
        "        # --- Carte REELLEMENT active ---\n"
        "        # Relue apres coup, pas deduite du parametre. Cette ligne est\n"
        "        # la trace qui permet de verifier, journal en main, sur quelle\n"
        "        # carte un run s'est deroule : la carte fait partie du\n"
        "        # scenario, ce n'est pas un detail de demarrage.\n"
        "        carte_effective = self.world.get_map().name.split('/')[-1]\n"
        "        if town and carte_effective != town:\n"
        "            self.get_logger().error(\n"
        "                f\"CARTE INCORRECTE : '{town}' demandee, \"\n"
        "                f\"'{carte_effective}' active. Le scenario mesure \"\n"
        "                f\"n'est pas celui demande.\"\n"
        "            )\n"
        "        else:\n"
        "            self.get_logger().info(f\"CARTE ACTIVE : {carte_effective}\")\n",
    ),
]


def main():
    if not os.path.exists(CIBLE):
        raise SystemExit("Fichier introuvable : %s" % CIBLE)

    src = io.open(CIBLE, encoding="utf-8").read()

    if "CARTE ACTIVE" in src:
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    for ancre, remplacement in BLOCS:
        n = src.count(ancre)
        if n != 1:
            raise SystemExit(
                "ANCRE TROUVEE %d FOIS (attendu 1) :\n---\n%s\n---\n"
                "Aucune modification effectuee." % (n, ancre[:300])
            )
        src = src.replace(ancre, remplacement)

    sauvegarde = "%s.bak_carte_%s" % (CIBLE, time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(CIBLE, sauvegarde)
    io.open(CIBLE, "w", encoding="utf-8").write(src)

    print("2 blocs corriges dans bridge_active_slam.py")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Verifier, puis reconstruire :")
    print("  python3 -m py_compile %s" % CIBLE)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    return 0


if __name__ == "__main__":
    sys.exit(main())
