#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Corrige le blocage de la liste noire des points inatteignables.

LE DEFAUT
---------
Releve sur le run de chauffe de la campagne :

    AUCUN CANDIDAT EXPLOITABLE : les 23 candidats sont tous a moins de
    25 m d'un point deja declare inatteignable (7 points).
    ... repete toutes les 3 secondes, vehicule immobile a (0.00, -0.17)
    apres plus de 3 minutes.

Deux causes se combinent.

1. RAYON D'EXCLUSION DISPROPORTIONNE.
   unreachable_exclusion_radius vaut 25 m, alors que candidate_generator
   espace ses candidats de waypoint_spacing = 8 m. Chaque point rejete
   elimine donc une zone contenant 6 a 7 candidats. Sept rejets au
   demarrage suffisent a effacer les 23 candidats disponibles.

2. L'ATTENTE N'A PAS D'ISSUE.
   Le correctif precedent faisait attendre la publication de candidats
   suivante plutot que de rechoisir un point deja rejete. Mais cette
   liste est calculee depuis la position du vehicule : tant qu'il ne
   bouge pas, elle est identique. L'attente ne peut donc pas se
   debloquer d'elle-meme. Un echec rapide a ete remplace par un blocage
   permanent, ce qui est pire : le run consomme ses 900 s sans rien
   produire et sans rien signaler d'anormal.

LES CORRECTIONS
---------------
a) Le rayon d'exclusion passe de 25 m a 8 m, c'est-a-dire l'espacement
   des candidats. Un point rejete elimine desormais lui-meme et ses
   voisins immediats, pas tout un quartier.

b) Quand le filtre vide malgre tout le vivier, on ne se fige plus : on
   repart sur la liste complete, mais AU PLUS UNE FOIS TOUTES LES
   repli_cooldown SECONDES. La rafale de rejets -- 20 en 180 ms, relevee
   precedemment -- devient impossible, et le chien de garde de 60 s sans
   destination acceptee peut faire son travail et terminer le run
   proprement au lieu de le laisser tourner a vide.

C'est la combinaison qui compte : (a) rend la situation rare, (b)
garantit qu'elle ne bloque jamais.

Usage :
    python3 patch_liste_noire.py
"""

import io
import os
import shutil
import sys
import time

BASE = os.path.expanduser("~/active_slam_carla/ros2_ws/src/active_slam_decision/")
DECISION = BASE + "active_slam_decision/decision_maker.py"
LAUNCH = BASE + "launch/active_slam.launch.py"


LAUNCH_BLOCS = [
    (
        "            'unreachable_exclusion_radius': 25.0,\n",

        "            # Rayon d'exclusion autour d'un point declare\n"
        "            # inatteignable. DOIT rester de l'ordre de\n"
        "            # l'espacement des candidats (waypoint_spacing = 8 m\n"
        "            # dans candidate_generator), pas de l'ordre de la\n"
        "            # distance minimale d'un candidat.\n"
        "            #\n"
        "            # A 25 m, chaque point rejete eliminait 6 a 7 candidats :\n"
        "            # sept rejets au demarrage effacaient les 23 candidats\n"
        "            # disponibles, et le vehicule restait immobile pour tout\n"
        "            # le run. A 8 m, un rejet elimine le point lui-meme et\n"
        "            # ses voisins immediats.\n"
        "            'unreachable_exclusion_radius': 8.0,\n",
    ),
]


DECISION_BLOCS = [
    # --- Etat : instant du dernier repli ---
    (
        "        # Instant du premier rejet d'une serie ininterrompue. Sert au\n"
        "        # critere d'arret : c'est une DUREE sans progres qui doit\n"
        "        # terminer le run, pas un nombre de messages echanges.\n"
        "        self.premier_rejet_time = None\n",

        "        # Instant du premier rejet d'une serie ininterrompue. Sert au\n"
        "        # critere d'arret : c'est une DUREE sans progres qui doit\n"
        "        # terminer le run, pas un nombre de messages echanges.\n"
        "        self.premier_rejet_time = None\n"
        "        # Instant du dernier repli sur la liste complete de candidats,\n"
        "        # quand le filtre des points inatteignables les a tous exclus.\n"
        "        self.dernier_repli_time = None\n",
    ),

    # --- Ne plus se figer : repli cadence au lieu d'attente sans issue ---
    (
        "        else:\n"
        "            # Tous les candidats sont a moins de\n"
        "            # unreachable_exclusion_radius d'un point deja declare\n"
        "            # inatteignable. La version precedente retombait\n"
        "            # silencieusement sur la liste complete, donc rechoisissait\n"
        "            # un point deja rejete : la boucle\n"
        "            # rejet -> nouvelle decision -> rejet tournait a la vitesse\n"
        "            # des messages ROS. Trace : les memes 7 points en cycle,\n"
        "            # 20 rejets en 180 millisecondes.\n"
        "            #\n"
        "            # On attend plutot la publication de candidats suivante :\n"
        "            # elle sera calculee depuis la position courante du\n"
        "            # vehicule, donc differente. waiting_for_candidates fait\n"
        "            # que on_candidates relancera la decision.\n"
        "            self.get_logger().warn(\n"
        "                f\"AUCUN CANDIDAT EXPLOITABLE : les \"\n"
        "                f\"{len(reachable)} candidats sont tous a moins de \"\n"
        "                f\"{self.unreachable_exclusion_radius:.0f} m d'un point \"\n"
        "                f\"deja declare inatteignable \"\n"
        "                f\"({len(self.unreachable_points)} points). Attente de \"\n"
        "                f\"la prochaine liste de candidats.\"\n"
        "            )\n"
        "            self.waiting_for_candidates = True\n"
        "            return\n",

        "        else:\n"
        "            # Tous les candidats sont a moins de\n"
        "            # unreachable_exclusion_radius d'un point deja declare\n"
        "            # inatteignable.\n"
        "            #\n"
        "            # Deux mauvaises reponses ont ete essayees ici :\n"
        "            #\n"
        "            # 1. Retomber silencieusement sur la liste complete. Cela\n"
        "            #    rechoisit un point deja rejete, et la boucle\n"
        "            #    rejet -> decision -> rejet tourne a la vitesse des\n"
        "            #    messages ROS : 20 rejets en 180 ms, mesures.\n"
        "            #\n"
        "            # 2. Attendre la publication de candidats suivante. Mais\n"
        "            #    cette liste est calculee depuis la position du\n"
        "            #    vehicule : tant qu'il ne bouge pas, elle est\n"
        "            #    identique. L'attente n'a pas d'issue -- vehicule\n"
        "            #    immobile pendant plus de 3 minutes, mesure.\n"
        "            #\n"
        "            # La bonne reponse est le repli CADENCE : on repart sur la\n"
        "            # liste complete, mais au plus une fois toutes les\n"
        "            # repli_cooldown secondes. La rafale devient impossible, le\n"
        "            # vehicule continue d'essayer, et le chien de garde des\n"
        "            # 60 s sans destination acceptee peut terminer le run\n"
        "            # proprement si la situation ne se debloque pas.\n"
        "            repli_cooldown = 5.0\n"
        "            maintenant = self.get_clock().now()\n"
        "            depuis_dernier = (\n"
        "                float('inf') if self.dernier_repli_time is None\n"
        "                else (maintenant - self.dernier_repli_time).nanoseconds / 1e9\n"
        "            )\n"
        "\n"
        "            if depuis_dernier < repli_cooldown:\n"
        "                self.waiting_for_candidates = True\n"
        "                return\n"
        "\n"
        "            self.dernier_repli_time = maintenant\n"
        "            self.get_logger().warn(\n"
        "                f\"AUCUN CANDIDAT EXPLOITABLE : les \"\n"
        "                f\"{len(reachable)} candidats sont tous a moins de \"\n"
        "                f\"{self.unreachable_exclusion_radius:.0f} m d'un point \"\n"
        "                f\"deja declare inatteignable \"\n"
        "                f\"({len(self.unreachable_points)} points). Repli sur la \"\n"
        "                f\"liste complete pour ne pas figer le vehicule.\"\n"
        "            )\n",
    ),
]


def appliquer(chemin, blocs):
    src = io.open(chemin, encoding="utf-8").read()
    for ancre, remplacement in blocs:
        n = src.count(ancre)
        if n != 1:
            raise SystemExit(
                "ANCRE TROUVEE %d FOIS (attendu 1) dans %s :\n---\n%s\n---\n"
                "Aucun fichier n'a ete modifie."
                % (n, os.path.basename(chemin), ancre[:300])
            )
        src = src.replace(ancre, remplacement)
    return src


def main():
    for chemin in (DECISION, LAUNCH):
        if not os.path.exists(chemin):
            raise SystemExit("Fichier introuvable : %s" % chemin)

    if "repli_cooldown" in io.open(DECISION, encoding="utf-8").read():
        raise SystemExit(
            "Le correctif semble deja applique.\n"
            "Aucune modification effectuee."
        )

    resultats = {
        DECISION: appliquer(DECISION, DECISION_BLOCS),
        LAUNCH: appliquer(LAUNCH, LAUNCH_BLOCS),
    }

    suffixe = time.strftime("%Y%m%d_%H%M%S")
    for chemin, contenu in resultats.items():
        sauvegarde = "%s.bak_listenoire_%s" % (chemin, suffixe)
        shutil.copy2(chemin, sauvegarde)
        io.open(chemin, "w", encoding="utf-8").write(contenu)
        print("  %s" % os.path.basename(chemin))
        print("      sauvegarde : %s" % os.path.basename(sauvegarde))

    print("\n3 blocs corriges dans 2 fichiers.")
    print("\nReconstruire :")
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    return 0


if __name__ == "__main__":
    sys.exit(main())
