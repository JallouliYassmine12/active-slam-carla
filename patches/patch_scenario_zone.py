#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Patch : ajoute la VARIANTE G (zone bornee) a un_scenario.sh.

Trois modifications independantes, chacune sautee si deja appliquee :

  1. VARIANTE=G : poids de la configuration B, graine = repetition, et
     export de ZONE_HALF_SIZE=150 (carre de 300 m de cote centre sur le
     point d'apparition). Le suffixe G nomme la campagne.

  2. La garde qui interdit toute VARIANTE a 'predefinie' est assouplie
     pour la seule lettre G. La reference DOIT porter ce suffixe pour
     ne pas ecraser les runs predefinie deja archives -- et elle ignore
     la zone, puisqu'elle ne lance pas candidate_generator. C'est
     exactement l'asymetrie que l'experience mesure.

  3. La borne de repetition passe de 1-5 a 1-10.

Sauvegarde horodatee, ancres verifiees uniques, bash -n.
"""

import os
import shutil
import subprocess
import sys
import time

CIBLE = os.path.expanduser("~/un_scenario.sh")

BLOC_G = '''    G)
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
'''

MODIFS = [
    (
        "variante G",
        "ZONE_HALF_SIZE",
        '''    *)
        echo "ERREUR : VARIANTE doit valoir A, B, C, D, E ou F (recu '$VARIANTE')"
        exit 1
        ;;
esac''',
        BLOC_G + '''    *)
        echo "ERREUR : VARIANTE doit valoir A, B, C, D, E, F ou G (recu '$VARIANTE')"
        exit 1
        ;;
esac''',
    ),
    (
        "garde predefinie assouplie pour G",
        '[ "$VARIANTE" != "G" ]',
        'if [ -n "$VARIANTE" ] && [ "$STRATEGIE" = "predefinie" ]; then',
        'if [ -n "$VARIANTE" ] && [ "$VARIANTE" != "G" ] \\\n   && [ "$STRATEGIE" = "predefinie" ]; then',
    ),
    (
        "repetitions 1 a 10",
        "[1-9]|10)",
        '''    1|2|3|4|5) ;;
    *) echo "ERREUR : repetition doit valoir 1 a 5"; exit 1 ;;''',
        '''    [1-9]|10) ;;
    *) echo "ERREUR : repetition doit valoir 1 a 10"; exit 1 ;;''',
    ),
]


def echec(message):
    print("ECHEC : " + message)
    print("Aucune modification n'a ete appliquee.")
    sys.exit(1)


def main():
    if not os.path.isfile(CIBLE):
        echec("fichier introuvable : " + CIBLE)

    src = open(CIBLE, encoding="utf-8").read()
    a_faire = []

    for description, marqueur, ancre, remplacement in MODIFS:
        if marqueur in src:
            print("  deja applique, ignore : " + description)
            continue
        n = src.count(ancre)
        if n != 1:
            echec("ancre '%s' trouvee %d fois (attendu 1)." % (description, n))
        a_faire.append((description, ancre, remplacement))

    if not a_faire:
        print("Rien a faire : le patch est deja entierement applique.")
        sys.exit(0)

    print("  %d modification(s) a appliquer, ancres uniques." % len(a_faire))

    sauvegarde = CIBLE + ".bak_" + time.strftime("%Y%m%d_%H%M%S")
    shutil.copy2(CIBLE, sauvegarde)
    print("  sauvegarde : " + os.path.basename(sauvegarde))

    for description, ancre, remplacement in a_faire:
        src = src.replace(ancre, remplacement, 1)
        print("  applique : " + description)

    open(CIBLE, "w", encoding="utf-8").write(src)

    r = subprocess.run(["bash", "-n", CIBLE], capture_output=True, text=True)
    if r.returncode != 0:
        shutil.copy2(sauvegarde, CIBLE)
        print(r.stderr)
        echec("bash -n a echoue ; le fichier d'origine a ete restaure.")
    print("  bash -n : OK")

    print("")
    print("Patch applique. Aucune recompilation : ce script est hors workspace,")
    print("l'empreinte du code n'est pas affectee.")


main()
