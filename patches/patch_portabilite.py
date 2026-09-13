#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Patch de portabilite des scripts d'analyse.

Remplace les chemins ecrits en dur par des variables d'environnement, avec
le chemin actuel comme valeur par defaut. Rien ne change pour un lancement
depuis la machine de developpement ; le depot devient utilisable ailleurs.

    ACTIVE_SLAM_METRICS   defaut ~/active_slam_carla/metrics
    ACTIVE_SLAM_MAPS      defaut ~/active_slam_carla/maps

Regles de securite :
  - sauvegarde horodatee de chaque fichier avant modification ;
  - une ancre trouvee plusieurs fois interrompt TOUT le patch, sans rien
    ecrire ;
  - une ancre absente laisse le fichier intact et le signale ;
  - un fichier deja patche est reconnu et laisse tel quel ;
  - verification par py_compile apres ecriture, restauration en cas
    d'echec.

Idempotent : une seconde execution ne modifie rien.
"""
import os
import py_compile
import shutil
import sys
import time

HORODATAGE = time.strftime("%Y%m%d_%H%M%S")
MAISON = os.path.expanduser("~")

# (variable d'environnement, chemin par defaut)
REGLE_METRICS = ("ACTIVE_SLAM_METRICS", "~/active_slam_carla/metrics")
REGLE_MAPS = ("ACTIVE_SLAM_MAPS", "~/active_slam_carla/maps")

CIBLES = [
    ("resultats_G.py", [REGLE_METRICS]),
    ("analyse_ate_appariee.py", [REGLE_METRICS]),
    ("distance_seuil.py", [REGLE_METRICS]),
    ("analyse_couverture.py", [REGLE_METRICS]),
    ("fermetures_boucle.py", [REGLE_METRICS, REGLE_MAPS]),
]


def ancre_de(chemin_defaut):
    return 'os.path.expanduser("%s")' % chemin_defaut


def marqueur_de(variable):
    return 'os.environ.get("%s"' % variable


def appliquer(lignes, variable, chemin_defaut):
    """Remplace l'ancre sur son unique ligne, en alignant la continuation.

    Retourne (lignes_modifiees, etat) avec etat dans
    {'fait', 'deja', 'absent', 'multiple'}.
    """
    ancre = ancre_de(chemin_defaut)
    marqueur = marqueur_de(variable)

    if any(marqueur in l for l in lignes):
        return lignes, "deja"

    index = [i for i, l in enumerate(lignes) if ancre in l]
    if not index:
        return lignes, "absent"
    if len(index) > 1:
        return lignes, "multiple"

    i = index[0]
    ligne = lignes[i]
    debut = ligne.index(ancre)
    prefixe = ligne[:debut]
    suffixe = ligne[debut + len(ancre):]
    creux = " " * (len(prefixe) + len("os.environ.get("))

    lignes[i] = '%s%s,' % (prefixe, marqueur)
    lignes.insert(i + 1, '%s%s)%s' % (creux, ancre, suffixe))
    return lignes, "fait"


# ---------------------------------------------------- phase 1 : controle

print()
print("PATCH DE PORTABILITE - phase de controle (aucune ecriture)")
print()

plan = []
arret = False

for nom, regles in CIBLES:
    chemin = os.path.join(MAISON, nom)
    if not os.path.exists(chemin):
        print("  ABSENT     %-28s  ignore" % nom)
        continue
    with open(chemin, encoding="utf-8") as f:
        lignes = f.read().split("\n")

    etats, faits = [], 0
    for variable, defaut in regles:
        lignes, etat = appliquer(lignes, variable, defaut)
        etats.append((variable, etat))
        if etat == "fait":
            faits += 1
        elif etat == "multiple":
            print()
            print("  ARRET : l'ancre de %s apparait plusieurs fois dans %s."
                  % (variable, nom))
            print("  Aucun fichier n'a ete modifie. Verifie ce fichier.")
            arret = True
        elif etat == "absent":
            print("  ANCRE ABSENTE  %-24s  %s"
                  % (nom, ancre_de(defaut)[:46]))

    if arret:
        sys.exit(1)
    if faits:
        plan.append((nom, chemin, "\n".join(lignes), faits))
        print("  A PATCHER  %-28s  %d remplacement(s)" % (nom, faits))
    elif all(e == "deja" for _, e in etats) and etats:
        print("  DEJA FAIT  %-28s" % nom)

if not plan:
    print()
    print("  Rien a faire. Les scripts sont deja portables.")
    sys.exit(0)

# ---------------------------------------------------- phase 2 : ecriture

print()
print("PATCH DE PORTABILITE - ecriture")
print()

for nom, chemin, texte, faits in plan:
    sauvegarde = "%s.bak_%s" % (chemin, HORODATAGE)
    shutil.copy2(chemin, sauvegarde)
    with open(chemin, "w", encoding="utf-8") as f:
        f.write(texte)
    try:
        py_compile.compile(chemin, doraise=True)
    except py_compile.PyCompileError as e:
        shutil.copy2(sauvegarde, chemin)
        print("  ECHEC      %-28s  restaure depuis la sauvegarde" % nom)
        print("  %s" % e)
        sys.exit(1)
    print("  OK         %-28s  sauvegarde : %s"
          % (nom, os.path.basename(sauvegarde)))

print()
print("Termine. Lancement inchange sur cette machine :")
print("    python3 ~/analyse_ate_appariee.py")
print()
print("Lancement depuis un depot clone ailleurs :")
print("    ACTIVE_SLAM_METRICS=resultats/metrics \\")
print("        python3 analyse/analyse_ate_appariee.py")
print()
