#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Le transfert de poids vers la fermeture de boucle ne s'applique plus
que dans la mesure ou une fermeture de boucle est reellement possible.

LE FAIT MESURE
--------------
Journal de decision de weighted_1 (3 septembre, campagne unitaire),
decisions 10 a 28 -- dix-neuf decisions consecutives :

    10  incert=0.808  loop=0.000  gain=0.223
    11  incert=0.856  loop=0.000  gain=0.479
    ...
    28  incert=0.936  loop=0.000  gain=0.813

L'incertitude normalisee est au plus haut, donc le transfert est au
maximum :

    transfer = 0.20 x 0.93            = 0.186
    w_info   = 0.30 - 0.186           = 0.114
    w_loop   = 0.20 + 0.186           = 0.386

Or loop_closure vaut 0.000 pour TOUS les candidats sur ces dix-neuf
decisions. Le terme w_loop * 0 ne classe rien. Le classement s'est donc
joue entre :

    w_info  * information_gain   ->  au plus 0.114 x 0.81 = 0.092
    w_safety * safety            ->  jusqu'a 0.15
    - w_distance * distance      ->  jusqu'a -0.15

Autrement dit : pendant les deux tiers du run, la securite et la
distance ont pese plus lourd que le gain d'information, sur une methode
dont le critere dominant devait etre l'exploration avec un poids
nominal de 0.30.

La mesure le confirme : weighted_1 a visite 111 cellules de 10 m, le
moins des cinq strategies (122 a 126 ailleurs), avec une etendue de
gain d'information de 0.082 contre 0.24 a 0.50 pour les autres.

LA CAUSE
--------
    transfer = min(
        weights['localization_uncertainty'] * uncertainty,
        weights['information_gain']
    )

Le transfert ne depend que de l'incertitude. Il ne consulte jamais les
candidats pour savoir si se recaler est seulement possible. L'intention
-- "quand la localisation se degrade, privilegier le recalage plutot
que l'exploration" -- se degrade silencieusement en "quand la
localisation se degrade et qu'aucun recalage n'est possible, cesser
d'explorer sans rien obtenir en echange".

Le vehicule n'explore plus ET ne se recale pas.

LA CORRECTION
-------------
    opportunite = max(loop_closure sur les candidats du tour)
    transfer    = w_unc * uncertainty * opportunite

  - aucune opportunite (opportunite = 0) : aucun transfert. Le poids
    d'exploration reste a 0.30, ce qui est la seule chose sensee a
    faire quand se recaler est impossible.
  - opportunite maximale (1.0) : comportement actuel, inchange.
  - entre les deux : transfert proportionnel.

`scores` est deja passe a la fonction : l'information necessaire est
sous la main, elle n'etait simplement pas consultee. Aucune signature
ne change, aucun appelant n'est touche.

CE QUE CE CORRECTIF NE FAIT PAS
-------------------------------
Il ne garantit pas que 'weighted' devance les autres strategies. Il
corrige un mecanisme demontrable ligne a ligne dans le journal de
decision, independamment du resultat qu'il produit. Si apres correction
'info_gain' reste devant, c'est le resultat, et il se rapporte tel
quel.

POURQUOI LA NORMALISATION LOGARITHMIQUE N'EST PAS TOUCHEE
---------------------------------------------------------
Verification faite sur le meme run : l'incertitude BRUTE est passee de
2870 a 1770 (-38 %) pendant que l'erreur de localisation reelle passait
de 134 a 85 m (-37 %). La mesure d'incertitude suit donc l'erreur vraie
de tres pres, sans jamais voir la verite terrain. Elle est juste.

La normalisation logarithmique comprime cette baisse de 38 % en 6 %
(0.935 -> 0.878), ce qui rend l'incertitude normalisee peu sensible
dans le haut de sa plage. Mais une fois le transfert multiplie par
l'opportunite, ce n'est plus determinant : le transfert n'a lieu que
lorsqu'un recalage est possible, et dans ce cas u vaut de toute facon
"eleve". Une seconde modification simultanee rendrait la campagne
inattribuable -- on corrige une chose a la fois.

VERIFICATION
------------
    python3 -m py_compile ~/active_slam_carla/ros2_ws/src/\
active_slam_decision/active_slam_decision/strategies.py
    cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install

Puis, la campagne devant etre refaite entierement :

    cd ~/active_slam_carla/metrics
    mkdir -p campagne_v1_avant_correctif
    mv ate_*.csv campagne_*.csv campagne_v1_avant_correctif/ 2>/dev/null
    cd ~
    rm -f ~/logs_campagne2/empreinte_reference.txt
    bash ~/un_scenario.sh bilan

Le retrait de l'empreinte de reference est indispensable : le code a
change, l'ancienne empreinte ne vaut plus, et le script refuserait de
demarrer. La nouvelle sera etablie au premier run.

Usage :
    python3 patch_transfert_opportunite.py
"""

import io
import os
import shutil
import sys
import time

CIBLE = os.path.expanduser(
    "~/active_slam_carla/ros2_ws/src/active_slam_decision/"
    "active_slam_decision/strategies.py"
)


ANCRE_DOC = (
    "    - u = 0 (localisation fiable)    -> poids nominaux, priorite a l'exploration\n"
    "    - u -> 1 (localisation degradee) -> priorite a la fermeture de boucle\n"
    "\n"
    "    La somme w_info + w_loop reste constante : c'est un transfert, pas\n"
    "    un ajout. `weight_localization_uncertainty` fixe l'amplitude\n"
    "    maximale de ce transfert.\n"
    "\n"
    "    Exemple avec les poids par defaut (info=0.30, loop=0.20, unc=0.20) :\n"
    "        u = 0.0  ->  w_info = 0.30, w_loop = 0.20\n"
    "        u = 0.5  ->  w_info = 0.20, w_loop = 0.30\n"
    "        u = 1.0  ->  w_info = 0.10, w_loop = 0.40\n"
)

REMPLACEMENT_DOC = (
    "    - u = 0 (localisation fiable)    -> poids nominaux, priorite a l'exploration\n"
    "    - u -> 1 (localisation degradee) -> priorite a la fermeture de boucle\n"
    "\n"
    "    ... mais seulement dans la mesure ou une fermeture de boucle est\n"
    "    REELLEMENT possible ce tour-ci. Le transfert est donc module par\n"
    "    `opportunite`, le meilleur potentiel de fermeture de boucle offert\n"
    "    par les candidats de la decision en cours.\n"
    "\n"
    "    Sans ce facteur, le transfert avait lieu meme quand aucun candidat\n"
    "    n'offrait de recalage : le poids retire a l'exploration etait alors\n"
    "    donne a un critere nul pour tous les candidats, donc perdu. Mesure\n"
    "    sur weighted_1 (campagne du 3 septembre), decisions 10 a 28 :\n"
    "    u ~ 0.93 et loop_closure = 0.000 pour tous les candidats, soit\n"
    "    w_info ramene de 0.30 a 0.114 pendant dix-neuf decisions. La\n"
    "    securite (0.15) et la distance (0.15) pesaient alors plus lourd que\n"
    "    le gain d'information sur une methode censee explorer.\n"
    "\n"
    "    La somme w_info + w_loop reste constante : c'est un transfert, pas\n"
    "    un ajout. `weight_localization_uncertainty` fixe l'amplitude\n"
    "    maximale de ce transfert.\n"
    "\n"
    "    Exemple avec les poids par defaut (info=0.30, loop=0.20, unc=0.20)\n"
    "    et une opportunite maximale (un candidat a loop_closure = 1.0) :\n"
    "        u = 0.0  ->  w_info = 0.30, w_loop = 0.20\n"
    "        u = 0.5  ->  w_info = 0.20, w_loop = 0.30\n"
    "        u = 1.0  ->  w_info = 0.10, w_loop = 0.40\n"
    "\n"
    "    Avec opportunite = 0.0, quelle que soit l'incertitude :\n"
    "        w_info = 0.30, w_loop = 0.20   (aucun transfert)\n"
)


ANCRE_CODE = (
    "    transfer = min(\n"
    "        weights['localization_uncertainty'] * uncertainty,\n"
    "        weights['information_gain']\n"
    "    )\n"
)

REMPLACEMENT_CODE = (
    "    # Meilleur potentiel de fermeture de boucle offert par les\n"
    "    # candidats de CETTE decision. Un transfert de poids vers un\n"
    "    # critere nul pour tous les candidats ne change pas leur\n"
    "    # classement entre eux : il retire simplement du poids au seul\n"
    "    # critere exploratoire qui, lui, discrimine.\n"
    "    opportunite = max(\n"
    "        (scores[c.id]['loop_closure'] for c in candidates),\n"
    "        default=0.0\n"
    "    )\n"
    "\n"
    "    transfer = min(\n"
    "        weights['localization_uncertainty'] * uncertainty * opportunite,\n"
    "        weights['information_gain']\n"
    "    )\n"
)


def main():
    if not os.path.exists(CIBLE):
        raise SystemExit("Fichier introuvable : %s" % CIBLE)

    src = io.open(CIBLE, encoding="utf-8").read()

    if "opportunite" in src:
        raise SystemExit(
            "Le correctif semble deja applique ('opportunite' present).\n"
            "Aucune modification effectuee."
        )

    for nom, ancre in (("docstring", ANCRE_DOC), ("code", ANCRE_CODE)):
        n = src.count(ancre)
        if n != 1:
            raise SystemExit(
                "ANCRE '%s' TROUVEE %d FOIS (attendu 1).\n"
                "Aucune modification effectuee." % (nom, n)
            )

    src = src.replace(ANCRE_DOC, REMPLACEMENT_DOC)
    src = src.replace(ANCRE_CODE, REMPLACEMENT_CODE)

    sauvegarde = "%s.bak_opportunite_%s" % (
        CIBLE, time.strftime("%Y%m%d_%H%M%S")
    )
    shutil.copy2(CIBLE, sauvegarde)
    io.open(CIBLE, "w", encoding="utf-8").write(src)

    print("Transfert de poids module par l'opportunite reelle de")
    print("fermeture de boucle.")
    print("sauvegarde : %s" % os.path.basename(sauvegarde))
    print("")
    print("Verifier puis reconstruire :")
    print("  python3 -m py_compile %s" % CIBLE)
    print("  cd ~/active_slam_carla/ros2_ws && colcon build --symlink-install")
    print("")
    print("Puis remettre la campagne a zero :")
    print("  cd ~/active_slam_carla/metrics")
    print("  mkdir -p campagne_v1_avant_correctif")
    print("  mv ate_*.csv campagne_*.csv campagne_v1_avant_correctif/ 2>/dev/null")
    print("  cd ~")
    print("  rm -f ~/logs_campagne2/empreinte_reference.txt")
    print("  bash ~/un_scenario.sh bilan")
    return 0


if __name__ == "__main__":
    sys.exit(main())
