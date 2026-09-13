"""
Strategies de selection de la destination suivante, utilisees pour
l'analyse comparative demandee dans le sujet de stage :

  - 'weighted'  : fonction de decision ponderee multi-criteres (methode proposee)
  - 'random'    : selection aleatoire parmi les candidats
  - 'distance'  : plus proche candidat uniquement
  - 'info_gain' : gain d'information uniquement

Chaque fonction prend la liste des candidats (Candidate) et le dict
`scores` produit par DecisionMaker._evaluate_candidates (indexe par
candidate.id) et retourne le Candidate choisi.

La strategie 'weighted' recoit en plus l'incertitude de localisation
courante, qui module dynamiquement la ponderation (voir select_weighted).
"""

import random


def select_random(candidates):
    return random.choice(candidates)


def select_by_distance(candidates, scores):
    return min(candidates, key=lambda c: scores[c.id]['distance'])


def select_by_information_gain(candidates, scores):
    return max(candidates, key=lambda c: scores[c.id]['information_gain'])


def select_weighted(candidates, scores, weights, uncertainty=0.0):
    """
    Fonction de decision ponderee "consciente de l'incertitude".

    L'incertitude de localisation courante (`uncertainty`, normalisee
    entre 0 et 1) n'est pas un terme additif : c'est un facteur qui
    redistribue le poids entre le gain d'information (explorer du
    nouveau) et le potentiel de fermeture de boucle (revenir se
    recaler).

    - u = 0 (localisation fiable)    -> poids nominaux, priorite a l'exploration
    - u -> 1 (localisation degradee) -> priorite a la fermeture de boucle

    ... mais seulement dans la mesure ou une fermeture de boucle est
    REELLEMENT possible ce tour-ci. Le transfert est donc module par
    `opportunite`, le meilleur potentiel de fermeture de boucle offert
    par les candidats de la decision en cours.

    Sans ce facteur, le transfert avait lieu meme quand aucun candidat
    n'offrait de recalage : le poids retire a l'exploration etait alors
    donne a un critere nul pour tous les candidats, donc perdu. Mesure
    sur weighted_1 (campagne du 3 septembre), decisions 10 a 28 :
    u ~ 0.93 et loop_closure = 0.000 pour tous les candidats, soit
    w_info ramene de 0.30 a 0.114 pendant dix-neuf decisions. La
    securite (0.15) et la distance (0.15) pesaient alors plus lourd que
    le gain d'information sur une methode censee explorer.

    La somme w_info + w_loop reste constante : c'est un transfert, pas
    un ajout. `weight_localization_uncertainty` fixe l'amplitude
    maximale de ce transfert.

    Exemple avec les poids par defaut (info=0.30, loop=0.20, unc=0.20)
    et une opportunite maximale (un candidat a loop_closure = 1.0) :
        u = 0.0  ->  w_info = 0.30, w_loop = 0.20
        u = 0.5  ->  w_info = 0.20, w_loop = 0.30
        u = 1.0  ->  w_info = 0.10, w_loop = 0.40

    Avec opportunite = 0.0, quelle que soit l'incertitude :
        w_info = 0.30, w_loop = 0.20   (aucun transfert)

    La distance et la securite ne dependent pas de l'incertitude : ce
    sont des contraintes de faisabilite, pas des criteres d'exploration.
    """
    # Meilleur potentiel de fermeture de boucle offert par les
    # candidats de CETTE decision. Un transfert de poids vers un
    # critere nul pour tous les candidats ne change pas leur
    # classement entre eux : il retire simplement du poids au seul
    # critere exploratoire qui, lui, discrimine.
    opportunite = max(
        (scores[c.id]['loop_closure'] for c in candidates),
        default=0.0
    )

    # --- Ablation possible du critere de fermeture de boucle ---
    #
    # Quand weight_loop_closure vaut 0, le critere est retire de la
    # decision : le transfert n'a plus de destinataire et ne doit
    # pas s'executer.
    #
    # Sans cette garde, mettre le poids a zero ne suffirait PAS :
    # le transfert lit le SCORE des candidats, pas le poids. Il
    # continuerait a retirer du poids au gain d'information pour le
    # verser sur w_loop = 0 + transfer, et le vehicule resterait
    # tire vers les zones deja cartographiees. L'ablation serait
    # annulee en silence.
    #
    # Avec un poids strictement positif, le comportement est
    # INCHANGE : les vingt runs apparies et la campagne de 25 runs
    # deja archives restent comparables.
    if weights['loop_closure'] > 0.0:
        transfer = min(
            weights['localization_uncertainty'] * uncertainty
            * opportunite,
            weights['information_gain']
        )
    else:
        transfer = 0.0
    w_info = weights['information_gain'] - transfer
    w_loop = weights['loop_closure'] + transfer

    def total_score(c):
        s = scores[c.id]
        return (
            w_info * s['information_gain']
            + w_loop * s['loop_closure']
            + weights['safety'] * s['safety']
            - weights['distance'] * s['distance_normalized']
        )

    return max(candidates, key=total_score)
