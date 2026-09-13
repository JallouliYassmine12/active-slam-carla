"""
Estimation de l'incertitude de localisation courante a partir de
/localization_pose (geometry_msgs/msg/PoseWithCovarianceStamped).

On utilise la trace du bloc de covariance en position (x, y, z) comme
indicateur scalaire d'incertitude : plus elle est grande, moins la
localisation courante est fiable. Cette incertitude n'est pas calculee
"par candidat" mais sert de signal global qui module l'importance donnee
au critere de fermeture de boucle (c'est la partie "uncertainty-aware"
du sujet : quand le systeme est incertain sur sa position, il valorise
davantage les destinations qui permettent une fermeture de boucle).
"""

import math


def compute_localization_uncertainty(pose_with_covariance):
    """
    Retourne un scalaire >= 0 representant l'incertitude de position
    courante (m^2). 0 = localisation parfaitement connue / pas encore
    de message recu.
    """
    if pose_with_covariance is None:
        return 0.0

    cov = pose_with_covariance.pose.covariance  # matrice 6x6 aplatie (36 valeurs), row-major
    # Indices diagonaux pour x, y, z : 0, 7, 14
    var_x = cov[0]
    var_y = cov[7]
    var_z = cov[14]
    return var_x + var_y + var_z


def normalize_uncertainty(raw_uncertainty, reference=1.0):
    """
    Normalise l'incertitude brute (m^2) entre 0 et 1 sur une echelle
    LOGARITHMIQUE.

    Pourquoi logarithmique et non lineaire :

    L'incertitude brute est la trace de la covariance de pose, qui
    s'accumule le long de la chaine d'odometrie. Elle ne croit donc pas
    lineairement mais de facon geometrique : sur un run de deux minutes,
    les valeurs relevees vont de 2 a 5350, soit plus de trois ordres de
    grandeur. Une normalisation lineaire est structurellement incapable
    de couvrir une telle plage :

      - reference basse (15, 200)  -> le critere sature a 1.0 des les
        premieres dizaines de secondes et ne discrimine plus rien pour
        tout le reste du parcours ;
      - reference haute (5000)     -> le critere reste quasi nul
        pendant les trois quarts du run et n'influence les decisions
        qu'au tout dernier moment.

    Aucun reglage lineaire ne satisfait les deux extremites. Le
    logarithme resout le probleme en compressant l'echelle : avec
    reference = 5000 (le seuil critique), on obtient

        brute=2    -> 0.13      brute=200  -> 0.62
        brute=50   -> 0.46      brute=800  -> 0.78
        brute=100  -> 0.54      brute=5000 -> 1.00

    Le critere balaie donc reellement [0, 1] sur toute la plage de
    fonctionnement admise, ce qui est la condition pour qu'il pese sur
    les decisions du debut a la fin du run — et non pas seulement
    pendant une fenetre etroite.

    La reference peut desormais etre calee sur le seuil critique
    lui-meme : 1.0 correspond exactement a la limite au-dela de laquelle
    la localisation est jugee non fiable.
    """
    if reference <= 0:
        return 0.0

    if raw_uncertainty <= 0.0:
        return 0.0

    return min(
        math.log1p(raw_uncertainty) / math.log1p(reference),
        1.0
    )
