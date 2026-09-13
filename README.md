# Active SLAM conscient de l'incertitude sous ROS 2 et CARLA

Methode d'Active SLAM pour la cartographie autonome : un vehicule simule
choisit lui-meme ses destinations a partir de la carte qu'il est en train
de construire, au lieu de suivre un itineraire fixe.

Stage d'ingenieur — AUTRIS SAS.

## Resultat principal

Tache a zone bornee (carre de 300 m de cote, budget de 1200 m), cinq
graines appariees, plan pre-enregistre, test des signes unilateral exact.

| Metrique | `weighted` | `random` | `predefinie` | `weighted` vs `predefinie` |
|---|---|---|---|---|
| Surface cartographiee en zone (m²) | 24 720 | 24 940 | 15 140 | 5/5, p = 0,031 |
| Distance au seuil de 15 140 m² (m) | 508 | 506 | 1 141 (2 graines sur 5) | 5/5, p = 0,031 |
| Duree au seuil (s) | 74 | 74 | 128 | 5/5, p = 0,031 |
| Erreur de trajectoire, moyenne (m) | 3,45 | 3,45 | 46,78 | 5/5, p = 0,031 |
| Erreur de trajectoire, mediane (m) | 3,43 | 3,03 | 19,06 | — |
| Fermetures de boucle par run | 34,4 | 36,4 | 3,8 | 5/5, p = 0,031 |

Les trois objectifs — couverture, efficacite de deplacement, localisation —
sont ameliores simultanement.

**Limites, enoncees explicitement :**

- L'avantage depend du seuil vise. En dessous d'environ 14 000 m² de
  couverture, les deux strategies se valent en distance, et la strategie de
  reference arrive meme plus tot en temps : elle roule 50 % plus vite.
- `random` egale `weighted`. Ce qui est demontre, c'est le benefice de
  **decider dans une zone bornee**, pas celui de la ponderation
  multicritere.
- Un run de la strategie de reference a diverge (162 m d'erreur). La
  mediane est donnee a cote de la moyenne pour cette raison.
- Le comptage des fermetures de boucle n'etait pas pre-enregistre : c'est
  une analyse explicative, decidee apres lecture des erreurs de
  localisation.

## Reproduire les tableaux sans rien simuler

Les donnees brutes des quinze runs sont dans `resultats/metrics/`.

```bash
ACTIVE_SLAM_METRICS=resultats/metrics python3 analyse/resultats_G.py
ACTIVE_SLAM_METRICS=resultats/metrics python3 analyse/analyse_ate_appariee.py
ACTIVE_SLAM_METRICS=resultats/metrics python3 analyse/distance_seuil.py
```

Python 3.10, bibliotheque standard uniquement.

`analyse/fermetures_boucle.py` demande les bases RTAB-Map, trop volumineuses
pour etre publiees (environ 100 Mo par run). Son resultat est fourni dans
`resultats/resultats_fermetures_G.csv`.

## Organisation

```
ros2_ws/src/
  active_slam_decision/    generation de candidats, criteres, decision,
                           controleur Active SLAM
  carla_sensors_bridge/    pont CARLA <-> ROS 2 (LiDAR, camera, IMU)
  slam_evaluation/         chaine de reference et evaluation hors ligne
  active_slam_core/        premiere version des noeuds de decision,
                           conservee a titre historique
scripts/                   lancement d'un run, campagnes, demonstration
analyse/                   production des tableaux et des tests
patches/                   correctifs successifs appliques au cours du
                           developpement (section 3.7 du rapport)
resultats/                 donnees brutes et resultats agreges
docs/                      rapport et figures
demo/                      configuration RViz de la demonstration
```

## Dependances

- CARLA 0.9.16 (Windows), API Python, mode synchrone, Town03
- ROS 2 Humble sur Ubuntu 22.04 (WSL2)
- RTAB-Map (`rtabmap_ros`) — front-end `icp_odometry`, back-end `rtabmap`
- `imu_filter_madgwick`
- [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) —
  evalue comme point de comparaison, non inclus dans ce depot

## Lancer un run

```bash
# CARLA doit tourner sur 127.0.0.1:2000
bash scripts/un_scenario.sh weighted 1        # Active SLAM
VARIANTE=G bash scripts/un_scenario.sh predefinie 1   # reference
```

Variables : `VARIANTE` (A a G), `ZONE_HALF_SIZE` (demi-cote de la zone en
metres), `SEED_RUN` (graine de trafic).

## Reproductibilite

L'etiquette `campagne-G` marque l'etat exact du code ayant produit les
quinze runs du chapitre 4. Son empreinte SHA-256 est dans
`resultats/empreinte_reference.txt`.
