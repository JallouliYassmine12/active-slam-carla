#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Campagne G : erreur de localisation (ATE) par graine, pour les trois bras.

Lit exactement les memes fichiers que resultats_G.py :
    ~/active_slam_carla/metrics/ate_<bras>G_<graine>.csv
colonnes : timestamp,gt_x,gt_y,est_x,est_y,error,z_drift

L'ATE d'un run est la moyenne de la norme || pose_vraie - pose_estimee ||
sur les echantillons valides. La mediane est donnee a cote : la moyenne
d'un bras peut etre dominee par un seul run divergent, et le rapport doit
citer les deux.

Tests des signes unilateraux exacts, fonction identique a resultats_G.py.
Seules les graines 1 a 5 entrent dans les tests : ce sont celles du plan
pre-enregistre. Les runs supplementaires sont listes a part, sans test.

Sortie : tableau a l'ecran + ~/logs_campagne2/resultats_ate_G.csv
"""
import csv
import math
import os
import re

BASE = os.environ.get("ACTIVE_SLAM_METRICS",
                      os.path.expanduser("~/active_slam_carla/metrics"))
SORTIE = os.path.expanduser("~/logs_campagne2/resultats_ate_G.csv")
BRAS = ["weighted", "random", "predefinie"]
GRAINES = [1, 2, 3, 4, 5]          # plan pre-enregistre
FACTEUR_DIVERGENCE = 3.0           # run divergent si ATE > 3 x mediane du bras


# --------------------------------------------------------------- outils

def mediane(v):
    s = sorted(v)
    n = len(s)
    if n == 0:
        return float("nan")
    m = n // 2
    return s[m] if n % 2 else (s[m - 1] + s[m]) / 2.0


def signes(paires, plus_petit_est_mieux=True):
    """paires : liste de (a, b). Retourne (victoires_de_a, n_utile, p)."""
    if plus_petit_est_mieux:
        gagne = sum(1 for a, b in paires if a < b)
        perd = sum(1 for a, b in paires if a > b)
    else:
        gagne = sum(1 for a, b in paires if a > b)
        perd = sum(1 for a, b in paires if a < b)
    n = gagne + perd
    if n == 0:
        return 0, 0, 1.0
    k = max(gagne, perd)
    p = sum(math.comb(n, i) for i in range(k, n + 1)) / 2.0 ** n
    return gagne, n, p


def verdict(a, b, gagne, n, p):
    """Nomme le bras gagnant. Un compteur seul se lit a l'envers."""
    if n == 0:
        return "aucune graine departageable"
    if gagne * 2 > n:
        vain, k = a, gagne
    elif gagne * 2 < n:
        vain, k = b, n - gagne
    else:
        return "egalite %d/%d  p=%.3f" % (gagne, n, p)
    note = "CONCLUANT" if p < 0.05 else ("tendance" if k == n else "non concluant")
    return "%s meilleur %d/%d  p=%.3f  %s" % (vain, k, n, p, note)


def mesure(nom):
    """Retourne le dictionnaire de mesures d'un run, ou None."""
    f = os.path.join(BASE, "ate_%s.csv" % nom)
    if not os.path.exists(f):
        return None
    err, ecart_colonne = [], []
    with open(f, newline='') as fh:
        for l in csv.DictReader(fh):
            try:
                gx, gy = float(l['gt_x']), float(l['gt_y'])
                ex, ey = float(l['est_x']), float(l['est_y'])
            except (KeyError, TypeError, ValueError):
                continue
            # meme convention que resultats_G.py : la verite terrain a
            # (0,0) signale un echantillon avant reception du premier TF
            if gx == 0.0 and gy == 0.0:
                continue
            e = math.hypot(gx - ex, gy - ey)
            err.append(e)
            try:
                ecart_colonne.append(abs(e - float(l['error'])))
            except (KeyError, TypeError, ValueError):
                pass
    if len(err) < 2:
        return None
    controle = max(ecart_colonne) if ecart_colonne else 0.0
    return {
        "n": len(err),
        "moyenne": sum(err) / len(err),
        "mediane": mediane(err),
        "rmse": math.sqrt(sum(e * e for e in err) / len(err)),
        "max": max(err),
        "controle": controle,
    }


def fmt(x, l=13, d=2):
    return ("%*.*f" % (l, d, x)) if x == x else "%*s" % (l, "-")


# --------------------------------------------------------------- lecture

R = {b: {} for b in BRAS}
for b in BRAS:
    for g in GRAINES:
        m = mesure("%sG_%d" % (b, g))
        if m:
            R[b][g] = m

# runs presents sur le disque mais hors plan pre-enregistre
SUPP = []
for f in sorted(os.listdir(BASE)):
    m = re.match(r"^ate_(%s)G_(\d+)\.csv$" % "|".join(BRAS), f)
    if m and int(m.group(2)) not in GRAINES:
        d = mesure("%sG_%s" % (m.group(1), m.group(2)))
        if d:
            SUPP.append((m.group(1), int(m.group(2)), d))


# --------------------------------------------------------------- tableau

print()
print("ERREUR DE LOCALISATION (ATE) PAR RUN - CAMPAGNE G  [m]")
print("Plan pre-enregistre : graines 1 a 5, trois bras, 15 runs.")
print()
print("%7s" % "graine" + "".join("%13s" % b for b in BRAS))
print("-" * (7 + 13 * len(BRAS)))
for g in GRAINES:
    ligne = "%7d" % g
    for b in BRAS:
        ligne += fmt(R[b][g]["moyenne"]) if g in R[b] else "%13s" % "-"
    print(ligne)
print("-" * (7 + 13 * len(BRAS)))

for etiq, cle in (("moyenne", "moyenne"), ("mediane", "mediane")):
    ligne = "%7s" % etiq
    for b in BRAS:
        v = [R[b][g]["moyenne"] for g in sorted(R[b])]
        if etiq == "moyenne":
            ligne += fmt(sum(v) / len(v)) if v else "%13s" % "-"
        else:
            ligne += fmt(mediane(v)) if v else "%13s" % "-"
    print(ligne)
print()

# --------------------------------------------------------- divergences

alerte = []
for b in BRAS:
    v = [R[b][g]["moyenne"] for g in sorted(R[b])]
    if len(v) < 3:
        continue
    med = mediane(v)
    for g in sorted(R[b]):
        if R[b][g]["moyenne"] > FACTEUR_DIVERGENCE * med:
            alerte.append((b, g, R[b][g]["moyenne"], med))

if alerte:
    print("RUNS DIVERGENTS (ATE > %.0f x la mediane du bras)" % FACTEUR_DIVERGENCE)
    for b, g, v, med in alerte:
        print("  %s graine %d : %.2f m  (mediane du bras %.2f m)" % (b, g, v, med))
    print("  -> citer la mediane a cote de la moyenne dans le rapport.")
    print("  -> le test des signes n'est pas affecte : il ne lit que l'ordre.")
    print()

# --------------------------------------------------------- controle csv

pires = [(b, g, R[b][g]["controle"]) for b in BRAS for g in sorted(R[b])]
pire = max((c for _, _, c in pires), default=0.0)
if pire > 0.01:
    print("AVERTISSEMENT : la colonne 'error' des CSV s'ecarte de la norme")
    print("  recalculee de %.3f m au maximum. Les valeurs ci-dessus sont" % pire)
    print("  recalculees depuis gt_x/gt_y/est_x/est_y, pas lues telles quelles.")
    print()

# ---------------------------------------------------------------- tests

TESTS = [("PRINCIPALE ", "weighted", "predefinie"),
         ("controle   ", "random", "predefinie"),
         ("secondaire ", "weighted", "random")]

print("COMPARAISONS PRE-ENREGISTREES  (ATE : plus petit est meilleur)")
print()
for etiq, a, b in TESTS:
    com = sorted(set(R[a]) & set(R[b]))
    if not com:
        print("  %s %s vs %s : aucune graine complete" % (etiq, a, b))
        continue
    paires = [(R[a][g]["moyenne"], R[b][g]["moyenne"]) for g in com]
    k, n, p = signes(paires)
    ma = sum(x for x, _ in paires) / len(paires)
    mb = sum(y for _, y in paires) / len(paires)
    print("  %s %10s %8.2f  vs %10s %8.2f   ->  %s"
          % (etiq, a, ma, b, mb, verdict(a, b, k, n, p)))
print()

fait = sum(len(R[b]) for b in BRAS)
print("  %d / 15 runs du plan pre-enregistre" % fait)
if fait < 15:
    print("  Resultats PARTIELS : le verdict se lit a 15 runs.")

if SUPP:
    print()
    print("RUNS SUPPLEMENTAIRES (hors plan pre-enregistre, exclus des tests)")
    for b, g, d in SUPP:
        print("  %-11s graine %-2d : %6.2f m  (%d echantillons)"
              % (b, g, d["moyenne"], d["n"]))
    print("  Les inclure apres coup invaliderait la pre-enregistration.")

# --------------------------------------------------------------- sortie

os.makedirs(os.path.dirname(SORTIE), exist_ok=True)
with open(SORTIE, "w", newline='') as fh:
    w = csv.writer(fh)
    w.writerow(["bras", "graine", "plan", "n_echantillons",
                "ate_moyenne_m", "ate_mediane_m", "ate_rmse_m",
                "ate_max_m", "divergent"])
    div = {(b, g) for b, g, _, _ in alerte}
    for b in BRAS:
        for g in sorted(R[b]):
            d = R[b][g]
            w.writerow([b, g, "pre-enregistre", d["n"],
                        "%.4f" % d["moyenne"], "%.4f" % d["mediane"],
                        "%.4f" % d["rmse"], "%.4f" % d["max"],
                        "oui" if (b, g) in div else "non"])
    for b, g, d in SUPP:
        w.writerow([b, g, "supplementaire", d["n"],
                    "%.4f" % d["moyenne"], "%.4f" % d["mediane"],
                    "%.4f" % d["rmse"], "%.4f" % d["max"], ""])

print()
print("  ecrit : %s" % SORTIE)
