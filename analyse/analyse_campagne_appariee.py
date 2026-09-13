#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analyse de la campagne APPARIEE : cinq strategies, cinq graines.

CE QUE CE SCRIPT PRODUIT
------------------------
1. Le tableau complet : ATE moyenne de chaque run, par strategie et
   par graine, avec moyenne, mediane et maximum par strategie.

2. La comparaison PRINCIPALE, pre-enregistree avant le premier run :
   'weighted' contre 'predefinie'. C'est la question du sujet -- une
   exploration active consciente de l'incertitude fait-elle mieux
   qu'un SLAM classique sur trajectoire figee.

3. Les trois comparaisons SECONDAIRES (weighted contre random,
   distance, info_gain), affichees comme descriptives et sans
   conclusion statistique. Avec quatre tests a p = 0,031, en obtenir
   un "significatif" par hasard n'a rien d'improbable ; les presenter
   au meme rang que la comparaison principale serait de la peche aux
   resultats.

4. Le taux de DIVERGENCE : runs dont l'ATE depasse 10 % du budget de
   1200 m. Une moyenne cache mal la difference entre "un peu moins
   precis" et "a perdu sa position".

5. SECURITE ET COLLISIONS, lues dans les journaux de decision. Elles
   ne concernent que les quatre strategies de decision : 'predefinie'
   ne prend aucune decision et n'ecrit pas de journal.

6. La REPRODUCTIBILITE, mesuree gratuitement. weightedB_g et
   weightedP_g sont la MEME configuration -- memes poids 0.20/0.25,
   meme graine, memes seuils 0.05 -- executees a plusieurs heures
   d'ecart. L'ecart entre les deux est le plancher de bruit du banc
   d'essai, et il dit a partir de quelle taille une difference merite
   d'etre commentee.

LE TEST UTILISE
---------------
Le test des signes, exact, unilateral, sur les paires non nulles. Il
ne suppose rien de la distribution des ecarts, ce qui compte quand on
en a cinq.

    5 ecarts de meme signe  ->  p = 0,031
    4 sur 5                 ->  p = 0,19
    3 sur 5                 ->  p = 0,50

Aucun test t n'est calcule : sur cinq paires, sa p-valeur supposerait
une normalite inverifiable.

CE QUE LE SCRIPT NE DIT PAS
---------------------------
Le resultat vaut pour les graines 1 a 5. L'appariement retire la
variabilite du trafic DE LA COMPARAISON, il ne rend pas ces cinq
tirages representatifs de tout trafic urbain possible. Les valeurs
absolues ne se comparent donc pas a celles de la campagne
preliminaire, qui tirait son trafic au hasard.

USAGE
-----
    python3 ~/analyse_campagne_appariee.py
    python3 ~/analyse_campagne_appariee.py --csv
"""

import csv
import math
import os
import sys

METRICS = os.path.expanduser("~/active_slam_carla/metrics")

STRATEGIES = ["random", "distance", "info_gain", "weighted", "predefinie"]
GRAINES = [1, 2, 3, 4, 5]

BUDGET = 1200.0
# Au-dela de 10 % du budget parcouru, la pose estimee n'est plus
# exploitable pour naviguer : ce n'est plus de l'imprecision.
SEUIL_DIVERGENCE = 0.10 * BUDGET

PRINCIPALE = ("weighted", "predefinie")
SECONDAIRES = [("weighted", "random"),
               ("weighted", "distance"),
               ("weighted", "info_gain")]


def lire_ate(nom):
    """Statistiques d'erreur d'un run, ou None s'il n'est pas archive."""
    chemin = os.path.join(METRICS, "ate_%s.csv" % nom)
    if not os.path.exists(chemin):
        return None

    erreurs = []
    with open(chemin, newline="", encoding="utf-8", errors="replace") as f:
        for ligne in csv.DictReader(f):
            try:
                gx = float(ligne["gt_x"])
                gy = float(ligne["gt_y"])
                err = float(ligne["error"])
            except (KeyError, TypeError, ValueError):
                continue
            # Echantillon d'amorcage : tout a zero, ce n'est pas une mesure.
            if gx == 0.0 and gy == 0.0 and err == 0.0:
                continue
            erreurs.append(err)

    if not erreurs:
        return None
    erreurs_triees = sorted(erreurs)
    milieu = len(erreurs_triees) // 2
    if len(erreurs_triees) % 2:
        mediane = erreurs_triees[milieu]
    else:
        mediane = 0.5 * (erreurs_triees[milieu - 1] + erreurs_triees[milieu])

    return {
        "moyenne": sum(erreurs) / len(erreurs),
        "mediane": mediane,
        "max": erreurs_triees[-1],
        "n": len(erreurs),
    }


def lire_decisions(nom):
    """Securite moyenne et collisions d'un run, depuis son journal."""
    chemin = os.path.join(METRICS, "campagne_%s.csv" % nom)
    if not os.path.exists(chemin):
        return None

    securites = []
    collisions = [0, 0, 0]
    decisions = 0
    with open(chemin, newline="", encoding="utf-8", errors="replace") as f:
        for l in csv.DictReader(f):
            decisions += 1
            try:
                securites.append(float(l["safety"]))
            except (KeyError, TypeError, ValueError):
                pass
            # Les compteurs de collision sont cumulatifs : le maximum
            # de la colonne est le total du run.
            for i, cle in enumerate(("collisions_vehicules",
                                     "collisions_pietons",
                                     "collisions_obstacles")):
                try:
                    collisions[i] = max(collisions[i], int(float(l[cle])))
                except (KeyError, TypeError, ValueError):
                    pass

    if not decisions:
        return None
    return {
        "decisions": decisions,
        "securite": sum(securites) / len(securites) if securites else None,
        "collisions": sum(collisions),
    }


def collecter(suffixe="P"):
    donnees = {}
    for s in STRATEGIES:
        donnees[s] = {}
        for g in GRAINES:
            nom = "%s%s_%d" % (s, suffixe, g)
            donnees[s][g] = {
                "nom": nom,
                "ate": lire_ate(nom),
                "dec": lire_decisions(nom),
            }
    return donnees


def moyenne(valeurs):
    return sum(valeurs) / len(valeurs) if valeurs else None


def mediane(valeurs):
    if not valeurs:
        return None
    v = sorted(valeurs)
    m = len(v) // 2
    return v[m] if len(v) % 2 else 0.5 * (v[m - 1] + v[m])


def test_des_signes(ecarts):
    """Test des signes exact, unilateral, sur les paires non nulles.

    Les ecarts sont calcules comme b - a, ou 'a' est la strategie
    nommee en premier. L'ATE est une ERREUR : plus petit vaut mieux.
    Un ecart POSITIF signifie donc que b a plus d'erreur, c'est-a-dire
    que 'a' fait mieux sur cette graine.

    Retourne (a_mieux, b_mieux, n, k, p).
    """
    a_mieux = sum(1 for e in ecarts if e > 0)
    b_mieux = sum(1 for e in ecarts if e < 0)
    n = a_mieux + b_mieux
    if n == 0:
        return 0, 0, 0, 0, 1.0
    k = max(a_mieux, b_mieux)
    p = sum(math.comb(n, i) for i in range(k, n + 1)) / (2.0 ** n)
    return a_mieux, b_mieux, n, k, p


def ecarts_apparies(donnees, a, b):
    """b - a, graine par graine. Negatif = a est meilleure."""
    paires = []
    for g in GRAINES:
        ea = donnees[a][g]["ate"]
        eb = donnees[b][g]["ate"]
        # La valeur disponible reste affichee meme quand la paire est
        # incomplete : masquer les deux ferait croire a deux runs
        # manquants la ou il n'y en a qu'un.
        paires.append((g,
                       None if ea is None else ea["moyenne"],
                       None if eb is None else eb["moyenne"]))
    return paires


# ---------------------------------------------------------------------
# Affichage
# ---------------------------------------------------------------------

def afficher_tableau(donnees):
    print("=== CAMPAGNE APPARIEE : 5 STRATEGIES x 5 GRAINES ===")
    print("")
    print("  Chaque colonne est un trafic identique pour les cinq lignes.")
    print("  ATE moyenne du run, en metres.")
    print("")
    print("  %-12s %7s %7s %7s %7s %7s   %8s %8s %8s"
          % ("strategie", "g1", "g2", "g3", "g4", "g5",
             "moyenne", "mediane", "max"))
    print("  " + "-" * 76)

    for s in STRATEGIES:
        cases = []
        valeurs = []
        for g in GRAINES:
            a = donnees[s][g]["ate"]
            if a is None:
                cases.append("-")
            else:
                cases.append("%.1f" % a["moyenne"])
                valeurs.append(a["moyenne"])
        maxi = max((donnees[s][g]["ate"]["max"]
                    for g in GRAINES if donnees[s][g]["ate"]), default=None)
        print("  %-12s %7s %7s %7s %7s %7s   %8s %8s %8s"
              % (s, cases[0], cases[1], cases[2], cases[3], cases[4],
                 "-" if not valeurs else "%.1f" % moyenne(valeurs),
                 "-" if not valeurs else "%.1f" % mediane(valeurs),
                 "-" if maxi is None else "%.1f" % maxi))
    print("")
    print("  moyenne/mediane portent sur les cinq runs ; max est la plus")
    print("  grande erreur instantanee observee, tous runs confondus.")
    print("")


def afficher_comparaison(donnees, a, b, principale):
    titre = "COMPARAISON PRINCIPALE (pre-enregistree)" if principale \
        else "%s contre %s" % (a, b)
    if principale:
        print("=== %s ===" % titre)
        print("")
        print("  %s contre %s" % (a, b))
        print("")
    else:
        print("  --- %s ---" % titre)

    paires = ecarts_apparies(donnees, a, b)
    ecarts = []

    if principale:
        print("  %6s %10s %12s %10s"
              % ("graine", a[:10], b[:12], "ecart"))
        print("  " + "-" * 42)

    for g, va, vb in paires:
        if va is None or vb is None:
            if principale:
                print("  %6d %10s %12s %10s"
                      % (g,
                         "-" if va is None else "%.1f" % va,
                         "-" if vb is None else "%.1f" % vb,
                         "incomplet"))
            continue
        e = vb - va
        ecarts.append(e)
        if principale:
            print("  %6d %10.1f %12.1f %+10.1f" % (g, va, vb, e))

    if principale:
        print("")

    if len(ecarts) < len(GRAINES):
        print("  %sRESULTAT PROVISOIRE : %d paire(s) sur %d."
              % ("" if principale else "  ", len(ecarts), len(GRAINES)))
        if not ecarts:
            return

    a_mieux, b_mieux, n, k, p = test_des_signes(ecarts)
    m = moyenne(ecarts)

    # ecart = b - a, sur une ERREUR : positif => b se trompe plus,
    # donc 'a' fait mieux sur cette graine.
    gagnant, perdant = (a, b) if a_mieux > b_mieux else (b, a)

    if principale:
        print("  Ecart moyen %s - %s : %+.1f m" % (b, a, m))
        print("  %s fait mieux sur %d graine(s) sur %d ; %s sur %d"
              % (a, a_mieux, len(ecarts), b, b_mieux))
        print("  Test des signes : %d/%d, p = %.3f" % (k, n, p))
        print("")
        if n == 0:
            print("  LECTURE : aucune difference sur ces cinq trafics.")
        elif p <= 0.05:
            print("  LECTURE : %s est systematiquement meilleure que %s"
                  % (gagnant, perdant))
            print("  sur les cinq trafics testes. C'est le resultat")
            print("  principal de la campagne.")
        elif k == n:
            print("  LECTURE : les %d ecarts vont tous dans le meme sens," % n)
            print("  en faveur de %s, mais %d paires ne suffisent pas a"
                  % (gagnant, n))
            print("  exclure le hasard. Deux graines de plus trancheraient.")
        else:
            print("  LECTURE : les ecarts ne vont pas tous dans le meme")
            print("  sens. Aucun avantage systematique n'est etabli entre")
            print("  %s et %s sur ces cinq trafics. C'est un resultat," % (a, b))
            print("  pas un echec.")
    else:
        print("    ecart moyen %+.1f m  |  %s meilleure %d/%d  |  p = %.3f"
              % (m, a, a_mieux, len(ecarts), p))


def afficher_secondaires(donnees):
    print("=== COMPARAISONS SECONDAIRES (descriptives) ===")
    print("")
    for a, b in SECONDAIRES:
        afficher_comparaison(donnees, a, b, principale=False)
    print("")
    print("  Ces trois comparaisons n'etaient PAS pre-enregistrees. Avec")
    print("  quatre tests, en obtenir un a p < 0,05 par hasard est")
    print("  probable. Elles se lisent comme des indications, pas comme")
    print("  des conclusions.")
    print("")


def afficher_divergences(donnees):
    print("=== DIVERGENCES ===")
    print("")
    print("  Runs dont l'ATE moyenne depasse %.0f m, soit 10 %% du budget"
          % SEUIL_DIVERGENCE)
    print("  parcouru. Au-dela, la pose estimee n'est plus exploitable")
    print("  pour naviguer : ce n'est plus de l'imprecision.")
    print("")
    for s in STRATEGIES:
        divergents = [g for g in GRAINES
                      if donnees[s][g]["ate"]
                      and donnees[s][g]["ate"]["moyenne"] > SEUIL_DIVERGENCE]
        faits = sum(1 for g in GRAINES if donnees[s][g]["ate"])
        detail = "" if not divergents else \
            "   (graines %s)" % ", ".join(str(g) for g in divergents)
        print("  %-12s %d / %d%s" % (s, len(divergents), faits, detail))
    print("")


def afficher_securite(donnees):
    print("=== SECURITE ET COLLISIONS ===")
    print("")
    print("  Lues dans les journaux de decision. 'predefinie' n'en ecrit")
    print("  pas : elle ne prend aucune decision.")
    print("")
    print("  %-12s %10s %12s %12s"
          % ("strategie", "decisions", "securite", "collisions"))
    print("  " + "-" * 48)
    for s in STRATEGIES:
        decs, secs, cols = [], [], []
        for g in GRAINES:
            d = donnees[s][g]["dec"]
            if d is None:
                continue
            decs.append(d["decisions"])
            cols.append(d["collisions"])
            if d["securite"] is not None:
                secs.append(d["securite"])
        if not decs:
            print("  %-12s %10s %12s %12s" % (s, "-", "-", "-"))
            continue
        print("  %-12s %10.0f %12s %12d"
              % (s, moyenne(decs),
                 "-" if not secs else "%.3f" % moyenne(secs),
                 sum(cols)))
    print("")
    print("  securite = moyenne du critere sur les decisions du run")
    print("  collisions = total sur les cinq runs")
    print("")


def afficher_reproductibilite():
    """weightedB_g et weightedP_g : meme configuration, deux executions."""
    print("=== REPRODUCTIBILITE DU BANC D'ESSAI ===")
    print("")
    print("  weightedB_g et weightedP_g sont la MEME configuration :")
    print("  poids 0.20 / 0.25, meme graine, seuils 0.05. Deux")
    print("  executions a plusieurs heures d'ecart. L'ecart entre elles")
    print("  ne vient ni du reglage ni du trafic -- c'est le bruit")
    print("  propre du dispositif.")
    print("")
    print("  %6s %10s %10s %10s" % ("graine", "B (m)", "P (m)", "ecart"))
    print("  " + "-" * 40)

    ecarts = []
    relatifs = []
    for g in GRAINES:
        b = lire_ate("weightedB_%d" % g)
        p = lire_ate("weightedP_%d" % g)
        if b is None or p is None:
            print("  %6d %10s %10s %10s"
                  % (g,
                     "-" if b is None else "%.1f" % b["moyenne"],
                     "-" if p is None else "%.1f" % p["moyenne"],
                     "incomplet"))
            continue
        e = p["moyenne"] - b["moyenne"]
        ecarts.append(abs(e))
        if b["moyenne"] > 0:
            relatifs.append(abs(e) / b["moyenne"])
        print("  %6d %10.1f %10.1f %+10.1f"
              % (g, b["moyenne"], p["moyenne"], e))

    print("")
    if ecarts:
        print("  Ecart absolu moyen : %.1f m" % moyenne(ecarts))
        if relatifs:
            print("  soit %.0f %% de la valeur mesuree"
                  % (100 * moyenne(relatifs)))
        print("")
        print("  LECTURE : une difference entre deux strategies plus")
        print("  petite que cet ordre de grandeur ne se distingue pas du")
        print("  bruit d'execution. C'est la resolution du banc d'essai,")
        print("  et elle se cite dans le rapport.")
    print("")


def exporter(donnees):
    sortie = csv.writer(sys.stdout)
    sortie.writerow(["strategie", "graine", "run", "ate_moyenne",
                     "ate_mediane", "ate_max", "echantillons",
                     "decisions", "securite", "collisions"])
    for s in STRATEGIES:
        for g in GRAINES:
            d = donnees[s][g]
            a = d["ate"] or {}
            dec = d["dec"] or {}
            sortie.writerow([
                s, g, d["nom"],
                "" if not a else "%.4f" % a["moyenne"],
                "" if not a else "%.4f" % a["mediane"],
                "" if not a else "%.4f" % a["max"],
                a.get("n", ""),
                dec.get("decisions", ""),
                "" if dec.get("securite") is None
                else "%.4f" % dec["securite"],
                dec.get("collisions", ""),
            ])


def main():
    if not os.path.isdir(METRICS):
        print("Dossier introuvable : %s" % METRICS)
        return 1

    donnees = collecter()

    if "--csv" in sys.argv[1:]:
        exporter(donnees)
        return 0

    manquants = [donnees[s][g]["nom"]
                 for s in STRATEGIES for g in GRAINES
                 if donnees[s][g]["ate"] is None]

    afficher_tableau(donnees)
    afficher_comparaison(donnees, PRINCIPALE[0], PRINCIPALE[1],
                         principale=True)
    print("")
    afficher_secondaires(donnees)
    afficher_divergences(donnees)
    afficher_securite(donnees)
    afficher_reproductibilite()

    if manquants:
        print("=== RUNS MANQUANTS ===")
        print("")
        for m in manquants:
            print("  %s" % m)
        print("")
        print("  L'analyse ci-dessus est PROVISOIRE tant qu'ils manquent.")
        print("")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BrokenPipeError:
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, sys.stdout.fileno())
        sys.exit(0)
