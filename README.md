# Boids — simulation de nuées (C / SDL2)

> 🇫🇷 Version française ci-dessous — 🇬🇧 English version below.

## 🇫🇷 Français

Simulation de **boids** (nuées d'oiseaux/poissons) écrite principalement en **C**
avec **SDL2**, basée sur les trois règles classiques de Reynolds :
**cohésion**, **alignement** et **séparation**.

### Points techniques
- Rendu temps réel avec SDL2 pour un grand nombre de boids (jusqu'à ~1000).
- **Hachage spatial** (découpage de l'espace en cases) pour accélérer la recherche
  des voisins et éviter le coût quadratique.
- Variantes plus avancées : ajout d'un **prédateur** et d'une **« bombe »**
  (`boids_prédateur+bombe.c`).
- Prototypes et expérimentations également en **Python** (`boids.py`, `boids_rebond.py`…).

> ⚠️ **Dossier de travail en l'état.** Ce repo rassemble plusieurs versions
> successives et expérimentations (différents `.c`, scripts Python, archives,
> exécutables), gardées telles quelles. Les fichiers les plus aboutis sont
> `boids_v2.c` et `Projet final/boids_prédateur+bombe.c`.

### Compilation (exemple)
Nécessite **SDL2**.
```bash
gcc boids_v2.c -o boids -lSDL2 -lm
./boids
```
Le dossier `Squelette de code-20251211/` contient aussi un `makefile`.

### ⚖️ Code tiers
Le sous-dossier `Squelette de code-20251211/boids-master/` est un projet **tiers**
téléchargé (implémentation C++ de référence), avec sa **propre licence** (voir
`boids-master/LICENSE`). Il est inclus à des fins de comparaison/inspiration et
reste la propriété de ses auteurs d'origine.

---

## 🇬🇧 English

A **boids** (bird/fish flocking) simulation written mainly in **C** with **SDL2**,
based on Reynolds' three classic rules: **cohesion**, **alignment** and
**separation**.

### Technical highlights
- Real-time SDL2 rendering for a large number of boids (up to ~1000).
- **Spatial hashing** (grid partitioning) to speed up neighbor lookup and avoid
  the quadratic cost.
- More advanced variants: a **predator** and a **"bomb"**
  (`boids_prédateur+bombe.c`).
- Prototypes and experiments in **Python** as well (`boids.py`, `boids_rebond.py`…).

> ⚠️ **Working folder, as-is.** This repo gathers several successive versions and
> experiments (various `.c` files, Python scripts, archives, executables), kept
> as they were. The most polished files are `boids_v2.c` and
> `Projet final/boids_prédateur+bombe.c`.

### Build (example)
Requires **SDL2**.
```bash
gcc boids_v2.c -o boids -lSDL2 -lm
./boids
```
The `Squelette de code-20251211/` folder also contains a `makefile`.

### ⚖️ Third-party code
The subfolder `Squelette de code-20251211/boids-master/` is a **third-party**
downloaded project (a reference C++ implementation) with its **own license** (see
`boids-master/LICENSE`). It is included for comparison/inspiration and remains the
property of its original authors.
