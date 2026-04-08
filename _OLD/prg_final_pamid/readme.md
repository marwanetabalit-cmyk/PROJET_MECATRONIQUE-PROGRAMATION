# Robot 2 Roues - Navigation Autonome

## Introduction

### Objectif du programme
Ce programme implémente un système de navigation autonome pour un robot à deux roues piloté par un Raspberry Pi Pico. Il permet au robot de se déplacer en ligne droite vers une cible prédéfinie, tout en compensant les différences entre les moteurs et en simulant l'odométrie pour estimer sa position en temps réel.

### Fonctionnement général
Le programme gère la navigation du robot vers un point cible en utilisant :
- Une machine à états pour gérer les phases de mouvement et d’arrivée
- Une odométrie simulée pour estimer la position
- Un système d’interruption pour un contrôle temporel précis
- Un servomoteur pour une animation de célébration à l’arrivée

---

## Organisation du projet

### Structure des fichiers

| Fichier                 | Rôle                                                                 |
|-------------------------|----------------------------------------------------------------------|
| `main.cpp`              | Programme principal, machine à états et logique de navigation        |
| `hal_motors.cpp/h`      | Pilote des moteurs (contrôle PWM, direction)                         |
| `hal_servo.cpp/h`       | Contrôle du servomoteur pour l’animation                            |
| `hal_ultrasound.cpp/h`  | Capteur ultrason (actuellement désactivé)                           |
| `odometry.cpp/h`        | Simulation d’odométrie et calcul de position                         |
| `interrupt_timer.cpp/h` | Gestion des interruptions hardware pour le timing                    |
| `CMakeLists.txt`        | Configuration de compilation                                         |
| `pico_sdk_import.cmake` | Intégration du SDK Raspberry Pi Pico                                 |

### Fonctions principales

- `state_init()` : Initialise les périphériques et la position.
- `state_move()` : Gère le déplacement vers la cible avec arrêt automatique.
- `state_arrived()` : Exécute l’animation du servomoteur à l’arrivée.
- `interrupt_callback()` : Gère les interruptions timer pour la mise à jour de la machine à états.
- `odometry_update()` : Met à jour la position simulée du robot.

---

## Explications techniques

### Choix d’architecture
Le système repose sur une **machine à états** pour une gestion claire des phases de fonctionnement. L’utilisation d’**interruptions timer** remplace les `sleep_ms()` bloquants, améliorant la réactivité.

### Compensation des moteurs
Une variable `ajout_calibration` permet d’ajuster empiriquement la vitesse d’un moteur pour compenser les différences mécaniques. Cette calibration se fait manuellement en testant le déplacement sur une ligne droite.

### Odométrie simulée
En l’absence d’encodeurs réels, l’odométrie est simulée par incréments fixes (`0,5 cm par cycle`). Cette approximation permet de tester la logique de navigation avant l’ajout de capteurs réels.

### Animation avec servomoteur
À l’arrivée, le servomoteur exécute un balayage gauche–droite trois fois avant de revenir au centre.

---

## Instructions d’utilisation

### Prérequis
- Raspberry Pi Pico avec SDK installé
- Moteurs DC avec pont en H
- Servomoteur standard (50 Hz)
- Surface plane et ligne de référence pour la calibration

### Compilation
1. Cloner le projet
2. Configurer avec CMake :
```bash
mkdir build && cd build
cmake ..
make

#Calibration
Exécuter le programme avec ajout_calibration = 0

Observer la déviation après 2–3 mètres

Ajuster la valeur dans le code :

Déviation à droite → augmenter

Déviation à gauche → diminuer

Recompiler et répéter jusqu’à obtenir un déplacement droit.

#Navigation
Modifier les constantes dans main.cpp :

c
#define TARGET_X 70.0f   // en cm
#define TARGET_Y 40.0f   // en cm
Lancer le programme et suivre la sortie série (115200 bauds).

#Tests et validations
Vérifications effectuées
Initialisation correcte des périphériques (moteurs, servo)

Exécution de la machine à états sans blocage

Arrêt automatique à la distance cible (DIST_ARRIVEE = 15 cm)

Animation du servomoteur déclenchée à l’arrivée

#Observations
La calibration manuelle permet un déplacement droit acceptable

L’odométrie simulée donne une estimation cohérente pour la navigation en ligne droite

Les interruptions assurent un timing stable sans bloquer le CPU

#Limitations connues
Pas de détection d’obstacles (ultrason désactivé)

Odométrie simulée, donc précision limitée

Calibration statique nécessitant une recompilation

#Améliorations possibles
##Priorité haute
Remplacer l’odométrie simulée par des encodeurs réels

Implémenter un régulateur PID pour chaque moteur

Réactiver la détection d’obstacles avec l’ultrason

##Priorité moyenne
Interface de calibration via terminal série

Log des données de navigation pour analyse

Diagnostic automatique des défaillances moteur

##Priorité basse
Navigation avec virages et trajectoires complexes

Transmission sans fil des données (télémétrie)

Interface graphique de contrôle

#Conclusion
Ce projet démontre les bases de la navigation autonome sur un robot à deux roues, avec une approche pragmatique pour compenser les imperfections matérielles. La structure modulaire et l’utilisation d’interruptions en font une base solide pour des extensions futures, notamment l’ajout de capteurs et d’algorithmes de contrôle avancés.