# INTRODUCTION

Objectif du programme : Ce projet a pour but de piloter les moteurs à courant continu de notre PAMI en utilisant la modulation de largeur d'impulsion (PWM) sur un microcontrôleur Raspberry Pi Pico. Nous cherchons également à acquérir les données de l'environnement via un capteur à ultrasons et des codeurs incrémentaux afin d'assurer une sécurité de mouvement basique.

Fonctionnement général : Notre code initialise les périphériques matériels du Pico puis lance une boucle principale. Dans cette boucle, le robot avance à une vitesse définie par défaut. Le programme lit en permanence la distance mesurée par le capteur à ultrasons. Si nous détectons un obstacle à une distance inférieure à 5 cm, nous coupons l'alimentation des moteurs immédiatement. Le mouvement reprend automatiquement dès que l'obstacle est retiré. Parallèlement, nous envoyons les données brutes des capteurs (tics des codeurs et distance en cm) sur le port série pour permettre le suivi en temps réel sur un ordinateur.

# ORGANISATION DU PROJET

Liste des fichiers source et header et leur rôle : 
main.c : Fichier principal contenant notre boucle infinie, la logique de décision (arrêt si obstacle) et l'appel aux fonctions d'initialisation. 
moteurs.c et moteurs.h : Ces fichiers gèrent la configuration de nos broches PWM et les fonctions de contrôle de direction et de vitesse des moteurs. 
codeur.c et codeur.h : Ils contiennent le code nécessaire à la lecture des signaux des codeurs incrémentaux pour mesurer la rotation des roues. 
ultrason.c et ultrason.h : Ils gèrent l'envoi du signal de déclenchement (trigger) et la mesure de la durée du retour (echo) pour le capteur de distance.

# DESCRIPTION DES FONCTIONS PRINCIPALES DU PROGRAMME

Explications techniques : Notre projet repose sur une architecture modulaire où chaque capteur ou actionneur possède son propre fichier de pilote. Le contrôle des moteurs est réalisé en boucle ouverte : nous envoyons une consigne PWM fixe sans asservissement pour le moment. La lecture des codeurs permet de vérifier que les roues tournent, mais nous ne l'utilisons pas encore pour corriger la trajectoire.

Les choix que nous avons faits : Nous avons choisi de séparer la logique de détection d'obstacle de la logique de mouvement pour garder le code lisible. La sécurité est prioritaire : nous vérifions la condition d'arrêt à chaque itération de la boucle principale pour garantir une réaction rapide.

Formules utilisées Pour le calcul de la distance avec le capteur à ultrasons, nous utilisons la formule physique standard : Distance = (Temps x Vitesse du Son) / 2. Nous avons approximé la vitesse du son à 340 mètres par seconde dans l'air.

# INSTRUCTIONS D UTILISATION

Comment compiler ou lancer le programme

Ouvrir le projet dans VS Code.

Etre bien sûr que le Pico : Quick access est reconnue votre board ( regarder si N/A ou Pico )

Sélectionner le kit de compilation ARM GCC approprié via l'extension CMake.
ensuite :
- Cmake : Select a kit
- Cmake : configurer
- Cmake : Build (pour avoir le fichier build)

Lancer la compilation pour générer le fichier .uf2.

Connecter le Raspberry Pi Pico en mode BOOTSEL et copier le fichier généré dans le lecteur.

Paramètres éventuels Nous avons défini la vitesse des moteurs (rapport cyclique PWM) et la distance de sécurité (actuellement 5 cm) par des constantes au début des fichiers sources. Ces valeurs peuvent être modifiées avant la compilation.

Dépendances nécessaires Le projet nécessite le SDK Raspberry Pi Pico (version 2.x), l'environnement CMake, le compilateur arm-none-eabi-gcc et l'extension VS Code associée.

# TESTS ET VALIDATIONS

Comment nous avons vérifié que notre programme fonctionnait Nous avons effectué la validation par étapes :

Affichage des valeurs des codeurs sur le moniteur série en tournant les roues manuellement.

Vérification de la distance affichée par l'ultrason face à un obstacle fixe mesuré à la règle.

Test fonctionnel complet : nous avons posé le robot sur cales, puis au sol. Nous avons placé un objet devant lui pour confirmer l'arrêt immédiat des moteurs et leur redémarrage après retrait de l'objet.

Résultats ou observations Les moteurs répondent correctement à nos commandes PWM. La détection d'obstacle est fiable pour des surfaces planes et coupe l'alimentation comme prévu. Les données remontées sur le port série sont cohérentes.

# AMELIORATIONS POSSIBLES

Ce que nous aurions ajouté avec plus de temps L'ajout principal serait la génération de trajectoires et l'asservissement PID. Cela nous permettrait de garantir que le robot avance parfaitement droit ou tourne d'un angle précis, en corrigeant automatiquement les erreurs grâce aux données des codeurs. Nous pourrions également calculer l'odométrie (position X, Y et angle du robot) en temps réel. 

En termes d'organisation nous changerons prochainement la disposition des fichiers .h pour les mettre dans un dossier "Déclaration" afin d'améliorer la lisibilité.

# Limites actuelles du programme 

Actuellement, notre robot ne corrige pas sa trajectoire si une roue tourne plus vite que l'autre (boucle ouverte). De plus, notre fonction de lecture de l'ultrason est bloquante pendant la durée de l'écho, ce qui pourrait ralentir l'exécution du code si la distance mesurée est très grande.