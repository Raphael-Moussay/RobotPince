# ROBOPINCEBOT

https://wiki.fablab.sorbonne-universite.fr/BookStack/books/projets-due-2024-2025/page/projet-rob3-gaelle-raphael-sofiane-jean-yves-harold-stanislas

## Description

**ROBOPINCEBOT** est un robot mobile autonome développé dans le cadre du projet de robotique de 3e année à Polytech Sorbonne, spécialité Robotique (février – juin 2025). Le robot est conçu pour localiser, saisir, transporter et déposer un objet dans une arène délimitée à l’aide d’un bras articulé et d’une pince motorisée.

Ce projet a été réalisé au Fablab de Sorbonne Université et mobilise des compétences en mécatronique : conception mécanique (modélisation, usinage), électronique embarquée, programmation sur Arduino et mise en œuvre de capteurs.

## Objectifs

- Concevoir et fabriquer un robot mobile autonome.
- Développer un algorithme de navigation simple par capteurs ultrason.
- Implémenter un système de préhension d’objet via un bras motorisé.
- Réaliser une démonstration fonctionnelle dans un environnement contraint.

## Équipe

| Nom                  | Email                             |
|----------------------|-----------------------------------|
| Sofiane BENRABIA     | s.benavron@gmail.com              |
| Gaelle MARDIKIAN     | gaellemardikian29@gmail.com       |
| Raphaël MOUSSAY      | raphael.moussay@gmail.com         |
| Jean ORIEUX          | jeanorx@gmail.com                 |
| Stanislas PINART     | stanislas.pinart@gmail.com        |
| Yves-Harold VALCIUS  | yvesharoldvalcius11@gmail.com     |

## Matériel utilisé

- Carte Arduino UNO
- 3 moteurs MS4015 avec encodeurs
- Bus CAN (MCP2515)
- 2 capteurs à ultrasons HC-SR04
- 1 servomoteur pour la pince
- 1 bouton poussoir
- 2 roues + 1 roue folle
- Batterie
- 2 LabDecks
- Pièces usinées en MDF (laser) via fichiers DXF SolidWorks

## Fonctionnalités

- Déplacement du robot à l’aide de moteurs contrôlés par bus CAN
- Odométrie de position et d’angle
- Détection d’obstacles avec ultrasons (horizontal et vertical)
- Préhension et dépose d’un objet avec une pince articulée
- Démarrage d’une séquence automatisée via bouton poussoir

## Installation

1. **Branchements** : suivre le schéma de câblage dans `https://wiki.fablab.sorbonne-universite.fr/BookStack/books/projets-due-2024-2025/page/projet-rob3-gaelle-raphael-sofiane-jean-yves-harold-stanislas`.
2. **Chargement du code** : ouvrir `Code_Pincebot_final.ino` dans l’IDE Arduino et téléverser sur la carte.
3. **Alimentation** : connecter la batterie.
4. **Démarrage** : appuyer sur le bouton poussoir pour lancer la séquence automatique.

## Dépendances logicielles (Arduino)

- `Servo.h` – contrôle du servomoteur
- `SPI.h` – communication SPI
- `mcp_can.h` / `mcp2515_can.h` – gestion du module CAN MCP2515

## Avancement et journal de bord

Le projet s’est déroulé sur plusieurs mois selon une approche itérative : prototypage, test, ajustement mécanique, implémentation électronique, puis intégration finale.

Les activités hebdomadaires détaillées sont disponibles dans le fichier `Gantt.pdf`.

## Solutions techniques

- **Mobilité** : deux roues arrière motorisées, une roue folle avant
- **Préhension** : pince fixée sur un bras articulé
- **Capteurs** : ultrasons pour éviter les obstacles et mesurer la hauteur de l’objet
- **Commande** : via Arduino et bus CAN, odométrie implémentée par retour encodeur

## Problèmes rencontrés

- Imprécisions des capteurs ultrasons à courte distance
- Stabilité du bras (ajout d’un élastique pour équilibrer le poids)
- Calcul de la position cible pour la dépose (approximation en fonction de l’objet)

## Licence

Ce projet est réalisé à but pédagogique, dans le cadre du cursus ingénieur de Polytech Sorbonne. Toute réutilisation doit être mentionnée comme projet étudiant.

---

*Pour toute question, se référer à la documentation ou contacter l’un des membres de l’équipe.*


