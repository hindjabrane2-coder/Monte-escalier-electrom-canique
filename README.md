# Monte-escalier electromecanique

Projet de conception d'un monte-escalier pour personne a mobilite reduite, realise chez empegec (juillet - novembre 2023). Le systeme transporte un fauteuil roulant le long d'un rail incline, entraine par un moteur a courant continu avec reducteur.

## Contenu

- `motor_sizing.py` : dimensionnement du moteur et du reducteur en fonction de la charge, de la pente et de la vitesse souhaitee. Calcul du couple, de la puissance et verification thermique.
- `stairlift_main.ino` : programme Arduino pour la commande du monte-escalier. Gestion du moteur via relais, capteurs de fin de course, boutons de commande et arret d'urgence.
- `control_panel.py` : interface de supervision en Python, communication serie avec l'Arduino, affichage de l'etat du systeme et journalisation.

## Principe de fonctionnement

Le fauteuil est fixe sur un chariot guide par un rail en profile aluminium. Un moteur DC 24V entraine le chariot via un systeme pignon-cremaillere. Deux capteurs de fin de course arretent le moteur en bout de rail. Un frein electromagnetique maintient le chariot a l'arret. L'utilisateur commande la montee ou la descente depuis un boitier filaire avec bouton homme-mort.

## Securites

- Bouton d'arret d'urgence (coupe l'alimentation puissance)
- Capteurs de fin de course haut et bas
- Detection de surintensité moteur
- Frein electromagnetique normalement ferme (s'ouvre sous tension)
- Vitesse limitee par rampe d'acceleration

## Dependances

- Arduino IDE 2.x
- Python 3.8+, pyserial
