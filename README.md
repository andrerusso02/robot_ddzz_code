# Lancer
nohup roslaunch diffbot_bringup bringup.launch &
puis :
exit

# Etapes homologation
A faire avant
- Activer point d'accès tel andré
- Alumer RPI
   - tester ssh dernière ip utilisés : ssh ddzz@192.168.164.212
   - si pas trouvée, scan IP avec PC andre lenovo
   - eteindre la rpi

Demarrer le robot
- vérifier que la tirette est enclenchée
- vérifier que le bouton d'arrêt d'urgence est relevé

- brancher petite batterie (puissance)
- brancher grosse batterie (rpi)

- attendre que la rpi soit détectée sur le réseau (nombre d'appareils sur le point d'accès)
- ssh : ssh ddzz@192.168.164.212 (ou autre ip)
- lancer la commande :
   - nohup roslaunch diffbot_bringup bringup.launch &

- choisir la couleur avec le bouton

- [attendre que le prog soit prêt (tf publiée). Comment ?]
   - rostopic list
   - récupérer tf entre odom et base footprint : rosrun tf tf_echo odom base_footprint

- exit

- tirer sur la tirette (et là peut être le robot part)

A FAIRE ABSOLUMENT AVANT L'HOMOLOGATION
- ajouter le node homologation à bringup
- tester arrêt devant des obstacles
- tester homologation
- tester lancement avec nohup
- tester rosrun tf tf_echo odom base_footprint

- tester toute la sequence d'homologation