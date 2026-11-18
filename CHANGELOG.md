# Historique des versions

Tags `v<majeur>.<mineur>[.<correctif>]`, version Zigbee `0xMMmmPP00`.

## v1.5

- LED éteinte en fonctionnement normal (plus de vert fixe une fois appairé) : elle ne s'allume que pour un événement
- Flash vert à chaque envoi (au lieu de bleu), flash rouge en cas d'échec d'envoi
- Non connecté : rouge clignotant. Bouton maintenu : orange puis violet à 3 s (réinitialisation)
- OTA en cours en bleu rapide (au lieu de cyan)
- Couleurs justes sur la Waveshare ESP32-H2-Zero : sa LED WS2812 a le rouge et le vert inversés par rapport au réglage « ESP32H2 Dev Module ». C'est ce qui affichait le vert fixe « appairé » de la v1.4 en rouge. La fonction `led()` inverse g et r

## v1.4

- Seuil d'envoi relevé de 0,1 °C à 0,2 °C pour réduire le nombre d'envois. Le DS18B20 mesurant par pas de 0,0625 °C, un envoi part en pratique à 0,25 °C d'écart (au lieu de 0,125 °C)

## v1.3

- Lecture des deux sondes en continu au lieu de toutes les 10 s : environ une lecture toutes les 0,8 s, le maximum en 12 bits (conversion de 750 ms). Une variation de 0,1 °C part donc en moins de 2 s
- Moniteur série : les lectures sans envoi ne sont affichées que toutes les 10 s

## v1.2

- Envoi sur variation : chaque température n'est envoyée que si elle a changé d'au moins 0,1 °C depuis le dernier envoi
- Envoi de sécurité au moins une fois par heure si la température ne bouge pas
- Suppression de l'envoi forcé toutes les 30 s ; mesure toutes les 10 s
- Reporting automatique de la pile Zigbee neutralisé (`setReporting(0, 65000, 600)`), convertisseur Z2M aligné
- L'appui court sur le bouton force toujours l'envoi immédiat des deux températures

## v1.1

Première mise à jour OTA.

- Tolérance de température annoncée à 0,1 °C (attribut ZCL Tolerance, informatif) au lieu de 0,5 °C

## v1.0

Première version avec client OTA, à flasher en USB.

- Routeur Zigbee (ZCZR, partitions `zigbee_zczr`) : la compilation échoue si un autre mode est sélectionné

- 2 sondes DS18B20 sur deux bus 1-Wire séparés : entrée du radiateur (GPIO4, endpoint 10) et sortie (GPIO5, endpoint 11)
- Mesure et envoi des deux températures toutes les 30 s, filtrage des lectures invalides
- Bouton d'appairage sur GPIO11 (appui 3 s) et bouton BOOT
- LED d'état : appairé, non appairé, envoi, OTA en cours
- Client OTA Zigbee (fabricant 0x1337, type 0x0002) avec garde-fou : redémarrage si l'OTA s'interrompt ou dure plus de 2 h ; envois et boutons suspendus pendant une OTA
- Publication automatique des images OTA par GitHub Actions
