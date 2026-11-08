# Historique des versions

Tags `v<majeur>.<mineur>[.<correctif>]`, version Zigbee `0xMMmmPP00`.

## v1.0

Première version avec client OTA, à flasher en USB.

- Routeur Zigbee (ZCZR, partitions `zigbee_zczr`) : la compilation échoue si un autre mode est sélectionné

- 2 sondes DS18B20 sur deux bus 1-Wire séparés : entrée du radiateur (GPIO4, endpoint 10) et sortie (GPIO5, endpoint 11)
- Mesure et envoi des deux températures toutes les 30 s, filtrage des lectures invalides
- Bouton d'appairage sur GPIO11 (appui 3 s) et bouton BOOT
- LED d'état : appairé, non appairé, envoi, OTA en cours
- Client OTA Zigbee (fabricant 0x1337, type 0x0002) avec garde-fou : redémarrage si l'OTA s'interrompt ou dure plus de 2 h ; envois et boutons suspendus pendant une OTA
- Publication automatique des images OTA par GitHub Actions
