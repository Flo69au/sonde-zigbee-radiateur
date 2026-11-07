# Historique des versions

Numérotation v<majeur>.<mineur> : chaque mise à jour incrémente le mineur (v1.1, v1.2…).

## v1.0

Première version, à flasher en USB.

- 2 sondes DS18B20 sur deux bus 1-Wire séparés : entrée du radiateur (GPIO4, endpoint 10) et sortie (GPIO5, endpoint 11)
- Mesure et envoi des deux températures toutes les 30 s, filtrage des lectures invalides
- Bouton d'appairage sur GPIO11 (appui 3 s) et bouton BOOT
- LED d'état : appairé, non appairé, envoi
- Client OTA Zigbee : mises à jour depuis Zigbee2MQTT
