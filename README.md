# Sonde de température radiateur Zigbee

Sonde Zigbee DIY à base d'**ESP32-H2** avec **deux DS18B20**, intégrée à Home Assistant via **Zigbee2MQTT**. Les mises à jour du firmware se font **en OTA par Zigbee**, sans câble.

- 2 températures (une sonde par bus 1-Wire), mesurées et envoyées toutes les 30 s (`SEND_INTERVAL_MS`)
- Bouton d'appairage : appui 3 s pour (ré)appairer, appui court pour forcer un envoi
- LED d'état : appairé / non appairé / envoi en cours
- Mises à jour OTA depuis l'onglet OTA de Zigbee2MQTT, versions v1.0, v1.1, v1.2…

## Matériel et câblage

| Élément | Broche ESP32-H2 | Remarque |
|---|---|---|
| DS18B20 **entrée** radiateur (DQ) | GPIO4 | pull-up 4,7 kΩ vers 3V3 → endpoint 10 / `temperature_entree` |
| DS18B20 **sortie** radiateur (DQ) | GPIO5 | pull-up 4,7 kΩ vers 3V3 → endpoint 11 / `temperature_sortie` |
| VDD des sondes | 3V3 | alimentation normale (pas de mode parasite) |
| GND des sondes | GND | |
| Bouton d'appairage | GPIO11 → GND | bouton poussoir, pull-up interne |
| LED RGB | GPIO8 | LED intégrée de l'ESP32-H2-DevKitM-1 (optionnelle) |

À éviter sur le H2 : GPIO 2, 3, 8, 9, 25 (strapping), 26/27 (USB), 23/24 (UART). Câbles longs (> 5–10 m) : pull-up de 2,2 kΩ.

## Compilation (Arduino IDE)

1. Core **Arduino-ESP32 3.x** (gestionnaire de cartes « esp32 » d'Espressif).
2. Bibliothèques : **OneWireNg** et **DallasTemperature** (installer cette dernière *sans les dépendances*). Ne pas garder la lib *OneWire* de Paul Stoffregen : elle ne compile pas sur ESP32-H2 et entre en conflit avec OneWireNg.
3. Menu Outils :
   - Carte : **ESP32H2 Dev Module**
   - USB CDC On Boot : **Enabled**
   - Zigbee mode : **Zigbee ED (end device)**, ou **Zigbee ZCZR** pour en faire un routeur si alimentation permanente
   - Partition Scheme : **Zigbee 4MB with spiffs** (ou la variante ZCZR) — ces schémas contiennent les deux partitions nécessaires à l'OTA
   - Erase All Flash Before Sketch Upload : **Enabled** au premier flash
4. Ouvrir `firmware/sonde_zigbee_2x_ds18b20/sonde_zigbee_2x_ds18b20.ino` et téléverser en USB.

## Zigbee2MQTT

1. Copier `z2m/h2_2xds18b20.mjs` dans `<data Z2M>/external_converters/` et redémarrer Z2M.
2. Activer *Permit join*, puis maintenir le bouton d'appairage 3 s.
3. La sonde apparaît comme `DIY H2-2xDS18B20` avec `temperature_entree` et `temperature_sortie`.

### Bouton et LED

| LED | Signification |
|---|---|
| Rouge clignotant | Non appairé : recherche d'un réseau, ou réseau perdu |
| Vert fixe faible | Appairé (éteinte si `LED_PAIRED_STEADY = false`) |
| Vert vif 1,5 s | Appairage réussi |
| Flash bleu | Envoi d'une température |
| Flash blanc | Appui court : mesure et envoi forcés |
| Orange fixe | Bouton maintenu (relâcher avant le violet annule) |
| Violet | Réinitialisation Zigbee puis redémarrage en mode appairage |

## Mises à jour OTA

La sonde n'a pas de Wi-Fi : le nouveau firmware lui est envoyé par Zigbee, par Zigbee2MQTT. La version **v1.0** (la première avec l'OTA) doit être flashée en USB. Toutes les suivantes peuvent passer en OTA.

### Index OTA

Ce dépôt publie son propre `ota_index.json` (les entrées de la sonde radiateur uniquement). Le dépôt
[zigbee-ota](https://github.com/Flo69au/zigbee-ota) le fusionne toutes les heures avec ceux du capteur SCD40
et du Levoit, et c'est cet index commun que lit Zigbee2MQTT :

```yaml
ota:
  zigbee_ota_override_index_location: https://raw.githubusercontent.com/Flo69au/zigbee-ota/main/ota_index.json
```

Les images `.ota` sont publiées en pièces jointes des **Releases GitHub** de ce dépôt-ci, et l'index pointe vers elles.

Identifiants OTA, identiques dans le firmware et dans le script : fabricant `0x1337` (4919, le même que le SCD40), type d'image `2` (le SCD40 est en type `1`). Chaque appareil ne peut donc recevoir que son propre firmware. La version Zigbee vaut `0xMMmm0000` (v1.1 = `0x01010000` = 16842752).

### À chaque nouvelle version

1. Dans le `.ino`, incrémenter la version :
   ```cpp
   #define FW_VERSION_MAJOR 1
   #define FW_VERSION_MINOR 1   // 1.0 -> 1.1 -> 1.2 ...
   ```
   Ajouter une ligne dans `CHANGELOG.md`.
2. Arduino IDE : **Croquis > Exporter les binaires compilés**. Le fichier utile est `build/…/sonde_zigbee_2x_ds18b20.ino.bin` (pas le `.merged.bin`).
3. Générer l'image OTA et mettre à jour l'index de ce dépôt (Python 3, sans dépendance) :
   ```bash
   python tools/make_ota.py firmware/sonde_zigbee_2x_ds18b20/build/esp32.esp32.esp32h2/sonde_zigbee_2x_ds18b20.ino.bin --index ota_index.json
   ```
   Le script lit la version dans le binaire et crée `dist/sonde-radiateur-v1.1.ota`. Dans l'index, il remplace seulement l'entrée de la sonde radiateur, avec l'URL de la release : les autres appareils ne sont pas touchés.
4. GitHub, ce dépôt : **Releases > Draft a new release**, tag `v1.1`, joindre `dist/sonde-radiateur-v1.1.ota`, publier.
5. Pousser `ota_index.json` dans ce dépôt. L'index commun `zigbee-ota` le reprend dans l'heure
   (ou tout de suite : dépôt `zigbee-ota` → *Actions* → *Fusion des index OTA* → *Run workflow*).
6. Zigbee2MQTT, onglet **OTA** : **Vérifier** sur la sonde, puis **Mettre à jour**. Avec `default_maximum_data_size: 50`, le transfert se fait par petits blocs : compter jusqu'à une heure environ selon la taille du firmware. La sonde continue d'envoyer ses températures pendant ce temps, puis redémarre sur la nouvelle version.

Pour contrôler une image : `python tools/make_ota.py --info dist/sonde-radiateur-v1.1.ota`.

## Structure du dépôt

```
firmware/sonde_zigbee_2x_ds18b20/   sketch Arduino
z2m/h2_2xds18b20.mjs                convertisseur externe Zigbee2MQTT 2.x
tools/make_ota.py                   création des images OTA + mise à jour de l'index
dist/                               images .ota générées (non versionnées, publiées en Releases)
CHANGELOG.md                        historique des versions
```
