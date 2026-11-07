# Sonde de température radiateur Zigbee

Sonde Zigbee DIY à base d'**ESP32-H2** avec **deux DS18B20** (entrée et sortie du radiateur), intégrée à Home Assistant via **Zigbee2MQTT**. Les mises à jour du firmware se font **en OTA par le réseau Zigbee**, publiées automatiquement par GitHub Actions.

- Températures d'**entrée** et de **sortie**, mesurées et envoyées toutes les 30 s (`SEND_INTERVAL_MS`)
- Bouton d'appairage : appui 3 s pour (ré)appairer, appui court pour forcer un envoi
- LED d'état : appairé / non appairé / envoi / OTA en cours
- Client OTA Zigbee avec garde-fou (redémarrage si une OTA échoue ou dure plus de 2 h)

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

## Flash USB (Arduino IDE)

1. Core **Arduino-ESP32 3.3.x** (la CI utilise la 3.3.12).
2. Bibliothèques : **OneWireNg** et **DallasTemperature** (installer cette dernière *sans les dépendances*). Ne pas garder la lib *OneWire* de Paul Stoffregen : elle ne compile pas sur ESP32-H2 et entre en conflit avec OneWireNg.
3. Menu Outils :
   - Carte : **ESP32H2 Dev Module**
   - USB CDC On Boot : **Enabled**
   - Zigbee mode : **Zigbee ED (end device)**
   - Partition Scheme : **Zigbee 4MB with spiffs** (contient `zb_storage` et les deux partitions OTA `app0`/`app1`)
   - Erase All Flash Before Sketch Upload : **Disabled** (garde l'appairage)
4. Ouvrir `firmware/sonde_zigbee_2x_ds18b20/sonde_zigbee_2x_ds18b20.ino` et téléverser.

La version flashée en USB est la valeur par défaut de `OTA_FW_VERSION` dans le sketch. Elle doit toujours être **égale au dernier tag publié**.

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
| Flash bleu | Envoi des températures |
| Flash blanc | Appui court : mesure et envoi forcés |
| Orange fixe | Bouton maintenu (relâcher avant le violet annule) |
| Violet | Réinitialisation Zigbee puis redémarrage en mode appairage |
| Cyan clignotant rapide | Mise à jour OTA en cours : envois et boutons suspendus, ne pas couper l'alimentation |

## Mises à jour OTA

Système commun à tous mes projets Zigbee : voir [Flo69au/zigbee-ota](https://github.com/Flo69au/zigbee-ota).

- Couple réservé à ce projet : fabricant **`0x1337`**, type d'image **`0x0002`**, modèle **`H2-2xDS18B20`**.
- Version Zigbee = **`0xMMmmPP00`** : v1.0 → `0x01000000`, v1.1 → `0x01010000`, v1.2.3 → `0x01020300`.
- Ce dépôt publie ses images dans ses Releases et tient son `ota_index.json` (uniquement la sonde). `zigbee-ota` le fusionne dans l'index commun lu par Zigbee2MQTT, toutes les heures (hh:23).

### Publier une nouvelle version

1. Modifier le code. Mettre la valeur par défaut de `OTA_FW_VERSION` à la future version (ex. `0x01010000 // v1.1`), ajouter une ligne dans `CHANGELOG.md`, pousser.
2. GitHub : **Releases → Draft a new release → nouveau tag `v1.1` → Publish release**.
3. Le workflow **Release OTA Zigbee** (`.github/workflows/ota-release.yml`) compile le firmware avec la version du tag, fabrique `1337-0002-01010000-ota-file.zigbee`, le joint à la release et met à jour `ota_index.json`.
4. Attendre la fusion de `zigbee-ota` (hh:23), ou la lancer : *Actions → Fusion des index OTA → Run workflow*.
5. Zigbee2MQTT : fiche de la sonde → onglet OTA → *Check for updates* → *Update*. Compter environ 1 h avec des blocs de 50 octets, sans redémarrer Z2M ni toucher la sonde pendant le transfert.
6. Vérifier `installed_version` dans l'onglet État de Z2M.

### En cas d'échec

- Le garde-fou redémarre la sonde si l'OTA s'interrompt. Sinon, la débrancher et la rebrancher avant de réessayer.
- Diagnostic : flasher avec *Outils → Core Debug Level → Info*, garder le moniteur série ouvert pendant l'OTA et relever les lignes `Zigbee - …`. Échec vers 0 % : début, en-tête ou taille de bloc. Au milieu : transfert ou écriture flash. Vers 100 % : vérification finale.
- Ne pas mettre `default_maximum_data_size: 100` dans Z2M : l'OTA échoue dès le début.

## Structure du dépôt

```
firmware/sonde_zigbee_2x_ds18b20/    sketch Arduino
z2m/h2_2xds18b20.mjs                 convertisseur externe Zigbee2MQTT 2.x
.github/workflows/ota-release.yml    compilation + image OTA + release + index, à chaque tag
ota_index.json                       index OTA de ce projet (mis à jour par la CI)
CHANGELOG.md                         historique des versions
```
