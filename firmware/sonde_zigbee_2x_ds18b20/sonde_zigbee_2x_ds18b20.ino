/*
 * Sonde de température Zigbee : ESP32-H2 + 2x DS18B20 (un bus 1-Wire par sonde)
 *                               + bouton d'appairage externe
 * ==============================================================================
 * Core Arduino-ESP32 3.x (bibliothèque Zigbee intégrée au core)
 *
 * Bibliothèques (gestionnaire de bibliothèques Arduino) :
 *   - OneWireNg          (Piotr Stolarz)  -> fournit un OneWire.h compatible
 *   - DallasTemperature  (Miles Burton)   -> installer "SANS les dépendances"
 *   /!\ Ne PAS garder la lib "OneWire" de Paul Stoffregen installée en parallèle :
 *       elle ne compile pas proprement sur ESP32-H2 et entre en conflit avec OneWireNg.
 *
 * Réglages Arduino IDE (menu Outils) :
 *   Carte             : ESP32H2 Dev Module
 *   USB CDC On Boot   : Enabled (logs sur l'USB natif)
 *   Zigbee mode       : "Zigbee ED (end device)"              -> appareil terminal (défaut)
 *                    ou "Zigbee ZCZR (coordinator/router)"    -> routeur (alim USB permanente)
 *   Partition Scheme  : "Zigbee 4MB with spiffs"               (mode ED)
 *                    ou "Zigbee ZCZR 4MB with spiffs"          (mode routeur)
 *   Erase All Flash Before Sketch Upload : Enabled au 1er flash / changement de mode
 *
 * Câblage : un GPIO + une pull-up 4,7 kΩ par sonde (alimentation normale, pas parasite)
 *
 *   3V3 ───┬──────────────┬──────────────┬──────────────┐
 *          │              │              │              │
 *       [4,7 kΩ]       [4,7 kΩ]     VDD #1 (rouge)  VDD #2 (rouge)
 *          │              │
 *   GPIO4 ─┴──────────────┼───────── DQ #1 (jaune/blanc)     -> ENTRÉE radiateur / endpoint 10
 *   GPIO5 ────────────────┴───────── DQ #2 (jaune/blanc)     -> SORTIE radiateur / endpoint 11
 *   GND ───────────────────────────── GND #1 et GND #2 (noir/bleu)
 *
 *   Bouton d'appairage (bouton poussoir, pas un interrupteur à bascule) :
 *   GPIO11 ──[ BP ]── GND        (pull-up interne activée, pas de résistance à ajouter)
 *
 *   Câbles longs (> 5-10 m) : descendre les pull-up à 2,2 kΩ.
 *   GPIO à éviter sur H2 : 2, 3, 8, 9, 25 (strapping), 26/27 (USB), 23/24 (UART).
 *
 * Bouton d'appairage (GPIO11) et bouton BOOT (GPIO9), même comportement :
 *   appui court     -> lecture + envoi immédiat des deux températures
 *   appui > 3 s     -> quitte le réseau Zigbee, reboot et repasse en mode appairage
 *                      (activer le "Permit join" dans Zigbee2MQTT)
 *
 * LED RGB de la carte (GPIO8 sur ESP32-H2-DevKitM-1), si présente :
 *   rouge clignotant -> NON appairé : recherche d'un réseau / réseau perdu
 *   vert fixe faible -> appairé (ou éteinte si LED_PAIRED_STEADY = false)
 *   vert vif 1,5 s   -> appairage réussi
 *   flash bleu       -> envoi d'une température
 *   flash blanc      -> appui court sur le bouton
 *   orange fixe      -> bouton maintenu (relâcher annule tant que ce n'est pas violet)
 *   violet           -> réinitialisation Zigbee, reboot en mode appairage
 *
 * Côté Zigbee : 2 endpoints "Temperature Measurement" (cluster 0x0402)
 *   endpoint 10 = température d'ENTRÉE (GPIO4)  -> temperature_entree dans Z2M
 *   endpoint 11 = température de SORTIE (GPIO5) -> temperature_sortie dans Z2M
 *   + client OTA Upgrade (cluster 0x0019) sur l'endpoint 10 : mise à jour via Zigbee2MQTT
 *
 * Mises à jour OTA (voir README.md) :
 *   1. incrémenter FW_VERSION_MINOR (1.0 -> 1.1 -> 1.2 ...)
 *   2. Arduino IDE : Croquis > Exporter les binaires compilés
 *   3. python tools/make_ota.py <le .ino.bin> --index <clone>/capteur-temp-rature-scd40_zigbee/ota_index.json
 *   4. Release GitHub v1.x avec le .ota (dist/) + pousser ota_index.json
 *   5. Zigbee2MQTT : onglet OTA > Vérifier > Mettre à jour
 *   La toute première version avec OTA (v1.0) doit être flashée en USB.
 */

#if !defined(ZIGBEE_MODE_ED) && !defined(ZIGBEE_MODE_ZCZR)
#error "Choisis Outils > Zigbee mode : 'Zigbee ED (end device)' ou 'Zigbee ZCZR (coordinator/router)'"
#endif

#include "Zigbee.h"
#include <OneWire.h>            // fourni par OneWireNg (interface compatible OneWire)
#include <DallasTemperature.h>
#include <math.h>
#include <string.h>

// ============================ Version firmware ============================
// Numérotation v<majeur>.<mineur> : à chaque mise à jour, incrémenter FW_VERSION_MINOR.
// Une image OTA n'est proposée par Z2M que si sa version est plus grande que celle installée.

#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 0

#define STR_(x) #x
#define STR(x)  STR_(x)
#define FW_VERSION_STR STR(FW_VERSION_MAJOR) "." STR(FW_VERSION_MINOR)

// Version Zigbee sur 32 bits : 0xMMmm0000 (v1.1 = 0x01010000, v1.2 = 0x01020000)
const uint32_t FW_FILE_VERSION = ((uint32_t)FW_VERSION_MAJOR << 24) | ((uint32_t)FW_VERSION_MINOR << 16);

// Marqueur lu par tools/make_ota.py dans le .bin pour fabriquer l'image OTA à la bonne version
__attribute__((used)) static const char FW_MARKER[] = "SONDE_FW_VERSION=" FW_VERSION_STR ";";

// Identifiants OTA : doivent être identiques dans tools/make_ota.py
// 0x1337 = code fabricant "maison" (comme le capteur SCD40), type 2 = sonde radiateur
// (le SCD40 est en type 1) : chaque appareil ne reçoit que son propre firmware.
#define OTA_MANUFACTURER 0x1337
#define OTA_IMAGE_TYPE   0x0002
#define OTA_HW_VERSION   0x0001

// ============================ Configuration ============================

#define ONEWIRE_PIN_ENTREE  4       // sonde d'entrée du radiateur
#define ONEWIRE_PIN_SORTIE  5       // sonde de sortie du radiateur
#define PAIR_BUTTON_PIN  11         // bouton d'appairage externe (vers GND)

#define USE_STATUS_LED     true     // false = LED de la carte toujours éteinte
#define LED_PAIRED_STEADY  true     // true = vert fixe quand appairé ; false = éteinte, seuls les flashs

const uint8_t  LED_BRIGHT   = 40;   // luminosité flashs / clignotements (0-255)
const uint8_t  LED_DIM      = 6;    // luminosité du vert fixe "appairé"
const uint16_t LED_BLINK_MS = 300;  // demi-période du clignotement rouge
const uint16_t LED_SEND_MS  = 150;  // durée du flash bleu à chaque envoi

#define EP_ENTREE     10
#define EP_SORTIE     11

#define ZB_MANUFACTURER "DIY"
#define ZB_MODEL        "H2-2xDS18B20"   // doit correspondre au convertisseur Z2M

const uint32_t SEND_INTERVAL_MS  = 30000;   // mesure + envoi des deux températures toutes les 30 s
const uint8_t  RESOLUTION_BITS   = 12;      // 12 bits = pas de 0,0625 °C, conversion 750 ms
const uint32_t REDISCOVER_MS     = 60000;   // nouvelle recherche si une sonde manque
const uint8_t  MAX_ERRORS        = 3;       // lectures invalides avant de déclarer la sonde absente
const uint32_t PAIR_HOLD_MS      = 3000;    // durée d'appui pour relancer l'appairage

// ============================ Objets ============================

// index 0 = entrée, index 1 = sortie
const char   *NOMS[2]    = {"Entrée", "Sortie"};
const uint8_t PINS[2]    = {ONEWIRE_PIN_ENTREE, ONEWIRE_PIN_SORTIE};
const uint8_t BUTTONS[2] = {PAIR_BUTTON_PIN, BOOT_PIN};

OneWire owEntree(ONEWIRE_PIN_ENTREE);
OneWire owSortie(ONEWIRE_PIN_SORTIE);
DallasTemperature dsEntree(&owEntree);
DallasTemperature dsSortie(&owSortie);
DallasTemperature *buses[2] = {&dsEntree, &dsSortie};

ZigbeeTempSensor zbEntree = ZigbeeTempSensor(EP_ENTREE);
ZigbeeTempSensor zbSortie = ZigbeeTempSensor(EP_SORTIE);
ZigbeeTempSensor *zbSondes[2] = {&zbEntree, &zbSortie};

struct Sonde {
  DeviceAddress addr;
  bool present;
  uint8_t errors;
  float lastReported;
  uint32_t lastDiscoverMs;
};
Sonde sondes[2];

enum CycleState { IDLE, CONVERTING };
CycleState state = IDLE;
uint32_t tCycle = 0;
uint32_t tConvStart = 0;

// ============================ LED d'état ============================

uint32_t flashUntil = 0;

// Écrit la couleur seulement si elle change
void led(uint8_t r, uint8_t g, uint8_t b) {
#if defined(RGB_BUILTIN)
  static int32_t last = -1;
  int32_t c = ((int32_t)r << 16) | ((int32_t)g << 8) | b;
  if (!USE_STATUS_LED || c == last) return;
  last = c;
  rgbLedWrite(RGB_BUILTIN, r, g, b);
#else
  (void)r; (void)g; (void)b;   // pas de LED RGB connue sur cette carte
#endif
}

// Flash temporaire ; la couleur de fond reprend toute seule ensuite
void ledFlash(uint8_t r, uint8_t g, uint8_t b, uint16_t ms) {
  led(r, g, b);
  flashUntil = millis() + ms;
}

// Couleur de fond selon l'état Zigbee, à appeler en boucle (non bloquant)
void ledUpdate() {
  uint32_t now = millis();
  if ((int32_t)(flashUntil - now) > 0) return;   // un flash est en cours

  if (!Zigbee.connected()) {
    // non appairé / réseau perdu : rouge clignotant
    if ((now / LED_BLINK_MS) % 2) led(LED_BRIGHT, 0, 0);
    else                          led(0, 0, 0);
  } else if (LED_PAIRED_STEADY) {
    led(0, LED_DIM, 0);          // appairé : vert fixe faible
  } else {
    led(0, 0, 0);                // appairé : éteinte entre deux envois
  }
}

// ============================ Utilitaires ============================

void printAddr(const uint8_t *a) {
  Serial.print("{");
  for (uint8_t i = 0; i < 8; i++) {
    Serial.printf("0x%02X%s", a[i], i < 7 ? ", " : "");
  }
  Serial.print("}");
}

uint32_t conversionTimeMs() {
  // 9 bits ≈ 94 ms, 10 ≈ 188, 11 ≈ 375, 12 ≈ 750 (+ marge)
  return (750UL >> (12 - RESOLUTION_BITS)) + 50;
}

// ============================ 1-Wire ============================

// Cherche la sonde sur le bus n° s (une sonde par bus)
void discoverSensor(uint8_t s) {
  Sonde &so = sondes[s];
  DallasTemperature *ds = buses[s];

  so.lastDiscoverMs = millis();
  ds->begin();
  ds->setWaitForConversion(false);   // conversion non bloquante, on gère l'attente nous-mêmes

  uint8_t n = ds->getDeviceCount();
  if (n > 1) {
    Serial.printf("[%s] /!\\ %u sondes sur GPIO%u, seule la première est utilisée\n",
                  NOMS[s], n, PINS[s]);
  }

  so.present = (n >= 1) && ds->getAddress(so.addr, 0);
  if (so.present) {
    ds->setResolution(so.addr, RESOLUTION_BITS);
    so.errors = 0;
    Serial.printf("[%s] GPIO%u OK, adresse ", NOMS[s], PINS[s]);
    printAddr(so.addr);
    Serial.println();
  } else {
    memset(so.addr, 0, 8);
    Serial.printf("[%s] GPIO%u : aucune sonde détectée\n", NOMS[s], PINS[s]);
  }
}

void readAndReport() {
  uint32_t now = millis();

  for (uint8_t s = 0; s < 2; s++) {
    Sonde &so = sondes[s];

    if (!so.present) {
      if ((now - so.lastDiscoverMs) >= REDISCOVER_MS) discoverSensor(s);
      continue;
    }

    float t = buses[s]->getTempC(so.addr);

    // -127 °C = sonde déconnectée / erreur CRC ; hors plage = lecture corrompue
    if (t == DEVICE_DISCONNECTED_C || t < -55.0f || t > 125.0f) {
      Serial.printf("[%s] lecture invalide (%.2f)\n", NOMS[s], t);
      if (++so.errors >= MAX_ERRORS) {
        Serial.printf("[%s] déclarée absente\n", NOMS[s]);
        so.present = false;
        so.lastDiscoverMs = 0;  // nouvelle recherche au prochain cycle
      }
      continue;
    }

    // 85,0 °C pile = valeur de mise sous tension (conversion pas faite) -> on ignore
    if (t == 85.0f && (isnan(so.lastReported) || fabsf(so.lastReported - 85.0f) > 5.0f)) {
      Serial.printf("[%s] 85 °C de démarrage ignoré\n", NOMS[s]);
      continue;
    }

    so.errors = 0;
    so.lastReported = t;
    zbSondes[s]->setTemperature(t);

    // envoi à chaque cycle (toutes les 30 s), que la température ait changé ou non
    if (Zigbee.connected()) {
      zbSondes[s]->reportTemperature();
      ledFlash(0, 0, LED_BRIGHT, LED_SEND_MS);   // flash bleu : envoi
      Serial.printf("[%s] %.2f °C -> envoyé\n", NOMS[s], t);
    } else {
      Serial.printf("[%s] %.2f °C (pas de réseau, non envoyé)\n", NOMS[s], t);
    }
  }
}

// ============================ Boutons ============================

// Bouton d'appairage externe et bouton BOOT : même comportement
void handleButtons() {
  for (uint8_t b = 0; b < 2; b++) {
    uint8_t pin = BUTTONS[b];
    if (digitalRead(pin) != LOW) continue;

    delay(50);                                  // anti-rebond
    if (digitalRead(pin) != LOW) continue;

    led(LED_BRIGHT, LED_BRIGHT / 3, 0);         // orange : bouton maintenu
    uint32_t t0 = millis();
    while (digitalRead(pin) == LOW) {
      if (millis() - t0 >= PAIR_HOLD_MS) {
        Serial.println("Appairage : on quitte le réseau, reboot en mode appairage...");
        led(LED_BRIGHT, 0, LED_BRIGHT);         // violet : réinitialisation
        delay(800);
        Zigbee.factoryReset();                  // efface le réseau et redémarre
      }
      delay(20);
    }

    // appui court : mesure + envoi immédiats (le cycle de 30 s repart de maintenant)
    Serial.println("Appui court : mesure + envoi forcés");
    ledFlash(LED_BRIGHT, LED_BRIGHT, LED_BRIGHT, 100);   // flash blanc
    tCycle = millis() - SEND_INTERVAL_MS;
    return;
  }
}

// ============================ Arduino ============================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\n=== Sonde Zigbee ESP32-H2 + 2x DS18B20 : firmware v%s (0x%08lX) ===\n",
                FW_VERSION_STR, (unsigned long)FW_FILE_VERSION);
  Serial.println(FW_MARKER);

  pinMode(PAIR_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BOOT_PIN, INPUT_PULLUP);
  led(0, 0, 0);

  for (uint8_t s = 0; s < 2; s++) {
    sondes[s].present = false;
    sondes[s].errors = 0;
    sondes[s].lastReported = NAN;
    sondes[s].lastDiscoverMs = 0;
    discoverSensor(s);
  }

  // Client OTA sur l'endpoint d'entrée (doit être ajouté avant addEndpoint)
  zbEntree.addOTAClient(FW_FILE_VERSION, FW_FILE_VERSION, OTA_HW_VERSION,
                        OTA_MANUFACTURER, OTA_IMAGE_TYPE);

  // Déclaration des deux endpoints température
  for (uint8_t s = 0; s < 2; s++) {
    zbSondes[s]->setManufacturerAndModel(ZB_MANUFACTURER, ZB_MODEL);
    zbSondes[s]->setMinMaxValue(-55, 125);  // plage du DS18B20
    zbSondes[s]->setTolerance(0.5);         // précision constructeur ±0,5 °C
    Zigbee.addEndpoint(zbSondes[s]);
  }

#if defined(ZIGBEE_MODE_ZCZR)
  bool ok = Zigbee.begin(ZIGBEE_ROUTER);
  Serial.println("Mode Zigbee : routeur");
#else
  bool ok = Zigbee.begin();
  Serial.println("Mode Zigbee : end device");
#endif

  if (!ok) {
    Serial.println("Zigbee : échec du démarrage, reboot...");
    led(LED_BRIGHT, 0, 0);
    delay(1000);
    ESP.restart();
  }

  // Pas encore appairé : la pile cherche un réseau ouvert en boucle.
  // Si déjà appairé, la connexion est quasi immédiate.
  Serial.print("Zigbee : recherche du réseau (active le Permit join dans Z2M)");
  uint32_t tDot = 0;
  while (!Zigbee.connected()) {
    ledUpdate();      // rouge clignotant
    handleButtons();  // reset possible même pendant l'attente
    if (millis() - tDot >= 1000) {
      tDot = millis();
      Serial.print(".");
    }
    delay(20);
  }
  Serial.println(" connecté !");
  ledFlash(0, LED_BRIGHT, 0, 1500);   // vert vif : appairage réussi

  // Interroge régulièrement le coordinateur pour une mise à jour OTA
  zbEntree.requestOTAUpdate();

  // Première mesure tout de suite, puis toutes les 30 s
  tCycle = millis() - SEND_INTERVAL_MS;
}

void loop() {
  handleButtons();
  ledUpdate();   // vert fixe si appairé, rouge clignotant si réseau perdu

  uint32_t now = millis();

  if (state == IDLE && (now - tCycle) >= SEND_INTERVAL_MS) {
    tCycle = now;
    // lance la conversion sur les deux bus en parallèle
    for (uint8_t s = 0; s < 2; s++) {
      if (sondes[s].present) buses[s]->requestTemperatures();
    }
    tConvStart = now;
    state = CONVERTING;
  }

  if (state == CONVERTING && (now - tConvStart) >= conversionTimeMs()) {
    readAndReport();
    state = IDLE;
  }

  delay(20);
}
