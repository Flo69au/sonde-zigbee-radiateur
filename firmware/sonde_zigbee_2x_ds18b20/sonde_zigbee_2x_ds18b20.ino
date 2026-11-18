/*
 * Sonde de température Zigbee : ESP32-H2 + 2x DS18B20 (un bus 1-Wire par sonde)
 *                               carte Waveshare ESP32-H2-Zero (ESP32-H2FH4, 4 Mo)
 *                               + bouton d'appairage externe
 * ==============================================================================
 * Core Arduino-ESP32 3.3.x (bibliothèque Zigbee intégrée au core), CI épinglée sur 3.3.12
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
 *   Zigbee mode       : Zigbee ZCZR (coordinator/router) -> la sonde est un routeur Zigbee
 *                       (alimentation permanente, elle relaie les autres appareils)
 *   Partition Scheme  : Zigbee ZCZR 4MB with spiffs (zb_storage + partitions OTA app0/app1)
 *   Erase All Flash Before Sketch Upload : Disabled (garde l'appairage)
 *                       Enabled UNE fois si la carte avait un firmware ED (table de
 *                       partitions différente), puis réappairer.
 *   Core Debug Level  : Info pour diagnostiquer une OTA (lignes "Zigbee - ...")
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
 * LED RGB WS2812 de la carte (RGB_BUILTIN, GPIO8). ÉTEINTE en fonctionnement normal,
 * elle ne s'allume que pour signaler un événement :
 *   rouge clignotant  -> NON connecté : recherche d'un réseau (appairage) / réseau perdu
 *   flash vert        -> température envoyée à Zigbee2MQTT
 *   flash rouge       -> échec d'envoi (nouvel essai à la lecture suivante)
 *   flash blanc       -> appui court sur le bouton
 *   orange fixe       -> bouton maintenu (relâcher avant le violet annule)
 *   violet            -> 3 s atteintes : réinitialisation Zigbee, reboot en mode appairage
 *   bleu rapide       -> mise à jour OTA en cours (envois et boutons suspendus,
 *                        ne pas couper l'alimentation)
 *   rouge 1-2 s       -> le Zigbee n'a pas démarré ou une OTA a échoué : redémarrage
 * Sur l'ESP32-H2-Zero, rouge et vert sont inversés par rapport au réglage par défaut
 * de "ESP32H2 Dev Module" : led() inverse g et r (comme ledColor() du capteur CO2).
 *
 * Côté Zigbee : 2 endpoints "Temperature Measurement" (cluster 0x0402)
 *   endpoint 10 = température d'ENTRÉE (GPIO4)  -> temperature_entree dans Z2M
 *   endpoint 11 = température de SORTIE (GPIO5) -> temperature_sortie dans Z2M
 *   + client OTA Upgrade (cluster 0x0019) sur l'endpoint 10 : mise à jour via Zigbee2MQTT
 *
 * Mises à jour OTA (voir README.md) :
 *   Publier une release GitHub avec un tag v<majeur>.<mineur>[.<correctif>] (v1.1, v1.2.1...).
 *   Le workflow .github/workflows/ota-release.yml compile ce sketch avec
 *   OTA_FW_VERSION = version du tag (0xMMmmPP00), fabrique l'image .zigbee avec
 *   image_builder_tool.py, la joint à la release et met à jour ota_index.json.
 *   L'index commun Flo69au/zigbee-ota (lu par Zigbee2MQTT) le fusionne toutes les heures.
 *   La première version avec client OTA (v1.0) se flashe en USB.
 */

#ifndef ZIGBEE_MODE_ZCZR
#error "Sélectionne 'Zigbee ZCZR (coordinator/router)' dans Outils -> Zigbee mode (et Partition Scheme 'Zigbee ZCZR 4MB with spiffs')"
#endif

#include "Zigbee.h"
#include <OneWire.h>            // fourni par OneWireNg (interface compatible OneWire)
#include <DallasTemperature.h>
#include <math.h>
#include <string.h>

// ============================ OTA ============================
// Version de CE firmware, format 0xMMmmPP00 (majeur.mineur.correctif) :
//   v1.0 -> 0x01000000, v1.1 -> 0x01010000, v1.2.3 -> 0x01020300
// Pour une release GitHub, elle est imposée depuis le tag via -DOTA_FW_VERSION.
// La valeur par défaut ci-dessous sert au flash USB depuis l'Arduino IDE : la garder
// ÉGALE au dernier tag publié, pour que Zigbee2MQTT ne repropose pas une mise à jour
// déjà installée.
#ifndef OTA_FW_VERSION
#define OTA_FW_VERSION       0x01050000  // v1.5 (LED éteinte en fonctionnement normal, couleurs du capteur CO2)
#endif
#define OTA_HW_VERSION       0x0101      // version matérielle (rarement à changer)
// Couple fabricant / type d'image réservé à ce projet (voir Flo69au/zigbee-ota) :
// doit correspondre à .github/workflows/ota-release.yml
#define OTA_MANUFACTURER_ID  0x1337
#define OTA_IMAGE_TYPE       0x0002

// Garde-fou OTA : la lib Zigbee ne prévient pas le sketch quand une mise à jour est
// annulée (ABORT) et garde alors un état interne incohérent qui fait échouer les
// tentatives suivantes. Si l'OTA se termine sans redémarrer, ou dure plus de 2 h,
// le firmware redémarre pour repartir sur un état propre.
#define OTA_MAX_DURATION_MS  (2UL * 60UL * 60UL * 1000UL)  // 2 h

volatile bool otaRunning = false;
volatile bool otaEnded = false;           // OTA terminée sans redémarrage auto (= échec)
volatile uint32_t otaStartMs = 0;

// ============================ Configuration ============================

#define ONEWIRE_PIN_ENTREE  4       // sonde d'entrée du radiateur
#define ONEWIRE_PIN_SORTIE  5       // sonde de sortie du radiateur
#define PAIR_BUTTON_PIN  11         // bouton d'appairage externe (vers GND)

#define USE_STATUS_LED     true     // false = LED de la carte toujours éteinte

const uint8_t  LED_BRIGHT   = 40;   // luminosité flashs / clignotements (0-255)
const uint16_t LED_BLINK_MS = 300;  // demi-période du clignotement rouge (non connecté)
const uint16_t LED_SEND_MS  = 150;  // durée du flash vert (ou rouge) à chaque envoi

#define EP_ENTREE     10
#define EP_SORTIE     11

#define ZB_MANUFACTURER "DIY"
#define ZB_MODEL        "H2-2xDS18B20"   // doit correspondre au convertisseur Z2M

// Envoi vers Zigbee2MQTT : chaque sonde n'envoie que si sa température a varié
// d'au moins TEMP_REPORT_DELTA_C depuis le dernier envoi, ou si elle n'a rien
// envoyé depuis REPORT_HEARTBEAT_MS (envoi de sécurité). Plus d'envoi forcé périodique.
// Lecture en continu : une nouvelle conversion démarre dès que la précédente est lue,
// soit une lecture toutes les ~0,8 s par sonde (en 12 bits une conversion dure 750 ms :
// c'est le maximum possible). Mettre par exemple 10000 pour lire toutes les 10 s.
const uint32_t MEASURE_INTERVAL_MS = 0;
const uint32_t LOG_INTERVAL_MS     = 10000;   // moniteur série : lectures sans envoi affichées toutes les 10 s
// Seuil de variation pour envoyer (°C). Le DS18B20 mesure par pas de 0,0625 °C :
// avec 0,2 °C, un envoi part en pratique à 0,25 °C d'écart (4 pas ; 3 pas = 0,1875).
const float    TEMP_REPORT_DELTA_C = 0.2f;
const uint32_t REPORT_HEARTBEAT_MS = 3600000; // envoi de sécurité : au moins 1 fois par heure
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
  float lastMeasured;       // dernière lecture valide
  float lastReported;       // dernière valeur envoyée (NAN = jamais envoyée)
  uint32_t lastReportMs;    // moment du dernier envoi
  uint32_t lastDiscoverMs;
};
Sonde sondes[2];

enum CycleState { IDLE, CONVERTING };
CycleState state = IDLE;
uint32_t tCycle = 0;
uint32_t tConvStart = 0;
bool forceReport = false;   // appui court sur le bouton : envoi immédiat des deux sondes

// ============================ LED d'état ============================

uint32_t flashUntil = 0;

// Écrit la couleur seulement si elle change
void led(uint8_t r, uint8_t g, uint8_t b) {
#if defined(RGB_BUILTIN)
  static int32_t last = -1;
  int32_t c = ((int32_t)r << 16) | ((int32_t)g << 8) | b;
  if (!USE_STATUS_LED || c == last) return;
  last = c;
  rgbLedWrite(RGB_BUILTIN, g, r, b);  // g et r inversés : LED de l'ESP32-H2-Zero
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
    // non connecté (appairage en cours / réseau perdu) : rouge clignotant
    if ((now / LED_BLINK_MS) % 2) led(LED_BRIGHT, 0, 0);
    else                          led(0, 0, 0);
  } else {
    led(0, 0, 0);                // connecté : éteinte, seuls les flashs d'envoi
  }
}

// ============================ OTA : suivi d'état ============================

// Appelé par la lib Zigbee au début et à la fin d'une mise à jour OTA
void onOTAStateChange(bool active) {
  if (active) {
    otaStartMs = millis();
    otaEnded = false;
  } else if (otaRunning) {
    otaEnded = true;   // en cas de succès la lib redémarre d'elle-même juste après
  }
  otaRunning = active;
  Serial.println(active ? "OTA : mise à jour démarrée" : "OTA : terminée");
}

// À appeler en tête de loop() : renvoie true tant qu'une OTA est en cours
// (le reste de la boucle est alors sauté : pas d'envoi, boutons désactivés).
bool otaGuard() {
  if (otaEnded) {
    Serial.println("OTA interrompue -> redémarrage pour repartir sur un état propre");
    led(LED_BRIGHT, 0, 0);
    delay(2000);
    ESP.restart();
  }
  if (!otaRunning) return false;

  if (millis() - otaStartMs > OTA_MAX_DURATION_MS) {
    Serial.println("OTA bloquée depuis plus de 2 h -> redémarrage");
    delay(500);
    ESP.restart();
  }
  // bleu rapide : OTA en cours, ne pas couper l'alimentation
  if ((millis() / 100) % 2) led(0, 0, LED_BRIGHT);
  else                      led(0, 0, 0);
  return true;
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

  // Pour ne pas inonder le moniteur série (2 lectures par seconde), les lectures
  // qui ne partent pas ne sont affichées que toutes les 10 s.
  static uint32_t lastLogMs = 0;
  bool logNow = (now - lastLogMs) >= LOG_INTERVAL_MS;
  if (logNow) lastLogMs = now;

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
    if (t == 85.0f && (isnan(so.lastMeasured) || fabsf(so.lastMeasured - 85.0f) > 5.0f)) {
      Serial.printf("[%s] 85 °C de démarrage ignoré\n", NOMS[s]);
      continue;
    }

    so.errors = 0;
    so.lastMeasured = t;
    zbSondes[s]->setTemperature(t);

    // Envoi seulement si : 1er envoi, variation >= TEMP_REPORT_DELTA_C, 1 h sans envoi,
    // ou appui court. (marge de 0,001 °C : en flottant 20,2 - 20,0 donne 0,1999...)
    bool first     = isnan(so.lastReported);
    bool changed   = !first && fabsf(t - so.lastReported) >= TEMP_REPORT_DELTA_C - 0.001f;
    bool heartbeat = !first && (now - so.lastReportMs) >= REPORT_HEARTBEAT_MS;

    if (!(first || changed || heartbeat || forceReport)) {
      if (logNow) Serial.printf("[%s] %.2f °C\n", NOMS[s], t);
      continue;
    }

    if (!Zigbee.connected()) {
      // pas de réseau (la LED clignote déjà en rouge) : nouvel essai à la prochaine lecture
      Serial.printf("[%s] %.2f °C (pas de réseau, nouvel essai)\n", NOMS[s], t);
    } else if (zbSondes[s]->reportTemperature()) {
      so.lastReported = t;
      so.lastReportMs = now;
      ledFlash(0, LED_BRIGHT, 0, LED_SEND_MS);   // flash vert : envoyé
      Serial.printf("[%s] %.2f °C -> envoyé (%s)\n", NOMS[s], t,
                    first ? "1er envoi" : changed ? "variation" : heartbeat ? "1 h" : "bouton");
    } else {
      ledFlash(LED_BRIGHT, 0, 0, LED_SEND_MS);   // flash rouge : échec d'envoi
      Serial.printf("[%s] %.2f °C (échec d'envoi, nouvel essai)\n", NOMS[s], t);
    }
  }
  forceReport = false;
}

// ============================ Boutons ============================

// Bouton d'appairage externe et bouton BOOT : même comportement
void handleButtons() {
  for (uint8_t b = 0; b < 2; b++) {
    uint8_t pin = BUTTONS[b];
    if (digitalRead(pin) != LOW) continue;

    delay(50);                                  // anti-rebond
    if (digitalRead(pin) != LOW) continue;

    led(LED_BRIGHT, LED_BRIGHT * 3 / 8, 0);     // orange fixe : bouton maintenu
    uint32_t t0 = millis();
    while (digitalRead(pin) == LOW) {
      if (millis() - t0 >= PAIR_HOLD_MS) {
        // 3 s atteintes : violet = réinitialisation, puis reboot en mode
        // appairage où la LED clignote en rouge
        Serial.println("Appairage : on quitte le réseau, reboot en mode appairage...");
        led(LED_BRIGHT, 0, LED_BRIGHT);         // violet : réinitialisation
        delay(800);
        Zigbee.factoryReset();                  // efface le réseau et redémarre
      }
      delay(20);
    }

    // appui court : mesure + envoi immédiats des deux sondes, même sans variation
    Serial.println("Appui court : mesure + envoi forcés");
    ledFlash(LED_BRIGHT, LED_BRIGHT, LED_BRIGHT, 100);   // flash blanc
    forceReport = true;
    tCycle = millis() - MEASURE_INTERVAL_MS;
    return;
  }
}

// ============================ Arduino ============================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\n=== Sonde Zigbee ESP32-H2 + 2x DS18B20 : firmware v%lu.%lu.%lu (0x%08lX) ===\n",
                (unsigned long)((OTA_FW_VERSION >> 24) & 0xFF),
                (unsigned long)((OTA_FW_VERSION >> 16) & 0xFF),
                (unsigned long)((OTA_FW_VERSION >> 8) & 0xFF),
                (unsigned long)OTA_FW_VERSION);

  pinMode(PAIR_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BOOT_PIN, INPUT_PULLUP);
  led(0, 0, 0);

  for (uint8_t s = 0; s < 2; s++) {
    sondes[s].present = false;
    sondes[s].errors = 0;
    sondes[s].lastMeasured = NAN;
    sondes[s].lastReported = NAN;
    sondes[s].lastReportMs = 0;
    sondes[s].lastDiscoverMs = 0;
    discoverSensor(s);
  }

  // Client OTA sur l'endpoint d'entrée (doit être ajouté avant addEndpoint)
  zbEntree.addOTAClient(OTA_FW_VERSION, OTA_FW_VERSION, OTA_HW_VERSION,
                        OTA_MANUFACTURER_ID, OTA_IMAGE_TYPE);
  zbEntree.onOTAStateChange(onOTAStateChange);

  // Déclaration des deux endpoints température
  for (uint8_t s = 0; s < 2; s++) {
    zbSondes[s]->setManufacturerAndModel(ZB_MANUFACTURER, ZB_MODEL);
    zbSondes[s]->setMinMaxValue(-55, 125);  // plage du DS18B20
    zbSondes[s]->setTolerance(0.1);         // tolérance annoncée ±0,1 °C (attribut ZCL informatif)
    Zigbee.addEndpoint(zbSondes[s]);
  }

  bool ok = Zigbee.begin(ZIGBEE_ROUTER);
  Serial.println("Mode Zigbee : routeur");

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
  led(0, 0, 0);   // connecté : LED éteinte (le 1er envoi fera un flash vert)

  // Reporting automatique de la pile Zigbee neutralisé : c'est le firmware qui
  // décide des envois (variation de 0,2 °C ou 1 h). max = 65000 s (~18 h) et seuil
  // de 600 °C hors d'atteinte. Doit rester aligné sur le convertisseur Z2M.
  // (après Zigbee.begin, et réappliqué à chaque démarrage)
  for (uint8_t s = 0; s < 2; s++) {
    zbSondes[s]->setReporting(0, 65000, 600);
  }

  // Interroge régulièrement le coordinateur pour une mise à jour OTA
  zbEntree.requestOTAUpdate();

  // Première mesure tout de suite, puis en continu
  tCycle = millis() - MEASURE_INTERVAL_MS;
}

void loop() {
  // Pendant une OTA : ni mesures, ni envois, ni boutons (reset en plein flash = brique)
  if (otaGuard()) {
    delay(10);
    return;
  }

  handleButtons();
  ledUpdate();   // éteinte si connecté, rouge clignotant si réseau perdu

  uint32_t now = millis();

  if (state == IDLE && (now - tCycle) >= MEASURE_INTERVAL_MS) {
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
