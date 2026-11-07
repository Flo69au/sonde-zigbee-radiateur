// Zigbee2MQTT 2.x : convertisseur externe pour la sonde ESP32-H2 + 2x DS18B20
//
// Installation : copier ce fichier dans  <dossier data de Z2M>/external_converters/
// puis redémarrer Zigbee2MQTT. Si la sonde est déjà appairée : bouton "Reconfigure"
// (ou "Interview") sur la page de l'appareil.
//
// Résultat : deux entités, temperature_entree (endpoint 10, GPIO4)
//            et temperature_sortie (endpoint 11, GPIO5).

import * as m from 'zigbee-herdsman-converters/lib/modernExtend';

export default {
    zigbeeModel: ['H2-2xDS18B20'],          // = ZB_MODEL dans le firmware
    model: 'H2-2xDS18B20',
    vendor: 'DIY',
    description: 'Sonde de température 2x DS18B20 (ESP32-H2)',
    ota: true,                               // mises à jour firmware depuis l'onglet OTA de Z2M
    extend: [
        m.deviceEndpoints({endpoints: {entree: 10, sortie: 11}}),
        m.temperature({
            endpointNames: ['entree', 'sortie'],
            // La sonde envoie elle-même ses deux températures toutes les 30 s (firmware).
            // Cette config sert à créer la liaison (bind) vers le coordinateur, indispensable
            // pour que ces envois arrivent. Le reporting automatique de la pile Zigbee est
            // volontairement neutralisé (max ~18 h, seuil 100 °C) pour éviter les doublons.
            reporting: {min: 30, max: 65000, change: 10000},
        }),
    ],
};
