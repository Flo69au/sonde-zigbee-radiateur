#!/usr/bin/env python3
"""
Fabrique l'image OTA Zigbee de la sonde radiateur (ESP32-H2) et met à jour
l'index OTA partagé utilisé par Zigbee2MQTT.

Fonctionnement (même principe que les autres appareils de Florian) :
  - l'image .ota est publiée en asset d'une Release GitHub du dépôt sonde-zigbee-radiateur
    (tag v1.1, v1.2, ...)
  - l'index ota_index.json, hébergé dans le dépôt capteur-temp-rature-scd40_zigbee et pointé
    par zigbee_ota_override_index_location dans Z2M, référence cette image.

Utilisation :
    python tools/make_ota.py <firmware.ino.bin> --index <clone>/capteur-temp-rature-scd40_zigbee/ota_index.json

  - La version (ex. 1.1) est lue automatiquement dans le .bin (marqueur
    SONDE_FW_VERSION=x.y; écrit par le firmware). --version permet de la vérifier.
  - L'image est créée dans dist/ (non versionné) : sonde-radiateur-v1.1.ota
  - Dans l'index, seule l'entrée de la sonde radiateur est remplacée ; les autres
    appareils (SCD40, Levoit...) ne sont pas touchés.

Autre commande :
    python tools/make_ota.py --info <fichier.ota>     affiche l'en-tête d'une image OTA

Format : en-tête OTA Zigbee standard (ZCL OTA Upgrade, 56 octets) + un seul
sous-élément "Upgrade Image" (tag 0x0000) contenant le binaire applicatif ESP32,
identique à ce que produit image_builder_tool.py de l'esp-zigbee-sdk.

Uniquement la bibliothèque standard Python 3 (aucune installation nécessaire).
"""

import argparse
import hashlib
import json
import re
import struct
import sys
from pathlib import Path

# Doivent correspondre aux #define du firmware (OTA_MANUFACTURER / OTA_IMAGE_TYPE / ZB_MODEL)
MANUFACTURER_CODE = 0x1337   # 4919, même code fabricant "maison" que le capteur SCD40
IMAGE_TYPE = 0x0002          # 2 = sonde radiateur (le SCD40 utilise 1)
MODEL_ID = "H2-2xDS18B20"
FILE_PREFIX = "sonde-radiateur"
DEFAULT_RELEASE_URL = "https://github.com/Flo69au/sonde-zigbee-radiateur/releases/download"

OTA_FILE_ID = 0x0BEEF11E
OTA_HEADER_VERSION = 0x0100
OTA_HEADER_LEN = 56
ZIGBEE_STACK_VERSION = 0x0002
TAG_UPGRADE_IMAGE = 0x0000

MARKER_RE = re.compile(rb"SONDE_FW_VERSION=(\d{1,3})\.(\d{1,3});")
HEADER_FMT = "<IHHHHHIH32sI"  # = 56 octets
REPO_ROOT = Path(__file__).resolve().parent.parent


def parse_version(text):
    m = re.fullmatch(r"v?(\d{1,3})\.(\d{1,3})", text.strip())
    if not m:
        raise ValueError(f"version invalide '{text}' (attendu : 1.1, 1.2...)")
    major, minor = int(m.group(1)), int(m.group(2))
    if major > 255 or minor > 255:
        raise ValueError("majeur et mineur doivent être <= 255")
    return major, minor


def file_version(major, minor):
    return (major << 24) | (minor << 16)


def version_str(fv):
    return f"{fv >> 24}.{(fv >> 16) & 0xFF}"


def build_ota(app_bin, fv, header_string):
    hs = header_string.encode("ascii", "replace")[:32].ljust(32, b"\x00")
    sub = struct.pack("<HI", TAG_UPGRADE_IMAGE, len(app_bin)) + app_bin
    total = OTA_HEADER_LEN + len(sub)
    header = struct.pack(
        HEADER_FMT,
        OTA_FILE_ID,
        OTA_HEADER_VERSION,
        OTA_HEADER_LEN,
        0x0000,  # field control : aucun champ optionnel
        MANUFACTURER_CODE,
        IMAGE_TYPE,
        fv,
        ZIGBEE_STACK_VERSION,
        hs,
        total,
    )
    assert len(header) == OTA_HEADER_LEN
    return header + sub


def read_header(data):
    if len(data) < OTA_HEADER_LEN:
        raise ValueError("fichier trop court pour être une image OTA")
    (fid, hver, hlen, fctl, manuf, itype, fv, stack, hs, total) = struct.unpack_from(HEADER_FMT, data)
    if fid != OTA_FILE_ID:
        raise ValueError(f"identifiant OTA invalide 0x{fid:08X}")
    tag, tlen = struct.unpack_from("<HI", data, hlen)
    return {
        "header_length": hlen,
        "manufacturer_code": manuf,
        "image_type": itype,
        "file_version": fv,
        "header_string": hs.rstrip(b"\x00").decode("ascii", "replace"),
        "total_size": total,
        "file_size": len(data),
        "tag": tag,
        "tag_length": tlen,
    }


def cmd_info(path):
    data = Path(path).read_bytes()
    h = read_header(data)
    print(f"Image OTA : {path}")
    print(f"  version           v{version_str(h['file_version'])}  ({h['file_version']}, 0x{h['file_version']:08X})")
    print(f"  fabricant / type  {h['manufacturer_code']} (0x{h['manufacturer_code']:04X}) / {h['image_type']}")
    print(f"  en-tête           '{h['header_string']}', {h['header_length']} octets")
    print(f"  taille            {h['file_size']} octets (annoncée {h['total_size']})")
    print(f"  sous-élément      tag 0x{h['tag']:04X}, {h['tag_length']} octets")
    ok = (h["total_size"] == h["file_size"]
          and h["header_length"] + 6 + h["tag_length"] == h["file_size"])
    print("  cohérence         " + ("OK" if ok else "ERREUR"))
    return 0 if ok else 1


def is_ours(entry):
    """Entrée de la sonde radiateur dans l'index (les autres appareils sont conservés)."""
    if entry.get("modelId") == MODEL_ID:
        return True
    return (entry.get("manufacturerCode") == MANUFACTURER_CODE
            and entry.get("imageType") == IMAGE_TYPE
            and "modelId" not in entry)


def update_index(index_path, ota_name, ota_bytes, fv, url, header_string):
    index_path = Path(index_path)
    entries = []
    if index_path.exists():
        try:
            entries = json.loads(index_path.read_text(encoding="utf-8") or "[]")
        except json.JSONDecodeError as e:
            sys.exit(f"Index illisible ({index_path}) : {e}")
        if not isinstance(entries, list):
            sys.exit(f"Index inattendu ({index_path}) : la racine doit être une liste")

    kept = [e for e in entries if not is_ours(e)]
    removed = len(entries) - len(kept)
    kept.append({
        "fileName": ota_name,
        "fileVersion": fv,
        "fileSize": len(ota_bytes),
        "url": url,
        "imageType": IMAGE_TYPE,
        "manufacturerCode": MANUFACTURER_CODE,
        "modelId": MODEL_ID,
        "otaHeaderString": header_string,
        "sha512": hashlib.sha512(ota_bytes).hexdigest(),
    })
    index_path.parent.mkdir(parents=True, exist_ok=True)
    index_path.write_text(json.dumps(kept, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return removed, len(kept) - 1


def main():
    p = argparse.ArgumentParser(description="Image OTA Zigbee pour la sonde radiateur ESP32-H2")
    p.add_argument("bin", nargs="?", help="binaire applicatif exporté par l'IDE Arduino (.ino.bin)")
    p.add_argument("--version", help="vérifie que le .bin est bien dans cette version (ex. 1.1)")
    p.add_argument("--index", help="ota_index.json à mettre à jour (clone local du dépôt de l'index)")
    p.add_argument("--release-url", default=DEFAULT_RELEASE_URL,
                   help="base des téléchargements des Releases GitHub (défaut : %(default)s)")
    p.add_argument("--out-dir", default=str(REPO_ROOT / "dist"),
                   help="dossier de sortie du .ota (défaut : dist/ du dépôt)")
    p.add_argument("--info", metavar="OTA", help="affiche l'en-tête d'une image .ota existante")
    a = p.parse_args()

    if a.info:
        return cmd_info(a.info)
    if not a.bin:
        p.error("indique le fichier .bin (ou --info <fichier.ota>)")

    bin_path = Path(a.bin)
    app = bin_path.read_bytes()
    if not app or app[0] != 0xE9:
        sys.exit(f"{bin_path} ne ressemble pas à un binaire ESP32 (octet magique 0xE9 absent). "
                 "Utilise le .ino.bin, pas le .merged.bin ni le .bootloader.bin.")

    found = MARKER_RE.search(app)
    if not found:
        sys.exit("Version introuvable dans le .bin (marqueur SONDE_FW_VERSION absent). "
                 "Vérifie que c'est bien le firmware de la sonde radiateur.")
    major, minor = int(found.group(1)), int(found.group(2))

    if a.version and parse_version(a.version) != (major, minor):
        sys.exit(f"Le .bin est en version {major}.{minor}, pas {a.version}. Une image OTA doit "
                 "porter la version du firmware qu'elle contient, sinon la sonde se remettra à "
                 "jour en boucle. Incrémente FW_VERSION_MINOR et réexporte le binaire.")

    fv = file_version(major, minor)
    tag = f"v{major}.{minor}"
    header_string = f"SONDE-RADIATEUR {tag}"
    ota = build_ota(app, fv, header_string)

    out_dir = Path(a.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    ota_name = f"{FILE_PREFIX}-{tag}.ota"
    ota_path = out_dir / ota_name
    ota_path.write_bytes(ota)
    url = f"{a.release_url.rstrip('/')}/{tag}/{ota_name}"

    print(f"Image OTA {tag} ({fv}) : {ota_path}  ({len(ota)} octets)")
    print(f"URL attendue  : {url}")

    if a.index:
        removed, others = update_index(a.index, ota_name, ota, fv, url, header_string)
        print(f"Index mis à jour : {a.index}  ({others} autre(s) appareil(s) conservé(s)"
              + (f", ancienne entrée de la sonde remplacée" if removed else "") + ")")

    print("\nÀ faire ensuite :")
    print(f"  1. GitHub sonde-zigbee-radiateur > Releases > nouvelle release, tag {tag},")
    print(f"     joindre {ota_name}")
    if a.index:
        print("  2. Pousser ota_index.json (dépôt capteur-temp-rature-scd40_zigbee)")
    else:
        print("  2. Relancer avec --index <...>/ota_index.json pour mettre l'index à jour")
    print("  3. Zigbee2MQTT > OTA > Vérifier sur la sonde > Mettre à jour")
    return 0


if __name__ == "__main__":
    sys.exit(main())
