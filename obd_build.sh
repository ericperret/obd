#!/bin/bash
# ============================================================
# obd_build.sh
# ------------------------------------------------------------
# Compile / flash / monitor du firmware obd_can_bridge.ino
# Cible : Seeed XIAO ESP32-S3, arduino-cli dans ~/bin
# Auteur : Eric Perret - assistance Claude
# Version : 1.0 - 2026-07-06
# ------------------------------------------------------------
# Usage :
#   ./obd_build.sh          compile seulement
#   ./obd_build.sh flash    compile + televerse
#   ./obd_build.sh mon      moniteur serie (quitter: Ctrl-C)
#   ./obd_build.sh all      compile + televerse + moniteur
# ============================================================

set -e
export PATH="$PATH:$HOME/bin"

FQBN="esp32:esp32:XIAO_ESP32S3"
SKETCH_DIR="$HOME/obd_can_bridge"
SRC="$HOME/Downloads/odb/obd_can_bridge.ino"
BAUD=115200

# --- arduino-cli exige un dossier portant le nom du sketch ---
mkdir -p "$SKETCH_DIR"
if [ -f "$SRC" ]; then
    cp "$SRC" "$SKETCH_DIR/obd_can_bridge.ino"
fi
[ -f "$SKETCH_DIR/obd_can_bridge.ino" ] || { echo "ERREUR: .ino introuvable ($SRC)"; exit 1; }

# --- detection du port (XIAO S3 = USB natif -> ttyACM) --------
PORT=$(ls /dev/ttyACM* 2>/dev/null | head -1)
[ -z "$PORT" ] && PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -1)

case "${1:-compile}" in
  compile)
    arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"
    echo "=== COMPILATION OK ==="
    ;;
  flash)
    [ -z "$PORT" ] && { echo "ERREUR: aucun port serie (carte branchee ?)"; exit 1; }
    arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"
    arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$SKETCH_DIR"
    echo "=== FLASH OK sur $PORT ==="
    ;;
  mon)
    [ -z "$PORT" ] && { echo "ERREUR: aucun port serie"; exit 1; }
    echo "Moniteur $PORT @ $BAUD - commandes: PING? INIT? DTC? EGR? - Ctrl-C pour quitter"
    arduino-cli monitor -p "$PORT" -c baudrate=$BAUD
    ;;
  all)
    "$0" flash
    sleep 2
    "$0" mon
    ;;
  *)
    echo "Usage: $0 [compile|flash|mon|all]"
    exit 1
    ;;
esac
