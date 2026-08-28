#!/usr/bin/env bash
# Prepare this card for an image whose clones boot the REAL receiver.
#
#   sudo ./tools/image-real.sh
#
# This is the image for someone with physical ESP32 wearables: the clone brings
# up its own hidden WiFi AP, boards join it, and biometric data streams to the
# app over BLE. It is the default and the one most people want.
#
# Deliberately a thin wrapper — prepare-image.sh holds the one implementation,
# so the two variants cannot drift apart in the ways that actually hurt (a strip
# step fixed here and forgotten there).
set -euo pipefail
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/prepare-image.sh" --mode real "$@"
