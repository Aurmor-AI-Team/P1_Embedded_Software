#!/usr/bin/env bash
# Prepare this card for an image whose clones boot the MOCK receiver.
#
#   sudo ./tools/image-mock.sh
#
# This is the image for someone with NO hardware: the clone stands up ten
# emulated wearables in software (mock_receiver.py) and drives the whole group
# path — device picker, per-participant assignment, working modes, impacts,
# teardown — from a Pi and a phone alone.
#
# The clone brings up NO access point: emulated boards talk over loopback, so
# there is nothing for a physical wearable to join. The app side needs its
# "Use mock wearables" toggle on the New Group Session screen; both halves are
# required, one invents the devices and the other invents their data.
#
# Deliberately a thin wrapper — see image-real.sh.
set -euo pipefail
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/prepare-image.sh" --mode mock "$@"
