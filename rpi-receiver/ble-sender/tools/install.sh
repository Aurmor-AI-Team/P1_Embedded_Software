#!/usr/bin/env bash
# One-time receiver setup. Idempotent — safe to re-run after an update.
#
#   sudo ./tools/install.sh
#
# Exists because every manual step in a README is a step a colleague will skip,
# and the skipped ones fail SILENTLY: a receiver without NetworkManager
# privileges starts fine, streams BLE fine, and simply never brings up or
# repairs the WiFi AP that wearables need. The symptom surfaces days later as
# "the app can't see my wearable".
set -euo pipefail

PREFIX=/opt/aurmor
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # .../ble-sender
UNIT=aurmor-receiver.service
MOCK_UNIT=aurmor-receiver-mock.service
# The hand-written unit that predates this installer. Left enabled alongside
# $UNIT it gives the Pi two GATT servers on one Bluetooth adapter and two
# binders on UDP 5005 — which presents as flaky pairing that no amount of AP
# debugging explains.
LEGACY_UNIT=ble_sender.service
SERVICE_USER="${SUDO_USER:-$USER}"

[[ $EUID -eq 0 ]] || { echo "run with sudo" >&2; exit 1; }

echo "==> System packages"
apt-get update -qq
# bluezero builds against system dbus/gi; bluez provides bluetoothctl + the
# GATT stack; NetworkManager owns the AP.
apt-get install -y -qq \
  python3-venv python3-dbus python3-gi \
  libdbus-1-dev libgirepository1.0-dev \
  bluez network-manager

echo "==> Installing to $PREFIX"
install -d "$PREFIX"
# --delete keeps a re-run clean, but receiver_config.json lives in the install
# dir and IS this Pi's identity: deleting it makes the service mint a new one on
# next start, orphaning every wearable already provisioned to this receiver.
rsync -a --delete \
  --exclude='__pycache__' --exclude='*.pyc' \
  --exclude='receiver_config.json*' \
  "$SRC/" "$PREFIX/ble-sender/"

echo "==> Receiver identity"
# The rsync above excludes receiver_config.json, so a Pi migrating from the
# hand-deployed home-directory layout would arrive here with NO identity and
# mint a brand new one on first start — new SSID, AP password and pi_id,
# orphaning every provisioned wearable and the receiver's registration in the
# app. Carry the existing one across. Only ever fills a gap; never overwrites.
NEW_CFG="$PREFIX/ble-sender/receiver_config.json"
if [[ -f $NEW_CFG ]]; then
  echo "    keeping existing $NEW_CFG"
else
  for legacy in /home/*/rpi-receiver/ble-sender/receiver_config.json \
                /root/rpi-receiver/ble-sender/receiver_config.json; do
    [[ -f $legacy ]] || continue
    cp -a "$legacy" "$NEW_CFG"
    echo "    migrated identity from $legacy"
    break
  done
  [[ -f $NEW_CFG ]] || echo "    none found — a new identity is generated on first start"
fi

echo "==> Python environment"
[[ -d $PREFIX/venv ]] || python3 -m venv --system-site-packages "$PREFIX/venv"
"$PREFIX/venv/bin/pip" install -q --upgrade pip
"$PREFIX/venv/bin/pip" install -q -r "$PREFIX/ble-sender/requirements.txt"

echo "==> NetworkManager privileges for $SERVICE_USER"
# The service itself runs as root (see the unit), so this is for INTERACTIVE use
# — running ble_sender.py by hand over SSH to debug, without sudo.
usermod -aG netdev "$SERVICE_USER" || true
cat > /etc/polkit-1/rules.d/50-aurmor-networkmanager.rules <<'EOF'
// Let the netdev group manage NetworkManager connections without a password
// prompt. Without this, `nmcli connection add/modify` fails over SSH — polkit
// treats non-login sessions as inactive — and the AP can never be repaired.
polkit.addRule(function(action, subject) {
    if (action.id.indexOf("org.freedesktop.NetworkManager.") === 0 &&
        subject.isInGroup("netdev")) {
        return polkit.Result.YES;
    }
});
EOF
systemctl restart polkit || true

echo "==> Retiring the hand-written $LEGACY_UNIT, if present"
if [[ -f /etc/systemd/system/$LEGACY_UNIT ]]; then
  systemctl disable --now "$LEGACY_UNIT" 2>/dev/null || true
  # Moved, not deleted: it is the only record of how this Pi used to boot.
  install -d /var/backups
  mv -v "/etc/systemd/system/$LEGACY_UNIT" \
        "/var/backups/$LEGACY_UNIT.retired-$(date +%Y%m%d%H%M%S)"
  rm -rf "/etc/systemd/system/$LEGACY_UNIT.d"
  systemctl daemon-reload
else
  echo "    not present"
fi

echo "==> systemd unit"
install -m 644 "$SRC/tools/$UNIT" "/etc/systemd/system/$UNIT"
systemctl daemon-reload
systemctl enable "$UNIT"

# The emulated-wearable variant (mock_receiver.py) is installed but deliberately
# NOT enabled: it Conflicts= with the real receiver and is meant to be swapped in
# by hand for a test session. See the unit's header and README.md.
if [[ -f $SRC/tools/aurmor-receiver-mock.service ]]; then
  install -m 644 "$SRC/tools/aurmor-receiver-mock.service" /etc/systemd/system/
  systemctl daemon-reload
fi

echo "==> Stale-station eviction (ghost associations after an ungraceful reboot)"
if [[ -f $SRC/tools/evict_stale_stations.py ]]; then
  install -m 755 "$SRC/tools/evict_stale_stations.py" /usr/local/bin/
  install -m 644 "$SRC/tools/aurmor-wifi-evict.service" /etc/systemd/system/
  systemctl daemon-reload
  systemctl enable --now aurmor-wifi-evict.service || true
fi

echo "==> First start (creates the AP profile and the receiver identity)"
systemctl restart "$UNIT"
sleep 3

echo
echo "==> Self-test"
# Non-fatal here: the report is more useful than an abrupt exit, and a fresh Pi
# may still be settling. The unit re-runs this on every start via ExecStartPre.
"$PREFIX/venv/bin/python" "$PREFIX/ble-sender/ble_sender.py" --selftest || true

cat <<EOF

Installed. Two things worth doing now:

  1. Back up this Pi's identity — it exists in exactly one place and cannot be
     regenerated. Losing it orphans every wearable provisioned to this receiver:
       sudo cp $PREFIX/ble-sender/receiver_config.json ~/receiver_config.json.bak

  2. If you are capturing an image for other people, run tools/prepare-image.sh
     FIRST. Otherwise every clone ships this Pi's SSID, password and pi_id, and
     they will fight over the same identity.

  Logs:   journalctl -u $UNIT -f
  Check:  $PREFIX/venv/bin/python $PREFIX/ble-sender/ble_sender.py --selftest
EOF
