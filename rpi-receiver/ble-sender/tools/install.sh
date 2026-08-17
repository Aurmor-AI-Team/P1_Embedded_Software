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

echo "==> systemd unit"
install -m 644 "$SRC/tools/$UNIT" "/etc/systemd/system/$UNIT"
systemctl daemon-reload
systemctl enable "$UNIT"

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
