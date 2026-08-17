#!/usr/bin/env bash
# Strip this Pi's IDENTITY so the SD card can be imaged for other people.
#
#   sudo ./tools/prepare-image.sh     # then shut down and capture the card
#
# A receiver's identity is not just a name. It is the hidden SSID, the AP
# password and the pi_id that wearables verify against — all generated once, on
# first run, and stored in receiver_config.json. Clone an image without removing
# it and every colleague's receiver claims the SAME SSID and pi_id: co-located
# receivers become one roaming pool, a wearable provisioned to yours happily
# talks to theirs, and the app cannot tell them apart (it registers a receiver
# by name, and names would collide too).
#
# Each Pi regenerates its own identity on next boot, deriving the suffix from its
# own board serial, so a stripped image is self-configuring.
set -euo pipefail

# Where ble-sender actually lives. Auto-detects the two layouts in use — the
# install.sh one and a hand-deployed home directory — so this can't silently
# "strip" an identity file it never found and hand you an image that still
# carries one. Override with AURMOR_DIR=... if yours is elsewhere.
UNIT=aurmor-receiver.service
for candidate in "${AURMOR_DIR:-}" /opt/aurmor/ble-sender \
                 /home/*/rpi-receiver/ble-sender ~/rpi-receiver/ble-sender; do
    [[ -n $candidate && -d $candidate ]] && { BLE_DIR="$candidate"; break; }
done
: "${BLE_DIR:?could not find ble-sender; set AURMOR_DIR=/path/to/ble-sender}"
echo "==> Using $BLE_DIR"

[[ $EUID -eq 0 ]] || { echo "run with sudo" >&2; exit 1; }

if [[ -f "$BLE_DIR/receiver_config.json" ]]; then
    echo
    echo "!!  This DELETES this Pi's identity (SSID, AP password, pi_id)."
    echo "!!  On next boot it generates a new one, and wearables already"
    echo "!!  provisioned to this receiver will stop connecting."
    echo "!!  Back it up first if you want this Pi to keep working as-is:"
    echo "!!    cp $BLE_DIR/receiver_config.json ~/receiver_config.json.safe"
    echo
    read -rp "Continue? [y/N] " reply
    [[ ${reply,,} == y ]] || exit 1
fi

echo "==> Stopping services"
systemctl stop "$UNIT" ble_sender.service 2>/dev/null || true
systemctl stop aurmor-wifi-evict.service 2>/dev/null || true

echo "==> Removing this Pi's receiver identity"
# SSID + AP password + pi_id + receiver_name. The .bak too — it is the same
# identity, and shipping it would let a clone "restore" someone else's.
rm -fv "$BLE_DIR/receiver_config.json" "$BLE_DIR/receiver_config.json.bak"

echo "==> Removing the AP profile"
# Carries the old SSID and password; the service rebuilds it from the fresh
# config on first boot.
nmcli connection delete aurmor-ap 2>/dev/null || true

echo "==> Removing Bluetooth identity + bonds"
# Bonded phones would otherwise be inherited by every clone, and the adapter
# alias is set per-Pi to the receiver_name (which is about to change).
rm -rf /var/lib/bluetooth/* 2>/dev/null || true
bluetoothctl system-alias "" 2>/dev/null || true

echo "==> Clearing caches and logs"
find "$BLE_DIR" -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null || true
journalctl --rotate 2>/dev/null || true
journalctl --vacuum-time=1s 2>/dev/null || true
rm -f /var/log/*.gz /var/log/*.1 2>/dev/null || true

echo "==> Clearing host identity (so clones aren't twins)"
# Without this every clone shares SSH host keys — clients scream about changed
# keys — and a machine-id, which some DHCP setups use to hand out the same lease
# to two machines.
#
# Deleting the host keys ALONE bricks SSH: sshd refuses to start without them,
# and Raspberry Pi OS does NOT regenerate them on boot by default. The result is
# "Connection refused" on port 22 and a Pi that can only be recovered with a
# keyboard attached. So install the regeneration BEFORE removing the keys, and
# verify it is enabled — if that fails, keep the keys rather than hand someone
# an unreachable box.
cat > /etc/systemd/system/aurmor-regen-ssh-host-keys.service <<'EOF'
[Unit]
Description=Regenerate SSH host keys on first boot after imaging
# Must win the race with sshd, which will not start without keys.
Before=ssh.service sshd.service
ConditionPathExistsGlob=|!/etc/ssh/ssh_host_*_key

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/bin/ssh-keygen -A
# One-shot by design: disable itself once the keys exist.
ExecStartPost=/bin/systemctl disable aurmor-regen-ssh-host-keys.service

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
if systemctl enable aurmor-regen-ssh-host-keys.service 2>/dev/null; then
    rm -f /etc/ssh/ssh_host_* 2>/dev/null || true
    echo "    host keys removed; they regenerate on first boot"
else
    echo "!!  could not enable SSH host-key regeneration — KEEPING the existing" >&2
    echo "!!  host keys. Clones will share them (ssh will warn), but they will" >&2
    echo "!!  at least be reachable. Fix before shipping the image." >&2
fi
: > /etc/machine-id
rm -f /var/lib/dbus/machine-id 2>/dev/null || true

# Wipe the shell history of whoever built the image (it usually contains the
# WiFi password they typed while debugging).
rm -f /root/.bash_history /home/*/.bash_history 2>/dev/null || true

cat <<'EOF'

Ready to image. Now:

  sudo shutdown -h now      # then capture the card

On first boot each clone will:
  * generate its own receiver_config.json (SSID/password/pi_id from ITS board serial)
  * build its own aurmor-ap profile from that config
  * regenerate SSH host keys and machine-id

Verify on a fresh clone before handing it over:
  sudo systemctl status aurmor-receiver
  /opt/aurmor/venv/bin/python /opt/aurmor/ble-sender/ble_sender.py --selftest

The self-test must print all PASS. If it does not, the clone is broken in one of
the ways that otherwise only shows up as "the app can't see my wearable".
EOF
