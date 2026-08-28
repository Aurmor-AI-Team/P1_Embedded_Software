#!/usr/bin/env bash
# Strip this Pi's IDENTITY and choose which receiver a clone boots into, so the
# SD card can be imaged for other people.
#
#   sudo ./tools/prepare-image.sh                 # real receiver (default)
#   sudo ./tools/prepare-image.sh --mode mock     # emulated-wearable receiver
#
# Or use the wrappers, which say the same thing more legibly:
#   sudo ./tools/image-real.sh
#   sudo ./tools/image-mock.sh
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
# own board serial, so a stripped image is self-configuring. Both modes need
# this: mock_receiver.py calls wifi_ap.load_config() exactly like the real one.
set -euo pipefail

REAL_UNIT=aurmor-receiver.service
MOCK_UNIT=aurmor-receiver-mock.service
# The hand-written unit that predates tools/install.sh. Left enabled alongside
# the real one it would give a clone two GATT servers on one adapter and two
# binders on UDP 5005 — so imaging retires it rather than tolerating it.
LEGACY_UNIT=ble_sender.service

MODE=real
while [[ $# -gt 0 ]]; do
    case $1 in
        --mode)   MODE="${2:-}"; shift 2 ;;
        --mode=*) MODE="${1#*=}"; shift ;;
        -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1 (try --help)" >&2; exit 2 ;;
    esac
done

case $MODE in
    real) WANTED=$REAL_UNIT; UNWANTED=$MOCK_UNIT
          BLURB="the REAL receiver — streams from physical ESP32 wearables over the hidden AP" ;;
    mock) WANTED=$MOCK_UNIT; UNWANTED=$REAL_UNIT
          BLURB="the MOCK receiver — ten EMULATED wearables, no boards and no AP needed" ;;
    *) echo "--mode must be 'real' or 'mock' (got '${MODE}')" >&2; exit 2 ;;
esac

# Where ble-sender actually lives. Auto-detects the two layouts in use — the
# install.sh one and a hand-deployed home directory — so this can't silently
# "strip" an identity file it never found and hand you an image that still
# carries one. Override with AURMOR_DIR=... if yours is elsewhere.
for candidate in "${AURMOR_DIR:-}" /opt/aurmor/ble-sender \
                 /home/*/rpi-receiver/ble-sender ~/rpi-receiver/ble-sender; do
    [[ -n $candidate && -d $candidate ]] && { BLE_DIR="$candidate"; break; }
done
: "${BLE_DIR:?could not find ble-sender; set AURMOR_DIR=/path/to/ble-sender}"

[[ $EUID -eq 0 ]] || { echo "run with sudo" >&2; exit 1; }

echo "==> Using $BLE_DIR"
echo "==> Mode: $MODE — clones will boot $BLURB"

# Both units must exist before we can pick one. Without this check a Pi that
# never ran install.sh would strip happily, enable nothing, and produce an image
# whose clones boot into no receiver at all.
missing=()
for u in "$REAL_UNIT" "$MOCK_UNIT"; do
    [[ -f "/etc/systemd/system/$u" ]] || missing+=("$u")
done
if (( ${#missing[@]} )); then
    echo >&2
    echo "!!  Not installed: ${missing[*]}" >&2
    echo "!!  Run 'sudo ./tools/install.sh' first — otherwise this script cannot" >&2
    echo "!!  choose which receiver the image boots into." >&2
    exit 1
fi

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
# The mock is in this list deliberately: mock_receiver.py calls load_config(),
# so a mock left running would regenerate receiver_config.json seconds after the
# strip deletes it, and the image would ship this Pi's identity after all.
systemctl stop "$REAL_UNIT" "$MOCK_UNIT" "$LEGACY_UNIT" 2>/dev/null || true
systemctl stop aurmor-wifi-evict.service 2>/dev/null || true

echo "==> Retiring the hand-written $LEGACY_UNIT, if present"
if [[ -f "/etc/systemd/system/$LEGACY_UNIT" ]]; then
    systemctl disable "$LEGACY_UNIT" 2>/dev/null || true
    install -d /var/backups
    mv -v "/etc/systemd/system/$LEGACY_UNIT" \
          "/var/backups/$LEGACY_UNIT.retired-$(date +%Y%m%d%H%M%S)"
    rm -rf "/etc/systemd/system/$LEGACY_UNIT.d"
    systemctl daemon-reload
else
    echo "    not present"
fi

echo "==> Selecting the boot receiver"
# Exactly one. Conflicts= in the units stops them running together, but only an
# enable/disable pair decides which one a clone comes up in.
systemctl disable "$UNWANTED" 2>/dev/null || true
systemctl enable "$WANTED"

echo "==> Removing this Pi's receiver identity"
# SSID + AP password + pi_id + receiver_name. The .bak too — it is the same
# identity, and shipping it would let a clone "restore" someone else's.
rm -fv "$BLE_DIR/receiver_config.json" "$BLE_DIR/receiver_config.json.bak"

# Copies outside the install dir are NOT deleted: one of them is usually the
# operator's own restore point, and destroying that is how a working Pi gets
# lost. They are inert (nothing reads them automatically) but they do carry this
# Pi's identity into the image, so name them and let a human decide.
strays=()
while IFS= read -r -d '' f; do strays+=("$f"); done < <(
    find /home /root -maxdepth 6 -name 'receiver_config*' \
         -not -path "$BLE_DIR/*" -print0 2>/dev/null || true)
if (( ${#strays[@]} )); then
    echo
    echo "!!  Other copies of a receiver identity are still on this card:"
    printf '!!    %s\n' "${strays[@]}"
    echo "!!  They are inert — nothing loads them automatically — but they do"
    echo "!!  ship in the image. Delete them yourself if that matters to you."
    echo
fi

echo "==> Marking first boot"
# Without this the strip and the start gate deadlock: aurmor-receiver.service
# runs --selftest as ExecStartPre, the self-test FAILs on the missing config we
# just deleted, systemd therefore never runs ExecStart, and ExecStart is the
# only thing that would have created that config. The clone boots, retries ten
# times, and parks in 'failed' with no AP and no BLE. The marker tells the
# self-test this absence is expected exactly once.
install -d /var/lib/aurmor
: > /var/lib/aurmor/first-boot

echo "==> Removing the AP profile"
# Carries the old SSID and password; the real service rebuilds it from the fresh
# config on first boot. (A mock image simply never brings one up.)
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

echo
echo "==> Verification"
fail=0
for u in "$WANTED" "$UNWANTED"; do
    state="$(systemctl is-enabled "$u" 2>&1 || true)"
    want=enabled; [[ $u == "$UNWANTED" ]] && want=disabled
    if [[ $state == "$want" ]]; then
        printf '    [PASS] %-32s %s\n' "$u" "$state"
    else
        printf '    [FAIL] %-32s %s (expected %s)\n' "$u" "$state" "$want"
        fail=1
    fi
done
if [[ -e $BLE_DIR/receiver_config.json ]]; then
    echo "    [FAIL] receiver_config.json still present — something regenerated it"
    fail=1
else
    printf '    [PASS] %-32s removed\n' "receiver_config.json"
fi
if [[ -e /var/lib/aurmor/first-boot ]]; then
    printf '    [PASS] %-32s present\n' "first-boot marker"
else
    echo "    [FAIL] first-boot marker missing — clones would deadlock at ExecStartPre"
    fail=1
fi
(( fail == 0 )) || { echo; echo "!!  NOT ready to image — fix the FAILs above." >&2; exit 1; }

SUGGESTED="aurmor-rpi-$(date +%Y%m%d)-${MODE}.img.gz"
cat <<EOF

Ready to image ($MODE). Now:

  sudo shutdown -h now      # then capture the card, and do NOT boot it again

Capture (from a Mac, card as /dev/diskN — check with 'diskutil list'):

  diskutil unmountDisk /dev/diskN
  sudo dd if=/dev/rdiskN bs=4m | pigz -c > $SUGGESTED
  pigz -t $SUGGESTED && shasum -a 256 $SUGGESTED | tee $SUGGESTED.sha256

Booting the card again regenerates an identity and makes it dirty: re-run this
script before capturing if that happens.

On first boot each clone will:
  * generate its own receiver_config.json (identity from ITS board serial)
  * start $WANTED
  * regenerate SSH host keys and machine-id
EOF

if [[ $MODE == real ]]; then
cat <<EOF
  * build its own aurmor-ap profile from that config

Verify on a fresh clone before handing it over:
  sudo systemctl status ${WANTED%.service}
  /opt/aurmor/venv/bin/python /opt/aurmor/ble-sender/ble_sender.py --selftest

The self-test must print all PASS. If it does not, the clone is broken in one of
the ways that otherwise only shows up as "the app can't see my wearable".
EOF
else
cat <<EOF

This image brings up NO WiFi access point — emulated boards talk over loopback,
so there is nothing for a physical wearable to join. Pair it with the app's
"Use mock wearables" toggle on the New Group Session screen; both halves are
needed.

Verify on a fresh clone before handing it over:
  sudo systemctl status ${WANTED%.service}
  journalctl -u $WANTED -b | grep -i "mock\|presence"
EOF
fi
