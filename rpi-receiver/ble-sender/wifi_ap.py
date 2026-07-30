"""Hidden WiFi access point on the Pi, managed through NetworkManager (nmcli).

The receiver permanently hosts a hidden AP; ESP32 wearables join it after the
mobile app forwards the credentials over BLE (see the WIFI_CREDS
characteristic in ble_sender.py). Requires Raspberry Pi OS Bookworm or any
system where nmcli manages wlan0.

Credentials and identity live in ``receiver_config.json`` next to this file,
generated with random secrets on first run so every Pi gets its own.
"""
from __future__ import annotations

import json
import secrets
import subprocess
import sys
from pathlib import Path

# NetworkManager assigns this gateway address to the AP interface when the
# connection uses ipv4.method=shared.
AP_IP = "10.42.0.1"

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "receiver_config.json"

# The old shared default. Several receivers side by side must NOT share an SSID
# (the ESP32 would treat them as one roaming network and pick by signal, not
# identity), so a config still carrying this literal is migrated to a unique one.
_LEGACY_SHARED_SSID = "aurmor-pi-ap"

_DEFAULTS = {
    "ap_con_name": "aurmor-ap",
    # The Pi's built-in radio (brcmfmac) associates roughly 8-10 stations before
    # it becomes unreliable, which is a chip-firmware limit, not a setting. For
    # a squad, point this at a USB adapter (usually wlan1) whose driver uses
    # mac80211 properly. Run `python3 wifi_ap.py --probe` to see what the Pi
    # has and which interfaces actually advertise AP mode.
    "ap_ifname": "wlan0",
    # Channels 1/6/11 are active-scannable in every regulatory domain. An
    # auto-picked 12/13 is passive-scan-only for world-domain ESP32s, and a
    # hidden AP can't be found by passive scan at all.
    "ap_channel": 6,
    "udp_port": 5005,
}


def main(argv=None) -> int:
    import argparse
    p = argparse.ArgumentParser(description="Hidden AP helper / interface probe")
    p.add_argument("--probe", action="store_true",
                   help="list wireless interfaces and their AP capability")
    args = p.parse_args(argv)
    if args.probe:
        return print_probe()
    cfg = load_config()
    print(json.dumps({k: v for k, v in cfg.items() if k != "ap_password"}, indent=2))
    return 0


def _pi_serial_suffix():
    """Last 4 hex chars of the Raspberry Pi's unique board serial, or None off-Pi.

    The board serial is stable, globally unique per unit, readable before any
    network interface comes up, and immune to MAC randomization — a better
    identity source than the WiFi MAC.
    """
    serial = ""
    # Device-tree node (preferred): a NUL-terminated ASCII serial string.
    try:
        raw = Path("/sys/firmware/devicetree/base/serial-number").read_bytes()
        serial = raw.split(b"\x00", 1)[0].decode("ascii", "ignore")
    except OSError:
        pass
    if not serial:
        # Fallback: the "Serial" line in /proc/cpuinfo.
        try:
            for line in Path("/proc/cpuinfo").read_text().splitlines():
                if line.lower().startswith("serial"):
                    serial = line.split(":", 1)[1]
                    break
        except OSError:
            pass
    serial = "".join(c for c in serial.lower() if c in "0123456789abcdef")
    return serial[-4:] if len(serial) >= 4 else None


def _unique_suffix() -> str:
    # Pi board serial is stable + human-recognizable; random is the fallback
    # (e.g. on a dev host). Either way it is stored once in receiver_config.json.
    return _pi_serial_suffix() or secrets.token_hex(2)


def load_config(path: Path = DEFAULT_CONFIG_PATH) -> dict:
    """Read the receiver config, creating it with generated secrets + a unique
    per-Pi identity if absent."""
    path = Path(path)
    if path.exists():
        cfg = json.loads(path.read_text())
    else:
        cfg = {}
    changed = False
    for key, value in _DEFAULTS.items():
        if key not in cfg:
            cfg[key] = value
            changed = True

    # Per-Pi identity: a unique hidden SSID (so the ESP32 targets exactly this
    # receiver) and a matching human-readable receiver name, sharing one suffix.
    ssid = cfg.get("ap_ssid")
    if not ssid or ssid == _LEGACY_SHARED_SSID:
        suffix = _unique_suffix()
        cfg["ap_ssid"] = f"aurmor-pi-{suffix}"
        cfg["receiver_name"] = f"aurmor-rpi-{suffix}"
        changed = True
    elif not cfg.get("receiver_name"):
        cfg["receiver_name"] = f"aurmor-rpi-{ssid.rsplit('-', 1)[-1]}"
        changed = True

    if not cfg.get("ap_password"):
        cfg["ap_password"] = secrets.token_urlsafe(9)  # 12 chars, WPA2-valid
        changed = True
    if not cfg.get("pi_id"):
        cfg["pi_id"] = secrets.randbelow(0xFFFFFFFE) + 1  # nonzero u32
        changed = True
    if changed:
        path.write_text(json.dumps(cfg, indent=2) + "\n")
        print(f"# wrote receiver config -> {path}", file=sys.stderr)
    return cfg


def wifi_creds_json(cfg: dict) -> bytes:
    """The WIFI_CREDS characteristic payload the app forwards to the ESP32."""
    return json.dumps({
        "ssid": cfg["ap_ssid"],
        "password": cfg["ap_password"],
        "ip": AP_IP,
        "port": cfg["udp_port"],
        "pi_id": cfg["pi_id"],
        "receiver_name": cfg["receiver_name"],
    }, separators=(",", ":")).encode("utf-8")


# --------------------------------------------------------------------------- #
# Interface probe: which radio should host the AP?
# --------------------------------------------------------------------------- #
# Chipsets whose in-kernel mac80211 drivers do AP mode properly. The number is
# a realistic associated-station count for a squad of wearables that all
# transmit on a cadence — NOT the driver's theoretical max, which is meaningless
# once 802.11 contention is taken into account.
KNOWN_AP_CHIPSETS = {
    "mt7921u":  ("MediaTek MT7921AU (WiFi 6)", 64),
    "mt7921":   ("MediaTek MT7921", 64),
    "mt76x2u":  ("MediaTek MT7612U", 48),
    "mt76":     ("MediaTek mt76 family", 48),
    "ath9k_htc": ("Atheros AR9271 (2.4 GHz only)", 24),
    "carl9170": ("Atheros AR9170", 16),
    "brcmfmac": ("Broadcom/Cypress — Pi built-in, AP mode is firmware-limited", 8),
    "rtl8xxxu": ("Realtek (in-kernel)", 16),
}


def _iw(args: list) -> str:
    try:
        r = subprocess.run(["iw", *args], capture_output=True, text=True)
        return r.stdout if r.returncode == 0 else ""
    except (FileNotFoundError, OSError):
        return ""


def probe_ap_interfaces() -> list:
    """Report every wireless interface and whether it can host the AP.

    Answers the question you actually have when a dongle arrives: did the Pi
    see it, does its driver advertise AP mode, and is it the built-in radio
    (~8 stations) or something that can carry a squad?
    """
    out = []
    dev = _iw(["dev"])
    ifaces = [ln.split()[-1] for ln in dev.splitlines()
              if ln.strip().startswith("Interface")]
    for name in ifaces:
        driver = ""
        try:
            driver = Path(f"/sys/class/net/{name}/device/driver").resolve().name
        except OSError:
            pass
        phy = ""
        try:
            phy = Path(f"/sys/class/net/{name}/phy80211").resolve().name
        except OSError:
            pass
        info = _iw(["phy", phy, "info"]) if phy else ""
        modes = info.split("Supported interface modes:", 1)
        ap_capable = False
        if len(modes) > 1:
            block = modes[1].split("Band ", 1)[0]
            ap_capable = any(ln.strip() == "* AP" for ln in block.splitlines())
        label, stations = KNOWN_AP_CHIPSETS.get(
            driver, (driver or "unknown driver", None))
        out.append({
            "interface": name,
            "phy": phy,
            "driver": driver,
            "chipset": label,
            "ap_capable": ap_capable,
            "realistic_stations": stations,
            "builtin": driver == "brcmfmac",
        })
    return out


def print_probe() -> int:
    rows = probe_ap_interfaces()
    if not rows:
        print("no wireless interfaces found (is `iw` installed? "
              "sudo apt install -y iw)", file=sys.stderr)
        return 1
    print(f"{'iface':8} {'driver':12} {'AP?':4} {'~stations':10} chipset")
    for r in rows:
        n = r["realistic_stations"]
        print(f"{r['interface']:8} {r['driver'][:12]:12} "
              f"{'yes' if r['ap_capable'] else 'NO':4} "
              f"{(str(n) if n else '?'):10} {r['chipset']}")
    best = max((r for r in rows if r["ap_capable"]),
               key=lambda r: r["realistic_stations"] or 0, default=None)
    if best is None:
        print("\nNo interface advertises AP mode. A USB adapter with an "
              "in-kernel mac80211 driver (mt7921u / mt76x2u) is the fix.")
        return 1
    print(f"\nBest AP interface: {best['interface']} ({best['chipset']})")
    if best["builtin"]:
        print("This is the Pi's built-in radio — expect ~8 stations. For a "
              "squad, add a USB adapter and set \"ap_ifname\" in "
              "receiver_config.json to its interface (usually wlan1).")
    else:
        print(f'Set "ap_ifname": "{best["interface"]}" in receiver_config.json.')
    return 0


def _nmcli(args: list, **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(["nmcli", *args], capture_output=True, text=True, **kwargs)


_PRIV_HINT = ("creating/modifying the AP profile needs privileges: run once "
              "with sudo, or grant them permanently (usermod -aG netdev + a "
              "polkit rule; see README.md)")


def _settings_match(cfg: dict) -> bool:
    """True when the existing profile already carries our SSID/AP/hidden/channel
    settings. Reads no secrets, so it works without privileges (the password
    came from the same config file, so an SSID match is trusted)."""
    r = _nmcli(["-g", "802-11-wireless.ssid,802-11-wireless.hidden,"
                "802-11-wireless.mode,802-11-wireless.channel",
                "connection", "show", cfg["ap_con_name"]])
    return r.returncode == 0 and r.stdout.splitlines() == [
        cfg["ap_ssid"], "yes", "ap", str(cfg["ap_channel"])]


def _is_active(con: str) -> bool:
    r = _nmcli(["-t", "-f", "NAME", "connection", "show", "--active"])
    return r.returncode == 0 and con in r.stdout.splitlines()


def ensure_ap(cfg: dict) -> bool:
    """Idempotently create + activate the hidden AP profile. Unprivileged runs
    are read-only when the profile already matches the config (at most a
    `connection up`). Never raises: on any failure it logs and returns False
    so BLE streaming still starts."""
    con = cfg["ap_con_name"]
    try:
        existing = _nmcli(["-t", "-f", "NAME", "connection", "show"])
        if existing.returncode != 0:
            print(f"# wifi-ap: nmcli unavailable ({existing.stderr.strip()}) — "
                  f"no access point; is NetworkManager running?", file=sys.stderr)
            return False
        exists = con in existing.stdout.splitlines()

        if exists and _settings_match(cfg):
            if _is_active(con):
                print(f"# wifi-ap: hidden AP '{cfg['ap_ssid']}' already active "
                      f"on {cfg['ap_ifname']} ({AP_IP})", file=sys.stderr)
                return True
        else:
            settings = [
                "802-11-wireless.mode", "ap",
                "802-11-wireless.band", "bg",
                "802-11-wireless.channel", str(cfg["ap_channel"]),
                "802-11-wireless.hidden", "yes",
                "802-11-wireless.ssid", cfg["ap_ssid"],
                "ipv4.method", "shared",
                "wifi-sec.key-mgmt", "wpa-psk",
                "wifi-sec.psk", cfg["ap_password"],
                # Pin plain WPA2-CCMP: NM's default lets wpa_supplicant offer a
                # WPA3-SAE transition AP, which ESP32 stations reject with
                # disconnect reason 211 (no AP with compatible security).
                "wifi-sec.proto", "rsn",
                "wifi-sec.pairwise", "ccmp",
                "wifi-sec.group", "ccmp",
                "wifi-sec.pmf", "disable",
                "connection.autoconnect", "yes",
            ]
            if exists:
                result = _nmcli(["connection", "modify", con, *settings])
            else:
                result = _nmcli(["connection", "add", "type", "wifi",
                                 "ifname", cfg["ap_ifname"], "con-name", con,
                                 *settings])
            if result.returncode != 0:
                print(f"# wifi-ap: configuring '{con}' failed: "
                      f"{result.stderr.strip()} — {_PRIV_HINT}", file=sys.stderr)
                return False

        up = _nmcli(["connection", "up", con])
        if up.returncode != 0:
            print(f"# wifi-ap: activating '{con}' failed: {up.stderr.strip()} — "
                  f"try `sudo nmcli con up {con}`", file=sys.stderr)
            return False

        print(f"# wifi-ap: hidden AP '{cfg['ap_ssid']}' up on "
              f"{cfg['ap_ifname']} ({AP_IP})", file=sys.stderr)
        return True
    except FileNotFoundError:
        print("# wifi-ap: nmcli not found — no access point. Install "
              "NetworkManager or run with --no-ap.", file=sys.stderr)
        return False
    except Exception as exc:  # AP is best-effort; the BLE stream must come up
        print(f"# wifi-ap: unexpected error: {exc}", file=sys.stderr)
        return False


if __name__ == "__main__":
    sys.exit(main())