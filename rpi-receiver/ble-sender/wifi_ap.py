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
    "ap_ifname": "wlan0",
    # Channels 1/6/11 are active-scannable in every regulatory domain. An
    # auto-picked 12/13 is passive-scan-only for world-domain ESP32s, and a
    # hidden AP can't be found by passive scan at all.
    "ap_channel": 6,
    "udp_port": 5005,
}


def _iface_mac_suffix(ifname: str):
    """Last two bytes of an interface MAC as 4 hex chars, or None if unreadable."""
    try:
        mac = Path(f"/sys/class/net/{ifname}/address").read_text().strip()
    except OSError:
        return None
    parts = mac.split(":")
    if len(parts) != 6:
        return None
    return (parts[4] + parts[5]).lower()


def _unique_suffix(ifname: str) -> str:
    # MAC-derived is stable + human-recognizable; random is the fallback (e.g.
    # on a dev host with no such interface). Either way it is stored once.
    return _iface_mac_suffix(ifname) or secrets.token_hex(2)


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
        suffix = _unique_suffix(cfg["ap_ifname"])
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
