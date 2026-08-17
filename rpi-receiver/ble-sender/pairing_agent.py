"""A BlueZ pairing agent, so iOS can finish pairing with the receiver ONCE.

Why this exists
---------------
BlueZ cannot complete a pairing — not even "Just Works", which involves no
passkey — unless some process has registered an ``org.bluez.Agent1`` and been
made the default agent. ``bluetoothctl`` registers one only while it is running
interactively; ``bluezero`` never does. On a headless receiver that leaves the
system with no agent at all, and the symptom is nasty:

    iOS connects, decides it wants to bond, shows
    "aurmor-rpi-xxxx would like to pair with your iPhone", the user accepts,
    BlueZ has nobody to authorise it, pairing fails, nothing is stored — and
    because nothing is stored it happens again on the very next connection,
    forever. GATT work stalls behind the prompt each time.

Confirmed on the receiver by all three of these being empty/absent:
    bluetoothctl devices Paired
    sudo ls /var/lib/bluetooth/*/
    busctl --system tree org.bluez | grep -i agent

Registering an agent makes the pairing actually succeed, so the key lands in
/var/lib/bluetooth and the prompt appears at most once per phone.

Capability is NoInputNoOutput — the receiver has no keypad or display, which is
what makes LE pairing take the Just Works path and lets us auto-accept. That is
the correct model here: this box is a shared appliance on a hidden AP, and
requiring someone to read a passkey off a headless Pi would be theatre. It does
NOT authenticate anybody — the wearable's own enrolment secret is what does that
(peq0-v1-head-tests/components/peripherals/ble_auth.h).

Everything here is best-effort: a receiver that cannot register an agent must
still stream, exactly as it did before this file existed.
"""
from __future__ import annotations

import pathlib
import sys

AGENT_PATH = "/org/aurmor/agent"
CAPABILITY = "NoInputNoOutput"

# Runtime marker so --selftest can tell whether registration actually happened.
# Needed because there is no way to ask BlueZ "who is your agent?": the agent
# object is exported by OUR bus connection, so it never shows up under
# `busctl tree org.bluez`, and checking there reports "no agent" on a perfectly
# healthy receiver. /run is tmpfs, so this cannot survive a reboot and go stale.
MARKER_PATH = "/run/aurmor-pairing-agent"

_BLUEZ = "org.bluez"
_AGENT_IFACE = "org.bluez.Agent1"
_DEVICE_IFACE = "org.bluez.Device1"
_PROPS_IFACE = "org.freedesktop.DBus.Properties"


def _build_agent_class(dbus, dbus_service):
    """Define the agent class lazily, so importing this module never requires
    dbus (the --stdout dry-run path runs on machines without it)."""

    class PairingAgent(dbus_service.Object):
        """Auto-accepts. See the module docstring for why that is right here."""

        def __init__(self, bus, path):
            super().__init__(bus, path)
            self._bus = bus

        def _trust(self, device):
            """Mark the peer trusted so BlueZ stops asking to authorise each
            service on later connections — without this the pairing succeeds but
            the phone can still be prompted again per-service."""
            try:
                props = dbus.Interface(
                    self._bus.get_object(_BLUEZ, device), _PROPS_IFACE)
                props.Set(_DEVICE_IFACE, "Trusted", dbus.Boolean(True))
            except Exception as exc:  # noqa: BLE001 - never break a pairing
                print(f"# agent: could not trust {device}: {exc}", file=sys.stderr)

        @dbus_service.method(_AGENT_IFACE, in_signature="", out_signature="")
        def Release(self):
            pass

        @dbus_service.method(_AGENT_IFACE, in_signature="o", out_signature="")
        def RequestAuthorization(self, device):
            # The Just Works path for a NoInputNoOutput agent.
            print(f"# agent: authorising pairing with {device}", file=sys.stderr)
            self._trust(device)

        @dbus_service.method(_AGENT_IFACE, in_signature="ou", out_signature="")
        def RequestConfirmation(self, device, passkey):
            print(f"# agent: confirming {device} (passkey {passkey:06d})",
                  file=sys.stderr)
            self._trust(device)

        @dbus_service.method(_AGENT_IFACE, in_signature="os", out_signature="")
        def AuthorizeService(self, device, uuid):
            self._trust(device)

        # Never reached with NoInputNoOutput, but Agent1 implementations are
        # expected to expose the whole interface; omitting them makes BlueZ log
        # UnknownMethod if it ever probes a different pairing method.
        @dbus_service.method(_AGENT_IFACE, in_signature="o", out_signature="s")
        def RequestPinCode(self, device):
            return "0000"

        @dbus_service.method(_AGENT_IFACE, in_signature="o", out_signature="u")
        def RequestPasskey(self, device):
            return dbus.UInt32(0)

        @dbus_service.method(_AGENT_IFACE, in_signature="os", out_signature="")
        def DisplayPinCode(self, device, pincode):
            pass

        @dbus_service.method(_AGENT_IFACE, in_signature="ouq", out_signature="")
        def DisplayPasskey(self, device, passkey, entered):
            pass

        @dbus_service.method(_AGENT_IFACE, in_signature="", out_signature="")
        def Cancel(self):
            print("# agent: pairing cancelled by the remote", file=sys.stderr)

    return PairingAgent


def _set_marker(registered: bool) -> None:
    """Best-effort: a diagnostic aid must never affect whether we stream."""
    try:
        if registered:
            pathlib.Path(MARKER_PATH).write_text("ok\n")
        else:
            pathlib.Path(MARKER_PATH).unlink(missing_ok=True)
    except OSError:
        pass


def is_registered() -> bool:
    """Whether a live agent registered in this boot (see MARKER_PATH)."""
    return pathlib.Path(MARKER_PATH).exists()


def try_register(bus=None) -> bool:
    """Register (and make default) a Just Works pairing agent.

    Returns True on success. Never raises: the receiver streams fine without an
    agent, it just re-prompts on every connection.

    Must be called BEFORE the GLib main loop starts (peripheral.publish()); the
    agent's methods are dispatched once that loop is running.
    """
    try:
        import dbus
        import dbus.service as dbus_service
        from dbus.mainloop.glib import DBusGMainLoop
    except ImportError as exc:
        print(f"# no pairing agent (dbus unavailable: {exc}) — iOS will re-prompt "
              "on every connection", file=sys.stderr)
        return False

    try:
        # bluezero sets this up too; doing it again is harmless and makes the
        # agent work even if this module is used on its own.
        DBusGMainLoop(set_as_default=True)
        if bus is None:
            bus = dbus.SystemBus()

        agent_cls = _build_agent_class(dbus, dbus_service)
        agent_cls(bus, AGENT_PATH)   # stays alive on the bus connection

        manager = dbus.Interface(
            bus.get_object(_BLUEZ, "/org/bluez"), "org.bluez.AgentManager1")
        manager.RegisterAgent(AGENT_PATH, CAPABILITY)
        # Being the DEFAULT agent is the part that matters: BlueZ only consults
        # the default one for an incoming pairing it has no other agent for.
        manager.RequestDefaultAgent(AGENT_PATH)
        # stderr, like every other startup message here: systemd block-buffers
        # a process's stdout, so a success line printed there can sit unflushed
        # for ages and read as "the agent never registered".
        print(f"Pairing agent registered ({CAPABILITY}) — phones bond once, "
              "then stop asking.", file=sys.stderr)
        _set_marker(True)
        return True
    except Exception as exc:  # noqa: BLE001 - a receiver must still stream
        print(f"# could not register pairing agent: {exc}\n"
              "# iOS will keep asking to pair on every connection; see "
              "pairing_agent.py", file=sys.stderr)
        _set_marker(False)
        return False
