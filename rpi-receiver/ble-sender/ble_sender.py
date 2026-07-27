#!/usr/bin/env python3
"""Stream biometric data over BLE (GATT notifications) to the mobile app.

The Raspberry Pi acts as a BLE *peripheral*: it advertises its unique per-Pi
name (``aurmor-rpi-<macsuffix>`` from its config; ``aurmor-rpi`` only when no
config exists), exposes one service with Meta (read), Data (notify), Control
(write) and WifiCreds (read) characteristics, and streams binary-v1 records
(see protocol.py). The React Native app (react-native-ble-plx, a BLE *central*)
scans by service UUID, connects, verifies the receiver's name against the
registered devices, reads Meta for the decode tables, and subscribes to the
Data characteristic.

Two sources:
  --source udp (default)  live IMU samples from ESP32 wearables on the Pi's
                          hidden WiFi AP (see wifi_ap.py / udp_source.py)
  --source csv            the original mock-CSV replay

  Run on the Pi:        python3 ble_sender.py
  Dry-run on any host:  python3 ble_sender.py --source csv --stdout --speed 20
                        python3 ble_sender.py --source udp --stdout --no-ap

See README.md for the GATT layout, UUIDs, and Raspberry Pi setup.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import pose as pose_mod
import protocol
import replay
import wifi_ap
from udp_source import UdpImuSource


def resolve_data_dir(args) -> Path:
    if args.data_dir:
        return Path(args.data_dir).expanduser()
    repo_root = Path(__file__).resolve().parent.parent  # .../rpi-receiver
    return repo_root / "mock-csv" / args.exercise


def resolve_pose_file(args, exercise: str) -> Path:
    if args.pose_file:
        return Path(args.pose_file).expanduser()
    repo_root = Path(__file__).resolve().parent.parent  # .../rpi-receiver
    slug = exercise.replace(" ", "_").lower()
    return repo_root / "ik-model" / "results" / slug / "madgwick" / "pose_seq.csv"


def build_meta(exercise: str, frames, nodes, chunk_size: int,
               field_specs, layouts, node_layout, period_ms=None) -> bytes:
    per = replay.period_ms(frames) if period_ms is None else period_ms
    descriptor = {
        "exercise": exercise,
        "period_ms": per,
        "fps": round(1000.0 / per, 2) if per else 0,
        "frames": len(frames),
        "nodes": nodes,
        "chunk_size": chunk_size,
        "framing": "binary-v1",
        "schema": protocol.SCHEMA_VERSION,
        # Decode tables for binary-v1: the app reconstructs each sample from these.
        "field_specs": field_specs,   # name -> [type, scale]
        "layouts": layouts,           # ordered field-name lists (no node/t_s)
        "node_layout": node_layout,   # node index -> layouts index
    }
    return json.dumps(descriptor, separators=(",", ":")).encode("utf-8")


# --------------------------------------------------------------------------- #
# Dry-run (no BLE): stream framed NDJSON to stdout at the real cadence.
# --------------------------------------------------------------------------- #
def run_stdout(frames, speed: float, loop: bool, verbose: bool, pose_seq=None) -> None:
    out = sys.stdout.buffer
    try:
        while True:
            origin = time.monotonic()
            for frame in frames:
                delay = (origin + frame.t_s / speed) - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                for _node, sample in frame.samples:
                    out.write(protocol.sample_to_ndjson(sample))
                if pose_seq is not None:
                    tran, quats = pose_seq.nearest(frame.t_s)
                    out.write(protocol.pose_to_ndjson(round(frame.t_s, 3), tran, quats))
                out.flush()
                if verbose:
                    print(f"# t_s={frame.t_s:.3f} nodes={len(frame.samples)}",
                          file=sys.stderr)
            if not loop:
                break
    except (BrokenPipeError, KeyboardInterrupt):
        pass


# --------------------------------------------------------------------------- #
# BLE peripheral
# --------------------------------------------------------------------------- #
class BleSender:
    def __init__(self, frames, meta_bytes, name, adapter_addr,
                 chunk_size, speed, loop, verbose,
                 nodes, field_specs, layouts, node_layout,
                 button_pin=None, power_button=False, pose_seq=None,
                 source=None, live_period_ms=100, wifi_creds_bytes=b"{}"):
        self.frames = frames
        self.pose_seq = pose_seq
        self.meta_bytes = meta_bytes
        # binary-v1 encode tables (mirror what Meta publishes to the app).
        self.field_specs = field_specs
        self.layouts = layouts
        self.node_index = {node: i for i, node in enumerate(nodes)}
        self.node_layout = node_layout
        self.name = name
        self.adapter_addr = adapter_addr
        self.chunk_size = chunk_size
        self.source = source  # UdpImuSource when live; None = CSV replay
        self.wifi_creds_bytes = wifi_creds_bytes
        if source is not None:
            self.frame_interval_ms = max(1, int(live_period_ms))
        else:
            self.frame_interval_ms = max(1, round(replay.period_ms(frames) / speed))
        self.loop = loop
        self.verbose = verbose
        self.button_pin = button_pin
        self.power_button = power_button
        self.use_button = (button_pin is not None) or power_button
        self._last_wait_log = 0.0
        self._wearables_cache = b'{"active":[]}'
        self._periph = None
        self._last_presence = None

        self.data_char = None
        self.frame_idx = 0
        self.running = False
        self.timer_active = False
        self._async = None     # bluezero.async_tools, set in run()
        self._button = None    # gpiozero.Button, kept to avoid garbage collection
        self._pwr_dev = None   # evdev.InputDevice for the Pi 5 power button
        self._pwr_thread = None  # daemon thread reading power-button events

    # -- characteristic callbacks ------------------------------------------- #
    def meta_read(self, options):
        # The Meta blob is usually larger than the ATT MTU, so the central reads
        # it with successive Read-Blob requests at increasing offsets. bluezero
        # passes `options` (with the offset) only when the callback declares a
        # parameter — honor it, or every blob would restart at byte 0 and the
        # reassembled JSON would be corrupt.
        offset = int(options.get("offset", 0))
        return list(self.meta_bytes[offset:])

    def wifi_creds_read(self, options):
        # Same read-blob offset handling as meta_read (payload is small, but a
        # low negotiated MTU still splits the read).
        offset = int(options.get("offset", 0))
        return list(self.wifi_creds_bytes[offset:])

    def wearables_read(self, options):
        # Live presence for BLE-silent wearables: which boards the UDP source
        # heard from recently. Rendered once per read (offset 0) so blob
        # continuations can't tear across a state change.
        offset = int(options.get("offset", 0))
        if offset == 0:
            active = self.source.active_wearables() if self.source else {}
            self._wearables_cache = json.dumps(
                {"active": [{"wid": wid, "node": node}
                            for wid, node in sorted(active.items())]},
                separators=(",", ":")).encode("utf-8")
        return list(self._wearables_cache[offset:])

    def control_write(self, value, options):
        cmd = bytes(value).decode("utf-8", "ignore").strip().lower()
        parts = cmd.split()
        if cmd in ("start", "restart"):
            self.start()
        elif cmd == "stop":
            self.stop()
        elif parts and parts[0] == "forget":
            wid = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else None
            if self.source is not None:
                # Retries sleep between sends — keep them off the GLib/BLE loop.
                threading.Thread(target=self.source.send_forget, args=(wid,),
                                 daemon=True).start()
            else:
                print("# control: 'forget' ignored (csv source)", file=sys.stderr)
        if self.verbose:
            print(f"# control: {cmd!r}", file=sys.stderr)

    def data_notify(self, notifying, characteristic):
        self.data_char = characteristic
        if not notifying:
            self.stop()
            return
        if self.use_button:
            print(f"# subscribed — press {self._trigger_desc()} to start "
                  f"streaming", file=sys.stderr)
        else:
            self.start()

    def on_button(self):
        """Start replay from a button press. Runs on the GLib main-loop thread
        (marshalled there via GLib.idle_add, since gpiozero fires on its own)."""
        if self.data_char is None:
            print("# button pressed, but no subscriber yet — connect the app "
                  "and enable notifications first", file=sys.stderr)
            return False
        print("# button pressed — (re)starting replay", file=sys.stderr)
        self.start()  # start() resets frame_idx, so each press replays from the top
        return False  # idle_add: run once

    # -- replay control ----------------------------------------------------- #
    def _send_meta(self):
        """Publish the decode tables over the Data stream (framed + chunked) so
        the app has them before the first sample. The Meta characteristic can't
        carry them: the descriptor exceeds the 512-byte GATT attribute limit."""
        if self.data_char is None:
            return
        message = protocol.frame_record(protocol.MSG_META, bytes(self.meta_bytes))
        for chunk in protocol.chunk_bytes(message, self.chunk_size):
            self.data_char.set_value(list(chunk))

    def start(self):
        self.frame_idx = 0
        self.running = True
        if self.source is not None:
            self.source.drain()  # discard backlog accumulated while unsubscribed
        # Always (re)send Meta first so a fresh subscriber — or a restart/loop —
        # can decode the samples that follow.
        self._send_meta()
        if not self.timer_active:
            self.timer_active = True
            # bluezero renamed add_timeout() -> add_timer_ms(); support both.
            add_timer = getattr(self._async, 'add_timer_ms', None) or self._async.add_timeout
            emit = self._emit_live if self.source is not None else self._emit_frame
            add_timer(self.frame_interval_ms, emit)
        if self.verbose:
            print("# streaming started", file=sys.stderr)

    def stop(self):
        self.running = False  # _emit_frame tears the timer down on its next tick

    def _emit_frame(self):
        if not self.running or self.data_char is None:
            self.timer_active = False
            return False
        if self.frame_idx >= len(self.frames):
            if self.loop:
                self.frame_idx = 0
            else:
                self.timer_active = False
                if self.verbose:
                    print("# replay complete", file=sys.stderr)
                return False

        frame = self.frames[self.frame_idx]
        for node, sample in frame.samples:
            node_idx = self.node_index[node]
            layout = self.layouts[self.node_layout[node_idx]]
            payload = protocol.encode_sample_binary(
                sample, node_idx, layout, self.field_specs)
            message = protocol.frame_record(protocol.MSG_SAMPLE, payload)
            for chunk in protocol.chunk_bytes(message, self.chunk_size):
                self.data_char.set_value(list(chunk))
        if self.pose_seq is not None:
            tran, quats = self.pose_seq.nearest(frame.t_s)
            payload = protocol.encode_pose_binary(round(frame.t_s, 3), tran, quats)
            message = protocol.frame_record(protocol.MSG_POSE, payload)
            for chunk in protocol.chunk_bytes(message, self.chunk_size):
                self.data_char.set_value(list(chunk))
        self.frame_idx += 1
        if self.verbose:
            print(f"# sent t_s={frame.t_s:.3f} ({len(frame.samples)} nodes)",
                  file=sys.stderr)
        return True

    def _emit_live(self):
        """Timer tick in live mode: drain the UDP queue and notify each sample."""
        if not self.running or self.data_char is None:
            self.timer_active = False
            return False
        samples = self.source.drain()
        if not samples:
            now = time.monotonic()
            if now - self._last_wait_log >= 5.0:
                print("# waiting for UDP data from wearables…", file=sys.stderr)
                self._last_wait_log = now
            return True
        for sample in samples:
            node_idx = self.node_index.get(sample["node"])
            if node_idx is None:
                continue
            layout = self.layouts[self.node_layout[node_idx]]
            payload = protocol.encode_sample_binary(
                sample, node_idx, layout, self.field_specs)
            message = protocol.frame_record(protocol.MSG_SAMPLE, payload)
            for chunk in protocol.chunk_bytes(message, self.chunk_size):
                self.data_char.set_value(list(chunk))
        if self.verbose:
            print(f"# sent {len(samples)} live samples "
                  f"(t_s={samples[-1]['t_s']:.3f})", file=sys.stderr)
        return True

    # -- bring-up ----------------------------------------------------------- #
    def _trigger_desc(self):
        if self.power_button:
            return "the power button"
        if self.button_pin is not None:
            return f"GPIO{self.button_pin}"
        return "app subscribe"

    def _arm_gpio_button(self):
        try:
            from gi.repository import GLib
            from gpiozero import Button
        except ImportError as exc:
            raise SystemExit(
                "GPIO button mode needs gpiozero + GLib. On the Pi install: "
                "sudo apt install -y python3-gpiozero python3-lgpio python3-gi"
            ) from exc
        # pull_up=True -> button wired between the GPIO pin and GND (pressed = low).
        self._button = Button(self.button_pin, pull_up=True, bounce_time=0.1)
        # gpiozero fires this on its own thread; idle_add hops onto the BLE loop.
        self._button.when_pressed = lambda: GLib.idle_add(self.on_button)
        print(f"Button armed on BCM GPIO{self.button_pin} (wired to GND).")

    def _arm_power_button(self):
        try:
            from gi.repository import GLib
            from evdev import InputDevice, ecodes, list_devices
        except ImportError as exc:
            raise SystemExit(
                "power-button mode needs python-evdev + GLib. On the Pi install: "
                "sudo apt install -y python3-evdev python3-gi"
            ) from exc

        dev = None
        for path in list_devices():
            try:
                candidate = InputDevice(path)
            except OSError:
                continue  # no permission for this node (not in 'input' group?); skip
            if ecodes.KEY_POWER in candidate.capabilities().get(ecodes.EV_KEY, []):
                dev = candidate
                break
            candidate.close()
        if dev is None:
            raise SystemExit(
                "no input device exposing KEY_POWER was found. Is this a Pi 5, and "
                "is your user in the 'input' group? "
                "(sudo usermod -aG input $USER, then log out and back in.)")
        self._pwr_dev = dev

        # Grab the device so logind doesn't ALSO see the press and power off. The
        # grab releases automatically when this process exits, so the button shuts
        # the Pi down normally again once the sender stops. A long-press is a
        # hardware force-off below the input layer and is unaffected.
        try:
            dev.grab()
        except OSError as exc:
            print(f"# warning: could not grab {dev.path} ({exc}); a press may still "
                  f"shut down the Pi unless you set HandlePowerKey=ignore in "
                  f"/etc/systemd/logind.conf", file=sys.stderr)

        def watch():
            for event in dev.read_loop():  # blocks in this daemon thread
                if (event.type == ecodes.EV_KEY
                        and event.code == ecodes.KEY_POWER
                        and event.value == 1):  # 1 = key down
                    GLib.idle_add(self.on_button)

        self._pwr_thread = threading.Thread(target=watch, daemon=True)
        self._pwr_thread.start()
        print(f"Power button armed via {dev.path} ({dev.name!r}).")

    def run(self):
        try:
            from bluezero import adapter, async_tools, peripheral
        except ImportError as exc:
            raise SystemExit(
                "bluezero is not installed. On the Raspberry Pi run "
                "`pip install -r requirements.txt` (see README.md). "
                "For local testing without BLE, use --stdout."
            ) from exc

        self._async = async_tools

        addr = self.adapter_addr
        if addr is None:
            available = list(adapter.Adapter.available())
            if not available:
                raise SystemExit("no Bluetooth adapter found.")
            addr = available[0].address

        # The app registers/verifies a receiver by its GAP *adapter* name
        # (Android's BluetoothDevice.getName / iOS peripheral.name / GATT char
        # 0x2A00), NOT just the advertisement LocalName that `local_name` below
        # sets. The OS image ships a shared adapter name ("aurmor-device"), so
        # every Pi would look identical and collide on the backend's unique
        # serial. Align the adapter name with our unique per-Pi name so all three
        # (adapter name, GATT name, advertisement) agree. Best-effort: BlueZ/
        # bluezero setter support varies, and a failure just leaves the old name.
        try:
            dongle = adapter.Adapter(addr)
            if dongle.alias != self.name:
                dongle.alias = self.name
                print(f"Set adapter name to {self.name!r}.")
        except Exception as exc:  # noqa: BLE001 - defensive, name is non-critical
            print(f"# could not set adapter name to {self.name!r}: {exc}",
                  file=sys.stderr)

        periph = peripheral.Peripheral(addr, local_name=self.name)
        periph.add_service(srv_id=1, uuid=protocol.SERVICE_UUID, primary=True)
        periph.add_characteristic(
            srv_id=1, chr_id=1, uuid=protocol.META_UUID,
            value=list(self.meta_bytes), notifying=False,
            flags=["read"], read_callback=self.meta_read)
        periph.add_characteristic(
            srv_id=1, chr_id=2, uuid=protocol.DATA_UUID,
            value=[], notifying=False,
            flags=["notify"], notify_callback=self.data_notify)
        periph.add_characteristic(
            srv_id=1, chr_id=3, uuid=protocol.CONTROL_UUID,
            value=[], notifying=False,
            flags=["write", "write-without-response"],
            write_callback=self.control_write)
        periph.add_characteristic(
            srv_id=1, chr_id=4, uuid=protocol.WIFI_CREDS_UUID,
            value=list(self.wifi_creds_bytes), notifying=False,
            flags=["read"], read_callback=self.wifi_creds_read)
        periph.add_characteristic(
            srv_id=1, chr_id=5, uuid=protocol.WEARABLES_UUID,
            value=list(self._wearables_cache), notifying=False,
            flags=["read"], read_callback=self.wearables_read)

        if self.button_pin is not None:
            self._arm_gpio_button()
        if self.power_button:
            self._arm_power_button()

        # Broadcast live-wearable presence in the advertisement (live mode only)
        # so the app shows a BLE-silent ESP32 as detected without connecting.
        self._periph = periph
        if self.source is not None:
            self._arm_presence_adv()

        trigger = (f"press {self._trigger_desc()}" if self.use_button
                   else "subscribe from the app")
        src_desc = ("live UDP samples" if self.source is not None
                    else f"{len(self.frames)} frames")
        print(f"Advertising '{self.name}' on {addr} — {src_desc} "
              f"@ {self.frame_interval_ms} ms/tick, chunk={self.chunk_size}B. "
              f"Streaming starts when you {trigger}. Ctrl-C to stop.")
        try:
            periph.publish()
        except KeyboardInterrupt:
            print("\nStopped.")

    # -- presence advertisement --------------------------------------------- #
    def _presence_data(self):
        active = self.source.active_wearables() if self.source else {}
        return protocol.presence_manufacturer_data(active.keys())

    @staticmethod
    def _set_mfg_data(advert, mid, data):
        """Set manufacturer data across bluezero API variants (method name and
        signature differ between versions)."""
        fn = getattr(advert, "manufacturer_data", None) or \
            getattr(advert, "add_manufacturer_data", None)
        if fn is None:
            raise AttributeError("advertisement has no manufacturer_data setter")
        fn(mid, data)

    def _arm_presence_adv(self):
        """Set the initial presence manufacturer data and refresh it on a timer.
        Best-effort: bluezero advertisement details vary, so any failure is
        logged and simply disables the passive-presence broadcast."""
        try:
            self._set_mfg_data(self._periph.advert, protocol.PRESENCE_MFG_ID,
                               self._presence_data())
        except Exception as exc:  # noqa: BLE001 - never block streaming
            print(f"# presence-adv: could not set manufacturer data ({exc}); "
                  f"passive ESP32 presence disabled", file=sys.stderr)
            return
        self._last_presence = self._presence_data()
        add_timer = getattr(self._async, 'add_timer_ms', None) or self._async.add_timeout
        add_timer(4000, self._refresh_presence_adv)
        print("# presence-adv: broadcasting live-wearable presence", file=sys.stderr)

    def _refresh_presence_adv(self):
        data = self._presence_data()
        if data != self._last_presence:
            try:
                # Re-register the advertisement so BlueZ picks up the new payload.
                self._periph.ad_manager.unregister_advertisement(self._periph.advert)
                self._set_mfg_data(self._periph.advert, protocol.PRESENCE_MFG_ID, data)
                self._periph.ad_manager.register_advertisement(self._periph.advert, {})
                self._last_presence = data
            except Exception as exc:  # noqa: BLE001
                print(f"# presence-adv: refresh failed ({exc})", file=sys.stderr)
                return False  # stop the timer; stale presence is better than churn
        return True


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", choices=("udp", "csv"), default="udp",
                   help="data source: live UDP from ESP32 wearables (default) "
                        "or the mock-CSV replay")
    p.add_argument("--config", default=None,
                   help="receiver config JSON (default: receiver_config.json "
                        "next to this script; created with generated secrets "
                        "on first run)")
    p.add_argument("--udp-port", type=int, default=None,
                   help="UDP listen port in live mode (default: from config)")
    p.add_argument("--pi-id", type=int, default=None,
                   help="this Pi's ID for WELCOME/FORGET (default: from config)")
    p.add_argument("--no-ap", action="store_true",
                   help="don't bring up the hidden WiFi AP (live mode)")
    p.add_argument("--live-period-ms", type=int, default=100,
                   help="UDP queue drain interval in live mode "
                        "(default: %(default)s ms)")
    p.add_argument("--exercise", default="10_pushups_biometric_data_simulation",
                   help="folder under mock-csv/ to replay (default: %(default)s)")
    p.add_argument("--data-dir", default=None,
                   help="explicit path to a folder of node CSVs (overrides --exercise)")
    p.add_argument("--ik", action="store_true",
                   help="also stream IK skeletal pose: one pose line per frame, "
                        "synced to the biometric frames by t_s")
    p.add_argument("--pose-file", default=None,
                   help="UIP pose_seq.csv path (default: "
                        "<repo>/ik-model/results/<exercise>/madgwick/pose_seq.csv)")
    p.add_argument("--name", default=None,
                   help="BLE advertised name (default: the receiver's unique "
                        "name from its config, e.g. aurmor-rpi-3f48; falls back "
                        "to aurmor-rpi when no config exists)")
    p.add_argument("--chunk-size", type=int, default=protocol.DEFAULT_CHUNK_SIZE,
                   help="max notification payload in bytes; keep <= MTU-3 "
                        "(default: %(default)s; raise to ~180 once MTU is negotiated)")
    p.add_argument("--speed", type=float, default=1.0,
                   help="replay speed multiplier (default: 1.0 = real time)")
    p.add_argument("--loop", action="store_true", help="repeat the session forever")
    p.add_argument("--button-pin", type=int, default=None,
                   help="BCM GPIO pin of a push button (wired to GND) that starts "
                        "the stream; when set, replay waits for a press instead of "
                        "auto-starting on subscribe (e.g. 17 = physical pin 11)")
    p.add_argument("--power-button", action="store_true",
                   help="use the Raspberry Pi 5 onboard power button as the trigger "
                        "(reads KEY_POWER via evdev and grabs it so a press won't "
                        "shut down the Pi while the sender runs; needs the 'input' "
                        "group)")
    p.add_argument("--adapter", default=None,
                   help="Bluetooth adapter address (default: first available)")
    p.add_argument("--stdout", action="store_true",
                   help="dry run: stream NDJSON to stdout, no BLE (works on macOS)")
    p.add_argument("--verbose", action="store_true", help="log frames to stderr")
    return p.parse_args(argv)


def run_stdout_live(source, period_ms: int) -> None:
    """Dry-run for live mode: drain the UDP queue to stdout as NDJSON."""
    try:
        while True:
            for sample in source.drain():
                sys.stdout.buffer.write(protocol.sample_to_ndjson(sample))
            sys.stdout.buffer.flush()
            time.sleep(period_ms / 1000.0)
    except (BrokenPipeError, KeyboardInterrupt):
        pass


def main_udp(args) -> None:
    cfg_path = Path(args.config).expanduser() if args.config else wifi_ap.DEFAULT_CONFIG_PATH
    cfg = wifi_ap.load_config(cfg_path)
    if args.udp_port is not None:
        cfg["udp_port"] = args.udp_port
    if args.pi_id is not None:
        cfg["pi_id"] = args.pi_id
    if not args.no_ap:
        wifi_ap.ensure_ap(cfg)

    source = UdpImuSource(cfg["udp_port"], cfg["pi_id"], verbose=args.verbose)
    source.start()

    if args.stdout:
        run_stdout_live(source, args.live_period_ms)
        return

    nodes = source.nodes
    field_specs, layouts, node_layout = protocol.build_live_protocol_meta(nodes)
    meta = build_meta("live-udp", [], nodes, args.chunk_size,
                      field_specs, layouts, node_layout,
                      period_ms=args.live_period_ms)

    # Advertise the receiver's unique per-Pi name so the app registers it under a
    # distinct serial (the shared "aurmor-rpi" collides on the backend's unique
    # serial column). --name still overrides for manual runs.
    name = args.name or cfg["receiver_name"]
    BleSender([], meta, name, args.adapter,
              args.chunk_size, args.speed, args.loop, args.verbose,
              nodes, field_specs, layouts, node_layout,
              button_pin=args.button_pin, power_button=args.power_button,
              source=source, live_period_ms=args.live_period_ms,
              wifi_creds_bytes=wifi_ap.wifi_creds_json(cfg)).run()


def main(argv=None):
    args = parse_args(argv)
    if args.speed <= 0:
        raise SystemExit("--speed must be > 0")

    if args.source == "udp":
        main_udp(args)
        return

    data_dir = resolve_data_dir(args)
    frames, nodes = replay.load_frames(data_dir)
    exercise = Path(args.data_dir).name if args.data_dir else args.exercise
    field_specs, layouts, node_layout = protocol.build_protocol_meta(frames, nodes)
    meta = build_meta(exercise, frames, nodes, args.chunk_size,
                      field_specs, layouts, node_layout)

    pose_seq = None
    if args.ik:
        pose_path = resolve_pose_file(args, exercise)
        pose_seq = pose_mod.load_pose_csv(pose_path)

    if args.verbose or args.stdout:
        print(f"# loaded {len(frames)} frames, nodes={nodes} from {data_dir}",
              file=sys.stderr)
        if pose_seq is not None:
            print(f"# loaded IK pose: {pose_seq.count} frames, "
                  f"{pose_seq.n_joints} joints", file=sys.stderr)

    if args.stdout:
        run_stdout(frames, args.speed, args.loop, args.verbose, pose_seq=pose_seq)
        return

    # Serve WiFi creds in csv mode too when a receiver config already exists
    # (lets provisioning be tested against the replay source). When a config
    # exists, also advertise its unique per-Pi name so pairing registers a
    # distinct serial; otherwise fall back to the shared default.
    cfg_path = Path(args.config).expanduser() if args.config else wifi_ap.DEFAULT_CONFIG_PATH
    if cfg_path.exists():
        cfg = wifi_ap.load_config(cfg_path)
        creds = wifi_ap.wifi_creds_json(cfg)
        name = args.name or cfg["receiver_name"]
    else:
        creds = b"{}"
        name = args.name or protocol.DEFAULT_DEVICE_NAME

    BleSender(frames, meta, name, args.adapter,
              args.chunk_size, args.speed, args.loop, args.verbose,
              nodes, field_specs, layouts, node_layout,
              button_pin=args.button_pin, power_button=args.power_button,
              pose_seq=pose_seq, wifi_creds_bytes=creds).run()


if __name__ == "__main__":
    main()
