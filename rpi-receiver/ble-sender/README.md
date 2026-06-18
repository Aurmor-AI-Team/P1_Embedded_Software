# ble-sender

Replays the recorded **mock biometric session** over **Bluetooth Low Energy** so
the mobile app has a realistic live stream to develop against. This is the
**sender only** — the React Native receiver comes later.

## Why a BLE *peripheral*

The mobile app (`aurmor-sports-mobile`) uses **react-native-ble-plx**, which is a
BLE **central**: it scans for peripherals *by advertised name* and connects to
them. So the Raspberry Pi must be the **peripheral / GATT server** — it
advertises a name, the phone connects, and the Pi pushes data via
**notifications**.

```
 Raspberry Pi (this script)              Phone (later)
 ┌───────────────────────────┐           ┌──────────────────────────┐
 │ bluezero GATT peripheral  │  BLE      │ react-native-ble-plx     │
 │ advertise "aurmor-rpi"    │ ───────►  │ central: scan by name,   │
 │ notify binary-v1 samples  │  notify   │ connect, subscribe Data  │
 └───────────────────────────┘           └──────────────────────────┘
```

## Data source

`../mock-csv/10_pushups_biometric_data_simulation/` — 10 CSVs (one per body-worn
node: `HEAD`, `WA`…`WI`), **236 rows at 255 ms (3.92 Hz) ≈ 60 s**, all sharing the
same `t_s` timeline. Switch sessions with `--exercise <folder>` or `--data-dir <path>`.

## GATT layout

Advertised name: **`aurmor-rpi`** (override with `--name`).

| Characteristic | UUID | Props | Payload |
|----------------|------|-------|---------|
| Service | `5a8e0000-9b1a-4c7d-8e2f-1f3a5b7c9d10` | — | primary service |
| **Meta** | `5a8e0001-…` | read | JSON descriptor: `{…,"chunk_size","framing":"binary-v1","schema",` `"field_specs","layouts","node_layout"}` — carries the binary-v1 decode tables |
| **Data** | `5a8e0002-…` | notify | binary-v1 byte stream (see framing) |
| **Control** | `5a8e0003-…` | write | ASCII `start` / `stop` / `restart` (optional; replay auto-starts on Data subscribe) |

The UUIDs and framing live in [protocol.py](protocol.py) — the single source of
truth the RN receiver must mirror.

## Wire framing (binary-v1 + MTU chunking)

JSON repeats every field **name** on every sample (~39/s), which dominates the
byte cost. **binary-v1** drops the names — field order is implicit and published
once in **Meta** (`field_specs`/`layouts`/`node_layout`) — and packs each value as
a fixed-width number. On the mock pushups session this is **~5.4× smaller** than
NDJSON; the app reconstructs the *same* objects (values carry each field's scale
resolution). Constant columns (`timestamp_iso`, `label`, `version`,
`present_mask_hex`) are still dropped.

Records are length-prefixed and concatenated into the **Data** byte stream:

```
record  = msg_type:u8 | length:u16(LE) | payload[length]   (msg_type 1=sample, 2=pose)
sample  = node_idx:u8 | t_s_ms:u32 | <fields in the node's layout order, per field_specs>
pose    = t_s_ms:u32 | tran:3×i16(/1000) | 24 joints × 4×i16 quaternion(/10000)
```

`field_specs` maps each field → `[type, scale]` (`i16`/`u16`/`i32`/`u32`/`f32`/`str`);
decoded value = `raw / scale`. Per 255 ms tick one sample record is sent **per
node** (10/frame), plus one pose record when `--ik` is set.

The **Data** characteristic is a **byte stream**: each record is split into
`--chunk-size` byte pieces, one notification each. The receiver concatenates all
incoming bytes and reads the length prefix to recover whole records — so chunk
size need **not** align with record boundaries.

> **MTU / `--chunk-size`.** Default is **20 bytes**, which is safe for an
> un-negotiated 23-byte ATT MTU but slow (a long sample becomes many
> notifications). Once the phone negotiates a larger MTU (iOS ~185, Android can
> request up to 517 via `requestMTUForDevice`), raise it, e.g. `--chunk-size 180`.
> Keep it **≤ negotiated MTU − 3**; BlueZ silently truncates a larger notification.

## Run

### Raspberry Pi (real BLE)

```bash
# one-time system deps for bluezero (dbus-python / PyGObject build against these)
sudo apt update
sudo apt install -y python3-dbus python3-gi libdbus-1-dev libgirepository1.0-dev bluez

pip install -r requirements.txt
python3 ble_sender.py                 # advertises "aurmor-rpi", waits for a subscriber
python3 ble_sender.py --loop --verbose --chunk-size 180
```

Requires **BlueZ ≥ 5.43**. If advertising/GATT registration fails on your image,
enable the experimental interface (`bluetoothd --experimental`, e.g. add
`ExecStart=… --experimental` in `/etc/systemd/system/bluetooth.service.d/`).

### Any host incl. macOS (dry run, no BLE)

`--stdout` imports no BlueZ — it streams **human-readable NDJSON** (not the
binary-v1 BLE format) to stdout at the real cadence, for eyeballing values,
timing, and which fields each node emits:

```bash
python3 ble_sender.py --stdout                 # real time (~60 s)
python3 ble_sender.py --stdout --speed 20      # 20× faster
```

### Flags

`--exercise` · `--data-dir` · `--name` · `--chunk-size` · `--speed` · `--loop` ·
`--button-pin` · `--power-button` · `--adapter` · `--stdout` · `--verbose`
(see `python3 ble_sender.py --help`).

## Hardware button trigger (optional)

Start the stream with a physical button press instead of auto-starting on
subscribe. With either trigger below, replay **waits for a press**:

1. Start the sender — it advertises and arms the button.
2. On the phone, connect and enable notifications on **Data**.
3. **Press** → the ~60 s session streams from the top. Press again to replay
   (each press restarts from frame 0).

The BLE **Control** characteristic still works too (write `start`/`stop`/
`restart`), so you can also trigger from the app. Both triggers below fire on
their own thread and are marshalled onto the BLE main loop via `GLib.idle_add`,
so they're thread-safe.

### Raspberry Pi 5 onboard power button (`--power-button`)

The Pi 5's power button isn't on the GPIO header — it's a Linux *input device*
emitting `KEY_POWER` — so this path reads it with **evdev**:

```bash
python3 ble_sender.py --power-button --chunk-size 180 --verbose
```

The sender **grabs** the button while it runs, so a press goes to the script
instead of shutting the Pi down. The grab releases when the sender exits, so the
button behaves as a normal power/shutdown button again whenever the script isn't
running. (A *long* press is still a hardware force-off, below the input layer.)

Setup:

```bash
sudo apt install -y python3-evdev      # usually already present
sudo usermod -aG input $USER           # to read /dev/input/event*; then log out/in
```

If grabbing fails (e.g. permissions), a press may still trigger shutdown — as a
fallback, set `HandlePowerKey=ignore` in `/etc/systemd/logind.conf` and reboot.

### Wired GPIO push button (`--button-pin`)

Works on any Pi: wire a **momentary push button between a GPIO pin and a GND
pin** — e.g. BCM **GPIO17** (physical pin 11) to GND (physical pin 9). The
internal pull-up is used, so no resistor is needed (pressed = pin pulled low).

```bash
python3 ble_sender.py --button-pin 17 --chunk-size 180 --verbose
```

Needs gpiozero (preinstalled on Raspberry Pi OS; else
`sudo apt install python3-gpiozero python3-lgpio`). Button wired to **3V3**
instead of GND? Change `pull_up=True` to `False` in `_arm_gpio_button`.

## Verifying without the app

On the Pi, run the sender, then on the phone use a generic BLE tool such as
**nRF Connect**: scan → confirm **`aurmor-rpi`** appears → connect → **read Meta**
(JSON descriptor with `field_specs`/`layouts`) → enable notifications on **Data**
→ watch chunked binary records arrive at ~4 Hz. (For human-readable values, run
the sender with `--stdout` on any host instead.)

## For the React Native receiver (next step)

1. Connect to the device discovered by name in
   `features/devices/useBtScanner.ts`.
2. (Android) `requestMTUForDevice(deviceId, 185+)`; **read Meta** for the decode
   tables (`field_specs`/`layouts`/`node_layout`) — required before decoding Data.
3. `monitorCharacteristicForService(SERVICE_UUID, DATA_UUID, …)`; base64-decode each
   notification to bytes, append to a buffer, split on the length-prefixed record
   framing, and decode each record per `field_specs`. (Implemented in the app at
   `features/ble-stream/protocol.ts` + `connect.ts`.)
4. Use the UUIDs from [protocol.py](protocol.py) verbatim.

## Layout

```
ble-sender/
├── ble_sender.py     # entry point: argparse, bluezero peripheral, GLib-driven replay
├── replay.py         # CSV -> merged, time-ordered frames (no BLE deps)
├── protocol.py       # UUIDs, binary-v1 encode + framing + chunking (no BLE deps)
├── test_binary_protocol.py  # round-trip + upload-fidelity check (stdlib only)
├── requirements.txt
└── README.md
```
