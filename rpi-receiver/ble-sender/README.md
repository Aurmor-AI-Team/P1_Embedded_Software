# ble-sender

Streams biometric data over **Bluetooth Low Energy** to the mobile app, from one
of two sources:

- **`--source udp` (default)** — live IMU samples from ESP32-C6 wearables that
  joined the Pi's hidden WiFi access point (see *Live mode* below);
- **`--source csv`** — replays the recorded **mock biometric session** so the
  app has a realistic stream to develop against without hardware.

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
| **Control** | `5a8e0003-…` | write | ASCII `start` / `stop` / `restart` / `forget [wid]` (streaming auto-starts on Data subscribe; `forget` sends a UDP FORGET to a wearable — live mode only) |
| **WifiCreds** | `5a8e0004-…` | read | JSON `{"ssid","password","ip","port","pi_id"}` — the hidden AP credentials the app forwards to an ESP32 during pairing (`{}` when no config exists) |

The UUIDs and framing live in [protocol.py](protocol.py) — the single source of
truth the RN receiver must mirror.

> **Phone GATT cache.** The WifiCreds characteristic was added later; a phone
> that talked to the Pi before may serve a stale GATT table and fail the creds
> read. Toggle Bluetooth off/on (or forget the peripheral in system settings)
> to refresh it.

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

## Live mode (default): ESP32 wearables over WiFi/UDP

```
 ESP32-C6 wearable                Raspberry Pi (this script)             Phone
 ┌─────────────────┐   UDP :5005  ┌─────────────────────────────┐  BLE   ┌─────────┐
 │ joins hidden AP │ ───────────► │ udp_source.py → queue        │ ─────► │ app     │
 │ 52 B IMU pkts   │ ◄─────────── │ ble_sender.py  (binary-v1)   │ notify │         │
 └─────────────────┘ WELCOME/     │ wifi_ap.py     (nmcli AP)    │        └─────────┘
                     FORGET       └─────────────────────────────┘
```

- **Hidden AP** — on startup the script ensures a hidden WiFi access point via
  **NetworkManager/nmcli** (connection `aurmor-ap`, `ipv4.method shared`, Pi IP
  `10.42.0.1`). Skip with `--no-ap`. Note: shared mode claims `wlan0`, so use
  Ethernet for SSH while developing.
- **Privileges** — *creating* the AP profile needs NetworkManager rights, which
  plain users don't have over SSH (polkit treats SSH sessions as inactive).
  Either run the script once with `sudo` (the profile persists and autoconnects
  from then on; later unprivileged runs are read-only when it's already up), or
  grant them permanently:

  ```bash
  sudo usermod -aG netdev $USER    # then log out/in
  sudo tee /etc/polkit-1/rules.d/50-networkmanager-netdev.rules > /dev/null <<'EOF'
  polkit.addRule(function(action, subject) {
      if (action.id.indexOf("org.freedesktop.NetworkManager.") == 0 &&
          subject.isInGroup("netdev")) {
          return polkit.Result.YES;
      }
  });
  EOF
  sudo systemctl restart polkit
  ```
- **`receiver_config.json`** — created next to the script on first run with a
  generated AP password and `pi_id`, plus a unique per-Pi identity (hidden SSID
  `aurmor-pi-<suffix>` + BLE name `aurmor-rpi-<suffix>`) whose `<suffix>` is the
  last 4 hex of the **Raspberry Pi board serial** (`/proc/cpuinfo` Serial /
  device-tree `serial-number`; random fallback off-Pi). Served to the app via
  **WifiCreds**. Holds a secret — git-ignored, don't commit it. **Delete this
  file before capturing a distributable image** so each cloned Pi regenerates
  its own identity from its own serial instead of sharing the golden Pi's.
- **Wearable IDs** — every UDP packet carries a `wearable_id`; `udp_source.py`
  maps them to body nodes (`WID_TO_NODE = {1:HEAD, 2:WA, 3:WD, 4:WE}`, one per
  body position the app assigns at pairing). Packets from unmapped IDs are
  answered (HELLO→WELCOME) but not streamed.
- **Ghost stations** — after an ESP32 loses power ungracefully, the AP keeps a
  stale association for its MAC and rejects the board's re-auth on reboot
  (disconnect reason 2) until it's removed. Install the eviction service so a
  rebooting board rejoins in seconds instead:

  ```bash
  sudo install -m 755 tools/evict_stale_stations.py /usr/local/bin/
  sudo cp tools/aurmor-wifi-evict.service /etc/systemd/system/
  sudo systemctl daemon-reload && sudo systemctl enable --now aurmor-wifi-evict.service
  ```

  It evicts any station passing **no data packets** for a window (~10 s). Note:
  idle-*time* eviction does NOT work — a stuck board retries auth every ~2.5 s,
  which resets the AP's inactivity timer; only real data (a live board HELLOs
  every 2 s) distinguishes it. (One-off manual clear: `sudo iw dev wlan0 station
  del <mac>`.)
- **Unpair** — the app writes `forget <wid>` to Control; the Pi sends a UDP
  FORGET (msg_type 4) to that wearable's last-seen address, and the board wipes
  its credentials and leaves the network.
- **Dev without hardware** — terminal A:
  `python3 ble_sender.py --source udp --stdout --no-ap`, terminal B:
  `python3 tools/fake_esp32_sender.py` (sends the HEAD mock CSV as real packets).
- The standalone debug receiver `../udp_imu_receiver/udp_imu_receiver.py` is
  unchanged; don't run it at the same time as live mode (both bind :5005).

## Run

### Raspberry Pi (real BLE)

```bash
# one-time system deps for bluezero (dbus-python / PyGObject build against these)
sudo apt update
sudo apt install -y python3-dbus python3-gi libdbus-1-dev libgirepository1.0-dev bluez

pip install -r requirements.txt
python3 ble_sender.py                 # live mode: hidden AP + UDP→BLE bridge
python3 ble_sender.py --source csv --loop --verbose --chunk-size 180   # mock replay
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

## Mock mode: a group session without ten wearables

Testing a group session needs ten boards. `mock_receiver.py` stands ten of them
up in software and runs **instead of** `ble_sender.py`:

```bash
sudo systemctl stop aurmor-receiver
sudo systemctl start aurmor-receiver-mock      # installed, not enabled
journalctl -u aurmor-receiver-mock -f
```

Then on the phone: **New Group Session → "Use mock wearables" → Select
devices**. Both halves are needed — the toggle invents the devices, this
invents their data. Switch back with `stop aurmor-receiver-mock` and
`start aurmor-receiver`; the two `Conflicts=`, since both claim the Bluetooth
adapter and the UDP port.

**Nothing on the data path is faked.** The real `UdpImuSource` and the real
`BleSender` run unmodified, and the emulated boards genuinely send
IMU/HELLO/ALERT packets over the loopback and genuinely answer WELCOME / MODE /
FORGET / ALERT\_ACK. The only fiction is that ten boards exist. So the working
modes (`idle` / `live` / `alerts` / `mock`), the impact ack-and-retransmit path,
per-participant node attribution and the end-of-session release all behave as
they do with hardware.

The fleet occupies a **reserved wearable-id block, `0xE001`–`0xE00A`** — serials
`aurmor-mibs-E001` … `aurmor-mibs-E00A`. The app mirrors that block in
`features/devices/mock-devices.ts`; if you change `--wid-base` or `--count`, you
must change it there too or the phone attributes samples to devices it never
shows.

Mock mode does **not** broadcast wearable presence in the advertisement. Ten live
wids need 26 bytes of manufacturer data and only ~10 are free once Flags and the
128-bit service UUID are in — BlueZ rejects an oversized advertisement outright,
and a receiver that cannot advertise is invisible to every scan in the app, so
there is nothing to pair and no session to start. Nothing is lost by skipping it:
the app seeds the simulated fleet's wids itself. See **Presence advertisement
sizing** below for the same limit on real hardware.

Dry run on any host (no BLE, no config file, human-readable NDJSON):

```bash
python3 mock_receiver.py --stdout --mode live --count 3 --verbose
python3 test_mock_receiver.py           # loopback tests, stdlib only
```

Flags: `--count` · `--wid-base` · `--mode` · `--period-ms` · `--impact-every` ·
`--csv` · `--ap` · `--stdout` (see `python3 mock_receiver.py --help`). Boards
start in `idle` like real hardware, so nothing streams until the session screen
selects a mode.

## Presence advertisement sizing

The receiver names its live wearables in the manufacturer data of its BLE
advertisement, so the app can show a provisioned (BLE-silent) ESP32 as detected
without connecting. That advertisement is a **31-byte legacy payload and it is
shared**:

| AD structure | bytes |
| --- | --- |
| Flags | 3 |
| Complete 128-bit Service UUID list | 18 |
| manufacturer-data header (len, type, company id) | 4 |
| our `[version, count]` | 2 |
| **left for wids, at 2 bytes each** | **4 → two wids** |

Go over 31 and BlueZ refuses to register the advertisement **at all** — the
receiver then advertises nothing and disappears from the device picker, the
presence scan and the session connection simultaneously. The symptom ("the app
can't see the receiver") points nowhere near the presence code, which is why
`protocol.PRESENCE_MAX_WIDS` is now *computed* from the table above rather than
guessed, `test_udp_source.py` asserts the budget, and `publish()` retries without
the presence data if BlueZ rejects it anyway.

Because two wids is not a squad, the advertisement **rotates**: each refresh
(`PRESENCE_REFRESH_MS`, 4 s) names the next slice, and the app holds a device
present for `PRESENCE_TTL_MS` (25 s, `useKnownDeviceScan.ts`) — long enough for a
full cycle, so a whole fleet still reads as detected. If you add anything to the
advertisement, subtract it from `_ADV_FIXED_BYTES`; if you lengthen the rotation,
check `refresh × slices` still fits comfortably inside the TTL.

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
├── mock_receiver.py  # ALTERNATIVE entry point: same receiver + 10 emulated wearables
├── replay.py         # CSV -> merged, time-ordered frames (no BLE deps)
├── protocol.py       # UUIDs, binary-v1 encode + framing + chunking (no BLE deps)
├── test_binary_protocol.py  # round-trip + upload-fidelity check (stdlib only)
├── test_mock_receiver.py    # loopback tests for the emulated fleet (stdlib only)
├── requirements.txt
└── README.md
```
