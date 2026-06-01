/* uwb_ranging.c — TDMA DS-TWR with beacon synchronisation
 *
 * One ROUND, viewed on the air:
 *
 *   t=0          Beacon  (main broadcast, addressed to FFFF)
 *   t=1×slot     ──────────── slot 1 (peripheral 'A') ─────────────
 *                Poll  : peripheral → main
 *                Resp  : main       → peripheral   (delayed 6 ms after Poll RX)
 *                Final : peripheral → main         (delayed 6 ms after Resp RX,
 *                                                   carries 3 timestamps + IMU)
 *                Main computes distance for peripheral 'A'
 *   t=2×slot     ──────────── slot 2 (peripheral 'B') ─────────────
 *                ... same pattern ...
 *   ...
 *   t=15×slot    ──────────── slot 15 (peripheral 'O') ────────────
 *   t=round_end  guard band
 *   t=round      Next beacon
 *
 * Why beacon synchronisation:
 *   Two ESP32 host clocks running open-loop at "30 Hz" diverge at ~50 ppm
 *   = 6 ms after 2 minutes — exactly the size of the reply-delay budget,
 *   which is why the old code worked for ~2 minutes and then collapsed.
 *   The fix: the peripheral schedules its Poll using dwt_setdelayedtrxtime
 *   referenced to the DW3000's hardware RX timestamp of the beacon. That's
 *   the same 499.2 MHz clock DS-TWR uses, and it's freshly captured every
 *   round. Host-clock drift can be arbitrary; the slot still lands.
 *
 * Why no collisions:
 *   Each peripheral has a unique slot index (= suffix 'A'..'O' = 1..15).
 *   Its Poll is scheduled exactly at slot_index × UWB_SLOT_WIDTH_MS after
 *   the beacon. Slots don't overlap by construction. If a peripheral
 *   misses its beacon, it stays quiet for the round (no late Poll firing
 *   into a neighbor's slot) and re-syncs on the next beacon.
 *
 * Why this fixes thermal drift too:
 *   The recovery ladder (uwb_handle_cycle_result) is now actually wired
 *   in at the per-slot level, AND there's a proactive recal every
 *   RECAL_INTERVAL_US regardless of miss count. The DW3000's PLL retunes
 *   periodically without waiting for a miss streak.
 *
 * Compared to the previous (non-TDMA) 3-message version:
 *   - New beacon frame (function code FN_BEACON, header-only payload).
 *   - Peripheral cycle now BLOCKS on beacon RX before doing anything.
 *   - Peripheral schedules Poll as a delayed TX from beacon RX timestamp,
 *     instead of an immediate TX from "whenever the host gets around to it".
 *   - Main API is now uwb_perform_round (sweeps all slots) instead of
 *     uwb_perform_ranging (one peer at a time).
 *   - Frame-level DS-TWR math inside one slot is unchanged.
 *
 * Two scheduled-TX gotchas (unchanged from earlier — repeated for reference):
 *   - dwt_setdelayedtrxtime takes timestamp >> 8 (top 32 bits of a
 *     40-bit value).
 *   - The hardware ALSO zeros bit 0 of that value (= bit 8 of timestamp).
 *     So the embedded TX timestamp must mask with & 0xFFFFFFFEUL before
 *     shifting back. Skipping this gives a 256-tick alternating error
 *     (~0.6 m bimodal distance, halved by ToF math).
 */

#include "uwb_ranging.h"
#include "dwm3000.h"
#include "deca_device_api.h"
#include "dw3000_deca_regs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include <string.h>
#include <math.h>
#include "bio_telemetry.h"
#include "freertos/event_groups.h"

static const char *TAG = "uwb";



static EventGroupHandle_t s_uwb_evt = NULL;
#define EVT_RXFCG  (1 << 0)
#define EVT_RXTO   (1 << 1)
#define EVT_RXERR  (1 << 2)
#define EVT_TXFRS  (1 << 3)
#define EVT_ANY_RX (EVT_RXFCG | EVT_RXTO | EVT_RXERR)

static volatile uint16_t s_cb_rx_len = 0;

/* These run in the deca_irq task (task context, NOT ISR) — plain
 * xEventGroupSetBits is correct, no FromISR variant needed. */
static void cb_txdone(const dwt_cb_data_t *d){ (void)d; xEventGroupSetBits(s_uwb_evt, EVT_TXFRS); }
static void cb_rxok (const dwt_cb_data_t *d){ s_cb_rx_len = d->datalength; xEventGroupSetBits(s_uwb_evt, EVT_RXFCG); }
static void cb_rxto (const dwt_cb_data_t *d){ (void)d; xEventGroupSetBits(s_uwb_evt, EVT_RXTO); }
static void cb_rxerr(const dwt_cb_data_t *d){ (void)d; xEventGroupSetBits(s_uwb_evt, EVT_RXERR); }

static uint32_t wait_rx_event(int timeout_us)
{
    TickType_t to = pdMS_TO_TICKS((timeout_us / 1000) + 1);
    EventBits_t b = xEventGroupWaitBits(s_uwb_evt, EVT_ANY_RX, pdTRUE, pdFALSE, to);
    if (b & EVT_RXFCG) return SYS_STATUS_RXFCG_BIT_MASK;
    if (b & EVT_RXTO)  return SYS_STATUS_ALL_RX_TO;
    if (b & EVT_RXERR) return SYS_STATUS_ALL_RX_ERR;
    return 0;   /* genuine software timeout: no IRQ at all */
}

static bool wait_txfrs(int timeout_us)
{
    TickType_t to = pdMS_TO_TICKS((timeout_us / 1000) + 1);
    return (xEventGroupWaitBits(s_uwb_evt, EVT_TXFRS, pdTRUE, pdFALSE, to) & EVT_TXFRS) != 0;
}



/* was: + sizeof(lsm6_sample_t) */
#define FINAL_BIO_IDX    25
#define FINAL_FRAME_LEN  (25 + (int)sizeof(bio_telemetry_t))

/* shared snapshot, generalizes your s_local_imu */
static portMUX_TYPE     s_bio_lock = portMUX_INITIALIZER_UNLOCKED;
static bio_telemetry_t  s_local_bio;
static bool             s_local_bio_valid = false;

void uwb_publish_telemetry(const bio_telemetry_t *t)
{
    if (!t) return;
    portENTER_CRITICAL(&s_bio_lock);
    s_local_bio = *t;
    s_local_bio_valid = true;
    portEXIT_CRITICAL(&s_bio_lock);
}

/* --------------------------------------------------------------------- */
/* Tunables                                                              */
/* --------------------------------------------------------------------- */

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* UWB µs (uus) = 65536 DTU = ~1.0256 µs wall-clock. UL suffix prevents
 * signed-int overflow in delay × UUS_TO_DWT_TIME expressions. */
#define UUS_TO_DWT_TIME 65536UL

/* Peripheral: Response RX window after Poll TX. */
#define POLL_TX_TO_RESP_RX_DLY_UUS  4000
#define RESP_RX_TIMEOUT_UUS         8000

/* Main: scheduled Response TX delay after Poll RX. */
#define POLL_RX_TO_RESP_TX_DLY_UUS  6000

/* Peripheral: scheduled Final TX delay after Response RX. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 6000

/* Main: Final RX window after Response TX. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS  0
#define FINAL_RX_TIMEOUT_UUS         12000

/* Main per-slot Poll-listen window. CRITICAL: this must equal
 * UWB_SLOT_WIDTH_MS (the slot duration the peripheral uses to schedule
 * its Poll). If shorter, the main's listen for slot i closes before the
 * peripheral actually transmits its Poll, and exchanges miss even with a
 * perfect link.
 *
 * The peripheral schedules its Poll at (slot - 1) × UWB_SLOT_WIDTH_MS +
 * PERIPHERAL_POLL_LEAD_US after beacon RX (see PERIPHERAL_POLL_LEAD_US
 * below). The main starts listening for slot 1 immediately after the
 * beacon TX completes, so its slot N listen window covers roughly
 * (N-1) × SLOT_WIDTH_MS to N × SLOT_WIDTH_MS after beacon. As long as
 * the window widths match, the windows align.
 *
 * Original bug: I used 10 ms here while the peripheral schedules at 15
 * ms intervals, so by the time peripheral 'A' fired its Poll the main
 * had already moved on to listening for 'B'. */
#define POLL_RX_TIMEOUT_PER_SLOT_UUS  (UWB_SLOT_WIDTH_MS * 1000)
/* Software ceiling: hw timeout + margin for slow status polling. */
#define POLL_RX_SW_TIMEOUT_US         (UWB_SLOT_WIDTH_MS * 1000 + 3000)

/* Peripheral Poll TX is scheduled at (slot - 1) × UWB_SLOT_WIDTH_MS +
 * PERIPHERAL_POLL_LEAD_US after beacon RX. The lead pushes the Poll a
 * little past the slot boundary so it lands comfortably inside the
 * main's listen window (which is starting at the slot boundary).
 *
 * Sizing the lead: between beacon RX and dwt_starttx(DELAYED) on the
 * peripheral, the host must run ~5-6 SPI transactions (read frame, read
 * timestamp, write TX data, write fctrl, set RX-after-TX, set delayed
 * time, starttx). On ESP32-C6 over the QM33 SDK, this is empirically
 * 3-5 ms. The lead MUST exceed that, or dwt_starttx returns DWT_ERROR
 * ("late") and the slot is wasted.
 *
 * Previous bug: 2 ms lead was below host-latency floor and the Poll was
 * cancelled every single time. Visible as "Poll TX late/cancelled" with
 * the peripheral reaching 100% beacon RX but 0% Poll TX.
 *
 * 5 ms is conservative — eats only 5/15 ms of the slot budget, leaving
 * 10 ms for the Response listen + Final TX (plenty, since both use
 * delayed TX timed off DW3000 clocks, not host clocks). Bump higher if
 * you still see "Poll TX late" in production, or lower if you've
 * profiled the host path. */
#define PERIPHERAL_POLL_LEAD_US       5000

/* Peripheral beacon-listen window. The peripheral should be sitting in
 * receive when the beacon arrives; on cold start or after a miss, it
 * waits up to two full rounds before giving up. */
#define BEACON_RX_TIMEOUT_US  (2 * UWB_ROUND_PERIOD_MS * 1000)

/* Proactive recal cadence (microseconds). Independent of miss count; this
 * absorbs thermal drift before it causes misses. */
#define RECAL_INTERVAL_US     (30 * 1000 * 1000)   /* 30 seconds          */

/* Speed of light in air ≈ 299,702,547 m/s. */
#define SPEED_OF_LIGHT_M_PER_S      299702547.0

/* DW3000 timestamp tick period: 1 / (499.2e6 * 128) ≈ 15.65 ps. */
#define DWT_TIME_UNITS              (1.0 / 499200000.0 / 128.0)

float g_uwb_distance_offset_m = 0.0f;

/* --------------------------------------------------------------------- */
/* Frame layout                                                          */
/* --------------------------------------------------------------------- */

#define FRAME_FC_0      0x41
#define FRAME_FC_1      0x88
#define PAN_ID_LO       0xCA
#define PAN_ID_HI       0xDE

#define ADDR_MAIN_LO    'V'
#define ADDR_MAIN_HI    'E'

#define ADDR_PERIPH_PREFIX 'W'
#define DEFAULT_PERIPH_SUFFIX 'A'

/* Broadcast short-address used for the beacon dst field. */
#define ADDR_BCAST_LO   0xFF
#define ADDR_BCAST_HI   0xFF

#define FN_BEACON       0x40
#define FN_POLL         0x21
#define FN_RESPONSE     0x10
#define FN_FINAL        0x29

#define FCS_LEN         2

/* Beacon: header only (10 bytes). The round counter is embedded for
 * debug/jitter analysis but isn't required by the protocol. */
#define BEACON_FRAME_LEN 14
#define BEACON_ROUND_IDX 10   /* uint32 round counter, little-endian */

#define POLL_FRAME_LEN  10

/* Response: header + poll_rx_ts(5) + resp_tx_ts(5). */
#define RESP_FRAME_LEN  20
#define RESP_POLL_RX_TS_IDX 10
#define RESP_RESP_TX_TS_IDX 15

/* Final: header + poll_tx_ts(5) + resp_rx_ts(5) + final_tx_ts(5)
 *                + imu_sample (sizeof(lsm6_sample_t)). */
#define FINAL_POLL_TX_TS_IDX  10
#define FINAL_RESP_RX_TS_IDX  15
#define FINAL_FINAL_TX_TS_IDX 20
#define FINAL_IMU_IDX         25
#define FINAL_FRAME_LEN   (25 + (int)sizeof(lsm6_sample_t))

/* --------------------------------------------------------------------- */
/* State                                                                  */
/* --------------------------------------------------------------------- */

static uwb_role_t s_role;
static uint8_t    s_seq = 0;
static uint8_t    s_my_addr_lo;
static uint8_t    s_my_addr_hi;
static uint8_t    s_peer_addr_lo;
static uint8_t    s_peer_addr_hi;

/* Peripheral-side: this device's slot index (1..UWB_MAX_PERIPHERALS),
 * derived from its address suffix at init time. */
static uint8_t    s_my_slot_index = 0;

/* Main-side: round counter, advanced once per beacon. */
static uint32_t   s_round_counter = 0;

/* Last proactive recal time (host us). Used by both roles. */
static int64_t    s_last_recal_us = 0;

/* Local IMU snapshot, published by uwb_publish_local_imu and read by the
 * peripheral when assembling the Final frame. */
static portMUX_TYPE  s_imu_lock = portMUX_INITIALIZER_UNLOCKED;
static lsm6_sample_t s_local_imu;
static bool          s_local_imu_valid = false;

#define RX_BUF_LEN 96
static uint8_t rx_buf[RX_BUF_LEN];

/* Consecutive-miss counter drives reactive recovery. */
static int s_consec_miss = 0;
#define RECOVER_RECONFIG_AFTER  8
#define RECOVER_HARD_AFTER      40

/* Link-up state. Peripheral only.
 *
 *   s_ever_linked = false  until the very first beacon RX
 *   s_ever_linked = true   after at least one beacon has been received
 *
 * Before the first beacon, "missing the beacon" is the normal state and
 * we DON'T recalibrate — the main is probably just not powered yet, and
 * recalibrating an idle radio doesn't make it more likely to hear a
 * not-yet-transmitting peer. We just keep waiting.
 *
 * After s_ever_linked is set, beacon misses ARE evidence of a problem
 * (drift, thermal, interference) and the normal recal ladder applies.
 *
 * On the main this stays false; the main is never "waiting for a peer"
 * in the same blocking sense — it just transmits and listens. */
static bool s_ever_linked = false;

/* Diagnostic: count any RX event the chip reports (good frame, bad CRC,
 * frame error, preamble timeout, frame timeout). If this stays at 0 over
 * a long window the radio isn't seeing energy at all — strong hint of
 * antenna / RF / channel-mismatch issue rather than a protocol bug. */
static uint32_t s_rx_event_count    = 0;
static uint32_t s_rx_good_count     = 0;
static int64_t  s_last_diag_log_us  = 0;
#define RX_DIAG_INTERVAL_US (5 * 1000 * 1000)   /* every 5 s */

/* Why the last cycle failed (peripheral side). Recovery treats different
 * miss reasons differently — a "Poll late" miss means host scheduling is
 * tight, not that the radio is broken, so recalibration would only make
 * things worse (recal pauses the radio for ms, making the next Poll even
 * later). The next call to uwb_handle_cycle_result reads this and skips
 * recovery if the miss was timing-related. */
typedef enum {
    UWB_MISS_NONE = 0,
    UWB_MISS_NO_BEACON,        /* RX path issue / link down */
    UWB_MISS_POLL_LATE,        /* host scheduling — DON'T recal */
    UWB_MISS_RESP_RX,          /* main didn't reply / corrupted */
    UWB_MISS_FINAL_TX,         /* our own TX failed */
    UWB_MISS_OTHER,
} uwb_miss_reason_t;
static uwb_miss_reason_t s_last_miss_reason = UWB_MISS_NONE;

#define UWB_TX_ANT_DLY TX_ANT_DLY
#define UWB_RX_ANT_DLY RX_ANT_DLY

/* --------------------------------------------------------------------- */
/* Config                                                                 */
/* --------------------------------------------------------------------- */

static const dwt_config_t s_uwb_config = {
    .chan          = 5,
    .txPreambLength = DWT_PLEN_64,
    .rxPAC         = DWT_PAC8,
    .txCode        = 9,
    .rxCode        = 9,
    .sfdType       = 1,
    .dataRate      = DWT_BR_6M8,
    .phrMode       = DWT_PHRMODE_STD,
    .phrRate       = DWT_PHRRATE_STD,
    .sfdTO         = (64 + 1 + 8 - 8),
    .stsMode       = DWT_STS_MODE_OFF,
    .stsLength     = DWT_STS_LEN_64,
    .pdoaMode      = DWT_PDOA_M0,
};
static const dwt_txconfig_t s_txconfig = {
    .PGdly = 0x34, .power = 0xfdfdfdfd, .PGcount = 0,
};

#ifndef SYS_STATUS_ALL_RX_TO
#define SYS_STATUS_ALL_RX_TO   (SYS_STATUS_RXFTO_BIT_MASK | SYS_STATUS_RXPTO_BIT_MASK)
#endif
#ifndef SYS_STATUS_ALL_RX_ERR
#define SYS_STATUS_ALL_RX_ERR  (SYS_STATUS_RXPHE_BIT_MASK | SYS_STATUS_RXFCE_BIT_MASK | \
                                SYS_STATUS_RXFSL_BIT_MASK | SYS_STATUS_RXSTO_BIT_MASK | \
                                SYS_STATUS_ARFE_BIT_MASK)
#endif
#define SYS_STATUS_RX_ANY (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)

/* PLL-lock bit: CPLOCK is bit 1 of SYS_STATUS_LO. The SDK normally
 * exposes it as SYS_STATUS_CPLOCK_BIT_MASK; fall back to the raw bit if
 * not. The diagnostic prints whether the PLL is locked at the moment we
 * sample status — useful when "nothing works" might be a stuck PLL. */
#ifndef SYS_STATUS_CPLOCK_BIT_MASK
#define SYS_STATUS_CPLOCK_BIT_MASK (1u << 1)
#endif

/* --------------------------------------------------------------------- */
/* Frame helpers                                                          */
/* --------------------------------------------------------------------- */

static void fill_header(uint8_t *frame, uint8_t fn,
                        uint8_t dst_lo, uint8_t dst_hi,
                        uint8_t src_lo, uint8_t src_hi)
{
    frame[0] = FRAME_FC_0;
    frame[1] = FRAME_FC_1;
    frame[2] = s_seq;
    frame[3] = PAN_ID_LO;
    frame[4] = PAN_ID_HI;
    frame[5] = dst_lo;
    frame[6] = dst_hi;
    frame[7] = src_lo;
    frame[8] = src_hi;
    frame[9] = fn;
}

/* Non-broadcast header check (Poll/Response/Final): dest must be us. */
static bool header_ok(const uint8_t *f, uint8_t expected_fn)
{
    return f[0] == FRAME_FC_0 &&
           f[1] == FRAME_FC_1 &&
           f[3] == PAN_ID_LO  &&
           f[4] == PAN_ID_HI  &&
           f[5] == s_my_addr_lo &&
           f[6] == s_my_addr_hi &&
           f[7] == s_peer_addr_lo &&
           f[8] == s_peer_addr_hi &&
           f[9] == expected_fn;
}

/* Beacon header check: dst is broadcast, src is main, fn = BEACON. The
 * peripheral does NOT check s_peer_addr (it accepts beacons from any
 * 'VE' regardless of its own per-slot peer state). */
static bool beacon_header_ok(const uint8_t *f)
{
    return f[0] == FRAME_FC_0 &&
           f[1] == FRAME_FC_1 &&
           f[3] == PAN_ID_LO  &&
           f[4] == PAN_ID_HI  &&
           f[5] == ADDR_BCAST_LO &&
           f[6] == ADDR_BCAST_HI &&
           f[7] == ADDR_MAIN_LO  &&
           f[8] == ADDR_MAIN_HI  &&
           f[9] == FN_BEACON;
}

static void ts_to_frame(uint8_t *p, uint64_t ts)
{
    p[0] = (uint8_t)(ts);
    p[1] = (uint8_t)(ts >> 8);
    p[2] = (uint8_t)(ts >> 16);
    p[3] = (uint8_t)(ts >> 24);
    p[4] = (uint8_t)(ts >> 32);
}

static uint64_t ts_from_frame(const uint8_t *p)
{
    return  (uint64_t)p[0]
         | ((uint64_t)p[1] << 8)
         | ((uint64_t)p[2] << 16)
         | ((uint64_t)p[3] << 24)
         | ((uint64_t)p[4] << 32);
}

static uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    dwt_readtxtimestamp(ts_tab);
    uint64_t ts = 0;
    for (int i = 4; i >= 0; i--) { ts <<= 8; ts |= ts_tab[i]; }
    return ts;
}
static uint64_t get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    dwt_readrxtimestamp(ts_tab, DWT_COMPAT_NONE);
    uint64_t ts = 0;
    for (int i = 4; i >= 0; i--) { ts <<= 8; ts |= ts_tab[i]; }
    return ts;
}

/* --------------------------------------------------------------------- */
/* Wait loops                                                            */
/* --------------------------------------------------------------------- */

static bool wait_txfrs(int timeout_us)
{
    int waited = 0;
    while (waited < timeout_us) {
        if (dwt_readsysstatuslo() & SYS_STATUS_TXFRS_BIT_MASK) {
            dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
            return true;
        }
        esp_rom_delay_us(20);
        waited += 20;
    }
    return false;
}

static uint32_t wait_rx_event(int timeout_us)
{
    int waited = 0;
    uint32_t status = 0;
    const int poll_us  = 50;
    const int yield_us = 1000;
    int since_yield = 0;

    while (waited < timeout_us) {
        status = dwt_readsysstatuslo();
        if (status & SYS_STATUS_RX_ANY) return status;
        esp_rom_delay_us(poll_us);
        waited      += poll_us;
        since_yield += poll_us;
        if (since_yield >= yield_us) {
            vTaskDelay(1);
            since_yield = 0;
            waited += (portTICK_PERIOD_MS * 1000);
        }
    }
    return status;
}

/* --------------------------------------------------------------------- */
/* DS-TWR distance computation (used on the main node)                   */
/* --------------------------------------------------------------------- */

static double dstwr_distance_m(uint64_t poll_tx_ts, uint64_t resp_rx_ts,
                               uint64_t final_tx_ts,
                               uint64_t poll_rx_ts, uint64_t resp_tx_ts,
                               uint64_t final_rx_ts)
{
    int64_t Ra = (int64_t)((uint32_t)(resp_rx_ts  - poll_tx_ts));
    int64_t Rb = (int64_t)((uint32_t)(final_rx_ts - resp_tx_ts));
    int64_t Da = (int64_t)((uint32_t)(final_tx_ts - resp_rx_ts));
    int64_t Db = (int64_t)((uint32_t)(resp_tx_ts  - poll_rx_ts));

    int64_t num = Ra * Rb - Da * Db;
    int64_t den = Ra + Rb + Da + Db;
    if (den == 0) return -1.0;

    double tof_ticks = (double)num / (double)den;
    double tof_s     = tof_ticks * DWT_TIME_UNITS;
    return tof_s * SPEED_OF_LIGHT_M_PER_S;
}

/* --------------------------------------------------------------------- */
/* Recovery                                                              */
/* --------------------------------------------------------------------- */

static void uwb_handle_cycle_result(bool cycle_ok)
{
    if (cycle_ok) {
        s_consec_miss = 0;
        return;
    }

    s_consec_miss++;

    /* Peripheral-only: if we've never received a beacon, this is the
     * "main not yet powered / out of range" state. Recalibrating doesn't
     * help — the radio is fine, there's just no signal to hear yet. */
    if (s_role == UWB_ROLE_PERIPHERAL && !s_ever_linked) {
        if (s_consec_miss == 1 || s_consec_miss % 50 == 0) {
            ESP_LOGW(TAG, "Waiting for first beacon (%d attempts so far) — "
                          "is the main powered and on the same channel?",
                     s_consec_miss);
        }
        return;
    }

    /* Peripheral-only: Poll-late misses are a host-timing problem, not a
     * radio problem. Recalibrating would pause the radio for several ms,
     * making the next Poll EVEN MORE likely to be late — a death spiral.
     * Just count the miss and skip the recal ladder. The user-visible
     * "Poll TX late" log already includes the action to take (bump
     * PERIPHERAL_POLL_LEAD_US). */
    if (s_role == UWB_ROLE_PERIPHERAL && s_last_miss_reason == UWB_MISS_POLL_LATE) {
        return;
    }

    if (s_consec_miss >= RECOVER_HARD_AFTER) {
        ESP_LOGE(TAG, "%d consec misses -> hard recover", s_consec_miss);
        if (dwm3000_hard_recover() == ESP_OK) {
            dwm3000_reconfigure_recover(&s_uwb_config, &s_txconfig,
                                        UWB_TX_ANT_DLY, UWB_RX_ANT_DLY);
        }
        s_consec_miss = 0;
        s_last_recal_us = esp_timer_get_time();
        return;
    }

    if (s_consec_miss % RECOVER_RECONFIG_AFTER == 0) {
        ESP_LOGW(TAG, "%d consec misses -> reconfigure+recal (DW T=%.1f C)",
                 s_consec_miss, dwm3000_read_temp_c());
        dwm3000_reconfigure_recover(&s_uwb_config, &s_txconfig,
                                    UWB_TX_ANT_DLY, UWB_RX_ANT_DLY);
        s_last_recal_us = esp_timer_get_time();
    }
}

/* Proactive recal — run between rounds (main) or between cycles
 * (peripheral) regardless of miss count. Absorbs thermal drift before it
 * accumulates into misses. */
static void uwb_proactive_recal_if_due(void)
{
    int64_t now = esp_timer_get_time();
    if (s_last_recal_us == 0) {
        s_last_recal_us = now;   /* arm on first call so we don't recal at t=0 */
        return;
    }
    if (now - s_last_recal_us >= RECAL_INTERVAL_US) {
        ESP_LOGI(TAG, "Proactive recal (DW T=%.1f C, %lld s since last)",
                 dwm3000_read_temp_c(),
                 (long long)((now - s_last_recal_us) / 1000000));
        dwm3000_reconfigure_recover(&s_uwb_config, &s_txconfig,
                                    UWB_TX_ANT_DLY, UWB_RX_ANT_DLY);
        s_last_recal_us = now;
    }
}

/* --------------------------------------------------------------------- */
/* Per-slot exchange — runs once per slot on MAIN                        */
/* --------------------------------------------------------------------- */

/* Listens for Poll from the peripheral whose address is currently in
 * (s_peer_addr_lo, s_peer_addr_hi); does Response + Final RX; computes
 * distance into *out. Returns true on a fully successful exchange.
 *
 * Called only by main_round_tdma. */
static bool main_handle_one_slot(uwb_range_result_t *out)
{
    dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK
                       | SYS_STATUS_TXFRB_BIT_MASK
                       | SYS_STATUS_TXPRS_BIT_MASK
                       | SYS_STATUS_TXPHS_BIT_MASK
                       | SYS_STATUS_RX_ANY);

    /* === 1. Listen for Poll within this slot. === */
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(POLL_RX_TIMEOUT_PER_SLOT_UUS);
    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "slot rxenable failed");
        return false;
    }

    uint32_t status = wait_rx_event(POLL_RX_SW_TIMEOUT_US);
    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }
        dwt_forcetrxoff();
        return false;
    }
    dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

    /* === 2. Validate Poll: must be addressed to us AND from the
     * peripheral assigned to this slot. A wrong-suffix Poll means a
     * peripheral fired in the wrong slot — drop and let the slot expire. */
    uint8_t  ranging_bit = 0;
    uint16_t flen = dwt_getframelength(&ranging_bit);
    if (flen != POLL_FRAME_LEN + FCS_LEN || flen > RX_BUF_LEN) {
        dwt_forcetrxoff();
        return false;
    }
    dwt_readrxdata(rx_buf, POLL_FRAME_LEN, 0);
    if (!header_ok(rx_buf, FN_POLL)) {
        dwt_forcetrxoff();
        return false;
    }
    uint8_t peer_seq = rx_buf[2];

    /* === 3. Schedule Response, prepare for Final RX. === */
    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

    uint64_t poll_rx_ts = get_rx_timestamp_u64();
    uint32_t resp_tx_time = (uint32_t)((poll_rx_ts +
                                        (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME))
                                       >> 8);
    uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    uint8_t resp[RESP_FRAME_LEN];
    s_seq = peer_seq;
    fill_header(resp, FN_RESPONSE,
                s_peer_addr_lo, s_peer_addr_hi,
                s_my_addr_lo, s_my_addr_hi);
    ts_to_frame(&resp[RESP_POLL_RX_TS_IDX], poll_rx_ts);
    ts_to_frame(&resp[RESP_RESP_TX_TS_IDX], resp_tx_ts);

    dwt_writetxdata(RESP_FRAME_LEN, resp, 0);
    dwt_writetxfctrl(RESP_FRAME_LEN + FCS_LEN, 0, 1);
    dwt_setdelayedtrxtime(resp_tx_time);

    /* DW3000 errata workaround: stale RX timeout/error status bits cause
     * dwt_starttx(DELAYED) to spuriously return DWT_ERROR even when the
     * scheduled time is far in the future. Clear them right before the
     * call. See Qorvo forum 21963 (Pirmin's finding, Mar 2025). */
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

    int8_t txret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    if (txret != DWT_SUCCESS) {
        dwt_forcetrxoff();
        return false;
    }

    /* === 4. Wait for Final RX. === */
    status = wait_rx_event(20000);
    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }
        dwt_forcetrxoff();
        return false;
    }
    dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

    /* === 5. Validate Final and extract timestamps + IMU. === */
    flen = dwt_getframelength(&ranging_bit);
    if (flen != FINAL_FRAME_LEN + FCS_LEN || flen > RX_BUF_LEN) {
        dwt_forcetrxoff();
        return false;
    }
    dwt_readrxdata(rx_buf, FINAL_FRAME_LEN, 0);
    if (!header_ok(rx_buf, FN_FINAL)) {
        dwt_forcetrxoff();
        return false;
    }

    uint64_t poll_tx_ts  = ts_from_frame(&rx_buf[FINAL_POLL_TX_TS_IDX]);
    uint64_t resp_rx_ts  = ts_from_frame(&rx_buf[FINAL_RESP_RX_TS_IDX]);
    uint64_t final_tx_ts = ts_from_frame(&rx_buf[FINAL_FINAL_TX_TS_IDX]);
    uint64_t final_rx_ts = get_rx_timestamp_u64();

    /* === 6. Compute distance. === */
    double distance_m = dstwr_distance_m(poll_tx_ts, resp_rx_ts, final_tx_ts,
                                         poll_rx_ts, resp_tx_ts, final_rx_ts);

    if (distance_m < -1.0 || distance_m > 500.0) {
        dwt_forcetrxoff();
        return false;
    }

    double distance_m_calibrated = distance_m - g_uwb_distance_offset_m;
    if (out) {
        out->distance_m   = (float)distance_m_calibrated;
        out->timestamp_us = esp_timer_get_time();
        out->valid        = true;

       bio_telemetry_t bio;
        memcpy(&bio, &rx_buf[FINAL_BIO_IDX], sizeof(bio));
        if (bio.version == BIO_TELEM_VERSION) {
            out->peer_bio = bio;            /* change uwb_range_result_t to carry bio_telemetry_t */
            out->peer_bio_valid = true;
        } else {
            out->peer_bio_valid = false;    /* 0xFF sentinel or version mismatch */
        }
            }

            dwt_forcetrxoff();
            return true;
}

/* --------------------------------------------------------------------- */
/* MAIN: one full TDMA round                                             */
/* --------------------------------------------------------------------- */

static esp_err_t main_round_tdma(uwb_range_result_t *results)
{
    /* Pre-init all result slots to invalid. */
    for (int i = 0; i < UWB_MAX_PERIPHERALS; i++) {
        results[i].valid          = false;
        results[i].distance_m     = 0.0f;
        results[i].timestamp_us   = 0;
        results[i].peer_bio_valid = false;
    }

    /* Proactive recal between rounds (cheap, happens once per ~RECAL_INTERVAL). */
    uwb_proactive_recal_if_due();

    dwt_forcetrxoff();
    dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK
                       | SYS_STATUS_TXFRB_BIT_MASK
                       | SYS_STATUS_TXPRS_BIT_MASK
                       | SYS_STATUS_TXPHS_BIT_MASK
                       | SYS_STATUS_RX_ANY);

    /* === Slot 0: broadcast beacon. ===
     * The beacon's TX timestamp anchors every peripheral's slot timing
     * for this round. Send it as IMMEDIATE — peripherals are already
     * sitting in RX waiting for it. */
    uint8_t beacon[BEACON_FRAME_LEN];
    s_seq++;
    fill_header(beacon, FN_BEACON,
                ADDR_BCAST_LO, ADDR_BCAST_HI,
                s_my_addr_lo, s_my_addr_hi);
    beacon[BEACON_ROUND_IDX + 0] = (uint8_t)(s_round_counter);
    beacon[BEACON_ROUND_IDX + 1] = (uint8_t)(s_round_counter >> 8);
    beacon[BEACON_ROUND_IDX + 2] = (uint8_t)(s_round_counter >> 16);
    beacon[BEACON_ROUND_IDX + 3] = (uint8_t)(s_round_counter >> 24);

    dwt_writetxdata(BEACON_FRAME_LEN, beacon, 0);
    dwt_writetxfctrl(BEACON_FRAME_LEN + FCS_LEN, 0, 1);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "Beacon TX start failed");
        return ESP_FAIL;
    }
    bool txfrs_ok = wait_txfrs(5000);
    if (!txfrs_ok) {
        ESP_LOGW(TAG, "Beacon TXFRS timeout (round %lu)",
                 (unsigned long)s_round_counter);
        /* Don't bail — peripherals may have caught the preamble anyway. */
    }

    /* Periodic beacon-TX diagnostic. Confirms beacons are actually going
     * out (TXFRS asserted). If you see "TXFRS=NO" repeatedly here while
     * the peripheral sees no RX events, the main's TX path is the problem.
     *
     * NOTE on PLL: CPLOCK is an *edge* status bit (set when the PLL locks
     * coming out of reset/recal), not held while locked. Reading it later
     * always shows "UNLOCKED" even on a healthy radio — so it's NOT a
     * useful runtime PLL-health indicator. The real signal is whether
     * TXFRS keeps asserting; if TX completes, the PLL is fine. */
    {
        static int64_t s_last_main_diag_us = 0;
        static int     s_diag_txfrs_ok_total = 0;
        static int     s_diag_txfrs_fail_total = 0;
        if (txfrs_ok) s_diag_txfrs_ok_total++;
        else          s_diag_txfrs_fail_total++;

        int64_t now_us = esp_timer_get_time();
        if (s_last_main_diag_us == 0) s_last_main_diag_us = now_us;
        if (now_us - s_last_main_diag_us >= RX_DIAG_INTERVAL_US) {
            ESP_LOGI(TAG, "TX diag: round=%lu  beacons OK=%d/FAIL=%d  T=%.1f C",
                     (unsigned long)s_round_counter,
                     s_diag_txfrs_ok_total,
                     s_diag_txfrs_fail_total,
                     dwm3000_read_temp_c());
            s_last_main_diag_us = now_us;
            s_diag_txfrs_ok_total = 0;
            s_diag_txfrs_fail_total = 0;
        }
    }

    /* === Slots 1..UWB_MAX_PERIPHERALS: sweep peripherals. ===
     * We immediately re-enable RX after the beacon TX completes. The
     * peripherals' Polls land in their assigned slots; this loop just
     * processes them as they arrive. The hw RX timeout on each slot
     * (POLL_RX_TIMEOUT_PER_SLOT_UUS) keeps the loop from blocking
     * indefinitely on a quiet slot. */
    int ok_count = 0;
    for (int slot = 1; slot <= UWB_MAX_PERIPHERALS; slot++) {
        /* The peripheral assigned to this slot has suffix 'A' + (slot-1). */
        s_peer_addr_lo = ADDR_PERIPH_PREFIX;
        s_peer_addr_hi = (uint8_t)('A' + (slot - 1));

        bool slot_ok = main_handle_one_slot(&results[slot - 1]);
        if (slot_ok) {
            ok_count++;
        }
        /* main_handle_one_slot leaves the radio in IDLE on both paths,
         * so we can re-enter the next slot's RX immediately. */
    }

    /* Track miss-streaks per ROUND rather than per slot — a "round" is
     * the unit of progress at this layer. If no peripheral responded at
     * all, that's a real miss. If at least one did, the round was
     * useful. */
    uwb_handle_cycle_result(ok_count > 0);

    s_round_counter++;

    static int round_log = 0;
    if (round_log++ % 10 == 0) {
        ESP_LOGI(TAG, "Round %lu: %d/%d slots OK",
                 (unsigned long)s_round_counter, ok_count, UWB_MAX_PERIPHERALS);
    }

    return ESP_OK;
}

/* --------------------------------------------------------------------- */
/* PERIPHERAL: one TDMA cycle (wait beacon, schedule Poll, exchange)     */
/* --------------------------------------------------------------------- */

static esp_err_t peripheral_cycle_tdma(uwb_range_result_t *result)
{
    /* Optimistic: if any failure path runs, it'll overwrite this. */
    s_last_miss_reason = UWB_MISS_OTHER;

    uwb_proactive_recal_if_due();

    dwt_forcetrxoff();
    dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK
                       | SYS_STATUS_TXFRB_BIT_MASK
                       | SYS_STATUS_TXPRS_BIT_MASK
                       | SYS_STATUS_TXPHS_BIT_MASK
                       | SYS_STATUS_RX_ANY);

    /* === 1. Wait for the beacon. ===
     * Use a hardware RX timeout sized to TWO full rounds so a single
     * missed beacon doesn't permanently desync. */
    dwt_setrxaftertxdelay(0);
    /* RX timeout in uus; cap to ~16-bit because the register is 24-bit on
     * DW3000 but conservative caps prevent surprise on the driver port. */
    uint32_t beacon_rx_to_uus = (BEACON_RX_TIMEOUT_US * 1000UL) / 1026UL; /* ~1.0256 us per uus */
    if (beacon_rx_to_uus > 0xFFFFFFUL) beacon_rx_to_uus = 0xFFFFFFUL;
    dwt_setrxtimeout(beacon_rx_to_uus);

    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "beacon rxenable failed");
        return ESP_FAIL;
    }

    uint32_t status = wait_rx_event(BEACON_RX_TIMEOUT_US + 5000);

    /* Count anything the RX did — good or bad — so we can tell whether
     * the chip is seeing energy at all. */
    if (status & SYS_STATUS_RXFCG_BIT_MASK)                       s_rx_good_count++;
    if (status & (SYS_STATUS_RXFCG_BIT_MASK
                | SYS_STATUS_ALL_RX_TO
                | SYS_STATUS_ALL_RX_ERR))                          s_rx_event_count++;

    /* Periodic diagnostic. The key signal:
     *   - rx_event_count grows but rx_good_count stays low → RF energy is
     *     reaching the chip, but frames aren't decoding. Look at channel,
     *     preamble code, or interference.
     *   - both stay low/zero → the chip isn't seeing energy at all.
     *     Look at antenna, distance, or whether the main is actually TXing.
     *   - both grow but s_ever_linked still false → packets received but
     *     not from the expected source (PAN ID / address filter).
     *
     * (CPLOCK isn't reported here — it's an edge bit, not a level, so
     * reading it later always shows "unlocked" even on a healthy radio.
     * Use the beacons OK/FAIL counter on the main side for real PLL
     * health, or just observe whether RX events keep accumulating.) */
    int64_t now_us = esp_timer_get_time();
    if (s_last_diag_log_us == 0) s_last_diag_log_us = now_us;
    if (now_us - s_last_diag_log_us >= RX_DIAG_INTERVAL_US) {
        ESP_LOGI(TAG, "RX diag: events=%lu  good=%lu  linked=%s",
                 (unsigned long)s_rx_event_count,
                 (unsigned long)s_rx_good_count,
                 s_ever_linked ? "yes" : "NO");
        s_last_diag_log_us = now_us;
    }

    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        s_last_miss_reason = UWB_MISS_NO_BEACON;
        if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }
        static int bmiss = 0;
        if (bmiss < 5 || bmiss % 20 == 0) {
            ESP_LOGW(TAG, "Beacon RX miss #%d (status=0x%08lX)",
                     bmiss, (unsigned long)status);
        }
        bmiss++;
        dwt_forcetrxoff();
        return ESP_OK;
    }
    dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

    /* === 2. Validate beacon. === */
    uint8_t  ranging_bit = 0;
    uint16_t flen = dwt_getframelength(&ranging_bit);
    if (flen != BEACON_FRAME_LEN + FCS_LEN || flen > RX_BUF_LEN) return ESP_OK;
    dwt_readrxdata(rx_buf, BEACON_FRAME_LEN, 0);
    if (!beacon_header_ok(rx_buf)) return ESP_OK;

    /* Got a valid beacon from the main. Latch the linked state — from now
     * on, beacon misses are real problems worth recalibrating for. The
     * first transition gets a prominent log so the user can see the link
     * came up. */
    if (!s_ever_linked) {
        ESP_LOGI(TAG, "*** Linked to main *** (first beacon RX in slot %u)",
                 s_my_slot_index);
        s_ever_linked = true;
    }

    uint64_t beacon_rx_ts = get_rx_timestamp_u64();

    /* Snapshot host time at the start of "the work" so we can measure the
     * end-to-end host latency from beacon-validated to starttx-called.
     * That latency is what PERIPHERAL_POLL_LEAD_US must exceed. */
    int64_t host_t0_us = esp_timer_get_time();

    /* === 3. Schedule Poll TX at our slot offset from the beacon. ===
     *
     * Slot 1 starts at 0 ms after beacon (main's listen window for slot 1
     * opens immediately when slot loop begins). We add a small lead time
     * to push the actual Poll TX a few ms into the window, which gives
     * the main side a chance to finish entering RX after the beacon TX
     * completes. Slot N (1..15) -> offset (N-1) × SLOT_WIDTH + lead.
     *
     * The slot offset uses 64-bit arithmetic before shifting because the
     * raw tick count exceeds 32 bits for the later slots. Forgetting this
     * is a common source of "delayed TX fires immediately because the
     * scheduled time wrapped" bugs. */
    uint32_t slot_offset_us = (uint32_t)(s_my_slot_index - 1)
                            * (uint32_t)UWB_SLOT_WIDTH_MS
                            * 1000U
                            + PERIPHERAL_POLL_LEAD_US;
    uint64_t slot_offset_ticks = (uint64_t)slot_offset_us * UUS_TO_DWT_TIME;

    uint32_t poll_tx_time = (uint32_t)((beacon_rx_ts + slot_offset_ticks) >> 8);
    uint64_t poll_tx_ts   = (((uint64_t)(poll_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
    (void)poll_tx_ts;  /* read back from get_tx_timestamp_u64 after TX completes */

    /* Build Poll. */
    uint8_t poll[POLL_FRAME_LEN];
    s_seq++;
    fill_header(poll, FN_POLL,
                s_peer_addr_lo, s_peer_addr_hi,
                s_my_addr_lo, s_my_addr_hi);
    dwt_writetxdata(POLL_FRAME_LEN, poll, 0);
    dwt_writetxfctrl(POLL_FRAME_LEN + FCS_LEN, 0, 1);

    /* Set up the RX-after-TX window for the Response BEFORE arming the
     * delayed TX. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    dwt_setdelayedtrxtime(poll_tx_time);

    /* Measure host latency right before the critical starttx call. */
    int64_t host_t1_us = esp_timer_get_time();
    int64_t host_elapsed_us = host_t1_us - host_t0_us;

    /* DW3000 errata workaround: stale RX timeout/error status bits cause
     * dwt_starttx(DELAYED) to spuriously return DWT_ERROR even when the
     * scheduled time is far in the future. Clear them right before the
     * call. See Qorvo forum 21963 (Pirmin's finding, Mar 2025).
     *
     * This was the root cause of the "Poll TX late/cancelled" failures
     * we were seeing with 269 us host latency vs 5000 us lead — the chip
     * wasn't refusing because of timing, it was refusing because stale
     * RX-FTO bits from previous cycles were latched. */
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

    int8_t txret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    if (txret != DWT_SUCCESS) {
        /* dwt_starttx returned DWT_ERROR. Two possible causes:
         *   1. Scheduled time has actually passed (host took > lead).
         *      Distinguishable: host_elapsed_us > PERIPHERAL_POLL_LEAD_US.
         *   2. Stale RX-FTO/error bits or other internal state (errata).
         *      We just cleared the known-problematic ones above, but some
         *      other latched condition may still bite. Distinguishable:
         *      host_elapsed_us << PERIPHERAL_POLL_LEAD_US.
         *
         * The log message distinguishes between them so you know which
         * knob to turn (or whether it's something new to investigate). */
        s_last_miss_reason = UWB_MISS_POLL_LATE;
        static int late_log = 0;
        if (late_log < 5 || late_log % 50 == 0) {
            const char *likely = (host_elapsed_us > PERIPHERAL_POLL_LEAD_US - 500)
                                ? "host too slow — bump PERIPHERAL_POLL_LEAD_US"
                                : "chip refused despite ample time — stale status bits?";
            ESP_LOGW(TAG, "Poll TX cancelled #%d (slot %u): "
                          "host=%lld us lead=%d us — %s",
                     late_log, s_my_slot_index,
                     (long long)host_elapsed_us,
                     PERIPHERAL_POLL_LEAD_US,
                     likely);
        }
        late_log++;
        dwt_forcetrxoff();
        return ESP_OK;
    }

    /* Successful arm. Log the host latency occasionally so we can see how
     * close we are to the limit during normal operation. */
    static int arm_log_count = 0;
    if (arm_log_count++ % 50 == 0) {
        ESP_LOGI(TAG, "Poll armed OK (slot %u): host=%lld us, lead=%d us, margin=%lld us",
                 s_my_slot_index,
                 (long long)host_elapsed_us,
                 PERIPHERAL_POLL_LEAD_US,
                 (long long)(PERIPHERAL_POLL_LEAD_US - host_elapsed_us));
    }

    /* === 4. Wait for Response RX. === */
    status = wait_rx_event(15000);
    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }
        static int rmiss = 0;
        if (rmiss < 5 || rmiss % 50 == 0) {
            ESP_LOGW(TAG, "Resp RX miss #%d (status=0x%08lX)",
                     rmiss, (unsigned long)status);
        }
        rmiss++;
        dwt_forcetrxoff();
        return ESP_OK;
    }
    dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

    /* === 5. Validate Response. === */
    flen = dwt_getframelength(&ranging_bit);
    if (flen != RESP_FRAME_LEN + FCS_LEN || flen > RX_BUF_LEN) return ESP_OK;
    dwt_readrxdata(rx_buf, RESP_FRAME_LEN, 0);
    if (!header_ok(rx_buf, FN_RESPONSE)) return ESP_OK;

    uint64_t poll_tx_ts_actual = get_tx_timestamp_u64();
    uint64_t resp_rx_ts        = get_rx_timestamp_u64();
    /* poll_rx_ts and resp_tx_ts also live in the Response payload but
     * the peripheral doesn't use them — only the main needs them, and
     * the main has its own local copies. */

    /* === 6. Schedule Final TX with timestamps + IMU. ===
     * Fire-and-forget — there's no frame after this one in the slot. */
    uint32_t final_tx_time = (uint32_t)((resp_rx_ts +
                                         (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME))
                                        >> 8);
    uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    uint8_t final_frame[FINAL_FRAME_LEN];
    fill_header(final_frame, FN_FINAL,
                s_peer_addr_lo, s_peer_addr_hi,
                s_my_addr_lo, s_my_addr_hi);
    ts_to_frame(&final_frame[FINAL_POLL_TX_TS_IDX],  poll_tx_ts_actual);
    ts_to_frame(&final_frame[FINAL_RESP_RX_TS_IDX],  resp_rx_ts);
    ts_to_frame(&final_frame[FINAL_FINAL_TX_TS_IDX], final_tx_ts);

    lsm6_sample_t imu_snap;
    bool imu_snap_valid;
    portENTER_CRITICAL(&s_imu_lock);
    imu_snap       = s_local_imu;
    imu_snap_valid = s_local_imu_valid;
    portEXIT_CRITICAL(&s_imu_lock);
    if (imu_snap_valid) {
        memcpy(&final_frame[FINAL_IMU_IDX], &imu_snap, sizeof(imu_snap));
    } else {
        memset(&final_frame[FINAL_IMU_IDX], 0xFF, sizeof(imu_snap));
    }

    bio_telemetry_t bio_snap; bool bio_valid;
    portENTER_CRITICAL(&s_bio_lock);
    bio_snap = s_local_bio; bio_valid = s_local_bio_valid;
    portEXIT_CRITICAL(&s_bio_lock);
    if (bio_valid) {
        bio_snap.node_seq = s_seq;                 /* optional: track per-node continuity */
        memcpy(&final_frame[FINAL_BIO_IDX], &bio_snap, sizeof(bio_snap));
    } else {
        memset(&final_frame[FINAL_BIO_IDX], 0xFF, sizeof(bio_snap));  /* version=0xFF => ignore */
    }

    dwt_writetxdata(FINAL_FRAME_LEN, final_frame, 0);
    dwt_writetxfctrl(FINAL_FRAME_LEN + FCS_LEN, 0, 1);
    dwt_setdelayedtrxtime(final_tx_time);

    /* DW3000 errata workaround (same as Poll TX): clear stale RX status
     * bits or dwt_starttx(DELAYED) may spuriously return DWT_ERROR. */
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

    if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
        static int late_log = 0;
        if (late_log < 5 || late_log % 50 == 0) {
            ESP_LOGW(TAG, "Final TX late/cancelled #%d", late_log);
        }
        late_log++;
        dwt_forcetrxoff();
        return ESP_OK;
    }

    if (!wait_txfrs(5000)) {
        static int fmiss = 0;
        if (fmiss < 5 || fmiss % 50 == 0) {
            ESP_LOGW(TAG, "Final TXFRS timeout #%d", fmiss);
        }
        fmiss++;
        dwt_forcetrxoff();
        return ESP_OK;
    }

    if (result) {
        result->valid        = true;
        result->distance_m   = 0.0f;
        result->timestamp_us = esp_timer_get_time();
    }
    s_last_miss_reason = UWB_MISS_NONE;

    static int success_count = 0;
    success_count++;
    if (success_count % 50 == 1) {
        ESP_LOGI(TAG, "Peripheral cycle OK #%d (slot %u, Final sent)",
                 success_count, s_my_slot_index);
    }

    dwt_forcetrxoff();
    return ESP_OK;
}

/* --------------------------------------------------------------------- */
/* Public API                                                             */
/* --------------------------------------------------------------------- */

esp_err_t uwb_init(uwb_role_t role,
                   char peripheral_addr_suffix,
                   int mosi, int miso, int sclk, int cs, int rst)
{
    s_role = role;

    /* Suffix range is 'A'..'A' + UWB_MAX_PERIPHERALS - 1 = 'A'..'O' for
     * the default 15 peripherals. Anything outside that range (including
     * 0) falls back to 'A' / slot 1. */
    const char max_suffix = 'A' + UWB_MAX_PERIPHERALS - 1;
    char suffix = peripheral_addr_suffix;
    if (suffix < 'A' || suffix > max_suffix) suffix = DEFAULT_PERIPH_SUFFIX;

    if (role == UWB_ROLE_MAIN) {
        s_my_addr_lo    = ADDR_MAIN_LO;       s_my_addr_hi    = ADDR_MAIN_HI;
        /* Peer addr is set per-slot inside main_round_tdma. Default it
         * to slot 1 so logs read sensibly before the first round. */
        s_peer_addr_lo  = ADDR_PERIPH_PREFIX; s_peer_addr_hi  = DEFAULT_PERIPH_SUFFIX;
        s_my_slot_index = 0;
    } else {
        s_my_addr_lo    = ADDR_PERIPH_PREFIX; s_my_addr_hi    = suffix;
        s_peer_addr_lo  = ADDR_MAIN_LO;       s_peer_addr_hi  = ADDR_MAIN_HI;
        s_my_slot_index = (uint8_t)(suffix - 'A' + 1);   /* 1..15 */
    }
    ESP_LOGI(TAG, "Init UWB role=%s addr=%c%c peer=%c%c slot=%u (TDMA, %d slots)",
             role == UWB_ROLE_MAIN ? "main" : "peripheral",
             s_my_addr_lo, s_my_addr_hi,
             s_peer_addr_lo, s_peer_addr_hi,
             s_my_slot_index, UWB_MAX_PERIPHERALS);

    esp_err_t err = dwm3000_init(mosi, miso, sclk, cs, rst);
    if (err != ESP_OK) return err;

    uint32_t dev_id = 0;
    err = dwm3000_read_devid(&dev_id);
    if (err != ESP_OK) return err;
    if ((dev_id >> 16) != 0xDECA) {
        ESP_LOGE(TAG, "Wrong DEV_ID: 0x%08lX", (unsigned long)dev_id);
        return ESP_FAIL;
    }
    dwt_setpllcaltemperature(TEMP_INIT);

    if (dwt_configure((dwt_config_t *)&s_uwb_config) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "dwt_configure failed");
        return ESP_FAIL;
    }
    dwt_configuretxrf((dwt_txconfig_t *)&s_txconfig);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    ESP_LOGI(TAG, "UWB ready (chan 5, PLEN 64, 6.8 Mbps, TDMA DS-TWR)");

    if (!s_uwb_evt) s_uwb_evt = xEventGroupCreate();

    dwt_setcallbacks(cb_txdone, cb_rxok, cb_rxto, cb_rxerr, NULL, NULL, NULL);

    /* Enable TX-done AND every RX terminator. If you enable only TX you'll see
    * the classic "tx callback fires, rx never does" hang on DWT_RESPONSE_EXPECTED. */
    dwt_setinterrupt(SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK
                | SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK
                | SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK
                | SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK
                | SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK
                | SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK
                | SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK
                | SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK
                | SYS_ENABLE_LO_ARFE_ENABLE_BIT_MASK,
                    0, DWT_ENABLE_INT);

    port_set_dwic_isr(dwt_isr);
    port_enable_dw3000_irq();
    port_EnableEXT_IRQ();
    return ESP_OK;
}

esp_err_t uwb_perform_round(uwb_range_result_t *results)
{
    if (!results) return ESP_FAIL;

    if (s_role == UWB_ROLE_MAIN) {
        return main_round_tdma(results);
    } else {
        /* Peripheral writes only results[0]. Pre-init so the caller's
         * "success" check is well-defined even if the cycle bails early. */
        results[0].valid          = false;
        results[0].distance_m     = 0.0f;
        results[0].timestamp_us   = 0;
        results[0].peer_bio_valid = false;
        esp_err_t e = peripheral_cycle_tdma(&results[0]);
        uwb_handle_cycle_result(results[0].valid);
        return e;
    }
}

void uwb_publish_local_imu(const lsm6_sample_t *sample)
{
    if (!sample) return;
    portENTER_CRITICAL(&s_imu_lock);
    s_local_imu       = *sample;
    s_local_imu_valid = true;
    portEXIT_CRITICAL(&s_imu_lock);
}