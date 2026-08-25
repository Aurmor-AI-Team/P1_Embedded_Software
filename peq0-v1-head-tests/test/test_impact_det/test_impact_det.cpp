// ---------------------------------------------------------------------------
// Host tests for the impact detector.
//
// These run on the dev machine (`pio test -e native`), not on a board: the
// detector is pure state-machine logic over (sample, time) and the properties
// that matter — one record per hit, the PEAK not the first crossing, the
// refractory window, the backlog — are exactly the ones that are painful to
// verify by hitting a real helmet.
//
// The transports are stubbed here, so we also get to assert what the board
// does when NOTHING is listening, which is the case that loses data in the
// field and the hardest one to reproduce deliberately.
// ---------------------------------------------------------------------------
#include <unity.h>

#include <string.h>
#include <vector>

#include "impact_det.h"
#include "ble_stream.h"
#include "wifi_udp_tx.h"

// --- controllable clock (declared in the esp_timer.h shim) ------------------
static int64_t s_now_us = 0;
extern "C" int64_t esp_timer_get_time(void) { return s_now_us; }
extern "C" void test_clock_set_us(int64_t us) { s_now_us = us; }
extern "C" void test_clock_advance_us(int64_t us) { s_now_us += us; }

// --- stubbed transports -----------------------------------------------------
static bool s_ble_up = false;
static bool s_wifi_up = false;
static std::vector<impact_rec_t> s_sent;

extern "C" bool ble_stream_ready(void) { return s_ble_up; }
extern "C" esp_err_t ble_stream_send_impact(const impact_rec_t *r)
{
    if (!s_ble_up) return ESP_ERR_INVALID_STATE;
    s_sent.push_back(*r);
    return ESP_OK;
}
extern "C" bool wifi_udp_is_verified(void) { return s_wifi_up; }
extern "C" esp_err_t wifi_udp_send_alert(const impact_rec_t *r)
{
    if (!s_wifi_up) return ESP_ERR_INVALID_STATE;
    s_sent.push_back(*r);
    return ESP_OK;
}

// --- helpers ----------------------------------------------------------------
static lsm6_sample_t sample_at(float h_mag)
{
    // The detector only reads h_mag (the caller computes the resultant), but it
    // takes the sample too, so give it a self-consistent one.
    lsm6_sample_t s = {};
    s.hx_g = h_mag;
    return s;
}

/** Feed one sample and advance the clock by one 200 Hz period. */
static void feed(float h_mag)
{
    lsm6_sample_t s = sample_at(h_mag);
    impact_det_feed(&s, h_mag);
    test_clock_advance_us(5000);   // 200 Hz
}

void setUp(void)
{
    test_clock_set_us(1000000);
    s_ble_up = true;
    s_wifi_up = false;
    s_sent.clear();
    impact_det_init();
    // Delivery defaults OFF (a board boots into the IDLE working mode). These
    // tests are about the DETECTOR, so turn it on here and let the two mode
    // tests below own the off case.
    impact_det_set_delivery_enabled(true);
    // impact_det_init() does not reset the running totals (they are boot-scoped
    // by design), so tests assert on s_sent rather than on impact_det_count().
}

void tearDown(void) {}

// ---------------------------------------------------------------------------

void test_below_threshold_never_fires(void)
{
    for (int i = 0; i < 100; i++) feed(IMPACT_THRESHOLD_G - 0.5f);
    TEST_ASSERT_EQUAL_UINT32(0, s_sent.size());
}

void test_one_burst_produces_exactly_one_record(void)
{
    // 30 samples above threshold = a 150 ms contact at 200 Hz. Without the
    // peak-hold this would emit ~30 records for a single hit.
    for (int i = 0; i < 30; i++) feed(35.0f);
    feed(1.0f);   // fall back below threshold, closing the event

    TEST_ASSERT_EQUAL_UINT32(1, s_sent.size());
}

void test_record_carries_the_peak_not_the_first_crossing(void)
{
    // A realistic ramp: the first sample over the line is well below the peak.
    const float ramp[] = { 21.0f, 28.0f, 44.5f, 39.0f, 25.0f };
    for (float g : ramp) feed(g);
    feed(1.0f);

    TEST_ASSERT_EQUAL_UINT32(1, s_sent.size());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 44.5f, s_sent[0].peak_g);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, IMPACT_THRESHOLD_G, s_sent[0].threshold_g);
}

void test_refractory_suppresses_a_second_hit(void)
{
    feed(30.0f);
    feed(1.0f);                       // event 1 closes
    TEST_ASSERT_EQUAL_UINT32(1, s_sent.size());

    // Ringing 100 ms later, inside the 250 ms dead time: same impact, not a new
    // one. Counting the ring-down as a second hit would inflate every count.
    test_clock_advance_us(100000);
    feed(30.0f);
    feed(1.0f);
    TEST_ASSERT_EQUAL_UINT32(1, s_sent.size());
}

void test_a_genuine_second_hit_after_the_refractory_does_fire(void)
{
    feed(30.0f);
    feed(1.0f);
    test_clock_advance_us(IMPACT_REFRACTORY_US + 10000);
    feed(30.0f);
    feed(1.0f);
    TEST_ASSERT_EQUAL_UINT32(2, s_sent.size());
    // Sequence numbers are monotonic and 0 is reserved.
    TEST_ASSERT_NOT_EQUAL(0, s_sent[0].seq);
    TEST_ASSERT_EQUAL_UINT32(s_sent[0].seq + 1, s_sent[1].seq);
}

void test_sustained_overload_is_capped_by_the_window(void)
{
    // A wedged high-g axis, or a board being sat on: held above threshold for a
    // full second. That is a sensor fault, not 2.5 head impacts per second.
    // Exactly ONE record, with a capped duration — and critically the detector
    // does NOT re-arm on the refractory alone while the signal stays high.
    for (int i = 0; i < 200; i++) feed(50.0f);   // 1 s at 200 Hz
    TEST_ASSERT_EQUAL_UINT32(1, s_sent.size());
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(IMPACT_WINDOW_US / 1000 + 5, s_sent[0].dur_ms);
}

void test_overload_rearms_once_the_signal_actually_releases(void)
{
    // Same sustained overload...
    for (int i = 0; i < 200; i++) feed(50.0f);
    TEST_ASSERT_EQUAL_UINT32(1, s_sent.size());

    // ...then it finally comes down, and a genuine later hit still registers.
    // The release requirement must suppress a stuck sensor WITHOUT ever
    // suppressing a real impact.
    for (int i = 0; i < 10; i++) feed(1.0f);
    test_clock_advance_us(IMPACT_REFRACTORY_US + 10000);
    feed(30.0f);
    feed(1.0f);
    TEST_ASSERT_EQUAL_UINT32(2, s_sent.size());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, s_sent[1].peak_g);
}

void test_stalled_sample_stream_still_emits_via_service(void)
{
    // Arm the detector, then have the samples stop dead (I2C wedged, task
    // starved). The hit happened; it must not sit unreported forever.
    feed(30.0f);
    TEST_ASSERT_EQUAL_UINT32(0, s_sent.size());   // still open

    test_clock_advance_us(IMPACT_WINDOW_US + 1000);
    impact_det_service();
    TEST_ASSERT_EQUAL_UINT32(1, s_sent.size());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, s_sent[0].peak_g);
}

void test_impacts_are_held_when_no_transport_is_up(void)
{
    s_ble_up = false;
    s_wifi_up = false;

    feed(30.0f);
    feed(1.0f);
    TEST_ASSERT_EQUAL_UINT32(0, s_sent.size());       // nothing sent...
    TEST_ASSERT_EQUAL_UINT8(1, impact_det_backlog()); // ...but not lost either
}

void test_backlog_drains_when_a_transport_returns(void)
{
    s_ble_up = false;
    s_wifi_up = false;
    for (int i = 0; i < 3; i++) {
        feed(30.0f);
        feed(1.0f);
        test_clock_advance_us(IMPACT_REFRACTORY_US + 10000);
    }
    TEST_ASSERT_EQUAL_UINT8(3, impact_det_backlog());

    // Phone reconnects. This is the case the backlog exists for.
    s_ble_up = true;
    impact_det_service();
    TEST_ASSERT_EQUAL_UINT32(3, s_sent.size());
    TEST_ASSERT_EQUAL_UINT8(0, impact_det_backlog());
}

void test_backlog_falls_back_to_wifi(void)
{
    s_ble_up = false;
    s_wifi_up = true;
    feed(30.0f);
    feed(1.0f);
    // No BLE subscriber, but the receiver is verified — it goes out over UDP.
    TEST_ASSERT_EQUAL_UINT32(1, s_sent.size());
    TEST_ASSERT_EQUAL_UINT8(0, impact_det_backlog());
}

void test_backlog_overflow_drops_the_oldest(void)
{
    s_ble_up = false;
    s_wifi_up = false;
    for (int i = 0; i < IMPACT_BACKLOG_DEPTH + 4; i++) {
        feed(30.0f + i);
        feed(1.0f);
        test_clock_advance_us(IMPACT_REFRACTORY_US + 10000);
    }
    TEST_ASSERT_EQUAL_UINT8(IMPACT_BACKLOG_DEPTH, impact_det_backlog());

    s_ble_up = true;
    // Drain fully: service() only takes IMPACT_DRAIN_PER_TICK per call, on
    // purpose, so a reconnect does not stall the sample stream behind 32
    // notifies.
    for (int i = 0; i < IMPACT_BACKLOG_DEPTH; i++) impact_det_service();
    TEST_ASSERT_EQUAL_UINT32(IMPACT_BACKLOG_DEPTH, s_sent.size());

    // The 4 dropped are the OLDEST, so the newest hit always survives —
    // the newest is the one someone is about to be pulled off the field for.
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f + IMPACT_BACKLOG_DEPTH + 3,
                             s_sent.back().peak_g);
    // What survived is one contiguous run of the most recent DEPTH records.
    TEST_ASSERT_EQUAL_UINT32(IMPACT_BACKLOG_DEPTH - 1,
                             s_sent.back().seq - s_sent.front().seq);
}

void test_drain_is_rate_limited_per_service_call(void)
{
    s_ble_up = false;
    s_wifi_up = false;
    for (int i = 0; i < 10; i++) {
        feed(30.0f);
        feed(1.0f);
        test_clock_advance_us(IMPACT_REFRACTORY_US + 10000);
    }
    s_ble_up = true;
    impact_det_service();
    TEST_ASSERT_EQUAL_UINT32(IMPACT_DRAIN_PER_TICK, s_sent.size());
}

void test_running_totals_accumulate_across_hits(void)
{
    feed(30.0f);
    feed(1.0f);
    test_clock_advance_us(IMPACT_REFRACTORY_US + 10000);
    feed(50.0f);
    feed(1.0f);

    TEST_ASSERT_EQUAL_UINT32(2, s_sent.size());
    const impact_rec_t &first = s_sent[0];
    const impact_rec_t &last  = s_sent[1];
    // The totals are BOOT-scoped by design (impact_det_init does not clear
    // them), and these tests share a process, so assert the deltas rather than
    // absolute values.
    TEST_ASSERT_EQUAL_UINT32(first.count + 1, last.count);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, first.sum_g + 50.0f, last.sum_g);
    TEST_ASSERT_TRUE(last.max_g >= 50.0f);
    TEST_ASSERT_TRUE(last.max_g >= first.max_g);
}

void test_inject_produces_a_record(void)
{
    // The bring-up hook: the mock CSV peaks near 1 g, so this is the only way
    // to exercise the pipeline without real hardware.
    impact_det_inject(35.0f);
    TEST_ASSERT_EQUAL_UINT32(1, s_sent.size());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 35.0f, s_sent[0].peak_g);
}

// --- the working-mode delivery gate ----------------------------------------
// IDLE and MOCK promise that nothing goes on the wire. The thing that must NOT
// happen is that they also lose the hit: an athlete who takes a knock while the
// app happens to be sitting in idle still took that knock.

void test_delivery_disabled_holds_records_rather_than_dropping_them(void)
{
    impact_det_set_delivery_enabled(false);
    s_ble_up = true;   // a transport IS up — the mode is the only thing stopping us

    for (int i = 0; i < 5; i++) feed(35.0f);
    feed(1.0f);
    impact_det_service();

    TEST_ASSERT_EQUAL_UINT32(0, s_sent.size());      // nothing on the wire
    TEST_ASSERT_EQUAL_UINT8(1, impact_det_backlog()); // but not lost
}

void test_re_enabling_delivery_drains_what_idle_held(void)
{
    impact_det_set_delivery_enabled(false);
    s_ble_up = true;

    // Three hits while the board is quiet.
    for (int hit = 0; hit < 3; hit++) {
        for (int i = 0; i < 5; i++) feed(35.0f);
        feed(1.0f);
        test_clock_advance_us(IMPACT_REFRACTORY_US);
    }
    impact_det_service();
    TEST_ASSERT_EQUAL_UINT32(0, s_sent.size());

    // The user picks live/alerts: the backlog goes out on the next housekeeping
    // pass (IMPACT_DRAIN_PER_TICK is 4, so one call covers three).
    impact_det_set_delivery_enabled(true);
    impact_det_service();

    TEST_ASSERT_EQUAL_UINT32(3, s_sent.size());
    TEST_ASSERT_EQUAL_UINT8(0, impact_det_backlog());
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_below_threshold_never_fires);
    RUN_TEST(test_one_burst_produces_exactly_one_record);
    RUN_TEST(test_record_carries_the_peak_not_the_first_crossing);
    RUN_TEST(test_refractory_suppresses_a_second_hit);
    RUN_TEST(test_a_genuine_second_hit_after_the_refractory_does_fire);
    RUN_TEST(test_sustained_overload_is_capped_by_the_window);
    RUN_TEST(test_overload_rearms_once_the_signal_actually_releases);
    RUN_TEST(test_stalled_sample_stream_still_emits_via_service);
    RUN_TEST(test_impacts_are_held_when_no_transport_is_up);
    RUN_TEST(test_backlog_drains_when_a_transport_returns);
    RUN_TEST(test_backlog_falls_back_to_wifi);
    RUN_TEST(test_backlog_overflow_drops_the_oldest);
    RUN_TEST(test_drain_is_rate_limited_per_service_call);
    RUN_TEST(test_running_totals_accumulate_across_hits);
    RUN_TEST(test_inject_produces_a_record);
    RUN_TEST(test_delivery_disabled_holds_records_rather_than_dropping_them);
    RUN_TEST(test_re_enabling_delivery_drains_what_idle_held);
    return UNITY_END();
}
