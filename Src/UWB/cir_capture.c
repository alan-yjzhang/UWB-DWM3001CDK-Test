/**
 * @file  cir_capture.c
 * @brief Read the Ipatov CIR of each responder's ranging frame, hold it until the round's ranging
 *        result is known, then send it as a binary frame with responder address, distance and RSSI.
 *        See cir_capture.h for the frame layout.
 *
 *        Two tasks use this module: the MCPS task captures (cir_capture_note_tx/cir_capture_frame)
 *        and the uwbmac report task sends (cir_capture_report/cir_capture_round_done). Each buffer
 *        has a state, changed under a critical section, so a buffer is never filled and sent at once.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "deca_device_api.h"
#include "usb_uart_tx.h"
#include "critical_section.h"
#include "cir_capture.h"

#define SAMPLE_BYTES 6 /* 3 bytes real + 3 bytes imag */
#define FRAME_LEN    (CIR_FRAME_HEADER_LEN + CIR_NUM_SAMPLES * SAMPLE_BYTES)

typedef enum
{
    ENTRY_FREE = 0,
    ENTRY_FILLING, /* MCPS task is reading the CIR into it */
    ENTRY_READY,   /* holds a CIR, waiting for the ranging result */
    ENTRY_SENDING  /* report task is completing and sending it */
} entry_state_t;

typedef struct
{
    volatile uint8_t state;
    int32_t delta_rstu; /* time from our Poll to this frame, in RSTU */
    /* Header + payload. dwt_readaccdata() returns a dummy first byte, so we read into the slot just
     * before the payload; the header is written afterwards and overwrites that byte. */
    uint8_t buf[FRAME_LEN];
} entry_t;

static entry_t entries[CIR_MAX_RESPONDERS];

static volatile bool enabled = false;
static uint32_t seq = 0;
static uint32_t decim_counter = 0;

/* Last ranging frame we transmitted (in a round, the Poll comes before every response). */
static volatile uint32_t last_ranging_tx_dtu = 0;
static volatile bool have_ranging_tx = false;

/* Round being captured, identified by the time of its Poll. */
static uint32_t round_poll_dtu = 0;
static bool round_valid = false;
static bool round_capture = false;

void cir_capture_set_enabled(bool enable)
{
    enabled = enable;
}

bool cir_capture_is_enabled(void)
{
    return enabled;
}

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    put_u16(p, (uint16_t)v);
    put_u16(p + 2, (uint16_t)(v >> 16));
}

/* --------------------------------------------------------------------------------------------- */
/*  Capture side (MCPS task)                                                                     */
/* --------------------------------------------------------------------------------------------- */

void cir_capture_note_tx(uint32_t frame_start_dtu, bool ranging_frame)
{
    if (ranging_frame)
    {
        last_ranging_tx_dtu = frame_start_dtu;
        have_ranging_tx = true;
    }
}

static int16_t rssi_to_cdbm(float rssi_dbm)
{
    /* The driver reports a valid RSSI as a negative dBm value. */
    if (!(rssi_dbm < 0.0f) || rssi_dbm < -327.0f)
    {
        return CIR_RSSI_INVALID;
    }
    return (int16_t)(rssi_dbm * 100.0f - 0.5f);
}

void cir_capture_frame(uint64_t rx_timestamp, uint32_t rx_frame_start_dtu, int rstu_dtu,
                       int acc_len, float rssi_dbm)
{
    if (!enabled || !have_ranging_tx || rstu_dtu <= 0)
    {
        return;
    }

    /* New round? (a new Poll has been sent since the last capture) */
    const uint32_t poll_dtu = last_ranging_tx_dtu;
    if (!round_valid || poll_dtu != round_poll_dtu)
    {
        round_valid = true;
        round_poll_dtu = poll_dtu;
        round_capture = ((decim_counter++ % CIR_CAPTURE_DECIMATE) == 0);

        /* Free CIRs of an earlier round that were never reported. */
        enter_critical_section();
        for (int i = 0; i < CIR_MAX_RESPONDERS; i++)
        {
            if (entries[i].state == ENTRY_READY)
            {
                entries[i].state = ENTRY_FREE;
            }
        }
        leave_critical_section();
    }
    if (!round_capture)
    {
        return;
    }

    /* Take a free buffer. */
    entry_t *e = NULL;
    enter_critical_section();
    for (int i = 0; i < CIR_MAX_RESPONDERS; i++)
    {
        if (entries[i].state == ENTRY_FREE)
        {
            e = &entries[i];
            e->state = ENTRY_FILLING;
            break;
        }
    }
    leave_critical_section();
    if (e == NULL)
    {
        return; /* more responders than CIR_MAX_RESPONDERS: drop */
    }

    /* Slot offset from the Poll (frame start to frame start, both in DTU). */
    const int32_t delta_dtu = (int32_t)(rx_frame_start_dtu - poll_dtu);
    e->delta_rstu = (delta_dtu + rstu_dtu / 2) / rstu_dtu;

    /* Choose the 512-sample window: put the first path 128 samples after the window start,
     * clamped to the accumulator. */
    dwt_nlos_ipdiag_t idx;
    dwt_nlos_ipdiag(&idx);
    int fp = (int)(idx.index_fp_u32 >> 6); /* 10.6 fixed point -> integer index */
    int start = fp - 128;
    int max_start = acc_len - CIR_NUM_SAMPLES;
    if (start < 0)
    {
        start = 0;
    }
    if (start > max_start)
    {
        start = max_start;
    }

    /* Read N samples (+1 dummy byte). The dummy byte lands at buf[HEADER_LEN - 1]. */
    uint8_t *f = e->buf;
    dwt_readaccdata(&f[CIR_FRAME_HEADER_LEN - 1], CIR_NUM_SAMPLES * SAMPLE_BYTES + 1, (uint16_t)start);

    f[0] = 0xFF;
    f[1] = 'C';
    f[2] = 'I';
    f[3] = 'R';
    put_u16(&f[4], CIR_NUM_SAMPLES);
    put_u16(&f[6], (uint16_t)start);
    put_u32(&f[8], seq++);
    put_u32(&f[12], idx.index_fp_u32);
    put_u32(&f[16], idx.index_pp_u32);
    put_u32(&f[20], (uint32_t)rx_timestamp);
    put_u32(&f[24], (uint32_t)(rx_timestamp >> 32));
    put_u16(&f[28], CIR_RESPONDER_UNKNOWN);  /* filled in by cir_capture_report() */
    f[30] = CIR_STATUS_NO_RESULT;
    f[31] = 0;
    put_u32(&f[32], (uint32_t)CIR_DISTANCE_INVALID);
    put_u16(&f[36], (uint16_t)rssi_to_cdbm(rssi_dbm));
    memset(&f[38], 0, CIR_FRAME_HEADER_LEN - 38); /* reserved, also clears the dummy byte */

    e->state = ENTRY_READY;
}

/* --------------------------------------------------------------------------------------------- */
/*  Report side (uwbmac report task)                                                             */
/* --------------------------------------------------------------------------------------------- */

static int slot_of(const entry_t *e, int slot_duration_rstu)
{
    int32_t d = e->delta_rstu;
    int32_t k = (d + slot_duration_rstu / 2) / slot_duration_rstu;
    int32_t err = d - k * slot_duration_rstu;
    if (err < 0)
    {
        err = -err;
    }
    /* Must sit close to a slot boundary (ToF and clock errors are a tiny fraction of a slot). */
    if (d <= 0 || k > 255 || err > slot_duration_rstu / 4)
    {
        return -1;
    }
    return (int)k;
}

static void send_entry(entry_t *e)
{
    /* Copies the whole frame into the report ring buffer (dropped as a unit if it does not fit). */
    port_tx_msg(e->buf, FRAME_LEN);
    e->state = ENTRY_FREE;
}

void cir_capture_report(uint16_t responder_addr, int responder_slot, int slot_duration_rstu,
                        uint8_t status, int32_t distance_mm)
{
    if (!enabled || slot_duration_rstu <= 0)
    {
        return;
    }

    entry_t *e = NULL;
    enter_critical_section();
    for (int i = 0; i < CIR_MAX_RESPONDERS; i++)
    {
        if (entries[i].state == ENTRY_READY && slot_of(&entries[i], slot_duration_rstu) == responder_slot)
        {
            e = &entries[i];
            e->state = ENTRY_SENDING;
            break;
        }
    }
    leave_critical_section();
    if (e == NULL)
    {
        return; /* no CIR for this responder this round (frame missed, or round not captured) */
    }

    uint8_t *f = e->buf;
    put_u16(&f[28], responder_addr);
    f[30] = status;
    f[31] = (uint8_t)responder_slot;
    put_u32(&f[32], (uint32_t)((status == 0) ? distance_mm : CIR_DISTANCE_INVALID));
    send_entry(e);
}

void cir_capture_round_done(void)
{
    if (!enabled)
    {
        return;
    }
    for (int i = 0; i < CIR_MAX_RESPONDERS; i++)
    {
        entry_t *e = NULL;
        enter_critical_section();
        if (entries[i].state == ENTRY_READY)
        {
            e = &entries[i];
            e->state = ENTRY_SENDING;
        }
        leave_critical_section();
        if (e != NULL)
        {
            /* Not matched to any ranging result: send as is (responder 0xFFFF, status 0xFF, slot 0). */
            send_entry(e);
        }
    }
}
