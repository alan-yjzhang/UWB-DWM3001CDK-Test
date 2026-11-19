/**
 * @file  cir_capture.c
 * @brief Read the Ipatov CIR after each ranging frame and send it as a binary frame.
 *        See cir_capture.h for the frame layout.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "deca_device_api.h"
#include "usb_uart_tx.h"
#include "cir_capture.h"

#define SAMPLE_BYTES 6 /* 3 bytes real + 3 bytes imag */

/* One buffer holds header + payload. dwt_readaccdata() returns a dummy first byte, so we read
 * into the slot just before the payload and then overwrite that byte with the last header byte. */
static uint8_t frame[CIR_FRAME_HEADER_LEN + CIR_NUM_SAMPLES * SAMPLE_BYTES];

static volatile bool enabled = false;
static uint32_t seq = 0;
static uint32_t decim_counter = 0;

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

void cir_capture_frame(uint64_t rx_timestamp, int acc_len)
{
    if (!enabled)
    {
        return;
    }
    if ((decim_counter++ % CIR_CAPTURE_DECIMATE) != 0)
    {
        return;
    }

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

    /* Read N samples (+1 dummy byte). The dummy byte ends up at frame[HEADER_LEN - 1]. */
    dwt_readaccdata(&frame[CIR_FRAME_HEADER_LEN - 1], CIR_NUM_SAMPLES * SAMPLE_BYTES + 1, (uint16_t)start);

    frame[0] = 0xFF;
    frame[1] = 'C';
    frame[2] = 'I';
    frame[3] = 'R';
    put_u16(&frame[4], CIR_NUM_SAMPLES);
    put_u16(&frame[6], (uint16_t)start);
    put_u32(&frame[8], seq++);
    put_u32(&frame[12], idx.index_fp_u32);
    put_u32(&frame[16], idx.index_pp_u32);
    put_u32(&frame[20], (uint32_t)rx_timestamp);
    put_u32(&frame[24], (uint32_t)(rx_timestamp >> 32));

    /* Copies the whole frame into the report ring buffer (dropped as a unit if it does not fit). */
    port_tx_msg(frame, (int)sizeof(frame));
}
