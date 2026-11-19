#ifndef CIR_CAPTURE_H
#define CIR_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Ipatov-preamble CIR capture on the initiator, streamed as binary frames over the normal report
 * channel (USB CDC on J20 with the default build, since UART output is off).
 *
 * The CIR is read when each responder's ranging frame is received, but the frame is only sent
 * once the ranging round has been reported. That way each CIR frame also carries the
 * responder's short address, the measured distance, and the RSSI of the received frame.
 *
 * Which responder sent a frame: with SP3 frames there is no payload, so no source address.
 * The responder is worked out from the slot instead. It is the time between our Poll and the
 * received frame, divided by the slot duration. In FiRa DS-TWR, responder i (0-based, in
 * controlee-list order) answers i+1 slots after the Poll.
 *
 * Frame layout (all little-endian), followed directly by the sample payload:
 *
 *   offset size  field
 *   0      4     magic          FF 43 49 52   (0xFF 'C' 'I' 'R'; 0xFF never occurs in the JSON text)
 *   4      2     n_samples      number of complex samples in the payload (512)
 *   6      2     start_index    CIR index of the first sample sent
 *   8      4     seq            capture counter (counts every capture, wraps at 2^32)
 *   12     4     fp_index_raw   Ipatov first-path index from dwt_nlos_ipdiag(), raw (10.6 fixed point)
 *   16     4     pp_index_raw   Ipatov peak-path index, raw
 *   20     8     rx_timestamp   rctu timestamp of the received frame
 *   28     2     responder      responder short address (0xFFFF = could not be matched)
 *   30     1     status         ranging status: 0 = OK, 0xFF = no ranging result, other = stack error code
 *   31     1     slot           slot offset from the Poll that the frame was received in (responder i -> i+1)
 *   32     4     distance_mm    measured distance in mm, int32 (INT32_MIN when status != 0)
 *   36     2     rssi_cdbm      RSSI of the received frame in 0.01 dBm, int16 (INT16_MIN = not available;
 *                               needs diagnostics on, i.e. "DIAG 1", which auto_start.c turns on)
 *   38     2     reserved       0
 *   40     6*N   samples        per sample: real (3 bytes LE), imag (3 bytes LE); 18-bit signed values
 */
#define CIR_FRAME_HEADER_LEN 40
#define CIR_NUM_SAMPLES      512

/* Capture 1 out of every CIR_CAPTURE_DECIMATE ranging rounds (1 = every round). Every responder of
 * a captured round is captured. */
#define CIR_CAPTURE_DECIMATE 1

/* CIRs buffered per round (one per responder). Each buffer takes about 3.1 KB of RAM. */
#define CIR_MAX_RESPONDERS 4

#define CIR_RESPONDER_UNKNOWN 0xFFFF
#define CIR_STATUS_NO_RESULT  0xFF
#define CIR_DISTANCE_INVALID  INT32_MIN
#define CIR_RSSI_INVALID      INT16_MIN

void cir_capture_set_enabled(bool enable);
bool cir_capture_is_enabled(void);

/* --- Called from the UWB driver (dw3000_mcps_mcu.c, MCPS task) ---------------------------- */

/* Note every transmitted frame. frame_start_dtu is the scheduled frame start (tx info->timestamp_dtu).
 * Only ranging frames matter: the last one sent before a response is our Poll. */
void cir_capture_note_tx(uint32_t frame_start_dtu, bool ranging_frame);

/* Read the Ipatov CIR of the ranging frame just received and hold it until the round is reported.
 * rx_frame_start_dtu: frame start in the same timebase as the tx timestamps (rx info->timestamp_dtu).
 * rstu_dtu:  DTU per RSTU (llhw->rstu_dtu).
 * acc_len:   Ipatov accumulator length (1016 for 64 MHz PRF, 992 for 16 MHz PRF).
 * rssi_dbm:  RSSI of this frame in dBm; pass 0 (or any value >= 0) when it is not available. */
void cir_capture_frame(uint64_t rx_timestamp, uint32_t rx_frame_start_dtu, int rstu_dtu,
                       int acc_len, float rssi_dbm);

/* --- Called from the FiRa report callback (fira_app.c, initiator) ------------------------- */

/* Attach one ranging result to the CIR held for that responder and send the frame.
 * responder_slot: slot offset of this responder from the Poll (controlee-list index + 1). */
void cir_capture_report(uint16_t responder_addr, int responder_slot, int slot_duration_rstu,
                        uint8_t status, int32_t distance_mm);

/* End of the round's report: send any CIR still held (responder unknown, no ranging result). */
void cir_capture_round_done(void);

#endif /* CIR_CAPTURE_H */
