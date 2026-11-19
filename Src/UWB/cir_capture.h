#ifndef CIR_CAPTURE_H
#define CIR_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Ipatov-preamble CIR capture, streamed as binary frames over the normal report
 * channel (USB CDC on J20 with the default build, since UART output is off).
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
 *   (reserved 36 bytes)
 *   64     6*N   samples        per sample: real (3 bytes LE), imag (3 bytes LE); 18-bit signed values
 */
#define CIR_FRAME_HEADER_LEN 64
#define CIR_PRE_FP_SAMPLES   64
#define CIR_NUM_SAMPLES      512
#define CIR_IPATOV_MAX_SAMPLES  1016 // or 992 for 

/* Capture 1 out of every CIR_CAPTURE_DECIMATE qualifying received frames (1 = all). */
#define CIR_CAPTURE_DECIMATE 10

void cir_capture_set_enabled(bool enable);
bool cir_capture_is_enabled(void);

/* Read the Ipatov CIR of the frame just received and queue it for transmission.
 * acc_len: Ipatov accumulator length (1016 for 64 MHz PRF, 992 for 16 MHz PRF). */
void cir_capture_frame(uint64_t rx_timestamp, int acc_len);

#endif /* CIR_CAPTURE_H */
