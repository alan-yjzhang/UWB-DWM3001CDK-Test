/**
 * @file      auto_start.c
 *
 * @brief     Start FiRa ranging automatically at boot, with the role chosen per board.
 *
 *            One firmware image is flashed on every board. At boot, each board reads its
 *            unique nRF52833 FICR DEVICEID and looks it up in AUTO_START_BOARDS below:
 *              - role INITIATOR -> behaves as if "INITF ..." had been typed (the tag)
 *              - role RESPONDER -> behaves as if "RESPF ..." had been typed (an anchor)
 *            A board whose ID is not in the table starts normally (idle CLI), so you can
 *            still use the serial terminal on it.
 *
 *            This overrides the weak DefaultTaskHookEvent() in defaultTask.c.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "nrf.h"
#include "app.h"
#include "common_fira.h"
#include "debug_config.h"
#include "cir_capture.h"

extern const app_definition_t helpers_app_fira[]; /* [0] = INITF, [1] = RESPF (see fira_app.c) */

/* ------------------------------------------------------------------------- */
/*  USER CONFIGURATION                                                       */
/* ------------------------------------------------------------------------- */

/* Parameters shared by every board. They MUST be identical on all boards.
 * Order: [RFRAME BPRF set] [slot rstu] [block ms] [round slots] [RR usage]
 *        [session id] [vupper64] [multi node mode] [round hopping] [initiator addr]
 * (same meaning as the INITF / RESPF command arguments) */
#define AUTO_COMMON_PARAMS "4 2400 200 25 2 42 01:02:03:04:05:06:07:08 1 0 0"

/* Short addresses. The initiator is 0; anchors are 1..4. Keep the initiator address
 * in sync with the last number of AUTO_COMMON_PARAMS above. */
#define AUTO_RESPONDER_LIST "1 2 3 4"

typedef enum { ROLE_INITIATOR, ROLE_RESPONDER } auto_role_t;

typedef struct {
    uint32_t id0;       /* NRF_FICR->DEVICEID[0] */
    uint32_t id1;       /* NRF_FICR->DEVICEID[1] */
    auto_role_t role;
    int addr;           /* short address for responders (ignored for the initiator) */
} auto_board_t;

/* Fill in the DEVICEID of each of your five boards. Read it with the J-Link port connected:
 *     nrfjprog --memrd 0x10000060 --n 8 -f nrf52
 * which prints two 32-bit words: DEVICEID[0] (at ...060) then DEVICEID[1] (at ...064).
 * The values below are PLACEHOLDERS and will not match any real board. */
static const auto_board_t AUTO_START_BOARDS[] = {
    { 0x00000000, 0x00000000, ROLE_INITIATOR, 0 },  /* tag      */
    { 0x00000000, 0x00000001, ROLE_RESPONDER, 1 },  /* anchor 1 */
    { 0x00000000, 0x00000002, ROLE_RESPONDER, 2 },  /* anchor 2 */
    { 0x00000000, 0x00000003, ROLE_RESPONDER, 3 },  /* anchor 3 */
    { 0x00000000, 0x00000004, ROLE_RESPONDER, 4 },  /* anchor 4 */
};

/* ------------------------------------------------------------------------- */

#define INITF_OFFSET 0
#define RESPF_OFFSET 1

const app_definition_t *DefaultTaskHookEvent(void)
{
    const uint32_t id0 = NRF_FICR->DEVICEID[0];
    const uint32_t id1 = NRF_FICR->DEVICEID[1];

    for (unsigned i = 0; i < sizeof(AUTO_START_BOARDS) / sizeof(AUTO_START_BOARDS[0]); i++)
    {
        const auto_board_t *b = &AUTO_START_BOARDS[i];
        if (b->id0 != id0 || b->id1 != id1)
        {
            continue;
        }

        char cmd[160];
        if (b->role == ROLE_INITIATOR)
        {
            snprintf(cmd, sizeof(cmd), "INITF " AUTO_COMMON_PARAMS " " AUTO_RESPONDER_LIST);
            scan_fira_params(cmd, true);
            show_fira_params();
            get_debug_config()->diagEn = 1; /* same as the "DIAG 1" command: adds RSSI_dBm and NLOS_% to results */
            cir_capture_set_enabled(true);  /* stream the Ipatov CIR as binary frames (see cir_capture.h) */
            return &helpers_app_fira[INITF_OFFSET];
        }
        else
        {
            snprintf(cmd, sizeof(cmd), "RESPF " AUTO_COMMON_PARAMS " %d", b->addr);
            scan_fira_params(cmd, false);
            show_fira_params();
            return &helpers_app_fira[RESPF_OFFSET];
        }
    }

    /* Unknown board: fall back to the stock behaviour (idle CLI). */
    return AppGetDefaultEvent();
}
