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

/* Parameters shared by every board:
static const char INITF_CMD_COMMENT[] = {
    "INITF [RFRAME BPRF set] [Slot duration rstu] [Block duration ms] [Round duration slots] [RR usage] [Session id] [vupper64 xx:xx:xx:xx:xx:xx:xx:xx] [Multi node mode] [Round hopping] [Initiator Addr] [Responder 1 Addr] ... [Responder n Addr]"};
static const char RESPF_CMD_COMMENT[] = {
    "RESPF [RFRAME BPRF set] [Slot duration rstu] [Block duration ms] [Round duration slots] [RR usage] [Session id] [vupper64 xx:xx:xx:xx:xx:xx:xx:xx] [Multi node mode] [Round hopping] [Initiator Addr] [Responder Addr]"};
    */
#define APP_PARAMS "4 2400 200  25  2  42 01:02:03:04:05:06:07:08  1 0 0"
/* Short addresses. The initiator is 0; anchors are 1..4. Keep the initiator address
 * in sync with the last number of AUTO_COMMON_PARAMS above. */
#define RESPONDERS "1" //"addr1  addr2 addr3  addr4 ..." all responder addresses 
const int device_addr = 0;
static char cmd[255];

/* ------------------------------------------------------------------------- */

#define INITF_OFFSET 0
#define RESPF_OFFSET 1

const app_definition_t *DefaultTaskHookEvent(void)
{
    if(device_addr != 0 ){
      snprintf(cmd,sizeof(cmd), "RESPF " APP_PARAMS " %d", device_addr);
      scan_fira_params(cmd, false);
      //show_fira_params();
      return &helpers_app_fira[RESPF_OFFSET]; // RESPF task
    }else{
      snprintf(cmd,sizeof(cmd), "INITF " APP_PARAMS "  " RESPONDERS);
      scan_fira_params(cmd, true);
      show_fira_params();
      cir_capture_set_enabled(true);  /* stream the Ipatov CIR as binary frames (see cir_capture.h) */
      get_debug_config()->diagEn = 1; // same as "DIAG 1" command, add RSSI_dbm and NLOS_% to reports
      return &helpers_app_fira[INITF_OFFSET]; // INITF task    
    }
    return AppGetDefaultEvent();
}
