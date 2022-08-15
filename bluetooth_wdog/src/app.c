/***************************************************************************//**
 * @file app.c
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************
 * This code has not been formally tested and is provided as-is. It is not
 * suitable for production environments. In addition, this code will not be
 * maintained and there may be no bug maintenance planned for these resources.
 * Silicon Labs may update projects from time to time.
 ******************************************************************************/
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"

#include "sl_sleeptimer.h"

#include "em_cmu.h"
#include "em_rmu.h"
#include "em_wdog.h"
#include "em_gpio.h"

// Choose a bit that will signal the watchdog feeder
#define EXT_SIG_WDOG_FEEDER     (1 << 6)

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

static uint32_t reset_cause;

static sl_sleeptimer_timer_handle_t wdog_feeder_signal_handle;

static void wdog_feeder_signal_cb(sl_sleeptimer_timer_handle_t *handle, void *data);

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  // Store the cause of the last reset, and clear the reset cause register
  reset_cause = RMU_ResetCauseGet();
  // Clear Reset causes so we know which reset occurs the next time
  RMU_ResetCauseClear();

  if(reset_cause & EMU_RSTCAUSE_WDOG0) {
      // TODO: handle case of watchdog reset
      while(1);
  }

  // Create watchdog feeder
  // Already initialized by default in a bluetooth project, but there's no
  // issue with calling the function again.
  sl_sleeptimer_init();

  // Start periodic timer to feed wdog every 1 sec
  sl_sleeptimer_start_periodic_timer(&wdog_feeder_signal_handle,      /* wdog timer handler */
                                     sl_sleeptimer_ms_to_tick(1000),  /* 1 second periodic timer */
                                     wdog_feeder_signal_cb,           /* callback function to feed wdog */
                                     NULL,                            /* no data passed to callback */
                                     3,                               /* priority 3 (arbitrary in this case) */
                                     0);                              /* no options */


  // Initialize watchdog to reset if the watchdog is not fed within ~2 seconds
  WDOG_Init_TypeDef wdog_init = WDOG_INIT_DEFAULT;
  wdog_init.em2Run = true;             /* WDOG counting when in EM2 */
  wdog_init.em3Run = true,             /* WDOG counting when in EM3 */
  wdog_init.perSel = wdogPeriod_2k;    /* Set the watchdog period to 2049 clock periods (ie ~2 seconds)*/

  // Enable clock for the WDOG module; has no effect on xG21
  CMU_ClockEnable(cmuClock_WDOG0, true);

  // Make watchdog use 1kHz clock source
  CMU_ClockSelectSet(cmuClock_WDOG0, cmuSelect_ULFRCO); /* ULFRCO as clock source */

  // Initializing watchdog with chosen settings
  WDOG_Init(&wdog_init);
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  // Test watchdog by halting the application.
  // GPIO port B, pin 1 is mapped to a Button for series 2 radio boards.
  // Check the WSTK radio board's user manual for button pinouts.
  // TODO: remove for release code
  GPIO_PinModeSet(gpioPortB, 1, gpioModeInputPull, 1);
  while(GPIO_PinInGet(gpioPortB, 1) == 0);

}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_assert_status(sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      app_assert_status(sc);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);
      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Restart advertising after client has disconnected.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates 1 second has elapsed for the watchdog feeder timer
    case sl_bt_evt_system_external_signal_id:
      // Feed watchdog if wdog feeder bit is set
      if(evt->data.evt_system_external_signal.extsignals & EXT_SIG_WDOG_FEEDER) {
          WDOG_Feed();
      }
      break;


    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

// This function will be called in an interrupt handler when the timer has
// elapsed one period
static void wdog_feeder_signal_cb(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;

  // Generate an event in the Blueooth stack
  sl_bt_external_signal(EXT_SIG_WDOG_FEEDER);
}
