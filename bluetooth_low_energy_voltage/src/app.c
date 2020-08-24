/***************************************************************************//**
 * @file app.c
 * @brief Silicon Labs Empty Example Project
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 * @version 1.0.1
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
 *******************************************************************************
 * # Experimental Quality
 * This code has not been formally tested and is provided as-is. It is not
 * suitable for production environments. In addition, this code will not be
 * maintained and there may be no bug maintenance planned for these resources.
 * Silicon Labs may update projects from time to time.
 ******************************************************************************/

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

#include "app.h"

#include "em_cmu.h"
#include "em_emu.h"
#include "em_prs.h"
#include "em_adc.h"
#include "em_ldma.h"
#include "em_letimer.h"

#define BUFFER_SIZE		64
#define LDMA_CHANNEL	0
#define ADC_DVL			1		// Number of samples to get at once

/* Print boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);

void LETimertoADC_PRS_Setup(void);

void LDMA_Setup(void);
void ADC_Open(void);
void LETimer_Open(void);

/* Flag for indicating DFU Reset must be performed */
static uint8_t boot_to_dfu = 0;

/* Buffer that will be filled by DMA */
uint32_t ADCBuffer[BUFFER_SIZE];

LDMA_TransferCfg_t ldmaXfer = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_ADC0_SINGLE);;
LDMA_Descriptor_t ldmaDescr = LDMA_DESCRIPTOR_SINGLE_P2M_BYTE(&(ADC0->SINGLEDATA),
															  ADCBuffer,
															  BUFFER_SIZE);

/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0
  pconfig->sleep.flags = 0;
#endif

  /* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
  initLog();

  /* Initialize stack */
  gecko_init(pconfig);

  LETimertoADC_PRS_Setup();
  LDMA_Setup();

  uint32_t average = 0;

  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* if there are no events pending then the next call to gecko_wait_event() may cause
     * device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
    if (!gecko_event_pending()) {
      flushLog();
    }

    /* Check for stack event. This is a blocking event listener. If you want non-blocking please see UG136. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Do not call any stack commands before receiving the boot event.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:

        bootMessage(&(evt->data.evt_system_boot));
        printLog("boot event - starting advertising\r\n");

        /* Set advertising parameters. 100ms advertisement interval.
         * The first parameter is advertising set handle
         * The next two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6).
         * The last two parameters are duration and maxevents left as default. */
        gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);

        /* Start general advertising and enable connections. */
        gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        break;

      case gecko_evt_le_connection_opened_id:
        printLog("connection opened\r\n");
        break;

      case gecko_evt_le_connection_closed_id:

        printLog("connection closed, reason: 0x%2.2x\r\n", evt->data.evt_le_connection_closed.reason);

        /* Check if need to boot to OTA DFU mode */
        if (boot_to_dfu) {
          /* Enter to OTA DFU mode */
          gecko_cmd_system_reset(2);
        } else {
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        }
        break;

      case gecko_evt_system_external_signal_id:
		if((evt->data.evt_system_external_signal.extsignals & 0x01) == 0x01){
		  average = 0;
		  for(int i = 0; i < BUFFER_SIZE; i++){
			  average += *(ADCBuffer + i);
		  }
		  average = average / BUFFER_SIZE;
		  uint8_t data[2];
		  data[1] = average & 0x00FF;
		  data[0] = (average >> 8) & 0x00FF;
		  gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_adc, 2, data);
		}
		break;

      /* Events related to OTA upgrading
         ----------------------------------------------------------------------------- */

      /* Check if the user-type OTA Control Characteristic was written.
       * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
      case gecko_evt_gatt_server_user_write_request_id:

        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
          /* Set flag to enter to OTA mode */
          boot_to_dfu = 1;
          /* Send response to Write Request */
          gecko_cmd_gatt_server_send_user_write_response(
            evt->data.evt_gatt_server_user_write_request.connection,
            gattdb_ota_control,
            bg_err_success);

          /* Close connection to enter to DFU OTA mode */
          gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
        }

        printLog("write\r\n");
        break;

      /* Add additional event handlers as your application requires */

      default:
        break;
    }
  }
}

/* Print stack version and local Bluetooth address as boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt)
{
#if DEBUG_LEVEL
  bd_addr local_addr;
  int i;

  printLog("stack version: %u.%u.%u\r\n", bootevt->major, bootevt->minor, bootevt->patch);
  local_addr = gecko_cmd_system_get_bt_address()->address;

  printLog("local BT device address: ");
  for (i = 0; i < 5; i++) {
    printLog("%2.2x:", local_addr.addr[5 - i]);
  }
  printLog("%2.2x\r\n", local_addr.addr[0]);
#endif
}



void LETimertoADC_PRS_Setup(void)
{
  LETimer_Open();
  CMU_ClockEnable(cmuClock_PRS, true);
  PRS_SourceAsyncSignalSet(0,
						  PRS_CH_CTRL_SOURCESEL_LETIMER0,
						  PRS_CH_CTRL_SIGSEL_LETIMER0CH0);
  ADC_Open();
}

void ADC_Open(void)
{
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_ADC0, true);

  /* Select AUXHFRCO for ADC ASYNC mode so it can run in EM2 */
  CMU->ADCCTRL = CMU_ADCCTRL_ADC0CLKSEL_AUXHFRCO;

  ADC_Init_TypeDef adcInit = ADC_INIT_DEFAULT;
  adcInit.em2ClockConfig = adcEm2ClockOnDemand;
  adcInit.timebase = ADC_TimebaseCalc(CMU_AUXHFRCOBandGet());
  adcInit.prescale = ADC_PrescaleCalc(CMU_AUXHFRCOFreqGet(), CMU_AUXHFRCOBandGet());

  ADC_InitSingle_TypeDef adcSingleInit = ADC_INITSINGLE_DEFAULT;
  adcSingleInit.singleDmaEm2Wu = true;
  adcSingleInit.reference = adcRefVDD;
  //adcSingleInit.posSel = adcPosSelAPORT3YCH9;
  adcSingleInit.posSel = adcPosSelAPORT4XCH11;
  adcSingleInit.prsSel = adcPRSSELCh0;
  adcSingleInit.prsEnable = true;
  ADC_Init(ADC0, &adcInit);
  ADC_InitSingle(ADC0, &adcSingleInit);
}

void LDMA_Setup(void)
{
  CMU_ClockEnable(cmuClock_LDMA, true);
  LDMA_Init_t ldmaInit = LDMA_INIT_DEFAULT;
  LDMA_Init(&ldmaInit);

  /* ADC to memory */
  ldmaDescr.xfer.decLoopCnt = true;
  ldmaDescr.xfer.doneIfs = true;
  ldmaDescr.xfer.blockSize = ADC_DVL - 1;
  ldmaDescr.xfer.ignoreSrec = true;
  ldmaDescr.xfer.size = ldmaCtrlSizeWord;

  LDMA_StartTransfer(LDMA_CHANNEL, &ldmaXfer, &ldmaDescr);

  // Interrupt whenever dma completed the transfer
  LDMA_IntEnable(LDMA_IF_DONE_DEFAULT);

  NVIC_ClearPendingIRQ(LDMA_IRQn);
  NVIC_EnableIRQ(LDMA_IRQn);
}

void LETimer_Open(void)
{
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

  // Enable clock for LETIMER0
  CMU_ClockEnable(cmuClock_LETIMER0, true);

  LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;
  letimerInit.comp0Top = true;
  letimerInit.ufoa0 = letimerUFOAPulse;
  LETIMER_Init(LETIMER0, &letimerInit);

  //LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP0);

  LETIMER_CompareSet(LETIMER0, 0, CMU_ClockFreqGet(cmuClock_LETIMER0) / 480);

  // Need REP0 != 0 to pulse on underflow
  LETIMER_RepeatSet(LETIMER0, 0, 1);

  NVIC_ClearPendingIRQ(LETIMER0_IRQn);
  //NVIC_EnableIRQ(LETIMER0_IRQn);	// No interrupt
}

void LDMA_IRQHandler(void)
{
  LDMA_IntClear(LDMA_IntGet());
  gecko_external_signal(0x01);
  LDMA_StartTransfer(0, &ldmaXfer, &ldmaDescr);
}


