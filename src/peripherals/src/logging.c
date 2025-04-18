// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "logging.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

#if defined(ENABLE_AUDIO_DL) && ((7-ENABLE_AUDIO_DL-7 == 14) || (7-ENABLE_AUDIO_DL-7 != 0))

#define RTT_BUFFER_LENGTH (2*AUDIO_BUFFER_NUM_SAMPLES*sizeof(int16_t))
uint8_t rttAudioBuffer[RTT_BUFFER_LENGTH];

#endif


// Public API Functions ------------------------------------------------------------------------------------------------

void logging_init(void)
{
#if defined(ENABLE_LOGGING) && ((7-ENABLE_LOGGING-7 == 14) || (7-ENABLE_LOGGING-7 != 0))

   // Enable SWO
   if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 1) && (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 1))
   {
      uint32_t ui32dcuVal;
      am_hal_dcu_get(&ui32dcuVal);
      if (!(ui32dcuVal & AM_HAL_DCU_CPUTRC_TPIU_SWO) && (am_hal_dcu_update(true, AM_HAL_DCU_CPUTRC_TPIU_SWO) != AM_HAL_STATUS_SUCCESS))
         configASSERT0(1);
   }

   // Enable the ITM interface and the SWO pin
   am_hal_itm_enable();
   am_hal_tpiu_enable(AM_HAL_TPIU_BAUD_1M);
   am_hal_gpio_pincfg_t swo_pinconfig = g_AM_BSP_GPIO_ITM_SWO;
   swo_pinconfig.GP.cfg_b.uFuncSel = PIN_SWO_FUNCTION;
   am_hal_gpio_pinconfig(PIN_SWO, swo_pinconfig);

   // Attach the ITM to the STDIO driver
   am_util_stdio_printf_init(am_hal_itm_print);

#else

   logging_disable();

#endif

#if defined(ENABLE_AUDIO_DL) && ((7-ENABLE_AUDIO_DL-7 == 14) || (7-ENABLE_AUDIO_DL-7 != 0))

   // Set up SEGGER RTT data streaming
   SEGGER_RTT_Init();
   SEGGER_RTT_ConfigUpBuffer(1, "AudioLogger", rttAudioBuffer, RTT_BUFFER_LENGTH, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
   print("\nRTT Control Block Address:  0x%08X\n", (uint32_t)&_SEGGER_RTT);

#endif
}

void logging_disable(void)
{
   // Disable the ITM/TPIU and detach the ITM interface from the STDIO driver
   am_hal_itm_disable();
   am_util_stdio_printf_init(0);

   // Disconnect the SWO pin
   am_hal_gpio_pinconfig(PIN_SWO, am_hal_gpio_pincfg_disabled);
}

void print_reset_reason(const am_hal_reset_status_t* reason)
{
   print("\n----------------------------------------\n");
   print("Reset Reasons: ");
   if (reason->bEXTStat)
      print("External Reset, ");
   if (reason->bPORStat)
      print("HW Power-On Reset, ");
   if (reason->bBODStat)
      print("Brown-Out Reset, ");
   if (reason->bSWPORStat)
      print("SW Power-On Reset, ");
   if (reason->bSWPOIStat)
      print("SW Power-On Initialization, ");
   if (reason->bDBGRStat)
      print("Debugger Reset, ");
   if (reason->bWDTStat)
      print("Watch Dog Timer Reset, ");
   if (reason->bBOUnregStat)
      print("Unregulated Supply Brownout, ");
   if (reason->bBOCOREStat)
      print("Core Regulator Brownout, ");
   if (reason->bBOMEMStat)
      print("Memory Regulator Brownout, ");
   if (reason->bBOHPMEMStat)
      print("High-Power Memory Regulator Brownout, ");
   if (reason->bBOLPCOREStat)
      print("Low-Power Core Regulator Brownout, ");
   print("\n");
}
