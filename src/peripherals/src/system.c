// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "audio.h"
#include "battery.h"
#include "henrik.h"
#include "imu.h"
#include "led.h"
#include "logging.h"
#include "magnet.h"
#include "mram.h"
#include "rtc.h"
#include "storage.h"
#include "system.h"
#include "vhf.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

extern uint8_t _uid_base_address;


// Ambiq Interrupt Service Routines and MCU Functions ------------------------------------------------------------------

void _close(void) {}
void _lseek(void) {}
void _read(void) {}
void _write(void) {}
void _fstat(void) {}
void _getpid(void) {}
void _isatty(void) {}
void _kill(void) {}

void am_gpio0_001f_isr(void)
{
   static uint32_t status;
   AM_CRITICAL_BEGIN
   am_hal_gpio_interrupt_irq_status_get(GPIO0_001F_IRQn, false, &status);
   am_hal_gpio_interrupt_irq_clear(GPIO0_001F_IRQn, status);
   AM_CRITICAL_END
   am_hal_gpio_interrupt_service(GPIO0_001F_IRQn, status);
}

void am_gpio0_203f_isr(void)
{
   static uint32_t status;
   AM_CRITICAL_BEGIN
   am_hal_gpio_interrupt_irq_status_get(GPIO0_203F_IRQn, false, &status);
   am_hal_gpio_interrupt_irq_clear(GPIO0_203F_IRQn, status);
   AM_CRITICAL_END
   am_hal_gpio_interrupt_service(GPIO0_203F_IRQn, status);
}

void am_gpio0_405f_isr(void)
{
   static uint32_t status;
   AM_CRITICAL_BEGIN
   am_hal_gpio_interrupt_irq_status_get(GPIO0_405F_IRQn, false, &status);
   am_hal_gpio_interrupt_irq_clear(GPIO0_405F_IRQn, status);
   AM_CRITICAL_END
   am_hal_gpio_interrupt_service(GPIO0_405F_IRQn, status);
}

void am_gpio0_607f_isr(void)
{
   static uint32_t status;
   AM_CRITICAL_BEGIN
   am_hal_gpio_interrupt_irq_status_get(GPIO0_607F_IRQn, false, &status);
   am_hal_gpio_interrupt_irq_clear(GPIO0_607F_IRQn, status);
   AM_CRITICAL_END
   am_hal_gpio_interrupt_service(GPIO0_607F_IRQn, status);
}


// Helpful Debugging Functions and Macros ------------------------------------------------------------------------------

typedef struct __attribute__((packed)) ContextStateFrame
{ uint32_t r0, r1, r2, r3, r12, lr, return_address, xpsr; } sContextStateFrame;

#define HARDFAULT_HANDLING_ASM(_x)               \
  __asm volatile(                                \
      "tst lr, #4 \n"                            \
      "ite eq \n"                                \
      "mrseq r0, msp \n"                         \
      "mrsne r0, psp \n"                         \
      "b system_hard_fault_handler \n"           )

__attribute__((optimize("O0")))
void system_hard_fault_handler(sContextStateFrame *frame)
{
#ifdef AM_DEBUG_PRINTF
   do {
      if (CoreDebug->DHCSR & (1 << 0))
         __asm("bkpt 1");
   } while (0);
#else
   NVIC_SystemReset();
   while (true) {}
#endif
}

void HardFault_Handler(void) { HARDFAULT_HANDLING_ASM(); }

void vAssertCalled(const char * const pcFileName, unsigned long ulLine)
{
   volatile uint32_t ulSetToNonZeroInDebuggerToContinue = 0;
   while (ulSetToNonZeroInDebuggerToContinue == 0);
}


// Private Helper Functions --------------------------------------------------------------------------------------------

static void system_initialize_unused_pins(void)
{
   // Initialize all unused pins as pulled up and disabled
   const uint32_t unused_pins[] = UNUSED_PINS;
   const am_hal_gpio_pincfg_t unused_pin_config = AM_HAL_GPIO_PINCFG_PULLEDUP_DISABLED;
   for (uint32_t i = 0; i < (sizeof(unused_pins) / sizeof(unused_pins[0])); ++i)
      am_hal_gpio_pinconfig(unused_pins[i], unused_pin_config);
}


// Public API Functions ------------------------------------------------------------------------------------------------

void setup_hardware(void)
{
   // Read the hardware reset reason
   am_hal_reset_status_t reset_reason;
   am_hal_reset_status_get(&reset_reason);

   // Enable the floating point module
   am_hal_sysctrl_fpu_enable();
   am_hal_sysctrl_fpu_stacking_enable(true);

   // Configure the board to operate in low-power mode
   am_hal_pwrctrl_low_power_init();
   am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_SIMOBUCK_INIT, NULL);
#ifndef AM_DEBUG_PRINTF
   am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_CRYPTO_POWERDOWN, NULL);
#endif

   // Configure only the necessary memory
   am_hal_pwrctrl_dsp_memory_config_t dsp_mem_config =
   {
      .bEnableICache = false,
      .bRetainCache = false,
      .bEnableRAM = false,
      .bActiveRAM = false,
      .bRetainRAM = false
   };
   am_hal_pwrctrl_mcu_memory_config_t mcu_mem_config =
   {
      .eCacheCfg    = AM_HAL_PWRCTRL_CACHE_NONE,
      .bRetainCache = false,
      .eDTCMCfg     = AM_HAL_PWRCTRL_DTCM_384K,
      .eRetainDTCM  = AM_HAL_PWRCTRL_DTCM_384K,
      .bEnableNVM0  = true,
      .bRetainNVM0  = false
   };
   am_hal_pwrctrl_sram_memcfg_t sram_mem_config =
   {
#if defined(ENABLE_AUDIO_DL) && ((7-ENABLE_AUDIO_DL-7 == 14) || (7-ENABLE_AUDIO_DL-7 != 0))
      .eSRAMCfg           = AM_HAL_PWRCTRL_SRAM_ALL,
      .eActiveWithMCU     = AM_HAL_PWRCTRL_SRAM_ALL,
#else
      .eSRAMCfg           = AM_HAL_PWRCTRL_SRAM_1M_GRP0,
      .eActiveWithMCU     = AM_HAL_PWRCTRL_SRAM_1M_GRP0,
#endif
      .eActiveWithGFX     = AM_HAL_PWRCTRL_SRAM_NONE,
      .eActiveWithDISP    = AM_HAL_PWRCTRL_SRAM_NONE,
      .eActiveWithDSP     = AM_HAL_PWRCTRL_SRAM_NONE,
#if defined(ENABLE_AUDIO_DL) && ((7-ENABLE_AUDIO_DL-7 == 14) || (7-ENABLE_AUDIO_DL-7 != 0))
      .eSRAMRetain        = AM_HAL_PWRCTRL_SRAM_ALL
#else
      .eSRAMRetain        = AM_HAL_PWRCTRL_SRAM_1M_GRP0
#endif
   };
   am_hal_pwrctrl_dsp_memory_config(AM_HAL_DSP0, &dsp_mem_config);
   am_hal_pwrctrl_dsp_memory_config(AM_HAL_DSP1, &dsp_mem_config);
   am_hal_pwrctrl_mcu_memory_config(&mcu_mem_config);
   am_hal_pwrctrl_sram_config(&sram_mem_config);
   am_hal_cachectrl_disable();

   // Initialize all unused GPIO pins to a known state
   system_initialize_unused_pins();

   // Set up printing to the console
   mram_init();
   logging_init();
   print_reset_reason(&reset_reason);
}

void system_reset(void)
{
   // Gracefully shut down all peripherals and initiate a power-on reset
   system_deinitialize_peripherals();
   am_hal_reset_control(AM_HAL_RESET_CONTROL_SWPOR, NULL);
}

void system_initialize_peripherals(void)
{
   // Initialize peripherals and start up the RTC
   leds_init();
   rtc_init();
   vhf_init();
   storage_init();
   imu_init();
   henrik_init();
   magnet_sensor_init();
   battery_monitor_init();
   system_enable_interrupts(true);
   storage_setup_logs();
}

void system_deinitialize_peripherals(void)
{
   // De-initialize all peripherals except for RTC and VHF
   mram_deinit();
   storage_deinit();
   audio_deinit();
   am_hal_interrupt_master_disable();
   henrik_deinit();
   imu_deinit();
   magnet_sensor_deinit();
   battery_monitor_deinit();
   leds_deinit();
   logging_disable();
}

void system_enable_interrupts(bool enabled)
{
   // Enable or disable all system interrupts
   if (enabled)
      am_hal_interrupt_master_enable();
   else
      am_hal_interrupt_master_disable();
}

void system_enter_deep_sleep_mode(void)
{
   // Only enter Deep Sleep mode if not streaming audio for download
#if !defined(ENABLE_AUDIO_DL) || (7-ENABLE_AUDIO_DL-7 == 14) || (7-ENABLE_AUDIO_DL-7 == 0)
   am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
#endif
}

void system_enter_power_off_mode(uint32_t wake_on_magnet, uint32_t wake_on_timestamp, bool reinit_on_wakeup)
{
   // Turn off all peripherals
   print("WARNING: Powering off. Will awake on: [ %s%s]...\n", wake_on_magnet ? "Magnet " : "", wake_on_timestamp ? "Timestamp " : "");
   system_deinitialize_peripherals();

   // Power down the crypto module followed by all peripherals
   am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_CRYPTO_POWERDOWN, NULL);
   am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_DIS_PERIPHS_ALL, NULL);

   // Optionally allow a change on the magnet sensor GPIO pin to wake up the device
   if (wake_on_magnet)
   {
      am_hal_gpio_pincfg_t input_pin_config = AM_HAL_GPIO_PINCFG_INPUT;
      input_pin_config.GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_6K;
      input_pin_config.GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
      uint32_t wakeup_pin = wake_on_magnet, interrupt_status;
      am_hal_gpio_pinconfig(wakeup_pin, input_pin_config);
      AM_CRITICAL_BEGIN
      am_hal_gpio_interrupt_irq_status_get(GPIO0_001F_IRQn, false, &interrupt_status);
      am_hal_gpio_interrupt_irq_clear(GPIO0_001F_IRQn, interrupt_status);
      AM_CRITICAL_END
      am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0, AM_HAL_GPIO_INT_CTRL_INDV_ENABLE, &wakeup_pin);
      NVIC_SetPriority(GPIO0_001F_IRQn + GPIO_NUM2IDX(wakeup_pin), AM_IRQ_PRIORITY_DEFAULT);
      NVIC_EnableIRQ(GPIO0_001F_IRQn + GPIO_NUM2IDX(wakeup_pin));
      magnet_sensor_enable_for_wakeup();
   }

   // Optionally, configure the RTC to wake the device at a specific timestamp
   if (wake_on_timestamp)
      rtc_set_wakeup_timestamp(wake_on_timestamp);

   // Enable interrupts and enter Deep Sleep mode
   am_hal_interrupt_master_enable();
   am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

   // Reinitialize select peripherals upon waking from Deep Sleep mode, if requested
   if (reinit_on_wakeup)
   {
      logging_init();
      leds_init();
      storage_init();
      battery_monitor_init();
      magnet_sensor_init();
      storage_setup_logs();
      print("INFO: Device woke up!\n");
   }
}

void system_read_ID(uint8_t *id, uint32_t id_length)
{
   // Copy ID from flash memory location into the specified buffer
   bool uninitialized = true;
   uint8_t *_id = &_uid_base_address;
   for (uint32_t i = 0; i < id_length; ++i)
   {
      id[i] = _id[i];
      if (id[i] != 0xFF)
         uninitialized = false;
   }
   if (uninitialized)
   {
      const uint32_t id0 = MCUCTRL->CHIPID0, id1 = MCUCTRL->CHIPID1;
      memcpy(id, &id0, sizeof(id0));
      memcpy(id + sizeof(id0), &id1, id_length - sizeof(id0));
   }
}

void system_delay(uint32_t delay_us)
{
   am_hal_delay_us(delay_us);
}
