// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "audio.h"
#include "battery.h"
#include "imu.h"
#include "led.h"
#include "logging.h"
#include "magnet.h"
#include "mram.h"
#include "rtc.h"
#include "storage.h"
#include "system.h"
#include "tracker.h"
#include "vhf.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

#define SCRATCH_MAGIC_MASK          0xFFFF0000
#define SCRATCH_MAGIC               0xA3E30000
#define SCRATCH0_REASON_MASK        0x000000FF
#define SCRATCH1_RESET_COUNT_MASK   0x0000FFFF

extern uint8_t _uid_base_address;

static system_boot_info_t boot_info;
static volatile uint32_t hal_failure_count;
static const char *first_hal_failure_file;
static uint32_t first_hal_failure_line, first_hal_failure_status;
static bool watchdog_running, sram_active_in_deep_sleep;

#if ENABLE_CACHE_MONITOR
static uint64_t cache_total_accesses, cache_total_hits;
#endif


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
   // Record where the fault happened before restarting
   const uint32_t faulting_address = frame ? frame->return_address : 0;
   MCUCTRL->SCRATCH0 = SCRATCH_MAGIC | RESET_REASON_HARD_FAULT;
   MCUCTRL->SCRATCH1 = MCUCTRL->SCRATCH1;   // Preserve the reset counter across this path

   // Persist the faulting address as well, but only when it differs from what is already stored
   mram_boot_record_t stored_record;
   mram_get_boot_record(&stored_record);
   if (stored_record.fault_address != faulting_address)
      mram_store_fault(RESET_REASON_HARD_FAULT, SCB->CFSR, faulting_address);
   NVIC_SystemReset();
   while (true) {}
#endif
}

void HardFault_Handler(void) { HARDFAULT_HANDLING_ASM(); }
void MemManage_Handler(void)  { HARDFAULT_HANDLING_ASM(); }
void BusFault_Handler(void)   { HARDFAULT_HANDLING_ASM(); }
void UsageFault_Handler(void) { HARDFAULT_HANDLING_ASM(); }

void vAssertCalled(const char * const pcFileName, unsigned long ulLine)
{
   volatile uint32_t ulSetToNonZeroInDebuggerToContinue = 0;
   while (ulSetToNonZeroInDebuggerToContinue == 0);
}

void system_note_hal_failure(const char *file, uint32_t line, uint32_t status)
{
   // Capture the first failure in full and count the rest
   AM_CRITICAL_BEGIN
   if (!hal_failure_count)
   {
      first_hal_failure_file = file;
      first_hal_failure_line = line;
      first_hal_failure_status = status;
   }
   ++hal_failure_count;
   AM_CRITICAL_END
}

uint32_t system_get_hal_failure_count(void)
{
   return hal_failure_count;
}

const char* system_get_first_hal_failure(uint32_t *line, uint32_t *status)
{
   if (line)
      *line = first_hal_failure_line;
   if (status)
      *status = first_hal_failure_status;
   return first_hal_failure_file;
}


// Private Helper Functions --------------------------------------------------------------------------------------------

static void system_initialize_unused_pins(void)
{
   // Initialize all unused pins as fully disabled
   const uint32_t unused_pins[] = UNUSED_PINS;
   for (uint32_t i = 0; i < (sizeof(unused_pins) / sizeof(unused_pins[0])); ++i)
      am_hal_gpio_pinconfig(unused_pins[i], am_hal_gpio_pincfg_disabled);
}

static void system_capture_boot_info(void)
{
   // Read the hardware reset status latched by the reset generator
   am_hal_reset_status_get(&boot_info.hardware_status);

   // Recover the software reset reason handed over by the previous run.
   // A missing magic tag means the scratch register lost its contents, which only happens on a real power cycle.
   const uint32_t scratch0 = MCUCTRL->SCRATCH0, scratch1 = MCUCTRL->SCRATCH1;
   const bool scratch_valid = ((scratch1 & SCRATCH_MAGIC_MASK) == SCRATCH_MAGIC);
   boot_info.was_power_on = !scratch_valid;
   boot_info.resets_this_epoch = scratch_valid ? ((scratch1 & SCRATCH1_RESET_COUNT_MASK) + 1) : 0;
   if ((scratch0 & SCRATCH_MAGIC_MASK) == SCRATCH_MAGIC)
      boot_info.software_reason = scratch0 & SCRATCH0_REASON_MASK;
   else
      boot_info.software_reason = scratch_valid ? RESET_REASON_UNKNOWN : RESET_REASON_NONE;

   // A power cycle starts a new epoch, which is the only time the counter in MRAM is programmed
   if (boot_info.was_power_on)
      mram_increment_boot_epoch();
   boot_info.boot_epoch = mram_get_boot_epoch();

   // Recover the fault address recorded by a previous hard fault
   mram_boot_record_t stored_record;
   mram_get_boot_record(&stored_record);
   boot_info.fault_address = (boot_info.software_reason == RESET_REASON_HARD_FAULT) ? stored_record.fault_address : 0;

   // Re-stamp the scratch registers for the next boot
   MCUCTRL->SCRATCH0 = SCRATCH_MAGIC | RESET_REASON_UNKNOWN;
   MCUCTRL->SCRATCH1 = SCRATCH_MAGIC | (boot_info.resets_this_epoch & SCRATCH1_RESET_COUNT_MASK);
}


// Public API Functions ------------------------------------------------------------------------------------------------

void setup_hardware(void)
{
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
      .eCacheCfg    = AM_HAL_PWRCTRL_CACHEB0_ONLY,
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
      .eActiveWithMCU     = AM_HAL_PWRCTRL_SRAM_NONE,
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
   sram_active_in_deep_sleep = false;

   // Enable the MRAM instruction cache
   const am_hal_cachectrl_config_t cache_config =
   {
      .eDescript = AM_HAL_CACHECTRL_DESCR_1WAY_128B_512E,
      .eMode     = AM_HAL_CACHECTRL_CONFIG_MODE_INSTR,
      .bLRU      = false
   };
   configASSERT0(am_hal_cachectrl_config(&cache_config));
   configASSERT0(am_hal_cachectrl_enable());

#if ENABLE_CACHE_MONITOR
   // Turn on the cache hit/miss counters so the chosen cache size can be judged from measurement rather than argument
   configASSERT0(am_hal_cachectrl_control(AM_HAL_CACHECTRL_CONTROL_MONITOR_ENABLE, NULL));
   configASSERT0(am_hal_cachectrl_control(AM_HAL_CACHECTRL_CONTROL_STATISTICS_RESET, NULL));
#endif

   // Initialize all unused GPIO pins to a known state
   system_initialize_unused_pins();

   // Set up persistent storage and determine why the device restarted
   mram_init();
   system_capture_boot_info();
   logging_init();
   print_reset_reason(&boot_info.hardware_status);
}

const char* reset_reason_name(uint32_t reason)
{
   switch (reason)
   {
      case RESET_REASON_NONE:                return "POWER-ON";
      case RESET_REASON_HARD_FAULT:          return "HARD-FAULT";
      case RESET_REASON_PHASE_COMPLETE:      return "PHASE-DONE";
      case RESET_REASON_AUDIO_ERROR:         return "AUDIO-ERROR";
      case RESET_REASON_STORAGE_FAILURE:     return "SD-FAILURE";
      case RESET_REASON_RTC_STOPPED:         return "RTC-STOPPED";
      case RESET_REASON_BATTERY_LOW:         return "BATTERY-LOW";
      case RESET_REASON_MISSING_CONFIG:      return "NO-CONFIG";
      case RESET_REASON_MAGNET_DEACTIVATED:  return "MAGNET-OFF";
      case RESET_REASON_ACTIVATED:           return "MAGNET-ON";
      case RESET_REASON_PERIPHERAL_TIMEOUT:  return "PERIPH-TIMEOUT";
      default:                               return "UNKNOWN";
   }
}

const system_boot_info_t* system_get_boot_info(void)
{
   return &boot_info;
}

void system_reset(void)
{
   system_reset_with_reason(RESET_REASON_UNKNOWN);
}

void system_reset_with_reason(uint32_t reason)
{
   // Record why the device is restarting so that the next boot can report it
   MCUCTRL->SCRATCH0 = SCRATCH_MAGIC | (reason & SCRATCH0_REASON_MASK);
   system_deinitialize_peripherals();
   am_hal_reset_control(AM_HAL_RESET_CONTROL_SWPOR, NULL);
}

void system_initialize_peripherals(void)
{
   // Initialize peripherals and start up the RTC
   leds_init();
   rtc_init();
   vhf_init();
   imu_init();
   tracker_init();
   magnet_sensor_init();
   battery_monitor_init();
   system_enable_interrupts(true);
   storage_init();
   storage_setup_logs();
}

void system_deinitialize_peripherals(void)
{
   // De-initialize all peripherals except for RTC and VHF
   mram_deinit();
   storage_deinit();
   audio_deinit();
   am_hal_interrupt_master_disable();
   tracker_deinit();
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
   storage_flush_log();
   system_disable_watchdog();
   system_release_sram_retention();
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
      system_enable_watchdog();
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

void system_enable_watchdog(void)
{
   // Configure a watchdog timer which must be fed within WATCHDOG_TIMEOUT_SECONDS
   if (watchdog_running)
   {
      am_hal_wdt_restart(AM_HAL_WDT_MCU);
      return;
   }
   am_hal_wdt_config_t watchdog_config = {
      .eClockSource = AM_HAL_WDT_1HZ,
      .bInterruptEnable = false,
      .ui32InterruptValue = 0,
      .bResetEnable = true,
      .ui32ResetValue = WATCHDOG_TIMEOUT_SECONDS,
      .bAlertOnDSPReset = false
   };
   configASSERT0(am_hal_wdt_config(AM_HAL_WDT_MCU, &watchdog_config));
   configASSERT0(am_hal_wdt_start(AM_HAL_WDT_MCU, false));
   watchdog_running = true;
}

void system_disable_watchdog(void)
{
   // Stop the watchdog
   if (watchdog_running)
   {
      am_hal_wdt_stop(AM_HAL_WDT_MCU);
      watchdog_running = false;
   }
}

void system_feed_watchdog(void)
{
   if (watchdog_running)
      am_hal_wdt_restart(AM_HAL_WDT_MCU);
}

void system_set_sram_active(bool active_during_deep_sleep)
{
   // Hold the lower SSRAM group fully active only while a DMA transfer needs to reach it during Deep Sleep
   if (active_during_deep_sleep != sram_active_in_deep_sleep)
   {
      am_hal_pwrctrl_sram_memcfg_t sram_mem_config;
      am_hal_pwrctrl_sram_config_get(&sram_mem_config);
      sram_mem_config.eActiveWithMCU = active_during_deep_sleep ? AM_HAL_PWRCTRL_SRAM_1M_GRP0 : AM_HAL_PWRCTRL_SRAM_NONE;
      configASSERT0(am_hal_pwrctrl_sram_config(&sram_mem_config));
      sram_active_in_deep_sleep = active_during_deep_sleep;
   }
}

void system_release_sram_retention(void)
{
   // Stop retaining shared SRAM entirely for an intentional long sleep
   am_hal_pwrctrl_sram_memcfg_t sram_mem_config;
   am_hal_pwrctrl_sram_config_get(&sram_mem_config);
   sram_mem_config.eActiveWithMCU = AM_HAL_PWRCTRL_SRAM_NONE;
   sram_mem_config.eSRAMRetain = AM_HAL_PWRCTRL_SRAM_NONE;
   configASSERT0(am_hal_pwrctrl_sram_config(&sram_mem_config));
   sram_active_in_deep_sleep = false;
}

void system_accumulate_cache_stats(void)
{
#if ENABLE_CACHE_MONITOR
   uint32_t accesses = 0, hits = 0;
   AM_CRITICAL_BEGIN
   accesses = CPU->IMON0;
   hits = CPU->IMON2;
   CPU->CACHECTRL = CPU_CACHECTRL_RESETSTAT_Msk;
   am_hal_sysctrl_sysbus_write_flush();
   AM_CRITICAL_END
   cache_total_accesses += accesses;
   cache_total_hits += hits;
#endif
}

bool system_get_cache_stats(uint64_t *accesses, uint64_t *hits, float *hit_rate_percent)
{
#if ENABLE_CACHE_MONITOR
   system_accumulate_cache_stats();
   *accesses = cache_total_accesses;
   *hits = cache_total_hits;
   *hit_rate_percent = cache_total_accesses ? ((100.0f * (float)cache_total_hits) / (float)cache_total_accesses) : 0.0f;
   return true;
#else
   *accesses = *hits = 0;
   *hit_rate_percent = 0.0f;
   return false;
#endif
}
