// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <math.h>
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
static uint8_t storage_pattern[SELF_TEST_STORAGE_BYTES];

#if ENABLE_CACHE_MONITOR
static uint64_t cache_total_accesses, cache_total_hits, cache_total_line_hits;
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
   boot_info.scratch0_raw = scratch0;
   boot_info.scratch1_raw = scratch1;
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

static bool test_storage(uint32_t *bytes_verified)
{
   // Write a known pattern, read it back, and compare
   *bytes_verified = 0;
   for (uint32_t i = 0; i < sizeof(storage_pattern); ++i)
      storage_pattern[i] = (uint8_t)(i * 31u + 7u);
   if (!storage_open(SELF_TEST_STORAGE_FILE_NAME, true))
      return false;
   const bool written = storage_write(storage_pattern, sizeof(storage_pattern));
   storage_close();
   if (!written)
      return false;

   // Read back into the same buffer
   if (!storage_open(SELF_TEST_STORAGE_FILE_NAME, false))
      return false;
   const uint32_t read_bytes = storage_read(storage_pattern, sizeof(storage_pattern));
   storage_close();
   storage_delete(SELF_TEST_STORAGE_FILE_NAME);

   // Compare the read data against the expected pattern
   if (read_bytes != sizeof(storage_pattern))
      return false;
   for (uint32_t i = 0; i < sizeof(storage_pattern); ++i)
      if (storage_pattern[i] != (uint8_t)(i * 31u + 7u))
         return false;

   // Return the result
   *bytes_verified = sizeof(storage_pattern);
   return true;
}

static bool test_imu(float *magnitude_mg)
{
   // A stationary device reads one g in some direction, whatever its orientation
   float x = 0.0f, y = 0.0f, z = 0.0f;
   imu_read_accel_data(&x, &y, &z);
   *magnitude_mg = sqrtf((x * x) + (y * y) + (z * z));
   return (*magnitude_mg >= SELF_TEST_IMU_MIN_MG) && (*magnitude_mg <= SELF_TEST_IMU_MAX_MG);
}

static bool test_power_and_clock(uint32_t rtc_before, uint32_t rtc_after, battery_result_t battery)
{
   // The clock check is free: the microphone window took real time, so the RTC must have advanced across it
   const bool clock_ticking = (rtc_after > rtc_before);
   const bool battery_plausible = (battery.millivolts >= SELF_TEST_BATTERY_MIN_MV) && (battery.millivolts <= SELF_TEST_BATTERY_MAX_MV);
   const bool temperature_plausible = (battery.celcius >= SELF_TEST_TEMPERATURE_MIN_C) && (battery.celcius <= SELF_TEST_TEMPERATURE_MAX_C);
   if (!clock_ticking)
      log_event("SELF_TEST_DETAIL", "check=RTC,result=FAIL,before=%u,after=%u", rtc_before, rtc_after);
   if (!battery_plausible)
      log_event("SELF_TEST_DETAIL", "check=BATTERY,result=FAIL,mv=%u", battery.millivolts);
   if (!temperature_plausible)
      log_event("SELF_TEST_DETAIL", "check=TEMPERATURE,result=FAIL,c=%0.2f", battery.celcius);
   return clock_ticking && battery_plausible && temperature_plausible;
}

static bool test_microphone(uint32_t activation_number, const char *device_label, audio_health_t *health)
{
   // Record a short window, tracking the level on the green LED as it goes
   const uint32_t sample_rate = SELF_TEST_AUDIO_SAMPLE_RATE_HZ, clip_length = SELF_TEST_AUDIO_CLIP_LENGTH_SECONDS;

   // Cap the DMA buffer so the tap indicator is responsive
   audio_set_dma_period_limit((sample_rate * SELF_TEST_LEVEL_UPDATE_MS) / 1000u);
   const bool audio_ready = (config_get_mic_type() == MIC_ANALOG) ?
         audio_analog_init(AUDIO_NUM_CHANNELS, sample_rate, clip_length, config_get_mic_amplification_db(), AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0f, NULL) :
         audio_digital_init(AUDIO_NUM_CHANNELS, sample_rate, clip_length, config_get_mic_amplification_db());
   audio_set_dma_period_limit(0);
   if (!audio_ready)
   {
      print("ERROR: Unable to configure audio for the microphone test\n");
      log_event("SELF_TEST_DETAIL", "check=MICROPHONE,result=FAIL_INIT");
      return false;
   }
   const uint32_t samples_per_buffer = audio_num_samples_per_dma();
   const uint32_t total_samples_wanted = (uint32_t)SELF_TEST_AUDIO_SECONDS * sample_rate;
   const uint32_t hold_buffers = samples_per_buffer ?
         ((((SELF_TEST_LEVEL_HOLD_MS * sample_rate) / 1000u) + samples_per_buffer - 1u) / samples_per_buffer) : 1u;
   audio_health_reset();
   log_event("SELF_TEST_DETAIL", "check=MICROPHONE,phase=CONFIG,requested_hz=%u,actual_hz=%u,samples_per_buffer=%u,buffer_ms=%u",
             sample_rate, audio_get_actual_sample_rate(), samples_per_buffer,
             sample_rate ? ((samples_per_buffer * 1000u) / sample_rate) : 0u);

   // Announce the listening window: three quick green flashes mean "tap the housing now"
   print("INFO: Self test - listening for %u seconds, tap the microphone now\n", (uint32_t)SELF_TEST_AUDIO_SECONDS);
   for (uint32_t flash = 0; flash < 3; ++flash)
   {
      led_on(LED_GREEN);
      system_delay(120000);
      led_off(LED_GREEN);
      system_delay(120000);
   }
   led_off(LED_ALL);

   // Save the clip so the microphone port can be confirmed open by ear
   const bool file_open = storage_open_named_wav_file(SELF_TEST_CLIP_FILE_NAME, AUDIO_NUM_CHANNELS, sample_rate);
   audio_begin_reading();
   int16_t *buffer;
   bool audio_failed = false;
   uint32_t hold_remaining = 0;
   for (uint32_t captured = 0; captured < total_samples_wanted; )
   {
      if (audio_error_encountered())
      {
         print("ERROR: Audio DMA error during the microphone test\n");
         audio_failed = true;
         break;
      }
      if (audio_data_available() && (buffer = audio_read_data_direct()))
      {
         if (file_open)
            storage_write_audio(buffer, sizeof(int16_t) * samples_per_buffer, false);

         // Live level feedback: the LED follows the loudest sample in this buffer
         int32_t peak = 0;
         for (uint32_t i = 0; i < samples_per_buffer; ++i)
         {
            const int32_t magnitude = (buffer[i] < 0) ? -(int32_t)buffer[i] : (int32_t)buffer[i];
            if (magnitude > peak)
               peak = magnitude;
         }

         // Latch the indicator on briefly so a single sharp tap stays visible instead of flashing
         if (peak > SELF_TEST_LIVE_LEVEL_THRESHOLD)
            hold_remaining = hold_buffers;
         if (hold_remaining)
         {
            led_on(LED_GREEN);
            --hold_remaining;
         }
         else
            led_off(LED_GREEN);

         captured += samples_per_buffer;
      }
      else
         system_enter_deep_sleep_mode();
   }
   audio_stop_reading();
   led_off(LED_ALL);

   // Close the window with two slow flashes of both LEDs
   print("INFO: Self test - listening window closed\n");
   for (uint32_t flash = 0; flash < 2; ++flash)
   {
      led_on(LED_ALL);
      system_delay(400000);
      led_off(LED_ALL);
      system_delay(400000);
   }
   if (file_open)
      storage_close_audio();
   audio_deinit();

   *health = audio_health_get();
   if (audio_failed)
   {
      log_event("SELF_TEST_DETAIL", "check=MICROPHONE,result=FAIL_DMA");
      return false;
   }

   // A window this hot is not a recording
   if (health->rms >= SELF_TEST_MIC_MAX_PLAUSIBLE_RMS)
   {
      print("ERROR: Microphone level implausible (rms %u, peak %u) - signal path is not delivering audio\n",
            health->rms, health->peak);
      log_event("SELF_TEST_DETAIL", "check=MICROPHONE,result=FAIL_LEVEL,rms=%u,peak=%u", health->rms, health->peak);
      return false;
   }

   // A constant output means the signal path is dead
   if (health->constant_output)
   {
      log_event("SELF_TEST_DETAIL", "check=MICROPHONE,result=FAIL_CONSTANT,value=%d", (int)health->min_sample);
      return false;
   }
   if (config_get_mic_type() == MIC_ANALOG)
   {
      const int32_t dc = audio_get_dc_offset();
      const int32_t deviation = (dc > MIC_DC_OFFSET_NOMINAL) ? (dc - MIC_DC_OFFSET_NOMINAL) : (MIC_DC_OFFSET_NOMINAL - dc);
      if (deviation > MIC_DC_OFFSET_TOLERANCE)
      {
         log_event("SELF_TEST_DETAIL", "check=MICROPHONE,result=FAIL_DC_OFFSET,dc_offset=%d", (int)dc);
         return false;
      }
   }
   return true;
}

static void indicate_result(self_test_result_t result)
{
   // Communicate the verdict without a computer: solid green for a pass, otherwise one
   // red blink per subsystem number so a failure can be identified at the bench.
   led_off(LED_ALL);
   if (result == SELF_TEST_PASS)
   {
      led_on(LED_GREEN);
      system_delay(SELF_TEST_PASS_INDICATION_US);
      led_off(LED_GREEN);
      return;
   }
   // Each repeat is one long red pulse that marks "a count follows", then one slow blink per subsystem number
   for (uint32_t repeat = 0; repeat < SELF_TEST_FAIL_INDICATION_REPEATS; ++repeat)
   {
      led_on(LED_RED);
      system_delay(SELF_TEST_FAIL_MARKER_US);
      led_off(LED_RED);
      system_delay(SELF_TEST_FAIL_MARKER_GAP_US);
      for (uint32_t blink = 0; blink < (uint32_t)result; ++blink)
      {
         led_on(LED_RED);
         system_delay(SELF_TEST_FAIL_BLINK_ON_US);
         led_off(LED_RED);
         system_delay(SELF_TEST_FAIL_BLINK_OFF_US);
      }
      system_delay(SELF_TEST_FAIL_REPEAT_GAP_US);
   }
}

static void write_results(self_test_result_t result, const audio_health_t *health, uint32_t storage_bytes, float imu_magnitude_mg, battery_result_t battery, uint32_t rtc_timestamp)
{
   // Same KEY = "value" grammar as the configuration and device info files
   char label[1 + MAX_DEVICE_LABEL_LEN] = { 0 };
   config_get_device_label(label, sizeof(label));
   if (!storage_open(SELF_TEST_RESULTS_FILE_NAME, true))
      return;

   char line[128];
   #define WRITE_RESULT(fmt, ...) \
      do { \
         const int len = snprintf(line, sizeof(line), fmt "\n", ##__VA_ARGS__); \
         if (len > 0) storage_write(line, (uint32_t)len); \
      } while (0)

   WRITE_RESULT("FW_VERSION = \"%s\"", _FW_VERSION);
   WRITE_RESULT("DEVICE_LABEL = \"%s\"", label);
   WRITE_RESULT("TEST_TIMESTAMP = \"%u\"", (unsigned)rtc_timestamp);
   WRITE_RESULT("ACTIVATION_NUMBER = \"%u\"", (unsigned)config_get_activation_number());
   WRITE_RESULT("OVERALL = \"%s\"", (result == SELF_TEST_PASS) ? "PASS" : "FAIL");
   WRITE_RESULT("FAILED_SUBSYSTEM = \"%d\"", (int)result);
   WRITE_RESULT("MIC_TYPE = \"%s\"", (config_get_mic_type() == MIC_ANALOG) ? "ANALOG" : "DIGITAL");
   WRITE_RESULT("MIC_RESULT = \"%s\"", (result == SELF_TEST_FAIL_MICROPHONE) ? "FAIL" : (health->silent ? "PASS_SILENT" : "PASS"));
   WRITE_RESULT("MIC_RMS = \"%u\"", (unsigned)health->rms);
   WRITE_RESULT("MIC_PEAK = \"%u\"", (unsigned)health->peak);
   WRITE_RESULT("MIC_MIN = \"%d\"", (int)health->min_sample);
   WRITE_RESULT("MIC_MAX = \"%d\"", (int)health->max_sample);
   WRITE_RESULT("MIC_MEAN = \"%d\"", (int)health->mean);
   WRITE_RESULT("MIC_SAMPLES = \"%u\"", (unsigned)health->num_samples);
   WRITE_RESULT("MIC_DC_OFFSET = \"%d\"", (int)audio_get_dc_offset());
   WRITE_RESULT("MIC_CONSTANT_OUTPUT = \"%s\"", health->constant_output ? "True" : "False");
   WRITE_RESULT("SD_RESULT = \"%s\"", storage_bytes ? "PASS" : "FAIL");
   WRITE_RESULT("SD_BYTES_VERIFIED = \"%u\"", (unsigned)storage_bytes);
   WRITE_RESULT("SD_FREE_MB = \"%u\"", (unsigned)storage_get_free_space_mb());
   WRITE_RESULT("IMU_RESULT = \"%s\"", (result == SELF_TEST_FAIL_IMU) ? "FAIL" : "PASS");
   WRITE_RESULT("IMU_MAGNITUDE_MG = \"%d\"", (int)imu_magnitude_mg);
   WRITE_RESULT("BATTERY_MV = \"%u\"", (unsigned)battery.millivolts);
   WRITE_RESULT("TEMPERATURE_C = \"%0.2f\"", battery.celcius);
   WRITE_RESULT("RTC_VALID = \"%s\"", rtc_is_valid() ? "True" : "False");
   #undef WRITE_RESULT

   storage_close();
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
   am_hal_sysctrl_bus_write_flush();
   system_deinitialize_peripherals();
   am_hal_reset_control(AM_HAL_RESET_CONTROL_SWPOR, NULL);

   // Writing the SWPOR key only *requests* the reset; the core keeps executing until the reset generator asserts
   am_hal_sysctrl_bus_write_flush();
   while (true) {}
}

self_test_result_t system_run_self_test(void)
{
   log_event("SELF_TEST_START", "fw=%s", _FW_VERSION);

   // Force the LEDs on for the duration regardless of configuration
   const bool leds_were_enabled = leds_are_enabled();
   leds_enable(true);
   char device_label[1 + MAX_DEVICE_LABEL_LEN] = { 0 };
   config_get_device_label(device_label, sizeof(device_label));
   const uint32_t activation_number = config_get_activation_number();
   const uint32_t rtc_before = rtc_get_timestamp();

   // Storage first: everything after this requires somewhere to record results
   print("INFO: Self test - checking storage\n");
   uint32_t storage_bytes = 0;
   const bool storage_ok = test_storage(&storage_bytes);
   log_event("SELF_TEST_DETAIL", "check=STORAGE,result=%s,bytes=%u",  storage_ok ? "PASS" : "FAIL", storage_bytes);
   print("INFO: Self test - checking accelerometer\n");
   float imu_magnitude_mg = 0.0f;
   const bool imu_ok = test_imu(&imu_magnitude_mg);
   log_event("SELF_TEST_DETAIL", "check=IMU,result=%s,magnitude_mg=%d", imu_ok ? "PASS" : "FAIL", (int)imu_magnitude_mg);
   audio_health_t health = { 0 };
   const bool microphone_ok = test_microphone(activation_number, device_label, &health);
   log_event("SELF_TEST_DETAIL", "check=MICROPHONE,result=%s,rms=%u,peak=%u,silent=%s", microphone_ok ? "PASS" : "FAIL", health.rms, health.peak, health.silent ? "True" : "False");
   print("INFO: Self test - checking power and clock\n");
   const battery_result_t battery = battery_monitor_get_details();
   const uint32_t rtc_after = rtc_get_timestamp();
   const bool power_ok = test_power_and_clock(rtc_before, rtc_after, battery);
   log_event("SELF_TEST_DETAIL", "check=POWER,result=%s,mv=%u,temp_c=%0.2f", power_ok ? "PASS" : "FAIL", battery.millivolts, battery.celcius);

   // Report the first failure, in the order a failure most likely matters
   self_test_result_t result = SELF_TEST_PASS;
   if (!microphone_ok)
      result = SELF_TEST_FAIL_MICROPHONE;
   else if (!storage_ok)
      result = SELF_TEST_FAIL_STORAGE;
   else if (!imu_ok)
      result = SELF_TEST_FAIL_IMU;
   else if (!power_ok)
      result = SELF_TEST_FAIL_POWER_OR_CLOCK;
   write_results(result, &health, storage_bytes, imu_magnitude_mg, battery, rtc_after);
   log_event("SELF_TEST_END", "result=%s,failed_subsystem=%d", (result == SELF_TEST_PASS) ? "PASS" : "FAIL", (int)result);
   storage_flush_log();
   static const char *const subsystem_names[] = { "none", "MICROPHONE", "STORAGE", "IMU", "POWER/CLOCK" };
   if (result == SELF_TEST_PASS)
      print("INFO: Self test PASSED - solid green for 3 seconds\n");
   else
      print("ERROR: Self test FAILED on subsystem %d (%s) - long red pulse then %d red blink(s), repeated %d times\n",
            (int)result, subsystem_names[(uint32_t)result < 5 ? (uint32_t)result : 0],
            (int)result, (int)SELF_TEST_FAIL_INDICATION_REPEATS);
   indicate_result(result);
   leds_enable(leds_were_enabled);
   return result;
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
#ifndef AM_DEBUG_PRINTF
   am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_CRYPTO_POWERDOWN, NULL);
#endif
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

   // Reinitialize peripherals upon waking from Deep Sleep mode, if requested
   if (reinit_on_wakeup)
   {
      logging_init();
      mram_init();
      leds_init();
      imu_init();
      if (config_gps_available())
         tracker_init();
      magnet_sensor_init();
      battery_monitor_init();
      storage_init();
      storage_setup_logs();
      system_enable_interrupts(true);
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
   uint32_t accesses = 0, hits = 0, line_hits = 0;
   AM_CRITICAL_BEGIN
   accesses = CPU->IMON0;
   hits = CPU->IMON2;
   line_hits = CPU->IMON3;
   CPU->CACHECTRL = CPU_CACHECTRL_RESETSTAT_Msk;
   am_hal_sysctrl_sysbus_write_flush();
   AM_CRITICAL_END
   cache_total_accesses += accesses;
   cache_total_hits += hits;
   cache_total_line_hits += line_hits;
#endif
}

bool system_get_cache_stats(uint64_t *accesses, uint64_t *hits, float *hit_rate_percent)
{
#if ENABLE_CACHE_MONITOR
   system_accumulate_cache_stats();
   const uint64_t served = cache_total_hits + cache_total_line_hits;
   *accesses = cache_total_accesses;
   *hits = served;
   *hit_rate_percent = cache_total_accesses ? ((100.0f * (float)served) / (float)cache_total_accesses) : 0.0f;
   return true;
#else
   *accesses = *hits = 0;
   *hit_rate_percent = 0.0f;
   return false;
#endif
}
