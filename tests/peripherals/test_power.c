// Power characterization firmware.
//
// Walks the device through each power state and event the forecaster charges for, one step at a time, and marks
// each step's measurement window on the external I2C test points so a PPK2 capture can be cut into steps afterward.
// A step sets its state up, holds it long enough to settle, opens its window with a marker frame carrying its code,
// holds the state for the window, closes the window, and tears the state down. Its results line is appended to the
// SD card only after teardown, so no card traffic ever lands inside a window that did not ask for it.
//
// A state step is measured as a mean current. An event step repeats one event at a fixed rate on top of a named
// baseline state step, so the charge per event is the step's excess over that baseline divided by its event count.
// A cycle step alternates a state on and off, and is charged against an on state and an off state in proportion to
// the time it spent in each. Build with 'make power': it strips the debug console and uses deployment optimization,
// because both change what is being measured.
//
// The brownout suite is different: it runs the heaviest realistic load forever, marking every completed cycle, while
// the capture tool lowers the supply until the cycles stop. That finds the lowest voltage the board can work at, which
// is where the battery is really used up.
//
// The suite and run details come from _power.cfg in the root of the card, one KEY = VALUE per line:
//    SUITE = core                 core, sd, mic, analog, dsp, imu, battery, or brownout
//    WINDOW_SECONDS = 60          State window; event and work steps hold for half of it
//    START_DELAY_SECONDS = 0      Time to sit at the floor first, such as for a thermal soak
//    REPEAT = 1                   Times to run the whole suite
//    LABEL, CARD, MIC, BATTERY, TEMPERATURE = free text copied into the results

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "audio.h"
#include "audio_filter.h"
#include "battery.h"
#include "ff.h"
#include "imu.h"
#include "led.h"
#include "logging.h"
#include "magnet.h"
#include "mram.h"
#include "opus_config.h"
#include "power_marker.h"
#include "rtc.h"
#include "silence.h"
#include "storage.h"
#include "system.h"
#include "tracker.h"
#include "vhf.h"

#ifndef POWER_MARKERS
#error "Build the power characterization firmware with 'make power', which enables the markers it relies on"
#endif


// Test Configuration --------------------------------------------------------------------------------------------------

#define POWER_CONFIG_FILE_NAME           "_power.cfg"
#define POWER_RESULTS_FILE_NAME          "_power.results"
#define POWER_SCRATCH_FILE_NAME          "_power_scratch.bin"
#define POWER_LOG_FILE_NAME              "_power_log.txt"
#define POWER_FILES_DIRECTORY            "_power_files"
#define POWER_RESULTS_FORMAT_VERSION     1
#define SUITE_DONE_CODE                  0x00
#define HEARTBEAT_OK_CODE                0xE0
#define HEARTBEAT_FAULT_CODE             0xE1

#define DEFAULT_WINDOW_SECONDS           60
#define MIN_WINDOW_SECONDS               10
#define MAX_WINDOW_SECONDS               3600
#define SETTLE_SECONDS                   3
#define MIC_CYCLE_ON_SECONDS             4
#define MIC_CYCLE_PERIOD_SECONDS         6
#define FILE_CYCLE_BYTES                 512
#define CPU_WORK_ITERATIONS              20000
#define MOTION_THRESHOLD_MG              200.0f
#define ANALOG_ARMED_THRESHOLD           0.9f
#define SILENCE_UNREACHABLE_THRESHOLD    1.0e30f

// Timers 0, 2, 4, 5, and 6 belong to the deployment firmware's peripherals
#define TICK_TIMER_NUMBER                07
#define CLOCK_TIMER_NUMBER               03
#define tick_timer_isr                   am_timer_isr1(TICK_TIMER_NUMBER)

// The deployment firmware's own buffers leave only a little of the retained SRAM free, and powering more would change
// what is being measured. Writes larger than this repeat it, which FatFs splits at the same 32 KB cluster boundaries
// a single large write would be split at; one second of audio is processed in chunks of it.
#define SCRATCH_BUFFER_BYTES             (32 * 1024)
#define SCRATCH_CHUNK_SAMPLES            16000


// Step Definitions ----------------------------------------------------------------------------------------------------

typedef enum { SUITE_CORE, SUITE_SD, SUITE_MIC, SUITE_ANALOG, SUITE_DSP, SUITE_IMU, SUITE_BATTERY, SUITE_BROWNOUT, NUM_SUITES } suite_t;
static const char *const suite_names[NUM_SUITES] = { "core", "sd", "mic", "analog", "dsp", "imu", "battery", "brownout" };

#define STEP_EVENT       0x01u   // Repeats an event on top of its baseline state
#define STEP_BUSY        0x02u   // Never sleeps: the service routine runs back to back for the whole window
#define STEP_NEEDS_CARD  0x04u   // The card is mounted before setup and stays mounted through the window
#define STEP_POWER_OFF   0x08u   // The window is one power-off sleep, opened by the firmware's own power-off marker
#define STEP_CYCLE       0x10u   // Alternates on and off, charged against an on state and the floor
#define STEP_MAGNET_WAKE 0x20u   // Power-off with the magnet sensor sampling for a wake
#define STEP_TRACK_VBAT  0x40u   // Records the lowest battery reading seen while the window was open
#define STEP_ENDLESS     0x80u   // Never ends: runs until the supply can no longer sustain it

typedef struct
{
   uint8_t code, baseline;
   suite_t suite;
   uint32_t flags, tick_hz, param;
   const char *name;
   void (*setup)(uint32_t param);
   uint32_t (*service)(uint32_t param);   // Runs after every wake; returns the number of events it completed
   void (*teardown)(uint32_t param);
} power_step_t;

typedef struct
{
   uint32_t window_ms, events, awake_ms, deep_sleeps, card_wakes, sd_writes, sd_kb, mic_starts, audio_buffers;
   uint32_t on_ms, vbat_rest_mv, vbat_loaded_mv, vbat_min_mv, hal_failures, samples_per_dma;
   int32_t temperature_centi_c;
   bool setup_ok;
} step_result_t;


// Static Global Variables ---------------------------------------------------------------------------------------------

__attribute__((section(".shared"), aligned(32)))
static union { uint8_t bytes[SCRATCH_BUFFER_BYTES]; int16_t samples[SCRATCH_BUFFER_BYTES / sizeof(int16_t)]; } scratch;

static struct
{
   char suite[16], label[48], card[48], mic[48], battery[48], temperature[24];
   uint32_t window_seconds, start_delay_seconds, repeat;
} settings;

static volatile bool tick_pending;
static volatile uint32_t imu_samples, magnet_changes;
static volatile bool device_activated = true;
static bool card_mounted, adc_ready, setup_succeeded, cycle_on;
static uint32_t cycle_ticks, cycle_on_since, cycle_on_ticks, vbat_min_mv, file_cycle_index;
static FIL log_file;

static bool append_results(const char *fmt, ...);


// Interrupt Service Routines ------------------------------------------------------------------------------------------

void am_rtc_isr(void)
{
   // The alarm only needs to wake the processor; window ends are decided by polling the deadline
   am_hal_rtc_interrupt_clear(AM_HAL_RTC_INT_ALM);
}

void tick_timer_isr(void)
{
   am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(TICK_TIMER_NUMBER, AM_HAL_TIMER_COMPARE_BOTH));
   am_hal_timer_clear(TICK_TIMER_NUMBER);
   tick_pending = true;
}

static void magnet_changed(bool field_present)
{
   (void)field_present;
   ++magnet_changes;
}

static void motion_changed(bool in_motion)
{
   (void)in_motion;
}

static void imu_sample_ready(float accel_x_mg, float accel_y_mg, float accel_z_mg)
{
   (void)accel_x_mg; (void)accel_y_mg; (void)accel_z_mg;
   ++imu_samples;
}


// Shared Helpers ------------------------------------------------------------------------------------------------------

static bool take_tick(void)
{
   // Consume one pending periodic wake, if there is one
   if (!tick_pending)
      return false;
   tick_pending = false;
   return true;
}

static void tick_timer_start(uint32_t hz)
{
   // A periodic wake on the 32 kHz crystal, the same source the deployment firmware's own timers use
   am_hal_timer_config_t config;
   am_hal_timer_default_config_set(&config);
   config.eInputClock = AM_HAL_TIMER_CLOCK_XT;
   config.eFunction = AM_HAL_TIMER_FN_UPCOUNT;
   config.ui32Compare0 = 32768u / hz;
   am_hal_timer_config(TICK_TIMER_NUMBER, &config);
   am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(TICK_TIMER_NUMBER, AM_HAL_TIMER_COMPARE_BOTH));
   am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(TICK_TIMER_NUMBER, AM_HAL_TIMER_COMPARE0));
   NVIC_SetPriority(TIMER0_IRQn + TICK_TIMER_NUMBER, AM_IRQ_PRIORITY_DEFAULT);
   NVIC_EnableIRQ(TIMER0_IRQn + TICK_TIMER_NUMBER);
   tick_pending = false;
   am_hal_timer_clear(TICK_TIMER_NUMBER);
}

static void tick_timer_stop(void)
{
   am_hal_timer_clear_stop(TICK_TIMER_NUMBER);
   am_hal_timer_interrupt_disable(AM_HAL_TIMER_MASK(TICK_TIMER_NUMBER, AM_HAL_TIMER_COMPARE0));
   NVIC_DisableIRQ(TIMER0_IRQn + TICK_TIMER_NUMBER);
   tick_pending = false;
}

static void card_mount(bool mounted)
{
   // Power the card up and mount it, or unmount it and cut its power entirely
   if (mounted && !card_mounted)
      storage_init();
   else if (!mounted && card_mounted)
      storage_deinit();
   card_mounted = mounted;
}

static battery_result_t read_battery(void)
{
   // Leave the battery monitor exactly as it was found
   const bool temporary = !adc_ready;
   if (temporary)
      battery_monitor_init();
   const battery_result_t result = battery_monitor_get_details();
   if (temporary)
      battery_monitor_deinit();
   return result;
}

static void note_vbat(void)
{
   const battery_result_t result = battery_monitor_get_details();
   if (result.valid && (!vbat_min_mv || (result.millivolts < vbat_min_mv)))
      vbat_min_mv = result.millivolts;
}

static void restore_sram_retention(void)
{
   // Power-off releases shared SRAM retention for good; the remaining steps need it back
   am_hal_pwrctrl_sram_memcfg_t sram_mem_config;
   am_hal_pwrctrl_sram_config_get(&sram_mem_config);
   sram_mem_config.eSRAMRetain = AM_HAL_PWRCTRL_SRAM_1M_GRP0;
   sram_mem_config.eActiveWithMCU = AM_HAL_PWRCTRL_SRAM_NONE;
   am_hal_pwrctrl_sram_config(&sram_mem_config);
}

static void fill_test_audio(void)
{
   // Broadband noise over a tone, so that neither the filters nor the encoder see a degenerate input
   uint32_t state = 0x12345678u;
   for (uint32_t i = 0; i < SCRATCH_CHUNK_SAMPLES; ++i)
   {
      state = (state * 1664525u) + 1013904223u;
      const int32_t noise = (int32_t)(state >> 20) - 2048;
      const int32_t tone = ((i % 32u) < 16u) ? 3000 : -3000;
      scratch.samples[i] = (int16_t)(noise + tone);
   }
}

static void establish_base_state(void)
{
   // The floor every step starts from and returns to: the sensors a deployment always configures are configured and
   // asleep, the HFRC trim is running as it is in the deployment firmware, and the card is unpowered
   rtc_init();
   am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFADJ_ENABLE, NULL);
   vhf_init();
   magnet_sensor_init();
   imu_init();
   imu_deinit();
   system_enable_interrupts(true);
}


// Step Bodies: Core ---------------------------------------------------------------------------------------------------

static uint32_t service_tick(uint32_t param)
{
   (void)param;
   return take_tick() ? 1 : 0;
}

static void setup_sram_active(uint32_t param)    { (void)param; system_set_sram_active(true); }
static void teardown_sram_active(uint32_t param) { (void)param; system_set_sram_active(false); }

static void setup_no_hfadj(uint32_t param)    { (void)param; am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFADJ_DISABLE, NULL); }
static void teardown_no_hfadj(uint32_t param) { (void)param; am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFADJ_ENABLE, NULL); }

static void setup_clock_timer(uint32_t clock)
{
   // A free-running timer with no interrupt, to isolate what its clock source costs in deep sleep
   am_hal_timer_config_t config;
   am_hal_timer_default_config_set(&config);
   config.eInputClock = (am_hal_timer_clock_e)clock;
   config.eFunction = AM_HAL_TIMER_FN_PWM;
   config.ui32Compare0 = 0xFFFF;
   config.ui32Compare1 = 0x7FFF;
   am_hal_timer_config(CLOCK_TIMER_NUMBER, &config);
   am_hal_timer_clear(CLOCK_TIMER_NUMBER);
}

static void teardown_clock_timer(uint32_t param)
{
   (void)param;
   am_hal_timer_clear_stop(CLOCK_TIMER_NUMBER);
}

static void setup_watchdog(uint32_t param)    { (void)param; system_enable_watchdog(); }
static void teardown_watchdog(uint32_t param) { (void)param; system_disable_watchdog(); }

static void setup_magnet_armed(uint32_t param)    { (void)param; magnet_sensor_register_callback(magnet_changed); }
static void teardown_magnet_armed(uint32_t param) { (void)param; magnet_sensor_deinit(); magnet_sensor_init(); }

static void setup_tracker(uint32_t param)    { (void)param; tracker_init(); }
static void teardown_tracker(uint32_t param) { (void)param; tracker_deinit(); }

static void setup_imu_idle(uint32_t param)    { (void)param; imu_init(); }
static void teardown_imu_idle(uint32_t param) { (void)param; imu_deinit(); }

static void setup_adc(uint32_t param)    { (void)param; battery_monitor_init(); adc_ready = true; }
static void teardown_adc(uint32_t param) { (void)param; battery_monitor_deinit(); adc_ready = false; }

static void setup_led(uint32_t color)
{
   leds_init();
   leds_enable(true);
   led_on((led_color_t)color);
}

static void teardown_led(uint32_t param)
{
   (void)param;
   led_off(LED_ALL);
   leds_deinit();
}

static void setup_vhf_on(uint32_t param)    { (void)param; vhf_activate(); }
static void teardown_vhf_on(uint32_t param) { (void)param; vhf_deactivate(); }

static void setup_mic_pins_driven(uint32_t param)
{
   // Drive both microphone power switches off rather than leaving whichever path is unused at its reset state
   (void)param;
   const am_hal_gpio_pincfg_t output_config = AM_HAL_GPIO_PINCFG_OUTPUT;
   am_hal_gpio_output_clear(PIN_MICROPHONE_ENABLE);
   am_hal_gpio_output_clear(PIN_DIGITAL_MIC_PWR);
   am_hal_gpio_pinconfig(PIN_MICROPHONE_ENABLE, output_config);
   am_hal_gpio_pinconfig(PIN_DIGITAL_MIC_PWR, output_config);
}

static void teardown_mic_pins_driven(uint32_t param)
{
   (void)param;
   am_hal_gpio_pinconfig(PIN_MICROPHONE_ENABLE, am_hal_gpio_pincfg_default);
   am_hal_gpio_pinconfig(PIN_DIGITAL_MIC_PWR, am_hal_gpio_pincfg_default);
}

static void setup_app_idle(uint32_t param)
{
   // What a deployment holds between clips: every peripheral initialized, the card powered, the magnet armed
   (void)param;
   system_initialize_peripherals();
   magnet_sensor_register_callback(magnet_changed);
   card_mounted = adc_ready = true;
}

static void teardown_app_idle(uint32_t param)
{
   (void)param;
   tracker_deinit();
   imu_deinit();
   battery_monitor_deinit();
   leds_deinit();
   magnet_sensor_deinit();
   adc_ready = false;
   establish_base_state();
}

static volatile float cpu_work_sink;

static uint32_t service_cpu_work(uint32_t param)
{
   // Integer and floating point work that stays in registers and the cache, as the processing paths mostly do
   (void)param;
   uint32_t state = 1;
   float accumulator = 0.0f;
   for (uint32_t i = 0; i < CPU_WORK_ITERATIONS; ++i)
   {
      state = (state * 1103515245u) + 12345u;
      accumulator += (float)(state >> 16) * 0.000015f;
   }
   cpu_work_sink = accumulator;
   if (vbat_min_mv && take_tick())
      note_vbat();
   return 1;
}

static uint32_t service_battery_read(uint32_t param)
{
   (void)param;
   if (!take_tick())
      return 0;
   battery_monitor_get_details();
   return 1;
}


// Step Bodies: SD Card ------------------------------------------------------------------------------------------------

static void setup_scratch_file(uint32_t param)
{
   (void)param;
   setup_succeeded = storage_open(POWER_SCRATCH_FILE_NAME, true);
}

static void teardown_scratch_file(uint32_t param)
{
   (void)param;
   storage_close();
   storage_delete(POWER_SCRATCH_FILE_NAME);
}

static uint32_t service_sd_session(uint32_t bytes)
{
   // One wake of the card that writes the requested amount, as a deployment's staged flushes do
   if (!take_tick())
      return 0;
   storage_sd_session_begin();
   for (uint32_t written = 0; written < bytes; )
   {
      const uint32_t chunk = MIN(bytes - written, (uint32_t)SCRATCH_BUFFER_BYTES);
      storage_write(scratch.bytes, chunk);
      written += chunk;
      if (vbat_min_mv)
         note_vbat();
   }
   storage_sd_session_end();
   return 1;
}

static void setup_file_cycle(uint32_t param)
{
   (void)param;
   storage_mkdir(POWER_FILES_DIRECTORY);
   file_cycle_index = 0;
}

static void teardown_file_cycle(uint32_t param)
{
   (void)param;
   char path[48];
   for (uint32_t i = 0; i < file_cycle_index; ++i)
   {
      snprintf(path, sizeof(path), "%s/f%05u.bin", POWER_FILES_DIRECTORY, (unsigned)i);
      storage_delete(path);
   }
   storage_delete(POWER_FILES_DIRECTORY);
}

static uint32_t service_file_cycle(uint32_t param)
{
   // Create, write, and finalize a small file: the overhead each clip pays beyond its audio
   (void)param;
   if (!take_tick())
      return 0;
   char path[48];
   snprintf(path, sizeof(path), "%s/f%05u.bin", POWER_FILES_DIRECTORY, (unsigned)file_cycle_index++);
   if (storage_open(path, true))
   {
      storage_write(scratch.bytes, FILE_CYCLE_BYTES);
      storage_close();
   }
   return 1;
}

static void setup_log_file(uint32_t param)
{
   (void)param;
   setup_succeeded = (f_open(&log_file, POWER_LOG_FILE_NAME, FA_OPEN_APPEND | FA_WRITE) == FR_OK);
}

static void teardown_log_file(uint32_t param)
{
   (void)param;
   f_close(&log_file);
   f_unlink(POWER_LOG_FILE_NAME);
}

static uint32_t service_log_line(uint32_t param)
{
   // An event line followed by a sync of the log inside one session, which is what every logged event costs
   (void)param;
   if (!take_tick())
      return 0;
   f_printf(&log_file, "[%u] EVT|POWER_TEST|t=%u,batt_mv=3600,temp_c=21.00,n=%u\n",
            (unsigned)rtc_get_timestamp(), (unsigned)rtc_get_timestamp(), (unsigned)cycle_ticks++);
   storage_sd_session_begin();
   f_sync(&log_file);
   storage_sd_session_end();
   return 1;
}

static uint32_t service_card_mount(uint32_t param)
{
   // A full power-up and mount followed by a power-down, as each boot and each wake from power-off pays
   (void)param;
   if (!take_tick())
      return 0;
   storage_init();
   storage_deinit();
   return 1;
}


// Step Bodies: Audio --------------------------------------------------------------------------------------------------

static uint32_t consume_audio(void)
{
   // Hand every captured buffer back so the DMA never overruns; storage costs are measured separately
   uint32_t buffers = 0;
   while (audio_data_available() && audio_read_data_direct())
      ++buffers;
   return buffers;
}

static void setup_digital_capture(uint32_t sample_rate_hz)
{
   audio_filter_initialize(FILTER_NONE, sample_rate_hz, 0, 0);
   setup_succeeded = audio_digital_init(AUDIO_NUM_CHANNELS, sample_rate_hz, 60, config_get_mic_amplification_db());
   if (setup_succeeded)
      audio_begin_reading();
}

static void setup_analog_capture(uint32_t sample_rate_hz)
{
   audio_filter_initialize(FILTER_NONE, sample_rate_hz, 0, 0);
   setup_succeeded = audio_analog_init(AUDIO_NUM_CHANNELS, sample_rate_hz, 60, config_get_mic_amplification_db(),
                                       AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0f, &device_activated);
   if (setup_succeeded)
      audio_begin_reading();
}

static void setup_analog_armed(uint32_t sample_rate_hz)
{
   // Amplitude-triggered recording waiting for a sound: microphone and comparator live, converter idle
   audio_filter_initialize(FILTER_NONE, sample_rate_hz, 0, 0);
   setup_succeeded = audio_analog_init(AUDIO_NUM_CHANNELS, sample_rate_hz, 60, config_get_mic_amplification_db(),
                                       AUDIO_MIC_BIAS_VOLTAGE, COMPARATOR_THRESHOLD, ANALOG_ARMED_THRESHOLD, &device_activated);
   if (setup_succeeded)
      audio_begin_reading();
}

static void teardown_capture(uint32_t param)
{
   (void)param;
   if (setup_succeeded)
      audio_stop_reading();
   audio_deinit();
   system_set_sram_active(false);
}

static uint32_t service_capture(uint32_t param)
{
   (void)param;
   consume_audio();
   return 0;
}

static uint32_t service_analog_armed(uint32_t param)
{
   // A trigger means a sound got through; re-arm and count it, since every trigger spoils the armed measurement
   (void)param;
   if (!audio_data_available())
      return 0;
   consume_audio();
   audio_stop_reading();
   audio_begin_reading();
   return 1;
}

static void setup_digital_cycle(uint32_t sample_rate_hz)
{
   audio_filter_initialize(FILTER_NONE, sample_rate_hz, 0, 0);
   setup_succeeded = audio_digital_init(AUDIO_NUM_CHANNELS, sample_rate_hz, 60, config_get_mic_amplification_db());
}

static void setup_analog_cycle(uint32_t sample_rate_hz)
{
   audio_filter_initialize(FILTER_NONE, sample_rate_hz, 0, 0);
   setup_succeeded = audio_analog_init(AUDIO_NUM_CHANNELS, sample_rate_hz, 60, config_get_mic_amplification_db(),
                                       AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0f, &device_activated);
}

static void teardown_cycle(uint32_t param)
{
   (void)param;
   if (cycle_on)
      audio_stop_reading();
   cycle_on = false;
   audio_deinit();
   system_set_sram_active(false);
}

static uint32_t service_cycle(uint32_t param)
{
   // Start the microphone at the top of each period and stop it partway through, as a scheduled recording does
   (void)param;
   consume_audio();
   if (!take_tick())
      return 0;
   const uint32_t phase = cycle_ticks++ % MIC_CYCLE_PERIOD_SECONDS;
   if ((phase == 0) && !cycle_on && setup_succeeded)
   {
      cycle_on_since = system_get_activity_ticks();
      audio_begin_reading();
      cycle_on = true;
      return 1;
   }
   if ((phase == MIC_CYCLE_ON_SECONDS) && cycle_on)
   {
      audio_stop_reading();
      cycle_on_ticks += system_get_activity_ticks() - cycle_on_since;
      cycle_on = false;
   }
   return 0;
}


// Step Bodies: Processing ---------------------------------------------------------------------------------------------

static void setup_biquad(uint32_t encoded)
{
   // The low byte selects the filter type and the rest is the sample rate in hertz
   const uint32_t sample_rate_hz = encoded >> 8;
   fill_test_audio();
   audio_filter_initialize((audio_filter_type_t)(encoded & 0xFF), sample_rate_hz, 500, sample_rate_hz / 4);
}

static void teardown_biquad(uint32_t encoded)
{
   audio_filter_initialize(FILTER_NONE, encoded >> 8, 0, 0);
}

static uint32_t service_biquad(uint32_t encoded)
{
   // One second of audio per event
   for (uint32_t done = 0, total = encoded >> 8; done < total; done += SCRATCH_CHUNK_SAMPLES)
      audio_filter_apply(scratch.samples, MIN(SCRATCH_CHUNK_SAMPLES, total - done));
   return 1;
}

static uint32_t silence_fft_length(uint32_t sample_rate_hz)
{
   // Mirrors the filter's own choice, which keeps each FFT bin between 7.81 and 11.72 Hz wide
   return (sample_rate_hz <= 8000) ? 1024 : ((sample_rate_hz <= 24000) ? 2048 : 4096);
}

static void setup_silence(uint32_t sample_rate_hz)
{
   // The filter stops at the first frame with sound in it, so an unreachable threshold makes every frame count as
   // silent and forces the full analysis, which is both the quiet-site case and the most it can ever cost
   fill_test_audio();
   setup_succeeded = silence_filter_initialize(sample_rate_hz, 500, sample_rate_hz / 4, SILENCE_UNREACHABLE_THRESHOLD);
}

static uint32_t service_silence(uint32_t sample_rate_hz)
{
   // One FFT frame per event: how many frames a buffer holds depends on the DMA size, which the forecaster knows
   silence_filter_is_silence(scratch.samples, silence_fft_length(sample_rate_hz));
   return 1;
}

static void setup_opus(uint32_t bitrate)
{
   fill_test_audio();
   opusenc_init((int32_t)bitrate);
}

static uint32_t service_opus(uint32_t bitrate)
{
   // One second of 48 kHz audio per event, the only rate the encoder accepts
   (void)bitrate;
   const opus_frame_t *begin, *end;
   for (uint32_t done = 0; done < OPUS_REQUIRED_SAMPLE_RATE_HZ; done += SCRATCH_CHUNK_SAMPLES)
      opusenc_encode(scratch.samples, MIN(SCRATCH_CHUNK_SAMPLES, OPUS_REQUIRED_SAMPLE_RATE_HZ - done), &begin, &end);
   return 1;
}


// Step Bodies: IMU ----------------------------------------------------------------------------------------------------

static void setup_imu_raw(uint32_t rate_hz)
{
   imu_init();
   imu_samples = 0;
   imu_enable_raw_data_output(true, LIS2DU12_2g, rate_hz, LIS2DU12_ODR_div_2, imu_sample_ready);
}

static void teardown_imu_raw(uint32_t rate_hz)
{
   imu_enable_raw_data_output(false, LIS2DU12_2g, rate_hz, LIS2DU12_ODR_div_2, NULL);
   imu_deinit();
}

static void setup_imu_motion(uint32_t param)
{
   (void)param;
   imu_init();
   imu_enable_motion_change_detection(true, MOTION_THRESHOLD_MG, motion_changed);
}

static void teardown_imu_motion(uint32_t param)
{
   (void)param;
   imu_enable_motion_change_detection(false, 0.0f, NULL);
   imu_deinit();
}


// Step Bodies: Battery ------------------------------------------------------------------------------------------------

static void setup_battery_cpu_leds(uint32_t param)
{
   // The heaviest sustained load the board can present without the card: processor flat out and both LEDs lit
   (void)param;
   battery_monitor_init();
   adc_ready = true;
   leds_init();
   leds_enable(true);
   led_on(LED_ALL);
}

static void teardown_battery_cpu_leds(uint32_t param)
{
   (void)param;
   led_off(LED_ALL);
   leds_deinit();
   battery_monitor_deinit();
   adc_ready = false;
}

static void setup_battery_sd(uint32_t param)
{
   battery_monitor_init();
   adc_ready = true;
   setup_scratch_file(param);
}

static void teardown_battery_sd(uint32_t param)
{
   teardown_scratch_file(param);
   battery_monitor_deinit();
   adc_ready = false;
}

static void setup_battery_capture(uint32_t sample_rate_hz)
{
   battery_monitor_init();
   adc_ready = true;
   setup_digital_capture(sample_rate_hz);
}

static void teardown_battery_capture(uint32_t param)
{
   teardown_capture(param);
   battery_monitor_deinit();
   adc_ready = false;
}

static uint32_t service_battery_capture(uint32_t param)
{
   (void)param;
   consume_audio();
   if (take_tick())
      note_vbat();
   return 0;
}


// Step Bodies: Brownout -----------------------------------------------------------------------------------------------

static void setup_brownout(uint32_t sample_rate_hz)
{
   // Capture at the highest rate with the card mounted and a file open, as the heaviest deployment phase does
   battery_monitor_init();
   adc_ready = true;
   setup_scratch_file(0);
   setup_digital_capture(sample_rate_hz);
}

static uint32_t service_brownout(uint32_t param)
{
   // One cycle per second: a full staging flush, a battery reading, and a results line, then a heartbeat saying whether
   // every write landed. The heartbeat is what the sweep watches; the results line says what the ADC thought.
   (void)param;
   consume_audio();
   if (!take_tick())
      return 0;
   bool written = true;
   storage_sd_session_begin();
   for (uint32_t done = 0; done < (512u * 1024u); done += SCRATCH_BUFFER_BYTES)
      written = storage_write(scratch.bytes, SCRATCH_BUFFER_BYTES) && written;
   storage_sd_session_end();
   const battery_result_t battery = battery_monitor_get_details();
   storage_close();
   written = append_results("BROWNOUT|cycle=%u,vbat_mv=%u,written=%u,t=%u\n", (unsigned)cycle_ticks++,
                            (unsigned)battery.millivolts, written ? 1u : 0u, (unsigned)rtc_get_timestamp()) && written;
   written = storage_open(POWER_SCRATCH_FILE_NAME, true) && written;
   power_marker_frame(written ? HEARTBEAT_OK_CODE : HEARTBEAT_FAULT_CODE);
   return 1;
}


// Step Table ----------------------------------------------------------------------------------------------------------

// Codes are permanent: the measurement catalogue and every capture ever taken refer to steps by them.
// A step's baseline is the code of the state it is charged against; zero means it is a state itself.
#define BIQUAD(type, rate)   (((rate) << 8) | (type))
#define S(code, name, suite, flags, hz, param, base, setup, service, teardown) \
   { code, base, suite, flags, hz, param, name, setup, service, teardown }

static const power_step_t steps[] =
{
   // Core: the floor and every always-on contributor, each measured as an increment over the floor
   S(0x01, "SLEEP_FLOOR",          SUITE_CORE, 0, 0, 0, 0, NULL, NULL, NULL),
   S(0x02, "SLEEP_NO_HFADJ",       SUITE_CORE, 0, 0, 0, 0, setup_no_hfadj, NULL, teardown_no_hfadj),
   S(0x03, "SLEEP_SRAM_ACTIVE",    SUITE_CORE, 0, 0, 0, 0, setup_sram_active, NULL, teardown_sram_active),
   S(0x04, "SLEEP_TIMER_XT",       SUITE_CORE, 0, 0, AM_HAL_TIMER_CLOCK_XT, 0, setup_clock_timer, NULL, teardown_clock_timer),
   S(0x05, "SLEEP_TIMER_HFRC",     SUITE_CORE, 0, 0, AM_HAL_TIMER_CLOCK_HFRC_DIV16, 0, setup_clock_timer, NULL, teardown_clock_timer),
   S(0x06, "SLEEP_WATCHDOG",       SUITE_CORE, 0, 0, 0, 0, setup_watchdog, NULL, teardown_watchdog),
   S(0x07, "SLEEP_TICK_1HZ",       SUITE_CORE, 0, 1, 0, 0, setup_adc, service_tick, teardown_adc),
   S(0x08, "SLEEP_MAGNET_ARMED",   SUITE_CORE, 0, 0, 0, 0, setup_magnet_armed, NULL, teardown_magnet_armed),
   S(0x09, "SLEEP_TRACKER_SLAVE",  SUITE_CORE, 0, 0, 0, 0, setup_tracker, NULL, teardown_tracker),
   S(0x0A, "SLEEP_IMU_IDLE",       SUITE_CORE, 0, 0, 0, 0, setup_imu_idle, NULL, teardown_imu_idle),
   S(0x0B, "SLEEP_ADC_IDLE",       SUITE_CORE, 0, 0, 0, 0, setup_adc, NULL, teardown_adc),
   S(0x0C, "SLEEP_MIC_PINS_DRIVEN",SUITE_CORE, 0, 0, 0, 0, setup_mic_pins_driven, NULL, teardown_mic_pins_driven),
   S(0x0D, "SLEEP_LED_RED",        SUITE_CORE, 0, 0, LED_RED, 0, setup_led, NULL, teardown_led),
   S(0x0E, "SLEEP_LED_GREEN",      SUITE_CORE, 0, 0, LED_GREEN, 0, setup_led, NULL, teardown_led),
   S(0x0F, "SLEEP_VHF_ENABLED",    SUITE_CORE, 0, 0, 0, 0, setup_vhf_on, NULL, teardown_vhf_on),
   S(0x10, "SLEEP_APP_IDLE",       SUITE_CORE, 0, 0, 0, 0, setup_app_idle, NULL, teardown_app_idle),
   S(0x11, "CPU_ACTIVE",           SUITE_CORE, STEP_BUSY, 0, 0, 0, NULL, service_cpu_work, NULL),
   S(0x18, "EVT_WAKE",             SUITE_CORE, STEP_EVENT, 64, 0, 0x04, NULL, service_tick, NULL),
   S(0x19, "EVT_BATTERY_READ",     SUITE_CORE, STEP_EVENT, 1, 0, 0x07, setup_adc, service_battery_read, teardown_adc),
   S(0x12, "OFF_RTC",              SUITE_CORE, STEP_POWER_OFF, 0, 0, 0, NULL, NULL, NULL),
   S(0x13, "OFF_MAGNET",           SUITE_CORE, STEP_POWER_OFF | STEP_MAGNET_WAKE, 0, 0, 0, NULL, NULL, NULL),

   // SD card: run once per card model, since all of these belong to the card rather than the board
   S(0x20, "SD_IDLE",              SUITE_SD, STEP_NEEDS_CARD, 0, 0, 0, NULL, NULL, NULL),
   S(0x21, "SD_IDLE_TICK_1HZ",     SUITE_SD, STEP_NEEDS_CARD, 1, 0, 0, setup_scratch_file, service_tick, teardown_scratch_file),
   S(0x22, "EVT_SD_SESSION_4K",    SUITE_SD, STEP_NEEDS_CARD | STEP_EVENT, 1, 4 * 1024, 0x21, setup_scratch_file, service_sd_session, teardown_scratch_file),
   S(0x23, "EVT_SD_SESSION_32K",   SUITE_SD, STEP_NEEDS_CARD | STEP_EVENT, 1, 32 * 1024, 0x21, setup_scratch_file, service_sd_session, teardown_scratch_file),
   S(0x24, "EVT_SD_SESSION_128K",  SUITE_SD, STEP_NEEDS_CARD | STEP_EVENT, 1, 128 * 1024, 0x21, setup_scratch_file, service_sd_session, teardown_scratch_file),
   S(0x25, "EVT_SD_SESSION_512K",  SUITE_SD, STEP_NEEDS_CARD | STEP_EVENT, 1, 512 * 1024, 0x21, setup_scratch_file, service_sd_session, teardown_scratch_file),
   S(0x26, "EVT_SD_FILE_CYCLE",    SUITE_SD, STEP_NEEDS_CARD | STEP_EVENT, 1, 0, 0x21, setup_file_cycle, service_file_cycle, teardown_file_cycle),
   S(0x27, "EVT_SD_LOG_LINE",      SUITE_SD, STEP_NEEDS_CARD | STEP_EVENT, 1, 0, 0x21, setup_log_file, service_log_line, teardown_log_file),
   S(0x28, "EVT_SD_MOUNT",         SUITE_SD, STEP_EVENT, 1, 0, 0x07, setup_adc, service_card_mount, teardown_adc),

   // Digital microphone: run once per microphone part
   S(0x40, "MIC_CAPTURE_8K",       SUITE_MIC, 0, 0, 8000, 0, setup_digital_capture, service_capture, teardown_capture),
   S(0x41, "MIC_CAPTURE_16K",      SUITE_MIC, 0, 0, 16000, 0, setup_digital_capture, service_capture, teardown_capture),
   S(0x42, "MIC_CAPTURE_24K",      SUITE_MIC, 0, 0, 24000, 0, setup_digital_capture, service_capture, teardown_capture),
   S(0x43, "MIC_CAPTURE_32K",      SUITE_MIC, 0, 0, 32000, 0, setup_digital_capture, service_capture, teardown_capture),
   S(0x44, "MIC_CAPTURE_48K",      SUITE_MIC, 0, 0, 48000, 0, setup_digital_capture, service_capture, teardown_capture),
   S(0x48, "EVT_MIC_CYCLE_16K",    SUITE_MIC, STEP_CYCLE | STEP_EVENT, 1, 16000, 0x41, setup_digital_cycle, service_cycle, teardown_cycle),
   S(0x49, "EVT_MIC_CYCLE_48K",    SUITE_MIC, STEP_CYCLE | STEP_EVENT, 1, 48000, 0x44, setup_digital_cycle, service_cycle, teardown_cycle),

   // Analog microphone path
   S(0x50, "ANALOG_CAPTURE_8K",    SUITE_ANALOG, 0, 0, 8000, 0, setup_analog_capture, service_capture, teardown_capture),
   S(0x51, "ANALOG_CAPTURE_16K",   SUITE_ANALOG, 0, 0, 16000, 0, setup_analog_capture, service_capture, teardown_capture),
   S(0x52, "ANALOG_CAPTURE_24K",   SUITE_ANALOG, 0, 0, 24000, 0, setup_analog_capture, service_capture, teardown_capture),
   S(0x53, "ANALOG_CAPTURE_32K",   SUITE_ANALOG, 0, 0, 32000, 0, setup_analog_capture, service_capture, teardown_capture),
   S(0x54, "ANALOG_CAPTURE_48K",   SUITE_ANALOG, 0, 0, 48000, 0, setup_analog_capture, service_capture, teardown_capture),
   S(0x58, "ANALOG_ARMED",         SUITE_ANALOG, 0, 0, 16000, 0, setup_analog_armed, service_analog_armed, teardown_capture),
   S(0x59, "EVT_ANALOG_CYCLE_16K", SUITE_ANALOG, STEP_CYCLE | STEP_EVENT, 1, 16000, 0x51, setup_analog_cycle, service_cycle, teardown_cycle),

   // Processing: each event is one second of audio (one FFT frame for the silence filter), charged against the floor
   // the processor would otherwise have slept at
   S(0x60, "EVT_LOWPASS_16K",      SUITE_DSP, STEP_BUSY | STEP_EVENT, 0, BIQUAD(FILTER_LOW_PASS, 16000), 0x01, setup_biquad, service_biquad, teardown_biquad),
   S(0x61, "EVT_HIGHPASS_16K",     SUITE_DSP, STEP_BUSY | STEP_EVENT, 0, BIQUAD(FILTER_HIGH_PASS, 16000), 0x01, setup_biquad, service_biquad, teardown_biquad),
   S(0x62, "EVT_BANDPASS_16K",     SUITE_DSP, STEP_BUSY | STEP_EVENT, 0, BIQUAD(FILTER_BAND_PASS, 16000), 0x01, setup_biquad, service_biquad, teardown_biquad),
   S(0x63, "EVT_LOWPASS_48K",      SUITE_DSP, STEP_BUSY | STEP_EVENT, 0, BIQUAD(FILTER_LOW_PASS, 48000), 0x01, setup_biquad, service_biquad, teardown_biquad),
   S(0x64, "EVT_HIGHPASS_48K",     SUITE_DSP, STEP_BUSY | STEP_EVENT, 0, BIQUAD(FILTER_HIGH_PASS, 48000), 0x01, setup_biquad, service_biquad, teardown_biquad),
   S(0x65, "EVT_BANDPASS_48K",     SUITE_DSP, STEP_BUSY | STEP_EVENT, 0, BIQUAD(FILTER_BAND_PASS, 48000), 0x01, setup_biquad, service_biquad, teardown_biquad),
   S(0x68, "EVT_SILENCE_FRAME_16K", SUITE_DSP, STEP_BUSY | STEP_EVENT, 0, 16000, 0x01, setup_silence, service_silence, NULL),
   S(0x69, "EVT_SILENCE_FRAME_48K", SUITE_DSP, STEP_BUSY | STEP_EVENT, 0, 48000, 0x01, setup_silence, service_silence, NULL),
   S(0x70, "EVT_OPUS_16KBPS",      SUITE_DSP, STEP_BUSY | STEP_EVENT, 0, 16000, 0x01, setup_opus, service_opus, NULL),
   S(0x71, "EVT_OPUS_24KBPS",      SUITE_DSP, STEP_BUSY | STEP_EVENT, 0, 24000, 0x01, setup_opus, service_opus, NULL),
   S(0x72, "EVT_OPUS_32KBPS",      SUITE_DSP, STEP_BUSY | STEP_EVENT, 0, 32000, 0x01, setup_opus, service_opus, NULL),
   S(0x73, "EVT_OPUS_64KBPS",      SUITE_DSP, STEP_BUSY | STEP_EVENT, 0, 64000, 0x01, setup_opus, service_opus, NULL),
   S(0x74, "EVT_OPUS_96KBPS",      SUITE_DSP, STEP_BUSY | STEP_EVENT, 0, 96000, 0x01, setup_opus, service_opus, NULL),
   S(0x75, "EVT_OPUS_128KBPS",     SUITE_DSP, STEP_BUSY | STEP_EVENT, 0, 128000, 0x01, setup_opus, service_opus, NULL),

   // IMU: every rate the configuration accepts, then motion detection waiting for movement
   S(0x80, "IMU_RAW_3HZ",          SUITE_IMU, 0, 0, 3, 0, setup_imu_raw, NULL, teardown_imu_raw),
   S(0x81, "IMU_RAW_6HZ",          SUITE_IMU, 0, 0, 6, 0, setup_imu_raw, NULL, teardown_imu_raw),
   S(0x82, "IMU_RAW_25HZ",         SUITE_IMU, 0, 0, 25, 0, setup_imu_raw, NULL, teardown_imu_raw),
   S(0x83, "IMU_RAW_50HZ",         SUITE_IMU, 0, 0, 50, 0, setup_imu_raw, NULL, teardown_imu_raw),
   S(0x84, "IMU_RAW_100HZ",        SUITE_IMU, 0, 0, 100, 0, setup_imu_raw, NULL, teardown_imu_raw),
   S(0x85, "IMU_RAW_200HZ",        SUITE_IMU, 0, 0, 200, 0, setup_imu_raw, NULL, teardown_imu_raw),
   S(0x86, "IMU_RAW_400HZ",        SUITE_IMU, 0, 0, 400, 0, setup_imu_raw, NULL, teardown_imu_raw),
   S(0x87, "IMU_RAW_800HZ",        SUITE_IMU, 0, 0, 800, 0, setup_imu_raw, NULL, teardown_imu_raw),
   S(0x88, "IMU_MOTION_ARMED",     SUITE_IMU, 0, 0, 0, 0, setup_imu_motion, NULL, teardown_imu_motion),

   // Battery: on a real cell, the voltage it holds under each kind of load, which is what decides the cutoff
   S(0x90, "BATT_FLOOR",           SUITE_BATTERY, 0, 0, 0, 0, NULL, NULL, NULL),
   S(0x91, "BATT_APP_IDLE",        SUITE_BATTERY, 0, 0, 0, 0, setup_app_idle, NULL, teardown_app_idle),
   S(0x92, "BATT_MIC_48K",         SUITE_BATTERY, STEP_TRACK_VBAT, 1, 48000, 0, setup_battery_capture, service_battery_capture, teardown_battery_capture),
   S(0x93, "BATT_SD_512K",         SUITE_BATTERY, STEP_NEEDS_CARD | STEP_TRACK_VBAT | STEP_EVENT, 1, 512 * 1024, 0x21, setup_battery_sd, service_sd_session, teardown_battery_sd),
   S(0x94, "BATT_CPU_LEDS",        SUITE_BATTERY, STEP_TRACK_VBAT | STEP_BUSY, 1, 0, 0, setup_battery_cpu_leds, service_cpu_work, teardown_battery_cpu_leds),

   // Brownout: on the PPK2 source meter while the capture tool lowers the supply; never returns
   S(0xA0, "BROWNOUT_HEAVY",       SUITE_BROWNOUT, STEP_ENDLESS | STEP_NEEDS_CARD, 1, 48000, 0, setup_brownout, service_brownout, NULL),
};


// Configuration and Results -------------------------------------------------------------------------------------------

static void copy_value(char *destination, uint32_t size, const char *value)
{
   strncpy(destination, value, size - 1);
   destination[size - 1] = '\0';
}

static void read_settings(void)
{
   // Defaults first, so that a missing or partial file still runs the core suite
   memset(&settings, 0, sizeof(settings));
   copy_value(settings.suite, sizeof(settings.suite), "core");
   settings.window_seconds = DEFAULT_WINDOW_SECONDS;
   settings.repeat = 1;
   if (!storage_open(POWER_CONFIG_FILE_NAME, false))
      return;

   static char line[128];
   int32_t length;
   while ((length = storage_read_line(line, sizeof(line))) >= 0)
   {
      line[MIN((uint32_t)length, sizeof(line) - 1)] = '\0';
      char *equals = strchr(line, '=');
      if ((line[0] == '#') || !equals)
         continue;
      *equals = '\0';
      char *key = line, *value = equals + 1;
      while (*key == ' ') ++key;
      while (*value == ' ') ++value;
      for (char *end = key + strlen(key); (end > key) && (end[-1] == ' '); --end) end[-1] = '\0';
      for (char *end = value + strlen(value); (end > value) && ((end[-1] == ' ') || (end[-1] == '"')); --end) end[-1] = '\0';
      if (*value == '"') ++value;

      if (!strcmp(key, "SUITE"))                     copy_value(settings.suite, sizeof(settings.suite), value);
      else if (!strcmp(key, "WINDOW_SECONDS"))       settings.window_seconds = (uint32_t)strtoul(value, NULL, 10);
      else if (!strcmp(key, "START_DELAY_SECONDS"))  settings.start_delay_seconds = (uint32_t)strtoul(value, NULL, 10);
      else if (!strcmp(key, "REPEAT"))               settings.repeat = (uint32_t)strtoul(value, NULL, 10);
      else if (!strcmp(key, "LABEL"))                copy_value(settings.label, sizeof(settings.label), value);
      else if (!strcmp(key, "CARD"))                 copy_value(settings.card, sizeof(settings.card), value);
      else if (!strcmp(key, "MIC"))                  copy_value(settings.mic, sizeof(settings.mic), value);
      else if (!strcmp(key, "BATTERY"))              copy_value(settings.battery, sizeof(settings.battery), value);
      else if (!strcmp(key, "TEMPERATURE"))          copy_value(settings.temperature, sizeof(settings.temperature), value);
   }
   storage_close();

   settings.window_seconds = MAX(MIN_WINDOW_SECONDS, MIN(MAX_WINDOW_SECONDS, settings.window_seconds));
   settings.repeat = MAX(1u, settings.repeat);
}

static bool append_results(const char *fmt, ...)
{
   // Opened and closed per line, so that a run cut short keeps everything it finished
   FIL results;
   if (f_open(&results, POWER_RESULTS_FILE_NAME, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
      return false;
   va_list args;
   va_start(args, fmt);
   f_vprintf(&results, fmt, args);
   va_end(args);
   return f_close(&results) == FR_OK;
}


// Step Runner ---------------------------------------------------------------------------------------------------------

static bool hold(const power_step_t *step, uint32_t seconds, uint32_t *events)
{
   // Keep the step's state until the RTC deadline passes, servicing it after every wake
   rtc_set_wakeup_timestamp(rtc_get_timestamp() + seconds);
   while ((step->flags & STEP_ENDLESS) || !rtc_wakeup_elapsed())
   {
      if (!(step->flags & STEP_BUSY))
         system_enter_deep_sleep_mode();
      if (step->service)
      {
         const uint32_t completed = step->service(step->param);
         if (events)
            *events += completed;
      }
   }
   rtc_clear_wakeup();
   return true;
}

static void run_power_off(const power_step_t *step, uint32_t seconds)
{
   // The step's frame names the window, and the firmware's own power-off marker opens it once teardown is done
   const uint32_t wake_pin = (step->flags & STEP_MAGNET_WAKE) ? PIN_MAG_SENSOR_INP : 0;
   power_marker_frame(step->code);
   system_enter_power_off_mode(wake_pin, rtc_get_timestamp() + seconds, false);
   power_marker_end();

   // Nothing comes back on its own after a power-off without reinitialization
   restore_sram_retention();
   mram_init();
   logging_init();
   if (wake_pin)
      magnet_sensor_deinit();
   establish_base_state();
}

static void run_step(const power_step_t *step, uint32_t run_index)
{
   // Every step starts from the floor with the card in the state it asks for
   step_result_t result = { 0 };
   card_mount(step->flags & STEP_NEEDS_CARD);
   const battery_result_t rest = read_battery();
   result.vbat_rest_mv = rest.millivolts;
   result.temperature_centi_c = (int32_t)(rest.celcius * 100.0f);
   const bool event_step = (step->flags & STEP_EVENT) != 0;
   const uint32_t window_seconds = (event_step || (step->flags & STEP_BUSY)) ? MAX(MIN_WINDOW_SECONDS, settings.window_seconds / 2) : settings.window_seconds;
   const uint32_t hal_failures_before = system_get_hal_failure_count();

   // Set up and settle
   setup_succeeded = true;
   cycle_on = false;
   cycle_ticks = cycle_on_ticks = vbat_min_mv = 0;
   if (step->setup)
      step->setup(step->param);
   result.setup_ok = setup_succeeded;
   if (step->tick_hz)
      tick_timer_start(step->tick_hz);
   if (!(step->flags & (STEP_POWER_OFF | STEP_CYCLE | STEP_ENDLESS)))
      hold(step, SETTLE_SECONDS, NULL);
   result.vbat_loaded_mv = read_battery().millivolts;
   if (step->flags & STEP_TRACK_VBAT)
      vbat_min_mv = result.vbat_loaded_mv ? result.vbat_loaded_mv : 1;

   // Snapshot every counter, then hold the state for the window
   storage_activity_t storage_before, storage_after;
   audio_stats_t audio_before, audio_after;
   uint64_t awake_before = 0, awake_after = 0;
   uint32_t sleeps_before = 0, sleeps_after = 0;
   storage_get_activity(&storage_before);
   audio_get_stats(&audio_before);
   system_get_activity_stats(&awake_before, &sleeps_before);
   const uint32_t window_start = system_get_activity_ticks();
   if (step->flags & STEP_POWER_OFF)
      run_power_off(step, window_seconds);
   else
   {
      power_marker_frame(step->code);
      hold(step, window_seconds, &result.events);
      power_marker_end();
   }
   const uint32_t window_ticks = system_get_activity_ticks() - window_start;
   if (cycle_on)
      cycle_on_ticks += system_get_activity_ticks() - cycle_on_since;
   system_get_activity_stats(&awake_after, &sleeps_after);
   audio_get_stats(&audio_after);
   storage_get_activity(&storage_after);

   // Collect what the window did before tearing anything down
   result.window_ms = (uint32_t)(((uint64_t)window_ticks * 1000u) / SYSTEM_ACTIVITY_TICKS_PER_SECOND);
   result.on_ms = (uint32_t)(((uint64_t)cycle_on_ticks * 1000u) / SYSTEM_ACTIVITY_TICKS_PER_SECOND);
   result.awake_ms = (uint32_t)(awake_after - awake_before);
   result.deep_sleeps = sleeps_after - sleeps_before;
   result.card_wakes = storage_after.card_wakes - storage_before.card_wakes;
   result.sd_writes = storage_after.write_transactions - storage_before.write_transactions;
   result.sd_kb = (storage_after.sectors_written - storage_before.sectors_written) / 2u;
   result.mic_starts = audio_after.mic_starts - audio_before.mic_starts;
   result.audio_buffers = audio_after.buffers_captured - audio_before.buffers_captured;
   result.samples_per_dma = (step->suite == SUITE_MIC) || (step->suite == SUITE_ANALOG) ? audio_num_samples_per_dma() : 0;
   result.vbat_min_mv = vbat_min_mv;
   if (step->tick_hz)
      tick_timer_stop();
   if (step->teardown)
      step->teardown(step->param);
   result.hal_failures = system_get_hal_failure_count() - hal_failures_before;

   // Record the step with the card powered, which is now outside every window
   const uint8_t off_baseline = (step->flags & STEP_CYCLE) ? 0x01 : 0;
   card_mount(true);
   append_results("STEP|run=%u,code=0x%02X,name=%s,suite=%s,kind=%s,baseline=0x%02X,off_baseline=0x%02X,"
                  "setup=%s,window_ms=%u,events=%u,on_ms=%u,awake_ms=%u,sleeps=%u,card_wakes=%u,sd_writes=%u,"
                  "sd_kb=%u,mic_starts=%u,audio_buffers=%u,samples_per_dma=%u,imu_samples=%u,magnet_changes=%u,"
                  "vbat_rest_mv=%u,vbat_loaded_mv=%u,vbat_min_mv=%u,temp_c=%s%u.%02u,hal_failures=%u,t=%u\n",
                  (unsigned)run_index, step->code, step->name, suite_names[step->suite],
                  (step->flags & STEP_CYCLE) ? "cycle" : (event_step ? "event" : "state"),
                  step->baseline, off_baseline, result.setup_ok ? "OK" : "FAIL",
                  (unsigned)result.window_ms, (unsigned)result.events, (unsigned)result.on_ms,
                  (unsigned)result.awake_ms, (unsigned)result.deep_sleeps, (unsigned)result.card_wakes,
                  (unsigned)result.sd_writes, (unsigned)result.sd_kb, (unsigned)result.mic_starts,
                  (unsigned)result.audio_buffers, (unsigned)result.samples_per_dma, (unsigned)imu_samples,
                  (unsigned)magnet_changes, (unsigned)result.vbat_rest_mv, (unsigned)result.vbat_loaded_mv,
                  (unsigned)result.vbat_min_mv, (result.temperature_centi_c < 0) ? "-" : "",
                  (unsigned)(abs(result.temperature_centi_c) / 100), (unsigned)(abs(result.temperature_centi_c) % 100),
                  (unsigned)result.hal_failures, (unsigned)rtc_get_timestamp());
   imu_samples = magnet_changes = 0;
}


// Main Function -------------------------------------------------------------------------------------------------------

int main(void)
{
   // Bring the board up to the deployment firmware's floor, then read what to run
   setup_hardware();
   establish_base_state();
   if (!rtc_is_valid())
      rtc_set_time_to_compile_time();
   card_mount(true);
   read_settings();
   suite_t suite = NUM_SUITES;
   for (uint32_t i = 0; i < NUM_SUITES; ++i)
      if (!strcmp(settings.suite, suite_names[i]))
         suite = (suite_t)i;
   uint32_t num_steps = 0;
   for (uint32_t i = 0; i < (sizeof(steps) / sizeof(steps[0])); ++i)
      num_steps += (steps[i].suite == suite) ? 1 : 0;
   append_results("RUN|format=%u,fw=%s,suite=%s,steps=%u,window_s=%u,repeat=%u,start_delay_s=%u,label=%s,card=%s,"
                  "card_mb=%u,card_au=%u,mic=%s,battery=%s,temperature=%s,t=%u\n",
                  POWER_RESULTS_FORMAT_VERSION, _FW_VERSION, settings.suite, (unsigned)num_steps,
                  (unsigned)settings.window_seconds, (unsigned)settings.repeat, (unsigned)settings.start_delay_seconds,
                  settings.label, settings.card, (unsigned)storage_get_total_space_mb(),
                  (unsigned)storage_get_allocation_unit_bytes(), settings.mic, settings.battery, settings.temperature,
                  (unsigned)rtc_get_timestamp());

   // An unknown suite is reported on the LEDs rather than silently running something else
   if (!num_steps)
   {
      append_results("ERROR|reason=unknown_suite,suite=%s\n", settings.suite);
      card_mount(false);
      leds_init();
      leds_enable(true);
      while (true)
      {
         led_toggle(LED_RED);
         system_delay(250000);
      }
   }

   // Sit at the floor for any requested soak, then run the suite
   if (settings.start_delay_seconds)
   {
      card_mount(false);
      const power_step_t soak = { 0 };
      hold(&soak, settings.start_delay_seconds, NULL);
   }
   for (uint32_t run = 1; run <= settings.repeat; ++run)
      for (uint32_t i = 0; i < (sizeof(steps) / sizeof(steps[0])); ++i)
         if (steps[i].suite == suite)
            run_step(&steps[i], run);
   append_results("DONE|t=%u\n", (unsigned)rtc_get_timestamp());

   // Finish at the floor with the card unpowered, so the end of the capture is unambiguous
   card_mount(false);
   power_marker_frame(SUITE_DONE_CODE);
   while (true)
      system_enter_deep_sleep_mode();
   return 0;
}
