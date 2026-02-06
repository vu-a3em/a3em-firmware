// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <math.h>
#include "audio.h"
#include "comparator.h"
#include "led.h"
#include "logging.h"
#include "system.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

#define PREAMP_FULL_GAIN           80
#define MICBIAS_VOLTAGE_MIN        0.9f
#define MICBIAS_VOLTAGE_MAX        1.5f

#define audio_pdm_isr       am_pdm_isr1(PDM_MODULE)
#define am_pdm_isr1(n)      am_pdm_isr(n)
#define am_pdm_isr(n)       am_pdm ## n ## _isr

__attribute__((section(".shared"), aligned(16)))
uint32_t sample_buffer[2*AUDIO_BUFFER_MAX_SAMPLES];

static void *audio_handle;
static float pga_gain_db;
static bool is_digital_mic;
static volatile uint32_t skip_samples;
static audio_trigger_t trigger_criterion;
static uint32_t num_audio_channels, sampling_rate_hz, dc_offset, num_samples_per_dma;
static volatile bool dma_complete = false, dma_error = false, adc_awake;
static am_hal_audadc_dma_config_t audadc_dma_config =
{
   .bDynamicPriority = false,
   .ePriority = AM_HAL_AUDADC_PRIOR_SERVICE_IMMED,
   .bDMAEnable = true,
   .ui32SampleCount = 0,
   .ui32TargetAddress = 0x0,
   .ui32TargetAddressReverse = 0x0
};
static am_hal_audadc_irtt_config_t audadc_irtt_config =
{
   .bIrttEnable        = true,
   .eClkDiv            = AM_HAL_AUDADC_RPTT_CLK_DIV8,
   .ui32IrttCountMax   = 0
};
static am_hal_pdm_transfer_t pdm_transfer_config = { 0 };
static am_hal_pdm_config_t pdm_config =
{
    .ePDMClkSpeed = AM_HAL_PDM_CLK_HFRC2ADJ_24_576MHZ,
    .eClkDivider = AM_HAL_PDM_MCLKDIV_1,
    .ePDMAClkOutDivder = AM_HAL_PDM_PDMA_CLKO_DIV3,
    .eStepSize = AM_HAL_PDM_GAIN_STEP_0_13DB,
    .bHighPassEnable = AM_HAL_PDM_HIGH_PASS_ENABLE,
    .ui32HighPassCutoff = 0x3,
    .bDataPacking = false,
    .bPDMSampleDelay = AM_HAL_PDM_CLKOUT_PHSDLY_NONE,
    .ui32GainChangeDelay = AM_HAL_PDM_CLKOUT_DELAY_NONE,
    .bSoftMute = false,
    .bLRSwap = false,
};


// Private Helper Functions --------------------------------------------------------------------------------------------

void audio_adc_start(void)
{
   if (is_digital_mic)
   {
      // Wake up the PDM peripheral
      if (!adc_awake)
         am_hal_pdm_power_control(audio_handle, AM_HAL_PDM_POWER_ON, true);
      skip_samples = (sampling_rate_hz <= AUDIO_BUFFER_MAX_SAMPLES) ? 1 : (sampling_rate_hz / AUDIO_BUFFER_MAX_SAMPLES);
      configASSERT0(am_hal_pdm_configure(audio_handle, &pdm_config));
      configASSERT0(am_hal_pdm_fifo_threshold_setup(audio_handle, 24));
      configASSERT0(am_hal_pdm_interrupt_enable(audio_handle, (AM_HAL_PDM_INT_DERR | AM_HAL_PDM_INT_DCMP | AM_HAL_PDM_INT_UNDFL | AM_HAL_PDM_INT_OVF)));
      configASSERT0(am_hal_pdm_enable(audio_handle));
      configASSERT0(am_hal_pdm_dma_start(audio_handle, &pdm_transfer_config));
      adc_awake = true;
   }
   else
   {
      // Wake up the AUDADC peripheral
      if (!adc_awake)
         configASSERT0(am_hal_audadc_power_control(audio_handle, AM_HAL_SYSCTRL_WAKE, true));
      AUDADCn(0)->DATAOFFSET = 0x0000UL;
      adc_awake = true;

      // Configure the AUDADC trigger timer
      configASSERT0(am_hal_audadc_configure_irtt(audio_handle, &audadc_irtt_config));
      configASSERT0(am_hal_audadc_irtt_enable(audio_handle));

      // Configure the DMA operation and interrupts to wake up the MCU on every conversion
      configASSERT0(am_hal_audadc_configure_dma(audio_handle, &audadc_dma_config));
      configASSERT0(am_hal_audadc_interrupt_enable(audio_handle, AM_HAL_AUDADC_INT_DERR | AM_HAL_AUDADC_INT_FIFOOVR1));

      // Set the desired gain configuration
      am_hal_audadc_gain_config_t audadc_gain_config =
      {
         .ui32LGA = (uint32_t)((pga_gain_db*2.0f + 10) * 0.8),
         .ui32HGADELTA = (uint32_t)((pga_gain_db*2.0f + 10) * 0.2),
         .ui32LGB = 0,
         .ui32HGBDELTA = 0,
         .eUpdateMode = AM_HAL_AUDADC_GAIN_UPDATE_IMME
      };
      configASSERT0(am_hal_audadc_internal_pga_config(audio_handle, &audadc_gain_config));
      configASSERT0(am_hal_audadc_sw_trigger(audio_handle));
   }
}

void am_audadc0_isr(void)
{
   // Clear the AUDADC interrupt
   static uint32_t status;
   am_hal_audadc_interrupt_status(audio_handle, &status, false);
   am_hal_audadc_interrupt_clear(audio_handle, status);

   // Handle a DMA completion or error event
   if ((status & AM_HAL_AUDADC_INT_FIFOOVR1) && AUDADCn(0)->DMASTAT_b.DMACPL)
   {
      am_hal_audadc_interrupt_service(audio_handle, &audadc_dma_config);
      dma_complete = true;
      dma_error = false;
   }
   if (status & AM_HAL_AUDADC_INT_DERR)
      dma_error = true;
}

void audio_pdm_isr(void)
{
   // Reset and service the interrupt
   static uint32_t status;
   am_hal_pdm_interrupt_status_get(audio_handle, &status, true);
   am_hal_pdm_interrupt_clear(audio_handle, status);
   am_hal_pdm_interrupt_service(audio_handle, status, &pdm_transfer_config);

   // Handle a DMA completion or error event
   if (status & AM_HAL_PDM_INT_DCMP)
   {
      if (skip_samples)
      {
         am_hal_pdm_fifo_flush(audio_handle);
         --skip_samples;
      }
      else
         dma_complete = true;
   }
   else
   {
      if (status & AM_HAL_PDM_INT_OVF)
         am_hal_pdm_fifo_flush(audio_handle);
      dma_error = status & AM_HAL_PDM_INT_DERR;
   }
}


// Public API Functions ------------------------------------------------------------------------------------------------

void audio_digital_init(uint32_t num_channels, uint32_t sample_rate_hz, uint32_t clip_length_seconds, float gain_db)
{
   // Turn on the external microphone
   am_hal_gpio_pincfg_t mic_en_config = AM_HAL_GPIO_PINCFG_OUTPUT;
   mic_en_config.GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X;
   configASSERT0(am_hal_gpio_pinconfig(PIN_DIGITAL_MIC_PWR, mic_en_config));
   am_hal_gpio_output_set(PIN_DIGITAL_MIC_PWR);

   // Initialize the PDM peripheral
   adc_awake = true;
   skip_samples = 0;
   is_digital_mic = true;
   num_audio_channels = num_channels;
   sampling_rate_hz = sample_rate_hz;
   configASSERT0(am_hal_pdm_initialize(PDM_MODULE, &audio_handle));
   configASSERT0(am_hal_pdm_power_control(audio_handle, AM_HAL_PDM_POWER_ON, false));

   // Determine the longest possible DMA period
   uint32_t max_seconds_per_dma = AUDIO_BUFFER_MAX_SAMPLES / sample_rate_hz;
   if (max_seconds_per_dma > clip_length_seconds)
      num_samples_per_dma = clip_length_seconds * sample_rate_hz;
   else if ((clip_length_seconds % max_seconds_per_dma) == 0)
      num_samples_per_dma = max_seconds_per_dma * sample_rate_hz;
   else
   {
      do { --max_seconds_per_dma; } while (clip_length_seconds % max_seconds_per_dma);
      num_samples_per_dma = max_seconds_per_dma * sample_rate_hz;
   }

   // Set up the DMA configuration structure
   pdm_transfer_config.ui32TotalCount = num_samples_per_dma * sizeof(uint32_t);
   pdm_transfer_config.ui32TargetAddr = (uint32_t)sample_buffer;
   pdm_transfer_config.ui32TargetAddrReverse = pdm_transfer_config.ui32TargetAddr + pdm_transfer_config.ui32TotalCount;

   // Determine the correct gain definition
   am_hal_pdm_gain_e gain_settings;
   if (gain_db < 0.75f)
      gain_settings = AM_HAL_PDM_GAIN_0DB;
   else if (gain_db < 2.25f)
      gain_settings = AM_HAL_PDM_GAIN_P15DB;
   else if (gain_db < 3.75f)
      gain_settings = AM_HAL_PDM_GAIN_P30DB;
   else if (gain_db < 5.25f)
      gain_settings = AM_HAL_PDM_GAIN_P45DB;
   else if (gain_db < 6.75f)
      gain_settings = AM_HAL_PDM_GAIN_P60DB;
   else if (gain_db < 8.25f)
      gain_settings = AM_HAL_PDM_GAIN_P75DB;
   else if (gain_db < 9.75f)
      gain_settings = AM_HAL_PDM_GAIN_P90DB;
   else if (gain_db < 11.25f)
      gain_settings = AM_HAL_PDM_GAIN_P105DB;
   else if (gain_db < 12.75f)
      gain_settings = AM_HAL_PDM_GAIN_P120DB;
   else if (gain_db < 14.25f)
      gain_settings = AM_HAL_PDM_GAIN_P135DB;
   else if (gain_db < 15.75f)
      gain_settings = AM_HAL_PDM_GAIN_P150DB;
   else if (gain_db < 17.25f)
      gain_settings = AM_HAL_PDM_GAIN_P165DB;
   else if (gain_db < 18.75f)
      gain_settings = AM_HAL_PDM_GAIN_P180DB;
   else if (gain_db < 20.25f)
      gain_settings = AM_HAL_PDM_GAIN_P195DB;
   else if (gain_db < 21.75f)
      gain_settings = AM_HAL_PDM_GAIN_P210DB;
   else if (gain_db < 23.25f)
      gain_settings = AM_HAL_PDM_GAIN_P225DB;
   else if (gain_db < 24.75f)
      gain_settings = AM_HAL_PDM_GAIN_P240DB;
   else if (gain_db < 26.25f)
      gain_settings = AM_HAL_PDM_GAIN_P255DB;
   else if (gain_db < 27.75f)
      gain_settings = AM_HAL_PDM_GAIN_P270DB;
   else if (gain_db < 29.25f)
      gain_settings = AM_HAL_PDM_GAIN_P285DB;
   else if (gain_db < 30.75f)
      gain_settings = AM_HAL_PDM_GAIN_P300DB;
   else if (gain_db < 32.25f)
      gain_settings = AM_HAL_PDM_GAIN_P315DB;
   else if (gain_db < 33.75f)
      gain_settings = AM_HAL_PDM_GAIN_P330DB;
   else
      gain_settings = AM_HAL_PDM_GAIN_P345DB;

   // Configure the PDM peripheral and HFRC2 clock source
   if (sample_rate_hz <= 16000)
   {
      pdm_config.ePDMAClkOutDivder = AM_HAL_PDM_PDMA_CLKO_DIV7;  // F_CLK = 1.536MHz
      pdm_config.ui32DecimationRate = 1536000 / (2 * sample_rate_hz);
   }
   else
   {
      pdm_config.ePDMAClkOutDivder = AM_HAL_PDM_PDMA_CLKO_DIV3;  // F_CLK = 3.072MHz
      pdm_config.ui32DecimationRate = 3072000 / (2 * sample_rate_hz);
   }
   pdm_config.eLeftGain = gain_settings;
   pdm_config.eRightGain = gain_settings;
   pdm_config.ePCMChannels = (num_channels == 1) ? AM_HAL_PDM_CHANNEL_LEFT : AM_HAL_PDM_CHANNEL_STEREO;
   configASSERT0(am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFRC2_START, NULL));
   am_util_delay_us(200);
   configASSERT0(am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HF2ADJ_ENABLE, NULL));
   am_util_delay_us(500);

   // Configure the necessary PDM pins
   am_bsp_pdm_pins_enable(PDM_MODULE);

   // Configure and enable PDM interrupts
   configASSERT0(am_hal_pdm_interrupt_enable(audio_handle, (AM_HAL_PDM_INT_DERR | AM_HAL_PDM_INT_DCMP | AM_HAL_PDM_INT_UNDFL | AM_HAL_PDM_INT_OVF)));
   NVIC_SetPriority(PDM0_IRQn + PDM_MODULE, AUDIO_ADC_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(PDM0_IRQn + PDM_MODULE);
}

void audio_analog_init(uint32_t num_channels, uint32_t sample_rate_hz, uint32_t clip_length_seconds, float gain_db, float mic_bias_voltage, audio_trigger_t trigger, float trigger_threshold_percent, volatile bool *device_activated)
{
   // Turn on the external microphone
   const am_hal_gpio_pincfg_t mic_en_config = AM_HAL_GPIO_PINCFG_OUTPUT;
   configASSERT0(am_hal_gpio_pinconfig(PIN_MICROPHONE_ENABLE, mic_en_config));
   am_hal_gpio_output_set(PIN_MICROPHONE_ENABLE);

   // Power up two programmable gain amplifiers per requested channel
   skip_samples = 0;
   num_audio_channels = num_channels;
   configASSERT0(am_hal_audadc_refgen_powerup());
   for (uint32_t i = 0; i < num_channels; ++i)
   {
      configASSERT0(am_hal_audadc_pga_powerup(2*i));
      configASSERT0(am_hal_audadc_pga_powerup(2*i + 1));
      configASSERT0(am_hal_audadc_gain_set(2*i, 2*PREAMP_FULL_GAIN));
      configASSERT0(am_hal_audadc_gain_set(2*i + 1, 2*PREAMP_FULL_GAIN));
   }

   // Store the requested microphone amplification level (max gain is 45dB)
   if (gain_db > 45.0f)
      pga_gain_db = 45.0f;
   else if (gain_db < 0.0f)
      pga_gain_db = 0.0f;
   else
      pga_gain_db = gain_db;

   // Power up the external microphone bias if requested
   if ((mic_bias_voltage >= MICBIAS_VOLTAGE_MIN) && (mic_bias_voltage <= MICBIAS_VOLTAGE_MAX))
   {
      am_hal_audadc_micbias_powerup((uint32_t)lroundf((mic_bias_voltage - 0.827913f) / 0.012481f));
      am_util_delay_ms(400);
   }
   else
      am_hal_audadc_micbias_powerdown();

   // Initialize the AUDADC peripheral
   adc_awake = true;
   is_digital_mic = false;
   configASSERT0(am_hal_audadc_initialize(0, &audio_handle));
   configASSERT0(am_hal_audadc_power_control(audio_handle, AM_HAL_SYSCTRL_WAKE, false));

   // Determine the longest possible DMA period
   uint32_t max_seconds_per_dma = AUDIO_BUFFER_MAX_SAMPLES / sample_rate_hz;
   if (max_seconds_per_dma > clip_length_seconds)
      num_samples_per_dma = clip_length_seconds * sample_rate_hz;
   else if ((clip_length_seconds % max_seconds_per_dma) == 0)
      num_samples_per_dma = max_seconds_per_dma * sample_rate_hz;
   else
   {
      do { --max_seconds_per_dma; } while (clip_length_seconds % max_seconds_per_dma);
      num_samples_per_dma = max_seconds_per_dma * sample_rate_hz;
   }

   // Set up the trigger timer and DMA configuration structures
   sampling_rate_hz = sample_rate_hz;
   const float sample_rate_khz = (float)sample_rate_hz / 1000.0;
   audadc_irtt_config.ui32IrttCountMax = (uint32_t)lroundf((6000.0f / sample_rate_khz) - 1.0f);   // Sample rate = eClock/eClkDiv/(ui32IrttCountMax+1)
   audadc_dma_config.ui32SampleCount = num_samples_per_dma;
   audadc_dma_config.ui32TargetAddress = (uint32_t)sample_buffer;
   audadc_dma_config.ui32TargetAddressReverse = audadc_dma_config.ui32TargetAddress + (sizeof(uint32_t) * audadc_dma_config.ui32SampleCount);

   // Configure the AUDADC peripheral and HFRC clock source
   am_hal_audadc_config_t audadc_config =
   {
      .eClock           = AM_HAL_AUDADC_CLKSEL_HFRC_48MHz,
      .ePolarity        = AM_HAL_AUDADC_TRIGPOL_RISING,
      .eTrigger         = AM_HAL_AUDADC_TRIGSEL_SOFTWARE,
      .eClockMode       = AM_HAL_AUDADC_CLKMODE_LOW_POWER,
      .ePowerMode       = AM_HAL_AUDADC_LPMODE1,
      .eRepeat          = AM_HAL_AUDADC_REPEATING_SCAN,
      .eRepeatTrigger   = AM_HAL_AUDADC_RPTTRIGSEL_INT,
      .eSampMode        = AM_HAL_AUDADC_SAMPMODE_MED
   };
   configASSERT0(am_hal_audadc_configure(audio_handle, &audadc_config));
   configASSERT0(am_hal_audadc_enable(audio_handle));

   // Configure the AUDADC slots for measurement
   am_hal_audadc_slot_config_t audadc_slot_config =
   {
      .eMeasToAvg      = AM_HAL_AUDADC_SLOT_AVG_1,
      .ePrecisionMode  = AM_HAL_AUDADC_SLOT_12BIT,
      .ui32TrkCyc      = 34,
      .bWindowCompare  = false,
      .bEnabled        = true
   };
   for (uint32_t i = 0; i < num_channels; ++i)
   {
      audadc_slot_config.eChannel = 2*i;
      configASSERT0(am_hal_audadc_configure_slot(audio_handle, 2*i, &audadc_slot_config));
      audadc_slot_config.eChannel = 2*i + 1;
      configASSERT0(am_hal_audadc_configure_slot(audio_handle, 2*i + 1, &audadc_slot_config));
   }

   // Set the correct interrupt priority
   trigger_criterion = trigger;
   NVIC_SetPriority(AUDADC0_IRQn, AUDIO_ADC_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(AUDADC0_IRQn);
   system_enable_interrupts(true);

   // Temporarily start the ADC to calculate the audio DC offset
   dc_offset = 0;
   led_off(LED_ALL);
   audio_adc_start();
   uint32_t dc_calculated = 0;
   print("INFO: Calculating analog microphone DC offset...\n");
   while (*device_activated && (dc_calculated < 4))
   {
      if (dma_error)
         system_reset();
      else if (!dma_complete)
         system_enter_deep_sleep_mode();
      else
      {
         led_toggle(LED_ALL);
         uint32_t dc_total = 0;
         const uint32_t *data = (uint32_t*)am_hal_audadc_dma_get_buffer(audio_handle);
         for (uint32_t i = 0; i < sampling_rate_hz; ++i)
            dc_total += (data[i] >> 20UL);
         dc_total /= sampling_rate_hz;
         dc_calculated = (abs((int32_t)dc_total - (int32_t)dc_offset) < (2 * (dc_calculated + 1))) ? (dc_calculated + 1) : 0;
         dc_offset = dc_total;
         dma_complete = false;
      }
   }
   led_off(LED_ALL);
   dc_offset <<= 4;
   print("INFO: Analog microphone DC offset calculated: %u\n", dc_offset);

   // Optionally connect the audio input to a comparator
   if (trigger == COMPARATOR_THRESHOLD)
      comparator_init(false, 0, trigger_threshold_percent, true);

   // Put the AUDADC to sleep
   audio_stop_reading();
}

void audio_deinit(void)
{
   // Disable all interrupts and power down the PDM or AUDADC peripheral
   if (is_digital_mic)
   {
      am_hal_gpio_output_clear(PIN_DIGITAL_MIC_PWR);
      if (audio_handle)
      {
         NVIC_DisableIRQ(PDM0_IRQn + PDM_MODULE);
         if (!adc_awake)
            am_hal_pdm_power_control(audio_handle, AM_HAL_PDM_POWER_ON, true);
         am_hal_pdm_dma_disable(audio_handle);
         am_hal_pdm_disable(audio_handle);
         am_hal_pdm_deinitialize(audio_handle);
         configASSERT0(am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFRC2_STOP, NULL));
         configASSERT0(am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HF2ADJ_DISABLE, NULL));
         audio_handle = NULL;
         adc_awake = false;
      }
   }
   else
   {
      comparator_deinit();
      am_hal_gpio_output_clear(PIN_MICROPHONE_ENABLE);
      if (audio_handle)
      {
         NVIC_DisableIRQ(AUDADC0_IRQn);
         if (!adc_awake)
            am_hal_audadc_power_control(audio_handle, AM_HAL_SYSCTRL_WAKE, true);
         am_hal_audadc_interrupt_disable(audio_handle, 0xFFFFFFFF);
         while (AUDADC->DMATOTCOUNT_b.TOTCOUNT != 0);
         configASSERT0(am_hal_audadc_control(audio_handle, AM_HAL_AUDADC_REQ_DMA_DISABLE, NULL));
         configASSERT0(am_hal_audadc_irtt_disable(audio_handle));
         configASSERT0(am_hal_audadc_disable(audio_handle));
         am_hal_audadc_micbias_powerdown();
         for (uint32_t i = 0; i < num_audio_channels; ++i)
         {
            configASSERT0(am_hal_audadc_pga_powerdown(2*i));
            configASSERT0(am_hal_audadc_pga_powerdown(2*i + 1));
         }
         configASSERT0(am_hal_audadc_refgen_powerdown());
         configASSERT0(am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_AUDADC));
         configASSERT0(am_hal_audadc_deinitialize(audio_handle));
         audio_handle = NULL;
         adc_awake = false;
      }
   }
}

void audio_begin_reading(void)
{
   // Start the audio peripheral and DMA data conversions
   if (is_digital_mic)
      audio_adc_start();
   else
   {
      if (trigger_criterion == IMMEDIATE)
         audio_adc_start();
      else
         comparator_start();
   }
}

void audio_stop_reading(void)
{
   // Stop sampling from the audio peripheral
   if (is_digital_mic)
   {
      if (!adc_awake)
         am_hal_pdm_power_control(audio_handle, AM_HAL_PDM_POWER_ON, true);
      am_hal_pdm_dma_disable(audio_handle);
      am_hal_pdm_disable(audio_handle);
      am_hal_pdm_power_control(audio_handle, AM_HAL_PDM_POWER_OFF, true);
   }
   else
   {
      if (!adc_awake)
         am_hal_audadc_power_control(audio_handle, AM_HAL_SYSCTRL_WAKE, true);
      configASSERT0(am_hal_audadc_control(audio_handle, AM_HAL_AUDADC_REQ_DMA_DISABLE, NULL));
      configASSERT0(am_hal_audadc_irtt_disable(audio_handle));
      configASSERT0(am_hal_audadc_power_control(audio_handle, AM_HAL_SYSCTRL_DEEPSLEEP, true));
   }
   adc_awake = false;
}

bool audio_data_available(void)
{
   // Determine if time to trigger an AUDADC conversion
   if (!adc_awake && comparator_triggered())
   {
      audio_adc_start();
      comparator_reset();
   }
   return dma_complete;
}

bool audio_error_encountered(void)
{
   return dma_error;
}

bool audio_read_data(int16_t *buffer)
{
   // Only read data if a DMA audio conversion is complete
   if (dma_complete)
   {
      if (is_digital_mic)
      {
         const int32_t *data = (int32_t*)am_hal_pdm_dma_get_buffer(audio_handle);
         for (uint32_t i = 0; i < num_samples_per_dma; ++i)
            buffer[i] = (int16_t)(data[i] >> 8);
      }
      else
      {
         // Read and calibrate the audio samples from the AUDADC DMA buffer
         const uint32_t *data = (uint32_t*)am_hal_audadc_dma_get_buffer(audio_handle);
         for (uint32_t i = 0; i < num_samples_per_dma; ++i)
            buffer[i] = (int16_t)((data[i] >> 16UL) - dc_offset);
      }
      dma_complete = false;
      return true;
   }
   return false;
}

int16_t* audio_read_data_direct(void)
{
   // Only read data if a DMA audio conversion is complete
   if (dma_complete)
   {
      int16_t *buffer;
      if (is_digital_mic)
      {
         int32_t *data = (int32_t*)am_hal_pdm_dma_get_buffer(audio_handle);
         buffer = (int16_t*)data;
         for (uint32_t i = 0; i < num_samples_per_dma; ++i)
            buffer[i] = (int16_t)(data[i] >> 8);
      }
      else
      {
         // Read and calibrate the audio samples from the AUDADC DMA buffer
         uint32_t *data = (uint32_t*)am_hal_audadc_dma_get_buffer(audio_handle);
         buffer = (int16_t*)data;
         for (uint32_t i = 0; i < num_samples_per_dma; ++i)
            buffer[i] = (int16_t)((data[i] >> 16UL) - dc_offset);
      }
      dma_complete = false;
      return buffer;
   }
   return NULL;
}

uint32_t audio_num_seconds_per_dma(void)
{
   return num_samples_per_dma / sampling_rate_hz;
}
