// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <math.h>
#include "audio.h"
#include "comparator.h"
#include "logging.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

#define PREAMP_FULL_GAIN           80
#define MICBIAS_VOLTAGE_MIN        0.9f
#define MICBIAS_VOLTAGE_MAX        1.5f

AM_SHARED_RW uint32_t sample_buffer[(2*AUDIO_BUFFER_NUM_SAMPLES) + 3];

static void *audadc_handle;
static float pga_gain_db;
static uint32_t num_audio_channels, sampling_rate_hz;
static volatile bool dma_complete = false, dma_error = false, adc_started;
static am_hal_offset_cal_coeffs_array_t offset_calibration;
static am_hal_audadc_dma_config_t audadc_dma_config =
{
   .bDynamicPriority = false,
   .ePriority = AM_HAL_AUDADC_PRIOR_SERVICE_IMMED,
   .bDMAEnable = true,
   .ui32SampleCount = AUDIO_BUFFER_NUM_SAMPLES,
   .ui32TargetAddress = 0x0,
   .ui32TargetAddressReverse = 0x0
};
static am_hal_audadc_irtt_config_t audadc_irtt_config =
{
   .bIrttEnable        = true,
   .eClkDiv            = AM_HAL_AUDADC_RPTT_CLK_DIV32,
   .ui32IrttCountMax   = 0
};


// Private Helper Functions --------------------------------------------------------------------------------------------

void audio_adc_start(void)
{
   // Wake up the AUDADC peripheral
   configASSERT0(am_hal_audadc_power_control(audadc_handle, AM_HAL_SYSCTRL_WAKE, true));
   AUDADCn(0)->DATAOFFSET = ((AUDADC->DATAOFFSET & ~AUDADC_DATAOFFSET_OFFSET_Msk) | _VAL2FLD(AUDADC_DATAOFFSET_OFFSET, 0x1800));

   // Configure the AUDADC trigger timer
   configASSERT0(am_hal_audadc_configure_irtt(audadc_handle, &audadc_irtt_config));
   configASSERT0(am_hal_audadc_irtt_enable(audadc_handle));

   // Configure the DMA operation and interrupts to wake up the MCU on every conversion
   configASSERT0(am_hal_audadc_configure_dma(audadc_handle, &audadc_dma_config));
   configASSERT0(am_hal_audadc_interrupt_enable(audadc_handle, AM_HAL_AUDADC_INT_DERR | AM_HAL_AUDADC_INT_FIFOOVR1));

   // Set the desired gain configuration
   am_hal_audadc_gain_config_t audadc_gain_config =
   {
      .ui32LGA = (uint32_t)((pga_gain_db*2.0f + 10) * 0.8),
      .ui32HGADELTA = (uint32_t)((pga_gain_db*2.0f + 10) * 0.2),
      .ui32LGB = 0,
      .ui32HGBDELTA = 0,
      .eUpdateMode = AM_HAL_AUDADC_GAIN_UPDATE_IMME
   };
   configASSERT0(am_hal_audadc_internal_pga_config(audadc_handle, &audadc_gain_config));
   configASSERT0(am_hal_audadc_sw_trigger(audadc_handle));
   adc_started = true;
}

void am_audadc0_isr(void)
{
   // Clear the AUDADC interrupt
   static uint32_t status;
   am_hal_audadc_interrupt_status(audadc_handle, &status, false);
   am_hal_audadc_interrupt_clear(audadc_handle, status);

   // Handle a DMA completion or error event
   if ((status & AM_HAL_AUDADC_INT_FIFOOVR1) && AUDADCn(0)->DMASTAT_b.DMACPL)
   {
      am_hal_audadc_interrupt_service(audadc_handle, &audadc_dma_config);
      dma_complete = true;
      dma_error = false;
   }
   if (status & AM_HAL_AUDADC_INT_DERR)
      dma_error = true;
}


// Public API Functions ------------------------------------------------------------------------------------------------

void audio_init(uint32_t num_channels, uint32_t sample_rate_hz, float gain_db, float mic_bias_voltage)
{
   // Turn on the external microphone
   const am_hal_gpio_pincfg_t mic_en_config = AM_HAL_GPIO_PINCFG_OUTPUT;
   configASSERT0(am_hal_gpio_pinconfig(PIN_MICROPHONE_ENABLE, mic_en_config));
   am_hal_gpio_output_set(PIN_MICROPHONE_ENABLE);

   // Power up two programmable gain amplifiers per requested channel
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

   // Initialize the AUDADC peripheral
   adc_started = false;
   configASSERT0(am_hal_audadc_initialize(0, &audadc_handle));
   configASSERT0(am_hal_audadc_power_control(audadc_handle, AM_HAL_SYSCTRL_WAKE, false));

   // Set up the trigger timer and DMA configuration structures
   sampling_rate_hz = sample_rate_hz;
   const float sample_rate_khz = (float)sample_rate_hz / 1000.0;
   audadc_irtt_config.ui32IrttCountMax = (uint32_t)lroundf((1500.0f / sample_rate_khz) - 1.0f);   // Sample rate = eClock/eClkDiv/(ui32IrttCountMax+1)
   audadc_dma_config.ui32TargetAddress = (uint32_t)((uint32_t)(sample_buffer + 3) & ~0xF);
   audadc_dma_config.ui32TargetAddressReverse = audadc_dma_config.ui32TargetAddress + (sizeof(uint32_t) * audadc_dma_config.ui32SampleCount);

   // Configure the AUDADC peripheral and HFRC clock source
   am_hal_audadc_config_t audadc_config =
   {
      .eClock           = AM_HAL_AUDADC_CLKSEL_HFRC_48MHz,
      .ePolarity        = AM_HAL_AUDADC_TRIGPOL_RISING,
      .eTrigger         = AM_HAL_AUDADC_TRIGSEL_SOFTWARE,
      .eClockMode       = AM_HAL_AUDADC_CLKMODE_LOW_LATENCY,
      .ePowerMode       = AM_HAL_AUDADC_LPMODE1,
      .eRepeat          = AM_HAL_AUDADC_REPEATING_SCAN,
      .eRepeatTrigger   = AM_HAL_AUDADC_RPTTRIGSEL_INT,
      .eSampMode        = AM_HAL_AUDADC_SAMPMODE_MED
   };
   configASSERT0(am_hal_audadc_configure(audadc_handle, &audadc_config));
   configASSERT0(am_hal_audadc_enable(audadc_handle));

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
      configASSERT0(am_hal_audadc_configure_slot(audadc_handle, 2*i, &audadc_slot_config));
      audadc_slot_config.eChannel = 2*i + 1;
      configASSERT0(am_hal_audadc_configure_slot(audadc_handle, 2*i + 1, &audadc_slot_config));
   }

   // Calculate the DC offset calibration parameters
   configASSERT0(am_hal_audadc_slot_dc_offset_calculate(audadc_handle, 2*num_channels, &offset_calibration));

   // Set the correct interrupt priority and put the AUDADC to sleep
   configASSERT0(am_hal_audadc_power_control(audadc_handle, AM_HAL_SYSCTRL_DEEPSLEEP, true));
   NVIC_SetPriority(AUDADC0_IRQn, AUDIO_ADC_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(AUDADC0_IRQn);
}

void audio_deinit(void)
{
   // Disable all interrupts and power down the AUDADC peripheral
   if (audadc_handle)
   {
      NVIC_DisableIRQ(AUDADC0_IRQn);
      if (!adc_started)
         am_hal_audadc_power_control(audadc_handle, AM_HAL_SYSCTRL_WAKE, true);
      am_hal_audadc_interrupt_disable(audadc_handle, 0xFFFFFFFF);
      while (AUDADC->DMATOTCOUNT_b.TOTCOUNT != 0);
      configASSERT0(am_hal_audadc_control(audadc_handle, AM_HAL_AUDADC_REQ_DMA_DISABLE, NULL));
      configASSERT0(am_hal_audadc_irtt_disable(audadc_handle));
      configASSERT0(am_hal_audadc_disable(audadc_handle));
      am_hal_audadc_micbias_powerdown();
      for (uint32_t i = 0; i < num_audio_channels; ++i)
      {
         configASSERT0(am_hal_audadc_pga_powerdown(2*i));
         configASSERT0(am_hal_audadc_pga_powerdown(2*i + 1));
      }
      configASSERT0(am_hal_audadc_refgen_powerdown());
      configASSERT0(am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_AUDADC));
      configASSERT0(am_hal_audadc_deinitialize(audadc_handle));
      audadc_handle = NULL;
      adc_started = false;
   }
}

uint32_t audio_num_reads_per_n_seconds(uint32_t seconds)
{
   return (uint32_t)(0.5f + ((float)seconds / ((float)AUDIO_BUFFER_NUM_SAMPLES / (float)sampling_rate_hz)));
}

void audio_begin_reading(audio_trigger_t criterion)
{
   // Trigger the next AUDADC conversion
   if (criterion == IMMEDIATE)
      audio_adc_start();
   else
      comparator_start();
}

void audio_stop_reading(void)
{
   // Stop sampling from the AUDADC
   configASSERT0(am_hal_audadc_control(audadc_handle, AM_HAL_AUDADC_REQ_DMA_DISABLE, NULL));
   configASSERT0(am_hal_audadc_irtt_disable(audadc_handle));
   configASSERT0(am_hal_audadc_power_control(audadc_handle, AM_HAL_SYSCTRL_DEEPSLEEP, true));
}

bool audio_data_available(void)
{
   // Determine if time to trigger an AUDADC conversion
   if (comparator_triggered())
   {
      print("INFO: Audio threshold triggered...beginning audio read\n");
      audio_adc_start();
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
   static int32_t offset_adjustment;
   if (dma_complete)
   {
      // Read and calibrate the audio samples from the AUDADC DMA buffer
      const uint32_t *data = (uint32_t*)am_hal_audadc_dma_get_buffer(audadc_handle);
      for (uint32_t i = 0; i < AUDIO_BUFFER_NUM_SAMPLES; ++i)
      {
         buffer[i] = AM_HAL_AUDADC_FIFO_HGDATA(data[i]) << 4;
         if (offset_calibration.sCalibCoeff[1].bValid)
         {
            offset_adjustment = offset_calibration.sCalibCoeff[1].i32DCOffsetAdj << 4;
            if ((buffer[i] >= 0) && (offset_adjustment > (32767 - buffer[i])))
               buffer[i] = 32767;
            else if ((buffer[i] < 0) && (offset_adjustment < (-32768 - buffer[i])))
               buffer[i] = -32768;
            else
               buffer[i] += offset_adjustment;
         }
      }
      dma_complete = false;
      return true;
   }
   return false;
}

int16_t* audio_read_data_direct(void)
{
   // Only read data if a DMA audio conversion is complete
   static int32_t offset_adjustment;
   if (dma_complete)
   {
      // Read and calibrate the audio samples from the AUDADC DMA buffer
      uint32_t *data = (uint32_t*)am_hal_audadc_dma_get_buffer(audadc_handle);
      int16_t *buffer = (int16_t*)data;
      for (uint32_t i = 0; i < AUDIO_BUFFER_NUM_SAMPLES; ++i)
      {
         buffer[i] = AM_HAL_AUDADC_FIFO_HGDATA(data[i]) << 4;
         if (offset_calibration.sCalibCoeff[1].bValid)
         {
            offset_adjustment = offset_calibration.sCalibCoeff[1].i32DCOffsetAdj << 4;
            if ((buffer[i] >= 0) && (offset_adjustment > (32767 - buffer[i])))
               buffer[i] = 32767;
            else if ((buffer[i] < 0) && (offset_adjustment < (-32768 - buffer[i])))
               buffer[i] = -32768;
            else
               buffer[i] += offset_adjustment;
         }
      }
      dma_complete = false;
      return buffer;
   }
   return NULL;
}
