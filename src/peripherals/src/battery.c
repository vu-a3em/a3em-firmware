// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "battery.h"
#include "logging.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

#define BATTERY_ADC_SLOT 0
#define TEMPERATURE_ADC_SLOT 7

static bool battery_low_latched, tempco_available;
static am_hal_adc_sample_t temperature_samples_raw[BATTERY_NUM_SAMPLES];
static volatile uint32_t battery_voltage_code, temperature_code, temperature_samples_raw_count;
static volatile bool conversion_complete;
static uint32_t consecutive_low_readings;
static void *adc_handle;


// Private Helper Functions --------------------------------------------------------------------------------------------

void am_adc_isr(void)
{
   // Clear the ADC interrupt
   static uint32_t status;
   am_hal_adc_interrupt_status(adc_handle, &status, true);
   am_hal_adc_interrupt_clear(adc_handle, status);

   // Read all values from the ADC FIFO
   static am_hal_adc_sample_t sample;
   while (AM_HAL_ADC_FIFO_COUNT(ADC->FIFO))
   {
      uint32_t samples_to_read = 1;
      am_hal_daxi_control(AM_HAL_DAXI_CONTROL_INVALIDATE, NULL);
      am_hal_adc_samples_read(adc_handle, true, NULL, &samples_to_read, &sample);
      if (sample.ui32Slot == BATTERY_ADC_SLOT)
         battery_voltage_code = AM_HAL_ADC_FIFO_SAMPLE(sample.ui32Sample);
      else if (sample.ui32Slot == TEMPERATURE_ADC_SLOT)
      {
         temperature_code = AM_HAL_ADC_FIFO_SAMPLE(sample.ui32Sample);
         if (temperature_samples_raw_count < BATTERY_NUM_SAMPLES)
            temperature_samples_raw[temperature_samples_raw_count++] = sample;
      }
   }

   // Set the conversion complete flag
   conversion_complete = true;
}


// Public API Functions ------------------------------------------------------------------------------------------------

void battery_monitor_init(void)
{
   // Define the ADC configuration structures
   am_hal_adc_config_t adc_config =
   {
      .eClock = AM_HAL_ADC_CLKSEL_HFRC_24MHZ,
      .ePolarity = AM_HAL_ADC_TRIGPOL_RISING,
      .eTrigger = AM_HAL_ADC_TRIGSEL_SOFTWARE,
      .eClockMode = AM_HAL_ADC_CLKMODE_LOW_POWER,
      .ePowerMode = AM_HAL_ADC_LPMODE1,
      .eRepeat = AM_HAL_ADC_SINGLE_SCAN,
      .eRepeatTrigger = AM_HAL_ADC_RPTTRIGSEL_INT
   };
   am_hal_adc_slot_config_t unused_slot_config =
   {
      .bEnabled = false,
      .bWindowCompare = false,
      .eChannel = AM_HAL_ADC_SLOT_CHSEL_SE0,
      .ePrecisionMode = AM_HAL_ADC_SLOT_12BIT,
      .ui32TrkCyc = AM_HAL_ADC_MIN_TRKCYC,
      .eMeasToAvg = AM_HAL_ADC_SLOT_AVG_1
   };
   am_hal_adc_slot_config_t battery_slot_config =
   {
      .bEnabled = true,
      .bWindowCompare = false,
      .eChannel = PIN_BATTERY_VOLTAGE_ADC_CHANNEL,
      .ePrecisionMode = AM_HAL_ADC_SLOT_12BIT,
      .ui32TrkCyc = 63,
      .eMeasToAvg = AM_HAL_ADC_SLOT_AVG_1
   };
   am_hal_adc_slot_config_t temperature_slot_config =
   {
      .bEnabled = true,
      .bWindowCompare = false,
      .eChannel = AM_HAL_ADC_SLOT_CHSEL_TEMP,
      .ePrecisionMode = AM_HAL_ADC_SLOT_12BIT,
      .ui32TrkCyc = 32,
      .eMeasToAvg = AM_HAL_ADC_SLOT_AVG_1
   };

   // Initialize all static variables
   conversion_complete = false;
   battery_voltage_code = temperature_code = 0;

   // Initialize the voltage input pin
   am_hal_gpio_pincfg_t voltage_pin_config = AM_HAL_GPIO_PINCFG_INPUT;
   voltage_pin_config.GP.cfg_b.uFuncSel = PIN_BATTERY_VOLTAGE_FUNCTION;
   configASSERT0(am_hal_gpio_pinconfig(PIN_BATTERY_VOLTAGE, voltage_pin_config));

   // Initialize and configure the ADC
   configASSERT0(am_hal_adc_initialize(0, &adc_handle));
   configASSERT0(am_hal_adc_power_control(adc_handle, AM_HAL_SYSCTRL_WAKE, false));
   configASSERT0(am_hal_adc_configure(adc_handle, &adc_config));

   // Configure all ADC conversion slots
   for (int slot = 0; slot < AM_HAL_ADC_MAX_SLOTS; ++slot)
      if (slot == BATTERY_ADC_SLOT)
         am_hal_adc_configure_slot(adc_handle, slot, &battery_slot_config);
      else if (slot == TEMPERATURE_ADC_SLOT)
         am_hal_adc_configure_slot(adc_handle, slot, &temperature_slot_config);
      else
         am_hal_adc_configure_slot(adc_handle, slot, &unused_slot_config);

   // Enable the TempCo power reduction optimization
   tempco_available = (am_hal_pwrctrl_tempco_init(adc_handle, TEMPERATURE_ADC_SLOT) == AM_HAL_STATUS_SUCCESS);
   if (!tempco_available)
      print("WARNING: TempCo power optimization unavailable on this device\n");

   // Put the ADC into Deep Sleep mode
   configASSERT0(am_hal_adc_power_control(adc_handle, AM_HAL_SYSCTRL_DEEPSLEEP, true));
}

void battery_monitor_deinit(void)
{
   // Deinitialize the ADC module
   if (adc_handle)
   {
      am_hal_adc_power_control(adc_handle, AM_HAL_SYSCTRL_WAKE, true);
      am_hal_adc_deinitialize(adc_handle);
      adc_handle = NULL;
   }
}

battery_result_t battery_monitor_get_details(void)
{
   // Wake up the ADC
   battery_result_t result = { 0 };
   battery_voltage_code = temperature_code = 0;
   if (am_hal_adc_power_control(adc_handle, AM_HAL_SYSCTRL_WAKE, true) != AM_HAL_STATUS_SUCCESS)
      return result;

   // Enable interrupts upon completion of an ADC conversion
   am_hal_adc_interrupt_enable(adc_handle, AM_HAL_ADC_INT_CNVCMP);
   NVIC_SetPriority(ADC_IRQn, BATT_ADC_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(ADC_IRQn);

   // Enable the ADC
   if (am_hal_adc_enable(adc_handle) != AM_HAL_STATUS_SUCCESS)
   {
      am_hal_adc_interrupt_disable(adc_handle, AM_HAL_ADC_INT_CNVCMP);
      am_hal_adc_power_control(adc_handle, AM_HAL_SYSCTRL_DEEPSLEEP, true);
      NVIC_DisableIRQ(ADC_IRQn);
      return result;
   }

   // Clear any stale state left over from a previous call
   uint32_t status;
   am_hal_adc_interrupt_status(adc_handle, &status, true);
   am_hal_adc_interrupt_clear(adc_handle, status);
   while (AM_HAL_ADC_FIFO_COUNT(ADC->FIFO))
   {
      uint32_t samples_to_read = 1;
      am_hal_adc_sample_t junk;
      am_hal_adc_samples_read(adc_handle, true, NULL, &samples_to_read, &junk);
   }

   // Take several independent measurements and use the median
   uint32_t voltage_samples[BATTERY_NUM_SAMPLES] = { 0 }, temperature_samples[BATTERY_NUM_SAMPLES] = { 0 }, num_valid = 0;
   temperature_samples_raw_count = 0;
   for (uint32_t i = 0; i < BATTERY_NUM_SAMPLES; ++i)
   {
      // Trigger an ADC measurement
      conversion_complete = false;
      battery_voltage_code = temperature_code = 0;
      if (am_hal_adc_sw_trigger(adc_handle) != AM_HAL_STATUS_SUCCESS)
         break;

      // Wait until the conversion has completed
      bool completed = false;
      for (uint32_t attempt = 0; (attempt < BATTERY_CONVERSION_MAX_WAITS) && !completed; ++attempt)
      {
         if (conversion_complete)
            completed = true;
         else
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
      }
      if (!completed)
         break;

      // The first conversion after enabling the ADC is discarded as unreliable
      if (i > 0)
      {
         voltage_samples[num_valid] = battery_voltage_code;
         temperature_samples[num_valid] = temperature_code;
         ++num_valid;
      }
   }

   // Disable the ADC
   am_hal_adc_interrupt_disable(adc_handle, AM_HAL_ADC_INT_CNVCMP);
   am_hal_adc_disable(adc_handle);
   am_hal_adc_power_control(adc_handle, AM_HAL_SYSCTRL_DEEPSLEEP, true);
   NVIC_DisableIRQ(ADC_IRQn);
   if (!num_valid)
      return result;

   // Insertion sort the samples and take the middle one
   for (uint32_t i = 1; i < num_valid; ++i)
   {
      const uint32_t voltage_key = voltage_samples[i], temperature_key = temperature_samples[i];
      int32_t j = (int32_t)i - 1;
      while ((j >= 0) && (voltage_samples[j] > voltage_key))
      {
         voltage_samples[j + 1] = voltage_samples[j];
         temperature_samples[j + 1] = temperature_samples[j];
         --j;
      }
      voltage_samples[j + 1] = voltage_key;
      temperature_samples[j + 1] = temperature_key;
   }
   const uint32_t median_voltage_code = voltage_samples[num_valid / 2];
   const uint32_t median_temperature_code = temperature_samples[num_valid / 2];

   // Hand the raw temperature samples to the TempCo trim handler
   if (tempco_available && (temperature_samples_raw_count >= AM_HAL_TEMPCO_NUMSAMPLES))
      am_hal_pwrctrl_tempco_sample_handler(temperature_samples_raw_count, temperature_samples_raw);

   // Calculate and return the battery voltage and temperature
   result.valid = true;
   float temperature_codes[3] = { (float)median_temperature_code * AM_HAL_ADC_VREF / 4096.0f, 0.0f, -123.456f };
   result.millivolts = (uint32_t)(((uint64_t)median_voltage_code * AM_HAL_ADC_VREFMV * (VOLTAGE_DIVIDER_UPPER + VOLTAGE_DIVIDER_LOWER)) / (4096ull * VOLTAGE_DIVIDER_LOWER));
   if (am_hal_adc_control(adc_handle, AM_HAL_ADC_REQ_TEMP_CELSIUS_GET, temperature_codes) == AM_HAL_STATUS_SUCCESS)
      result.celcius = temperature_codes[1];
   return result;
}

bool battery_monitor_is_critically_low(uint32_t threshold_millivolts)
{
   // Require several consecutive low readings before reporting a low battery, and require the voltage
   // to recover by a margin before clearing the condition
   const battery_result_t details = battery_monitor_get_details();
   if (!details.valid)
   {
      // A failed measurement is not evidence of a low battery
      print("WARNING: Battery measurement failed\n");
      return battery_low_latched;
   }
   if (details.millivolts <= threshold_millivolts)
   {
      if (consecutive_low_readings < BATTERY_LOW_CONSECUTIVE_READINGS)
         ++consecutive_low_readings;
      if (consecutive_low_readings >= BATTERY_LOW_CONSECUTIVE_READINGS)
         battery_low_latched = true;
   }
   else
   {
      consecutive_low_readings = 0;
      if (battery_low_latched && (details.millivolts > (threshold_millivolts + BATTERY_LOW_HYSTERESIS_MV)))
         battery_low_latched = false;
   }
   return battery_low_latched;
}

void battery_monitor_reset_low_state(void)
{
   consecutive_low_readings = 0;
   battery_low_latched = false;
}
