// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "battery.h"
#include "logging.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

#define BATTERY_ADC_SLOT 0
#define TEMPERATURE_ADC_SLOT 7

static volatile uint32_t battery_voltage_code, temperature_code;
static volatile bool conversion_complete;
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
         temperature_code = AM_HAL_ADC_FIFO_SAMPLE(sample.ui32Sample);
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
      .ui32TrkCyc = AM_HAL_ADC_MIN_TRKCYC,
      .eMeasToAvg = AM_HAL_ADC_SLOT_AVG_1
   };
   am_hal_adc_slot_config_t temperature_slot_config =
   {
      .bEnabled = true,
      .bWindowCompare = false,
      .eChannel = AM_HAL_ADC_SLOT_CHSEL_TEMP,
      .ePrecisionMode = AM_HAL_ADC_SLOT_10BIT,
      .ui32TrkCyc = AM_HAL_ADC_MIN_TRKCYC,
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

   // Put the ADC into Deep Sleep mode
   configASSERT0(am_hal_adc_power_control(adc_handle, AM_HAL_SYSCTRL_DEEPSLEEP, true));
}

void battery_monitor_deinit(void)
{
   // Deinitialize the ADC module
   am_hal_adc_power_control(adc_handle, AM_HAL_SYSCTRL_WAKE, true);
   am_hal_adc_deinitialize(adc_handle);
}

battery_result_t battery_monitor_get_details(void)
{
   // Wake up the ADC
   conversion_complete = false;
   battery_result_t result = { 0 };
   battery_voltage_code = temperature_code = 0;
   if (am_hal_adc_power_control(adc_handle, AM_HAL_SYSCTRL_WAKE, true) != AM_HAL_STATUS_SUCCESS)
      return result;

   // Enable interrupts upon completion of an ADC conversion
   am_hal_adc_interrupt_enable(adc_handle, AM_HAL_ADC_INT_CNVCMP);
   NVIC_SetPriority(ADC_IRQn, BATT_ADC_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(ADC_IRQn);

   // Enable the ADC
   if ((am_hal_adc_enable(adc_handle) != AM_HAL_STATUS_SUCCESS) || am_hal_adc_sw_trigger(adc_handle))
   {
      am_hal_adc_interrupt_disable(adc_handle, AM_HAL_ADC_INT_CNVCMP);
      am_hal_adc_power_control(adc_handle, AM_HAL_SYSCTRL_DEEPSLEEP, true);
      NVIC_DisableIRQ(ADC_IRQn);
      return result;
   }

   // Wait until the conversion has completed
   uint32_t retries_remaining = 25;
   while (!conversion_complete && retries_remaining--)
      am_hal_delay_us(10000);

   // Disable the ADC
   am_hal_adc_interrupt_disable(adc_handle, AM_HAL_ADC_INT_CNVCMP);
   am_hal_adc_power_control(adc_handle, AM_HAL_SYSCTRL_DEEPSLEEP, true);
   NVIC_DisableIRQ(ADC_IRQn);

   // Calculate and return the battery voltage and temperature
   battery_voltage_code += 500; // TODO: THIS SHOULD CERTAINLY NOT BE NECESSARY, SOMETHING IS WRONG
   float temperature_codes[3] = { (float)temperature_code * AM_HAL_ADC_VREF / 1024.0f, 0.0f, -123.456f };
   result.millivolts = (battery_voltage_code * AM_HAL_ADC_VREFMV / 4096) * (VOLTAGE_DIVIDER_UPPER + VOLTAGE_DIVIDER_LOWER) / VOLTAGE_DIVIDER_LOWER;
   if (am_hal_adc_control(adc_handle, AM_HAL_ADC_REQ_TEMP_CELSIUS_GET, temperature_codes) == AM_HAL_STATUS_SUCCESS)
      result.celcius = temperature_codes[1];
   return result;
}
