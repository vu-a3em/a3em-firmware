// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "magnet.h"
#include "system.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

#define VALIDATION_TIMER_TICK_RATE_MS 50

static uint32_t validation_ticks_requested;
static uint32_t input_pin = PIN_MAG_SENSOR_INP;
static magnet_sensor_callback_t detection_callback;
static magnetic_field_validation_callback_t validation_callback;
static volatile bool magnetic_field_present, timer_manually_started;
static volatile am_hal_gpio_pincfg_t input_pin_config;
static volatile uint32_t validation_tick_count;
static bool sensor_enabled;


// Private Helper Functions --------------------------------------------------------------------------------------------

static void magnet_sensor_isr(void*)
{
   // Toggle the interrupt direction (due to silicon errata ERR008) to capture the opposite edge interrupt
   if (input_pin_config.GP.cfg_b.eIntDir == AM_HAL_GPIO_PIN_INTDIR_LO2HI)
   {
      input_pin_config.GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
      magnetic_field_present = true;
   }
   else
   {
      input_pin_config.GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
      magnetic_field_present = false;
   }
   configASSERT0(am_hal_gpio_pinconfig(PIN_MAG_SENSOR_INP, input_pin_config));
   configASSERT0(am_hal_gpio_pinconfig(PIN_MAG_SENSOR_INP2, input_pin_config));

   // Forward this interrupt to the user callback
   if (detection_callback)
      detection_callback(magnetic_field_present);
}


void am_timer02_isr(void)
{
   // Verify that the magnetic field is still present
   const bool field_present = am_hal_gpio_input_read(PIN_MAG_SENSOR_INP);
   am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(MAG_DETECT_TIMER_NUMBER, AM_HAL_TIMER_COMPARE_BOTH));
   if (!field_present && !timer_manually_started)
      validation_callback = NULL;
   else if (++validation_tick_count >= validation_ticks_requested)
   {
      validation_callback(field_present);
      validation_callback = NULL;
   }
   else
      am_hal_timer_clear(MAG_DETECT_TIMER_NUMBER);
}

static void enable_sensor(bool enable)
{
   // Clear or set the DISABLE pin to wake up or shut down the sensor
   if (enable && !sensor_enabled)
   {
      // Delay long enough for the sensor to wake up and make a measurement (from datasheet)
      am_hal_gpio_output_clear(PIN_MAG_SENSOR_DIS);
      system_delay(1050);
   }
   else if (!enable && sensor_enabled)
      am_hal_gpio_output_set(PIN_MAG_SENSOR_DIS);
   sensor_enabled = enable;
}


// Public API Functions ------------------------------------------------------------------------------------------------

void magnet_sensor_init(void)
{
   // Ensure detection callbacks are initially unregistered
   detection_callback = NULL;
   validation_callback = NULL;

   // Initialize all magnet sensor GPIOs
   input_pin_config = (am_hal_gpio_pincfg_t)AM_HAL_GPIO_PINCFG_INPUT;
   input_pin_config.GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_6K;
   input_pin_config.GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
   const am_hal_gpio_pincfg_t disable_pin_config = AM_HAL_GPIO_PINCFG_OUTPUT;
   configASSERT0(am_hal_gpio_pinconfig(PIN_MAG_SENSOR_INP, input_pin_config));
   configASSERT0(am_hal_gpio_pinconfig(PIN_MAG_SENSOR_INP2, input_pin_config));
   configASSERT0(am_hal_gpio_pinconfig(PIN_MAG_SENSOR_DIS, disable_pin_config));

   // Ensure that the sensor is asleep
   enable_sensor(false);

   // Initialize the magnetic field validation timer
   am_hal_timer_config_t field_validation_timer_config;
   am_hal_timer_default_config_set(&field_validation_timer_config);
   field_validation_timer_config.ui32Compare0 = (uint32_t)(VALIDATION_TIMER_TICK_RATE_MS * MAGNET_FIELD_VALIDATION_TIMER_TICK_RATE_HZ / 1000);
   am_hal_timer_config(MAG_DETECT_TIMER_NUMBER, &field_validation_timer_config);
   am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(MAG_DETECT_TIMER_NUMBER, AM_HAL_TIMER_COMPARE0));
   NVIC_SetPriority(TIMER0_IRQn + MAG_DETECT_TIMER_NUMBER, MAGNET_SENSOR_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(TIMER0_IRQn + MAG_DETECT_TIMER_NUMBER);
}

void magnet_sensor_deinit(void)
{
   // Stop all running timers
   am_hal_timer_disable(MAG_DETECT_TIMER_NUMBER);
   NVIC_DisableIRQ(TIMER0_IRQn + MAG_DETECT_TIMER_NUMBER);
   am_hal_timer_interrupt_disable(AM_HAL_TIMER_MASK(MAG_DETECT_TIMER_NUMBER, AM_HAL_TIMER_COMPARE0));;

   // Disable magnet sensing interrupts
   NVIC_DisableIRQ(GPIO0_001F_IRQn + GPIO_NUM2IDX(PIN_MAG_SENSOR_INP));
   am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, PIN_MAG_SENSOR_INP, NULL, (void*)input_pin);
   am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0, AM_HAL_GPIO_INT_CTRL_INDV_DISABLE, (void*)&input_pin);
   validation_callback = NULL;
   detection_callback = NULL;

   // Put the sensor into sleep mode
   enable_sensor(false);
}

void magnet_sensor_enable_for_wakeup(void)
{
   // Simply power on the sensor without any interrupt machinery
   enable_sensor(true);
}

bool magnet_sensor_field_present(void)
{
   // Temporarily enable the sensor to make a measurement if not already active
   if (!sensor_enabled)
   {
      am_hal_gpio_output_clear(PIN_MAG_SENSOR_DIS);
      system_delay(1050);
   }
   const bool field_present = am_hal_gpio_input_read(PIN_MAG_SENSOR_INP);
   if (!sensor_enabled)
      am_hal_gpio_output_set(PIN_MAG_SENSOR_DIS);
   return field_present;
}

void magnet_sensor_register_callback(magnet_sensor_callback_t callback)
{
   // Enable magnet sensing interrupts
   NVIC_SetPriority(GPIO0_001F_IRQn + GPIO_NUM2IDX(PIN_MAG_SENSOR_INP), MAGNET_SENSOR_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(GPIO0_001F_IRQn + GPIO_NUM2IDX(PIN_MAG_SENSOR_INP));

   // Ensure that the interrupts are initially set to trigger upon a transition to high
   magnetic_field_present = false;
   input_pin_config.GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
   configASSERT0(am_hal_gpio_pinconfig(PIN_MAG_SENSOR_INP, input_pin_config));
   configASSERT0(am_hal_gpio_pinconfig(PIN_MAG_SENSOR_INP2, input_pin_config));

   // Register a private callback with the interrupt control mechanism
   detection_callback = callback;
   configASSERT0(am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, PIN_MAG_SENSOR_INP, magnet_sensor_isr, (void*)input_pin));
   configASSERT0(am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0, AM_HAL_GPIO_INT_CTRL_INDV_ENABLE, (void*)&input_pin));

   // Wake up the sensor
   enable_sensor(true);

   // Check for a pre-existing magnetic field so we don't miss firing the initial callback
   if (magnet_sensor_field_present())
      magnet_sensor_isr(NULL);
}

void magnet_sensor_verify_field(uint32_t milliseconds, magnetic_field_validation_callback_t callback, bool calling_from_detection_callback)
{
   // Start the field validation timer for the requested number of milliseconds
   validation_tick_count = 0;
   validation_callback = callback;
   timer_manually_started = !calling_from_detection_callback;
   validation_ticks_requested = milliseconds / VALIDATION_TIMER_TICK_RATE_MS;
   am_hal_timer_clear(MAG_DETECT_TIMER_NUMBER);
}
