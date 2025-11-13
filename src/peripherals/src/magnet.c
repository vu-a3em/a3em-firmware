// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "led.h"
#include "magnet.h"
#include "system.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

#define SENSOR_SAMPLING_TIME_US 15
#define SENSOR_SLEEP_TIME_MS 90
#define SENSOR_DEEP_SLEEP_TIME_MS 500

#define VALIDATION_TIMER_TICK_RATE_MS 100
#define VALIDATION_TRANSITION_TIME_BUFFER_MS 2000
#define VALIDATION_NUM_PHASES 5

static uint32_t input_pin = PIN_MAG_SENSOR_INP;
static magnet_sensor_callback_t detection_callback;
static int32_t validation_ticks_requested, transition_ticks;
static magnetic_field_validation_callback_t validation_callback;
static volatile int32_t validation_tick_count, transition_tick_count, validation_phase;
static volatile am_hal_gpio_pincfg_t input_pin_config;
static volatile bool magnetic_field_present;
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

   // Potentially move to the next stage of validation mode
   if (transition_tick_count >= 0)
      transition_tick_count = transition_ticks;
}

void am_timer02_isr(void)
{
   // Verify that the magnetic field is still as expected
   const bool required_field_presence = (validation_phase % 2) == 0;
   const bool field_present = am_hal_gpio_input_read(PIN_MAG_SENSOR_INP);
   am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(MAG_DETECT_TIMER_NUMBER, AM_HAL_TIMER_COMPARE_BOTH));

   // Handle verification based on the current phase
   if (transition_tick_count >= 0)
   {
      if (++transition_tick_count >= transition_ticks)
      {
         transition_tick_count = -1;
         if (field_present != required_field_presence)
         {
            led_indicate_validation_failed();
            validation_callback(false);
            validation_callback = NULL;
         }
         else
            am_hal_timer_clear(MAG_DETECT_TIMER_NUMBER);
      }
      else
      {
         led_toggle_validation_phase_change();
         am_hal_timer_clear(MAG_DETECT_TIMER_NUMBER);
      }
   }
   else if (field_present != required_field_presence)
   {
      validation_callback(false);
      validation_callback = NULL;
   }
   else if (++validation_tick_count >= validation_ticks_requested)
   {
      validation_tick_count = 0;
      if (++validation_phase == VALIDATION_NUM_PHASES)
      {
         validation_callback(true);
         validation_callback = NULL;
      }
      else
      {
         led_off(LED_ALL);
         transition_tick_count = 0;
         led_toggle_validation_phase_change();
         am_hal_timer_clear(MAG_DETECT_TIMER_NUMBER);
      }
   }
   else
      am_hal_timer_clear(MAG_DETECT_TIMER_NUMBER);
}

static void enable_sensor(bool enable)
{
   // Clear or set the DISABLE pin to wake up or shut down the sensor
   if (enable && !sensor_enabled)
   {
      // Give sensor time to make the first measurement
      am_hal_timer_clear(MAG_SAMPLING_TIMER_NUMBER);
      system_delay(1200);
   }
   else if (!enable && sensor_enabled)
   {
      am_hal_timer_disable(MAG_SAMPLING_TIMER_NUMBER);
      am_hal_gpio_output_set(PIN_MAG_SENSOR_DIS);
   }
   sensor_enabled = enable;
}


// Public API Functions ------------------------------------------------------------------------------------------------

void magnet_sensor_init(void)
{
   // Ensure detection callbacks are initially unregistered
   detection_callback = NULL;
   validation_callback = NULL;
   transition_tick_count = -1;

   // Initialize all magnet sensor GPIOs
   input_pin_config = (am_hal_gpio_pincfg_t)AM_HAL_GPIO_PINCFG_INPUT;
   input_pin_config.GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_6K;
   input_pin_config.GP.cfg_b.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
   am_hal_gpio_pincfg_t disable_pin_config = AM_HAL_GPIO_PINCFG_OUTPUT;
   disable_pin_config.GP.cfg_b.uFuncSel = PIN_MAG_SENSOR_DIS_FUNCTION;
   configASSERT0(am_hal_gpio_pinconfig(PIN_MAG_SENSOR_INP, input_pin_config));
   configASSERT0(am_hal_gpio_pinconfig(PIN_MAG_SENSOR_INP2, input_pin_config));
   configASSERT0(am_hal_gpio_pinconfig(PIN_MAG_SENSOR_DIS, disable_pin_config));

   // Ensure that the sensor is asleep
   sensor_enabled = true;
   enable_sensor(false);

   // Initialize the magnetic field validation timer
   am_hal_timer_config_t field_validation_timer_config;
   am_hal_timer_default_config_set(&field_validation_timer_config);
   field_validation_timer_config.ui32Compare0 = (uint32_t)(VALIDATION_TIMER_TICK_RATE_MS * MAGNET_FIELD_TIMER_TICK_RATE_HZ / 1000);
   am_hal_timer_config(MAG_DETECT_TIMER_NUMBER, &field_validation_timer_config);
   am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(MAG_DETECT_TIMER_NUMBER, AM_HAL_TIMER_COMPARE0));
   NVIC_SetPriority(TIMER0_IRQn + MAG_DETECT_TIMER_NUMBER, MAGNET_VALIDATION_TIMER_INTERRUPT_PRIORITY );
   NVIC_EnableIRQ(TIMER0_IRQn + MAG_DETECT_TIMER_NUMBER);

   // Initialize the magnetic field sampling timer
   am_hal_timer_config_t field_sampling_timer_config;
   am_hal_timer_default_config_set(&field_sampling_timer_config);
   field_sampling_timer_config.eFunction = AM_HAL_TIMER_FN_PWM;
   field_sampling_timer_config.ui32Compare0 = (uint32_t)(SENSOR_SLEEP_TIME_MS * MAGNET_FIELD_TIMER_TICK_RATE_HZ / 1000);
   field_sampling_timer_config.ui32Compare1 = (uint32_t)(SENSOR_SAMPLING_TIME_US * MAGNET_FIELD_TIMER_TICK_RATE_HZ / 1000000);
   am_hal_timer_config(MAG_SAMPLING_TIMER_NUMBER, &field_sampling_timer_config);
   am_hal_timer_output_config(PIN_MAG_SENSOR_DIS, 2 * MAG_SAMPLING_TIMER_NUMBER);
}

void magnet_sensor_deinit(void)
{
   // Stop all running timers
   am_hal_timer_disable(MAG_DETECT_TIMER_NUMBER);
   am_hal_timer_disable(MAG_SAMPLING_TIMER_NUMBER);
   NVIC_DisableIRQ(TIMER0_IRQn + MAG_DETECT_TIMER_NUMBER);
   am_hal_timer_interrupt_disable(AM_HAL_TIMER_MASK(MAG_DETECT_TIMER_NUMBER, AM_HAL_TIMER_COMPARE0));

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
   // Initialize the magnetic field sampling pin
   am_hal_gpio_pincfg_t disable_pin_config = AM_HAL_GPIO_PINCFG_OUTPUT;
   disable_pin_config.GP.cfg_b.uFuncSel = PIN_MAG_SENSOR_DIS_FUNCTION;
   configASSERT0(am_hal_gpio_pinconfig(PIN_MAG_SENSOR_DIS, disable_pin_config));

   // Initialize the magnetic field sampling timer using a low-power, always-on LFRC (nominally 900Hz)
   am_hal_timer_config_t field_sampling_timer_config;
   am_hal_timer_default_config_set(&field_sampling_timer_config);
   field_sampling_timer_config.eFunction = AM_HAL_TIMER_FN_PWM;
   field_sampling_timer_config.eInputClock = AM_HAL_TIMER_CLOCK_LFRC;
   field_sampling_timer_config.ui32Compare0 = (uint32_t)(SENSOR_DEEP_SLEEP_TIME_MS * 900 / 1000);
   field_sampling_timer_config.ui32Compare1 = 1;
   am_hal_timer_config(MAG_SAMPLING_TIMER_NUMBER, &field_sampling_timer_config);
   am_hal_timer_output_config(PIN_MAG_SENSOR_DIS, 2 * MAG_SAMPLING_TIMER_NUMBER);

   // Start the low-frequency sampling timer
   am_hal_timer_clear(MAG_SAMPLING_TIMER_NUMBER);
}

bool magnet_sensor_field_present(void)
{
   // Temporarily enable the sensor to make a measurement if not already active
   if (!sensor_enabled)
   {
      am_hal_gpio_output_clear(PIN_MAG_SENSOR_DIS);
      system_delay(1200);
   }
   const bool field_present = am_hal_gpio_input_read(PIN_MAG_SENSOR_INP);
   if (!sensor_enabled)
      am_hal_gpio_output_set(PIN_MAG_SENSOR_DIS);
   return field_present;
}

void magnet_sensor_register_callback(magnet_sensor_callback_t callback)
{
   // Register a private callback with the interrupt control mechanism
   detection_callback = callback;
   configASSERT0(am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, PIN_MAG_SENSOR_INP, magnet_sensor_isr, (void*)input_pin));
   configASSERT0(am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0, AM_HAL_GPIO_INT_CTRL_INDV_ENABLE, (void*)&input_pin));

   // Wake up the sensor
   enable_sensor(true);

   // Ensure that interrupts are initially set to trigger on the correct transition
   magnetic_field_present = am_hal_gpio_input_read(PIN_MAG_SENSOR_INP);
   input_pin_config.GP.cfg_b.eIntDir = magnetic_field_present ? AM_HAL_GPIO_PIN_INTDIR_HI2LO : AM_HAL_GPIO_PIN_INTDIR_LO2HI;
   configASSERT0(am_hal_gpio_pinconfig(PIN_MAG_SENSOR_INP, input_pin_config));
   configASSERT0(am_hal_gpio_pinconfig(PIN_MAG_SENSOR_INP2, input_pin_config));

   // Manually fire the initial callback
   detection_callback(magnetic_field_present);

   // Enable magnet sensing interrupts
   NVIC_SetPriority(GPIO0_001F_IRQn + GPIO_NUM2IDX(PIN_MAG_SENSOR_INP), MAGNET_SENSOR_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(GPIO0_001F_IRQn + GPIO_NUM2IDX(PIN_MAG_SENSOR_INP));
}

void magnet_sensor_verify_field(uint32_t milliseconds, magnetic_field_validation_callback_t callback)
{
   // Only proceed if there is not a pending verification in progress
   if (!validation_callback)
   {
      // Start the field validation timer for the requested number of milliseconds
      transition_tick_count = -1;
      validation_callback = callback;
      validation_phase = validation_tick_count = 0;
      validation_ticks_requested = milliseconds / VALIDATION_TIMER_TICK_RATE_MS;
      transition_ticks = VALIDATION_TRANSITION_TIME_BUFFER_MS / VALIDATION_TIMER_TICK_RATE_MS;
      am_hal_timer_clear(MAG_DETECT_TIMER_NUMBER);
   }
}
