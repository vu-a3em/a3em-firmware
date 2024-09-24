// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <math.h>
#include "comparator.h"
#include "digipot.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

static volatile bool threshold_exceeded = false;


// Private Helper Functions --------------------------------------------------------------------------------------------

void am_vcomp_isr(void)
{
   // Set the threshold exceeded flag and disable further interrupts
   threshold_exceeded = true;
   VCOMP->INTEN_b.OUTHI = 0;
}


// Public API Functions ------------------------------------------------------------------------------------------------

void comparator_init(bool internal_reference, uint32_t internal_threshold_millivolts, float external_threshold_percent, bool compare_to_audadc_input)
{
   // Determine the level selection constant closest to the desired threshold for the internal reference generator
   VCOMP_CFG_LVLSEL_Enum level_select;
   if (internal_threshold_millivolts < 770)
      level_select = VCOMP_CFG_LVLSEL_0P58V;
   else if (internal_threshold_millivolts < 970)
      level_select = VCOMP_CFG_LVLSEL_0P77V;
   else if (internal_threshold_millivolts < 1160)
      level_select = VCOMP_CFG_LVLSEL_0P97V;
   else if (internal_threshold_millivolts < 1350)
      level_select = VCOMP_CFG_LVLSEL_1P16V;
   else if (internal_threshold_millivolts < 1550)
      level_select = VCOMP_CFG_LVLSEL_1P35V;
   else if (internal_threshold_millivolts < 1740)
      level_select = VCOMP_CFG_LVLSEL_1P55V;
   else if (internal_threshold_millivolts < 1930)
      level_select = VCOMP_CFG_LVLSEL_1P74V;
   else if (internal_threshold_millivolts < 2130)
      level_select = VCOMP_CFG_LVLSEL_1P93V;
   else
      level_select = VCOMP_CFG_LVLSEL_2P13V;

   // Initialize the external voltage reference if enabled
   if (!internal_reference)
   {
      // Initialize a digital potentiometer-based voltage reference
      digipot_init();
      digipot_set_percent_output(external_threshold_percent);

      // Initialize the GPIO pin used for the comparator reference voltage
      am_hal_gpio_pincfg_t reference_input_config = AM_HAL_GPIO_PINCFG_DISABLED;
      reference_input_config.GP.cfg_b.uFuncSel = PIN_COMPARATOR_REF_FN;
      configASSERT0(am_hal_gpio_pinconfig(PIN_COMPARATOR_REF, reference_input_config));
   }

   // Initialize the GPIO pin used for the comparator input
   if (!compare_to_audadc_input)
   {
      am_hal_gpio_pincfg_t voltage_input_config = AM_HAL_GPIO_PINCFG_DISABLED;
      voltage_input_config.GP.cfg_b.uFuncSel = PIN_COMPARATOR_FN;
      configASSERT0(am_hal_gpio_pinconfig(PIN_COMPARATOR, voltage_input_config));
   }

   // Initialize and configure the voltage comparator
   VCOMP->PWDKEY = VCOMP_PWDKEY_PWDKEY_Key;
   VCOMP->CFG_b.LVLSEL = level_select;
   VCOMP->CFG_b.NSEL = internal_reference ? VCOMP_CFG_NSEL_DAC : PIN_COMPARATOR_REF_CFG_FN;
   VCOMP->CFG_b.PSEL = VCOMP_CFG_PSEL_VEXT2;
   VCOMP->STAT_b.PWDSTAT = VCOMP_STAT_PWDSTAT_POWERED_UP;
   if (compare_to_audadc_input)
   {
      MCUCTRL->PGACTRL1_b.VCOMPSELPGA = 1;            // Select a PGA output as the VCOMP input
      MCUCTRL->PGAADCIFCTRL_b.PGAADCIFVCOMPSEL = 0;   // Route PGA_A0 to the VCOMP input
      MCUCTRL->PGAADCIFCTRL_b.PGAADCIFVCOMPEN = 1;
   }
   else
   {
      MCUCTRL->PGACTRL1_b.VCOMPSELPGA = 0;
      MCUCTRL->PGAADCIFCTRL_b.PGAADCIFVCOMPEN = 0;
   }
   VCOMP->PWDKEY = 1;
}

void comparator_deinit(void)
{
   // Shut down the voltage comparator
   comparator_stop();
   VCOMP->PWDKEY = VCOMP_PWDKEY_PWDKEY_Key;
   VCOMP->INTEN_b.OUTHI = 0;
   MCUCTRL->PGAADCIFCTRL_b.PGAADCIFVCOMPEN = 0;
   VCOMP->CFG_b.NSEL = VCOMP_CFG_NSEL_VREFEXT1;
   VCOMP->STAT_b.PWDSTAT = VCOMP_STAT_PWDSTAT_POWERED_DOWN;
   VCOMP->PWDKEY = 1;

   // Disable the comparator input pin
   am_hal_gpio_pincfg_t voltage_input_config = AM_HAL_GPIO_PINCFG_DISABLED;
   configASSERT0(am_hal_gpio_pinconfig(PIN_COMPARATOR, voltage_input_config));

   // Shutdown the digital potentiometer-based voltage reference
   digipot_deinit();
}

void comparator_start(void)
{
   // Enable interrupts when the comparator threshold is exceeded
   comparator_reset();
   NVIC_SetPriority(VCOMP_IRQn, COMPARATOR_THRESHOLD_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(VCOMP_IRQn);
}

void comparator_stop(void)
{
   // Disable comparator interrupts
   NVIC_DisableIRQ(VCOMP_IRQn);
   VCOMP->INTEN_b.OUTHI = 0;
   comparator_reset();
}

void comparator_reset(void)
{
   // Reset the threshold exceeded flag and clear any previous interrupts
   threshold_exceeded = false;
   VCOMP->INTCLR = VCOMP->INTSTAT;
   VCOMP->INTEN_b.OUTHI = 1;
}

void comparator_set_threshold(float percent)
{
   digipot_set_percent_output(percent);
}

bool comparator_triggered(void)
{
   // Return whether the threshold has been exceeded
   return threshold_exceeded;
}
