// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "led.h"
#include "logging.h"
#include "system.h"
#include "tracker.h"


// Test Application Overview -------------------------------------------------------------------------------------------
//
// Pin-level bring-up tool for the GPS Tracker I2C link. Where test_tracker2.c brings the IO Slave up and waits to be
// addressed, this application never touches the IOS at all: it owns GPIO 0 and GPIO 1 as ordinary pins and answers the
// two questions that a slave-side test cannot.
//
//    1. Is GPIO 0 really the clock and GPIO 1 really the data line, all the way out to the connector?
//    2. What is the Tracker actually putting on those two wires?
//
// It runs in two stages. The master probe drives the bus itself, so a logic analyzer sees traffic that is known-good by
// construction; anything the analyzer decodes differently is a wiring fault between this die and the probe tips. The
// sniffer then releases both pins and passively records every transition, decoding the result twice -- once assuming
// GPIO 0 is the clock and once assuming GPIO 1 is -- and reports which assumption produces valid I2C.
//
// IMPORTANT: the IO Slave is never enabled here, so this board will NOT acknowledge its own address while this
// application is running. A NACK during the sniffer stage is expected and means nothing. Flash test_tracker2.c to test
// whether the slave answers.
//
//
// Stage 1, the master probe (runs at start-up, then repeats whenever the bus has stayed idle -- see
// MASTER_PROBE_INTERVAL_SEC). Four phases, in increasing order of what they need to be believed:
//
//    Drive check       Drives each line low on its own and reads both back. Catches a pad that cannot pull its line
//                      down, a line held low by something else, and the two lines shorted together. Needs no test
//                      equipment; the verdict is printed on this console.
//
//    Line identity     Emits an asymmetric pulse train: LINE_ID_SCL_PULSES low pulses on GPIO 0 while GPIO 1 is held
//                      low, an idle gap with both lines high, then LINE_ID_SDA_PULSES low pulses on GPIO 1 while
//                      GPIO 0 is held low. Count the pulses on each analyzer channel. The channel carrying 3 pulses
//                      first is GPIO 0 and must be the one wired to SCL; the channel carrying 7 pulses is GPIO 1 and
//                      must be SDA. If they come out the other way round, the lines are crossed and that alone
//                      explains why the slave never acknowledges. The idle line is parked low throughout each burst
//                      so that the pattern cannot be mistaken for I2C START/STOP activity.
//
//    Known frame       Bit-bangs a textbook write to EXT_HW_I2C_ADDRESS and reports whether anything acknowledged.
//                      The analyzer must decode this as a normal write; if it cannot, the analyzer and this board
//                      disagree about which wire is which.
//
//    Address scan      Probes every 7-bit address and lists whatever answers. Useful for finding a slave that came up
//                      at an unexpected address, and for proving that the ACK sampling in this file works at all.
//
// Arm the analyzer for at least a 2 ms window before the probe runs; the identity pattern alone spans roughly
// (3 + 7) * 2 * LINE_ID_PULSE_US plus the gaps.
//
//
// Stage 2, the sniffer (runs forever after the probe). Both pins become plain inputs with no pull, so the tool is
// invisible to the bus. A tight polling loop timestamps every transition with the cycle counter, which samples far
// faster than any legal I2C rate, and a burst ends once the bus has been quiet for SNIFFER_IDLE_GAP_US. Each burst is
// then reported as:
//
//    - the transition count on each line; the clock line carries roughly nine times the edges of a data line
//    - the decoded frames under both line assignments, and a verdict naming the one that produced valid I2C
//    - the peak clock rate, measured from the shortest interval between consecutive clock rising edges
//
// If the sniffer reports valid frames only under "GPIO 1 = clock", the two signals are swapped. If neither assignment
// decodes but edges are present, the capture is not I2C at all. If no edges arrive while the Tracker is transmitting,
// the wires reaching these pads are not the wires the Tracker is driving.
//
//
// Stage 3, watching the bus through the IO Slave's own pads (WATCH_THROUGH_IOS_PADS, and the default). Stages 1 and 2
// clear the wires and the pads but say nothing about what the peripheral behind them receives, because they only run
// with the pads configured as GPIOs. This stage runs the production tracker_init() so the IO Slave is fully enabled
// and owns both pads at FNCSEL 1, and then samples them anyway: Programmer's Guide Section 9.2.3 says a pad's input
// "can always be read from the GPIO_RDn register" as long as INPEN is set and the pad is not a GPIO with RDZERO high,
// so GPIO_RD0 shows exactly the two signals the peripheral is being fed. That closes the last gap in the chain.
//
// Each burst is reported three ways, which together isolate where the transfer dies:
//
//    what the pads saw        the decoded frame, including whether the address byte was acknowledged on the wire
//    what the IO Slave did    interrupts taken, INTSTAT, ADDPTR and REGACCINTSTAT, sampled either side of the burst
//    which address was armed  see below
//
// If the pads show a clean frame and the IO Slave shows no reaction at all, the signal is reaching the peripheral and
// the address comparison is what fails. That is worth testing directly, because the Programmer's Guide contradicts
// itself on how CFG_I2CADDR is encoded: Section 13.8.6 says a 7-bit address must "match the lower 7 bits of the
// CFG_I2CADDR field", while its own Figure 42 and Figure 44 show a field holding the address shifted left by one.
// The driver programs the shifted form. So this stage rotates CFG_I2CADDR through every plausible encoding, spending
// IOS_BURSTS_PER_CANDIDATE bursts on each, and names the one that produces a match. Leave the master writing to
// EXT_HW_I2C_ADDRESS in a loop and watch which line reports an acknowledgement.
//
// If no encoding ever matches while the pads clearly show valid frames, the receive path inside the peripheral is not
// working and the remaining suspects are the module instance, the silicon, or a board fault -- not the configuration.


// Static Global Variables and Definitions -------------------------------------------------------------------------------

#define TEST_APP_VERSION            0x01

#define SCL_PIN                     PIN_EXT_HW_I2C_SCL
#define SDA_PIN                     PIN_EXT_HW_I2C_SDA
#define SCL_MASK                    (1u << SCL_PIN)
#define SDA_MASK                    (1u << SDA_PIN)

// The capture loop reads the whole low GPIO bank in one shot, which only covers pins 0 through 31
_Static_assert((SCL_PIN < 32) && (SDA_PIN < 32), "The bus sniffer can only watch GPIO 0 through GPIO 31");

// Bit-bang timing. One quarter-period per state change puts the clock at roughly 1/(4 * BUS_QUARTER_US), so the
// default is about 83 kHz -- deliberately slow, and well below the 500 kHz floor of erratum ERR066
#define BUS_QUARTER_US              3
#define SCL_STRETCH_TIMEOUT_US      2000

// Line identity pattern. The two pulse counts must stay different from each other, and small enough to count by eye
#define LINE_ID_PULSE_US            20
#define LINE_ID_SCL_PULSES          3
#define LINE_ID_SDA_PULSES          7
#define LINE_ID_REPEATS             3
#define LINE_ID_PHASE_GAP_US        200
#define LINE_ID_REPEAT_GAP_US       5000

// Bring the IO Slave up and watch its own pads, rotating the address encoding. Set to 0 to go back to the pin-level
// tool of stages 1 and 2, which leaves the peripheral disabled and the pads configured as GPIOs
#define WATCH_THROUGH_IOS_PADS      1
#define IOS_BURSTS_PER_CANDIDATE    4

// Set to 0 for a purely passive tool that never drives the bus. Ignored while WATCH_THROUGH_IOS_PADS is set, because
// the pads then belong to the peripheral and cannot be driven as GPIOs
#define RUN_MASTER_PROBE            1

// Re-run the master probe this often, but only while the bus has stayed idle, so that an analyzer can be armed without
// having to race a reset. Set to 0 to probe once at start-up and never again
#define MASTER_PROBE_INTERVAL_SEC   15

#define SNIFFER_MAX_EDGES           4096
#define SNIFFER_IDLE_GAP_US         500
#define SNIFFER_ARM_TIMEOUT_MS      1000

// A level must hold this long before it counts as an edge. The pull-up driven rising edges of an I2C bus cross the
// pad's input threshold slowly, and polling them at several MHz catches that crossing as a burst of chatter, which
// shows up as duplicated or shifted bits in the decode. This must stay well under half a bit period: 500 ns leaves
// room down to a 400 kHz bus, where half a bit is still 1250 ns
#define GLITCH_FILTER_NS            500

// An LRAM offset inside the host-writable direct area, used only as a scratch location for the liveness probe
#define REG_LIVENESS_PROBE          0x20
#define SNIFFER_PRINT_EDGES         24
#define SNIFFER_MAX_PRINTED_FRAMES  16

static uint32_t capture_cycle[SNIFFER_MAX_EDGES];
static uint32_t capture_level[SNIFFER_MAX_EDGES];
static uint32_t capture_initial_level;
static uint32_t cycles_per_us;
static bool cycle_counter_running;

// Everything one pass of the I2C decoder learned about a capture
typedef struct
{
   uint32_t starts;            // START conditions, including repeated starts
   uint32_t restarts;          // of those, the ones that arrived without an intervening STOP
   uint32_t stops;
   uint32_t bytes;             // bytes that reached their acknowledge bit
   uint32_t acks, nacks;
   uint32_t address_acks;      // of those, acknowledged address bytes, which is the only proof a slave answered
   uint32_t last_address;      // the address byte of the most recent frame, as seen on the wire
   uint32_t complete_frames;   // START through STOP with no leftover bits
   uint32_t truncated_frames;  // a STOP arrived part way through a byte
   uint32_t simultaneous;      // samples in which both lines moved at once, so their order is unknowable
} decode_stats_t;


// Private Helper Functions ----------------------------------------------------------------------------------------------

static void timing_init(void)
{
   // The cycle counter is the sniffer's only time base. Calibrate it against the HAL's delay rather than assuming a
   // core frequency, and notice if the counter is not running at all -- decoding still works without it, since that
   // only depends on the order of transitions, but every reported time and frequency would be meaningless
   CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
   DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
   DWT->CYCCNT = 0;
   const uint32_t start = DWT->CYCCNT;
   am_hal_delay_us(10000);
   const uint32_t elapsed = DWT->CYCCNT - start;
   cycle_counter_running = (elapsed > 0);
   cycles_per_us = cycle_counter_running ? (elapsed / 10000) : 96;
   if (!cycles_per_us)
      cycles_per_us = 96;
}

static uint32_t cycles_to_ns(uint32_t cycles)
{
   return (uint32_t)(((uint64_t)cycles * 1000u) / cycles_per_us);
}

static void print_microseconds(uint32_t cycles)
{
   const uint32_t nanoseconds = cycles_to_ns(cycles);
   print("%u.%03u", nanoseconds / 1000u, nanoseconds % 1000u);
}

static void bus_configure_inputs(void)
{
   // Plain inputs with no pull of our own, so that the tool cannot influence what it is measuring
   configASSERT0(am_hal_gpio_pinconfig(SCL_PIN, am_hal_gpio_pincfg_input));
   configASSERT0(am_hal_gpio_pinconfig(SDA_PIN, am_hal_gpio_pincfg_input));
}

static void bus_configure_open_drain(void)
{
   // Park the output data registers high before the pads start driving, so that switching to open-drain cannot glitch
   // either line low. Input is left enabled because the ACK bit and any clock stretching have to be read back
   am_hal_gpio_pincfg_t open_drain = AM_HAL_GPIO_PINCFG_OPENDRAIN;
   open_drain.GP.cfg_b.eGPInput = AM_HAL_GPIO_PIN_INPUT_ENABLE;
   open_drain.GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X;
   am_hal_gpio_output_set(SCL_PIN);
   am_hal_gpio_output_set(SDA_PIN);
   configASSERT0(am_hal_gpio_pinconfig(SCL_PIN, open_drain));
   configASSERT0(am_hal_gpio_pinconfig(SDA_PIN, open_drain));
}

static bool scl_release_and_wait(void)
{
   // Release the clock and wait for it to actually rise, which both honors clock stretching and detects a line that
   // something else is holding down
   am_hal_gpio_output_set(SCL_PIN);
   for (uint32_t waited = 0; waited < SCL_STRETCH_TIMEOUT_US; ++waited)
   {
      if (am_hal_gpio_input_read(SCL_PIN))
         return true;
      am_hal_delay_us(1);
   }
   return false;
}

static void bus_start(void)
{
   am_hal_gpio_output_set(SDA_PIN);
   am_hal_delay_us(BUS_QUARTER_US);
   scl_release_and_wait();
   am_hal_delay_us(BUS_QUARTER_US);
   am_hal_gpio_output_clear(SDA_PIN);
   am_hal_delay_us(2 * BUS_QUARTER_US);
   am_hal_gpio_output_clear(SCL_PIN);
   am_hal_delay_us(BUS_QUARTER_US);
}

static void bus_stop(void)
{
   am_hal_gpio_output_clear(SDA_PIN);
   am_hal_delay_us(BUS_QUARTER_US);
   scl_release_and_wait();
   am_hal_delay_us(2 * BUS_QUARTER_US);
   am_hal_gpio_output_set(SDA_PIN);
   am_hal_delay_us(2 * BUS_QUARTER_US);
}

static void bus_write_bit(bool value)
{
   if (value)
      am_hal_gpio_output_set(SDA_PIN);
   else
      am_hal_gpio_output_clear(SDA_PIN);
   am_hal_delay_us(BUS_QUARTER_US);
   scl_release_and_wait();
   am_hal_delay_us(2 * BUS_QUARTER_US);
   am_hal_gpio_output_clear(SCL_PIN);
   am_hal_delay_us(BUS_QUARTER_US);
}

static bool bus_read_bit(void)
{
   // Release the data line first so that a slave can drive it, then sample while the clock is high
   am_hal_gpio_output_set(SDA_PIN);
   am_hal_delay_us(BUS_QUARTER_US);
   scl_release_and_wait();
   am_hal_delay_us(BUS_QUARTER_US);
   const bool value = am_hal_gpio_input_read(SDA_PIN) ? true : false;
   am_hal_delay_us(BUS_QUARTER_US);
   am_hal_gpio_output_clear(SCL_PIN);
   am_hal_delay_us(BUS_QUARTER_US);
   return value;
}

static bool bus_write_byte(uint8_t value)
{
   // Returns true if the receiver pulled the data line down during the acknowledge bit
   for (uint32_t bit = 0; bit < 8; ++bit)
      bus_write_bit((value & (0x80u >> bit)) != 0);
   return !bus_read_bit();
}

static bool bus_probe_address(uint8_t address)
{
   bus_start();
   const bool acknowledged = bus_write_byte((uint8_t)(address << 1));
   bus_stop();
   return acknowledged;
}

static void report_drive_check(void)
{
   // Drive each line low on its own and read both back. This is the one test in the file that needs no analyzer and
   // no cooperating master, and it separates three different faults from each other
   uint32_t scl_idle, sda_idle, scl_when_scl_driven, sda_when_scl_driven, scl_when_sda_driven, sda_when_sda_driven;

   am_hal_gpio_output_set(SCL_PIN);
   am_hal_gpio_output_set(SDA_PIN);
   am_hal_delay_us(200);
   scl_idle = am_hal_gpio_input_read(SCL_PIN);
   sda_idle = am_hal_gpio_input_read(SDA_PIN);

   am_hal_gpio_output_clear(SCL_PIN);
   am_hal_delay_us(200);
   scl_when_scl_driven = am_hal_gpio_input_read(SCL_PIN);
   sda_when_scl_driven = am_hal_gpio_input_read(SDA_PIN);
   am_hal_gpio_output_set(SCL_PIN);
   am_hal_delay_us(200);

   am_hal_gpio_output_clear(SDA_PIN);
   am_hal_delay_us(200);
   scl_when_sda_driven = am_hal_gpio_input_read(SCL_PIN);
   sda_when_sda_driven = am_hal_gpio_input_read(SDA_PIN);
   am_hal_gpio_output_set(SDA_PIN);
   am_hal_delay_us(200);

   print("   Both released:      SCL = %u, SDA = %u%s\n", scl_idle, sda_idle,
         (scl_idle && sda_idle) ? "" : "   <-- a released line is not being pulled up");
   print("   GPIO %u driven low:  SCL = %u, SDA = %u%s\n", (uint32_t)SCL_PIN, scl_when_scl_driven, sda_when_scl_driven,
         scl_when_scl_driven ? "   <-- the pad cannot pull its own line down" :
         !sda_when_scl_driven ? "   <-- SDA followed SCL down, the two lines are shorted together" : "");
   print("   GPIO %u driven low:  SCL = %u, SDA = %u%s\n", (uint32_t)SDA_PIN, scl_when_sda_driven, sda_when_sda_driven,
         sda_when_sda_driven ? "   <-- the pad cannot pull its own line down" :
         !scl_when_sda_driven ? "   <-- SCL followed SDA down, the two lines are shorted together" : "");
   if (scl_idle && sda_idle && !scl_when_scl_driven && sda_when_scl_driven && scl_when_sda_driven && !sda_when_sda_driven)
      print("   Both pads drive and release independently, so the pins themselves are healthy\n");
   else
      print("   The pins are NOT behaving as two independent open-drain lines; fix this before reading anything below\n");
}

static void emit_line_identification(void)
{
   // Park the line that is not being pulsed at low, which keeps each burst inert as far as any I2C decoder is
   // concerned, and leave both lines high between the bursts so that the two halves are easy to tell apart
   for (uint32_t repeat = 0; repeat < LINE_ID_REPEATS; ++repeat)
   {
      am_hal_gpio_output_clear(SDA_PIN);
      am_hal_delay_us(LINE_ID_PULSE_US);
      for (uint32_t pulse = 0; pulse < LINE_ID_SCL_PULSES; ++pulse)
      {
         am_hal_gpio_output_clear(SCL_PIN);
         am_hal_delay_us(LINE_ID_PULSE_US);
         am_hal_gpio_output_set(SCL_PIN);
         am_hal_delay_us(LINE_ID_PULSE_US);
      }
      am_hal_gpio_output_set(SDA_PIN);
      am_hal_delay_us(LINE_ID_PHASE_GAP_US);

      am_hal_gpio_output_clear(SCL_PIN);
      am_hal_delay_us(LINE_ID_PULSE_US);
      for (uint32_t pulse = 0; pulse < LINE_ID_SDA_PULSES; ++pulse)
      {
         am_hal_gpio_output_clear(SDA_PIN);
         am_hal_delay_us(LINE_ID_PULSE_US);
         am_hal_gpio_output_set(SDA_PIN);
         am_hal_delay_us(LINE_ID_PULSE_US);
      }
      am_hal_gpio_output_set(SCL_PIN);
      am_hal_delay_us(LINE_ID_REPEAT_GAP_US);
   }
}

static void run_address_scan(void)
{
   // Sweep the whole 7-bit space, skipping the reserved addresses at either end
   uint32_t found = 0;
   for (uint32_t address = 0x08; address <= 0x77; ++address)
      if (bus_probe_address((uint8_t)address))
      {
         print("   Address 0x%02X acknowledged\n", address);
         ++found;
      }
   if (!found)
      print("   Nothing on the bus acknowledged any address from 0x08 to 0x77\n");
   print("   Remember that this board's own IO Slave is disabled here, so 0x%02X is not expected to answer\n",
         (uint32_t)EXT_HW_I2C_ADDRESS);
}

static void run_master_probe(void)
{
   print("\n[PROBE] Driving the bus as a master. Arm the analyzer for at least a 2 ms window.\n");
   bus_configure_open_drain();

   print("[PROBE] Phase 1, drive check:\n");
   report_drive_check();

   print("[PROBE] Phase 2, line identity: %u pulses on GPIO %u, an idle gap, then %u pulses on GPIO %u, repeated %u times\n",
         (uint32_t)LINE_ID_SCL_PULSES, (uint32_t)SCL_PIN, (uint32_t)LINE_ID_SDA_PULSES, (uint32_t)SDA_PIN,
         (uint32_t)LINE_ID_REPEATS);
   print("        The analyzer channel showing %u pulses FIRST is GPIO %u and must be the one probing SCL\n",
         (uint32_t)LINE_ID_SCL_PULSES, (uint32_t)SCL_PIN);
   emit_line_identification();

   print("[PROBE] Phase 3, known frame: write 0x00 0xA0 to address 0x%02X at roughly %u kHz\n",
         (uint32_t)EXT_HW_I2C_ADDRESS, (uint32_t)(1000u / (4u * BUS_QUARTER_US)));
   bus_start();
   const bool address_acked = bus_write_byte((uint8_t)(EXT_HW_I2C_ADDRESS << 1));
   const bool offset_acked = bus_write_byte(0x00);
   const bool data_acked = bus_write_byte(0xA0);
   bus_stop();
   print("        Address %s, offset byte %s, data byte %s\n", address_acked ? "ACKed" : "NACKed",
         offset_acked ? "ACKed" : "NACKed", data_acked ? "ACKed" : "NACKed");
   print("        The analyzer must decode this as a write to 0x%02X; if it does not, it is not probing these two pins\n",
         (uint32_t)EXT_HW_I2C_ADDRESS);

   print("[PROBE] Phase 4, address scan:\n");
   run_address_scan();

   bus_configure_inputs();
   print("[PROBE] Done, both pins released back to inputs\n\n");
}

static uint32_t capture_burst(void)
{
   // Poll both lines as fast as the core allows and timestamp every change. At the default core clock this samples
   // well above 1 MHz, which is more than an order of magnitude faster than any legal I2C rate, so no edge that
   // matters can be missed. Nothing may print from in here
   const uint32_t idle_cycles = cycles_per_us * SNIFFER_IDLE_GAP_US;
   const uint32_t arm_cycles = cycles_per_us * 1000u * SNIFFER_ARM_TIMEOUT_MS;

   // Wait for the bus to move with interrupts still live, so that a quiet bus does not lock the rest of the system
   // out for a second at a time
   uint32_t last = GPIO->RD0 & (SCL_MASK | SDA_MASK), level;
   const uint32_t armed_at = DWT->CYCCNT;
   capture_initial_level = last;
   for (;;)
   {
      level = GPIO->RD0 & (SCL_MASK | SDA_MASK);
      if (level != last)
         break;
      if ((DWT->CYCCNT - armed_at) > arm_cycles)
         return 0;
   }

   // The burst is live from here on, and an interrupt taken now could hide a transition completely
   const uint32_t glitch_cycles = (cycles_per_us * GLITCH_FILTER_NS) / 1000u;
   const uint32_t interrupt_state = am_hal_interrupt_master_disable();
   uint32_t last_edge = DWT->CYCCNT, count = 1;
   capture_cycle[0] = last_edge;
   capture_level[0] = level;
   last = level;

   // A change is only believed once it has held for the glitch window. The edge is timestamped from the first sample
   // that saw the new level, so debouncing costs nothing in timing accuracy
   uint32_t candidate = level, candidate_time = last_edge;
   while (count < SNIFFER_MAX_EDGES)
   {
      const uint32_t sample = GPIO->RD0 & (SCL_MASK | SDA_MASK);
      const uint32_t now = DWT->CYCCNT;
      if (sample != candidate)
      {
         candidate = sample;
         candidate_time = now;
      }
      else if (candidate != last)
      {
         if ((now - candidate_time) >= glitch_cycles)
         {
            capture_cycle[count] = candidate_time;
            capture_level[count] = candidate;
            ++count;
            last = candidate;
            last_edge = now;
         }
      }
      else if ((now - last_edge) > idle_cycles)
         break;   // the bus has gone quiet, so the burst is over
   }
   am_hal_interrupt_master_set(interrupt_state);
   return count;
}

static void decode_capture(uint32_t count, uint32_t clock_mask, uint32_t data_mask, bool verbose, decode_stats_t *stats)
{
   // Walk the captured transitions as an I2C bus. Which physical line plays which role is the caller's choice, which
   // is the whole point: running this twice with the roles reversed is what identifies a swap
   memset(stats, 0, sizeof(*stats));
   bool clock = (capture_initial_level & clock_mask) != 0, data = (capture_initial_level & data_mask) != 0;
   bool in_frame = false, printing = false;
   uint32_t bit_count = 0, shift_register = 0, frame_bytes = 0, frame_index = 0;
   for (uint32_t i = 0; i < count; ++i)
   {
      const bool new_clock = (capture_level[i] & clock_mask) != 0;
      const bool new_data = (capture_level[i] & data_mask) != 0;
      if ((new_clock != clock) && (new_data != data))
         ++stats->simultaneous;

      if (clock && new_clock && (new_data != data))
      {
         // A data-line transition while the clock is high is a START or a STOP, never a data bit
         if (!new_data)
         {
            ++stats->starts;
            if (in_frame)
            {
               ++stats->restarts;
               if (printing)
                  print(", RESTART");
            }
            else
            {
               ++frame_index;
               printing = verbose && (frame_index <= SNIFFER_MAX_PRINTED_FRAMES);
               if (printing)
                  print("      Frame %u: START", frame_index);
            }
            in_frame = true;
            bit_count = shift_register = frame_bytes = 0;
         }
         else
         {
            ++stats->stops;
            if (in_frame)
            {
               if (bit_count)
                  ++stats->truncated_frames;
               else
                  ++stats->complete_frames;
               if (printing)
                  print("%s, STOP\n", bit_count ? ", (truncated mid-byte)" : "");
            }
            in_frame = false;
            printing = false;
            bit_count = shift_register = 0;
         }
      }
      else if (!clock && new_clock && in_frame)
      {
         // Data is valid on the rising clock edge; the ninth bit of every group is the acknowledge
         if (bit_count < 8)
         {
            shift_register = (shift_register << 1) | (new_data ? 1u : 0u);
            ++bit_count;
         }
         else
         {
            ++stats->bytes;
            if (new_data)
               ++stats->nacks;
            else
               ++stats->acks;
            if (!frame_bytes)
            {
               stats->last_address = shift_register >> 1;
               if (!new_data)
                  ++stats->address_acks;
            }
            if (printing)
            {
               if (!frame_bytes)
                  print(", addr 0x%02X %s %s", shift_register >> 1, (shift_register & 1u) ? "read" : "write",
                        new_data ? "NACK" : "ACK");
               else
                  print(", data 0x%02X %s", shift_register, new_data ? "NACK" : "ACK");
            }
            ++frame_bytes;
            bit_count = shift_register = 0;
         }
      }
      clock = new_clock;
      data = new_data;
   }
   if (in_frame && printing)
      print(", (no STOP before the bus went idle)\n");
}

static uint32_t shortest_clock_period(uint32_t count, uint32_t clock_mask)
{
   // The gap between consecutive clock rising edges, taken at its minimum, is the master's peak bit rate. An average
   // would be dragged down by the pauses between bytes and between frames
   uint32_t shortest = 0, previous_rise = 0;
   bool clock = (capture_initial_level & clock_mask) != 0, have_previous = false;
   for (uint32_t i = 0; i < count; ++i)
   {
      const bool new_clock = (capture_level[i] & clock_mask) != 0;
      if (!clock && new_clock)
      {
         if (have_previous)
         {
            const uint32_t period = capture_cycle[i] - previous_rise;
            if (!shortest || (period < shortest))
               shortest = period;
         }
         previous_rise = capture_cycle[i];
         have_previous = true;
      }
      clock = new_clock;
   }
   return shortest;
}

static uint32_t count_line_edges(uint32_t count, uint32_t mask)
{
   uint32_t edges = 0;
   bool level = (capture_initial_level & mask) != 0;
   for (uint32_t i = 0; i < count; ++i)
   {
      const bool new_level = (capture_level[i] & mask) != 0;
      if (new_level != level)
         ++edges;
      level = new_level;
   }
   return edges;
}

static void print_raw_edges(uint32_t count)
{
   const uint32_t shown = (count < SNIFFER_PRINT_EDGES) ? count : SNIFFER_PRINT_EDGES;
   print("   First %u transitions, timed from the first edge (GPIO %u = SCL as wired, GPIO %u = SDA as wired):\n",
         shown, (uint32_t)SCL_PIN, (uint32_t)SDA_PIN);
   for (uint32_t i = 0; i < shown; ++i)
   {
      print("      %3u  ", i);
      print_microseconds(capture_cycle[i] - capture_cycle[0]);
      print(" us   GPIO %u = %u, GPIO %u = %u\n", (uint32_t)SCL_PIN, (capture_level[i] & SCL_MASK) ? 1u : 0u,
            (uint32_t)SDA_PIN, (capture_level[i] & SDA_MASK) ? 1u : 0u);
   }
   if (count > shown)
      print("      ... %u more\n", count - shown);
}

static void report_capture(uint32_t count)
{
   // Summarize the raw capture before interpreting it, so that the numbers stay trustworthy even if both decodes fail
   const uint32_t scl_edges = count_line_edges(count, SCL_MASK), sda_edges = count_line_edges(count, SDA_MASK);
   print("[SNIFF] Captured %u transitions over ", count);
   print_microseconds(capture_cycle[count - 1] - capture_cycle[0]);
   print(" us%s\n", (count >= SNIFFER_MAX_EDGES) ? " (buffer filled, the burst may have been cut short)" : "");
   print("   GPIO %u (wired as SCL): %u edges      GPIO %u (wired as SDA): %u edges\n",
         (uint32_t)SCL_PIN, scl_edges, (uint32_t)SDA_PIN, sda_edges);
   print("   The clock line carries far more edges than the data line, so the busier line above is the clock\n");
   if (SNIFFER_PRINT_EDGES)
      print_raw_edges(count);

   // Decode twice, once for each way round the two signals could be
   decode_stats_t as_wired, as_swapped;
   decode_capture(count, SCL_MASK, SDA_MASK, false, &as_wired);
   decode_capture(count, SDA_MASK, SCL_MASK, false, &as_swapped);
   print("   Decoded assuming GPIO %u = clock: %u START, %u STOP, %u bytes (%u ACK / %u NACK), %u complete frames\n",
         (uint32_t)SCL_PIN, as_wired.starts, as_wired.stops, as_wired.bytes, as_wired.acks, as_wired.nacks,
         as_wired.complete_frames);
   print("   Decoded assuming GPIO %u = clock: %u START, %u STOP, %u bytes (%u ACK / %u NACK), %u complete frames\n",
         (uint32_t)SDA_PIN, as_swapped.starts, as_swapped.stops, as_swapped.bytes, as_swapped.acks, as_swapped.nacks,
         as_swapped.complete_frames);
   if (as_wired.simultaneous || as_swapped.simultaneous)
      print("   %u samples caught both lines moving at once, so a few edge orderings are guesses\n",
            (as_wired.simultaneous > as_swapped.simultaneous) ? as_wired.simultaneous : as_swapped.simultaneous);

   // Pick a winner on completed frames first, then on how many bytes reached an acknowledge bit
   const bool wired_wins = (as_wired.complete_frames > as_swapped.complete_frames) ||
                           ((as_wired.complete_frames == as_swapped.complete_frames) && (as_wired.bytes >= as_swapped.bytes));
   const decode_stats_t *winner = wired_wins ? &as_wired : &as_swapped;
   const uint32_t winning_clock_mask = wired_wins ? SCL_MASK : SDA_MASK;
   const uint32_t winning_data_mask = wired_wins ? SDA_MASK : SCL_MASK;
   if (!winner->starts || !winner->bytes)
   {
      print("   VERDICT: neither assignment produced valid I2C, so these edges are not an I2C transfer\n");
      return;
   }

   const uint32_t period = shortest_clock_period(count, winning_clock_mask);
   print("   Frames under the winning assignment (GPIO %u = clock, GPIO %u = data):\n",
         wired_wins ? (uint32_t)SCL_PIN : (uint32_t)SDA_PIN, wired_wins ? (uint32_t)SDA_PIN : (uint32_t)SCL_PIN);
   decode_stats_t printed;
   decode_capture(count, winning_clock_mask, winning_data_mask, true, &printed);
   if (cycle_counter_running && cycles_to_ns(period))
   {
      print("   Peak clock rate %u kHz (shortest period ", 1000000u / cycles_to_ns(period));
      print_microseconds(period);
      print(" us)\n");
   }
   if (wired_wins)
      print("   VERDICT: the bus decodes correctly as wired, so GPIO %u really is SCL and GPIO %u really is SDA\n",
            (uint32_t)SCL_PIN, (uint32_t)SDA_PIN);
   else
      print("   VERDICT: the bus only decodes with the lines REVERSED. SCL and SDA are swapped between the connector\n"
            "            and GPIO %u / GPIO %u, which is why the IO Slave never acknowledges its address\n",
            (uint32_t)SCL_PIN, (uint32_t)SDA_PIN);
}


// Watching the Bus Through the IO Slave's Own Pads ------------------------------------------------------------------------

// Every way CFG_I2CADDR could plausibly be encoded for a 7-bit address, given that the Programmer's Guide describes
// two different schemes. Exactly one of these can be right, and a match names it
static const struct { uint32_t field; const char *description; } address_candidates[] = {
   { EXT_HW_I2C_ADDRESS << 1, "shifted left one, which is what the driver programs today" },
   { EXT_HW_I2C_ADDRESS,      "unshifted, for a comparison against I2CADDR bits 6 through 0" },
   { EXT_HW_I2C_ADDRESS >> 1, "shifted right one, in case something in the path shifts twice" },
};
#define ADDRESS_CANDIDATE_COUNT  (sizeof(address_candidates) / sizeof(address_candidates[0]))

static volatile uint32_t observer_interrupts, observer_sticky_status;

void tracker_isr_observer(uint32_t interrupt_status)
{
   // Overrides the weak no-op in the tracker driver. The driver clears the interrupt status before anything else can
   // read it, so this is the only place an interrupt can be observed at all
   ++observer_interrupts;
   observer_sticky_status |= interrupt_status;
}

static void print_interrupt_names(uint32_t interrupt_status)
{
   // Spell out every flag, because GENAD is the one that matters most here and it is easy to miss in a bitmap
   static const struct { uint32_t mask; const char *name; } flags[] = {
      { AM_HAL_IOS_INT_FSIZE,  "FSIZE"  }, { AM_HAL_IOS_INT_FOVFL,  "FOVFL"  },
      { AM_HAL_IOS_INT_FUNDFL, "FUNDFL" }, { AM_HAL_IOS_INT_FRDERR, "FRDERR" },
      { AM_HAL_IOS_INT_GENAD,  "GENAD"  }, { AM_HAL_IOS_INT_IOINTW, "IOINTW" },
      { AM_HAL_IOS_INT_XCMPWR, "XCMPWR" }, { AM_HAL_IOS_INT_XCMPWF, "XCMPWF" },
      { AM_HAL_IOS_INT_XCMPRR, "XCMPRR" }, { AM_HAL_IOS_INT_XCMPRF, "XCMPRF" } };
   bool first = true;
   for (uint32_t i = 0; i < (sizeof(flags) / sizeof(flags[0])); ++i)
      if (interrupt_status & flags[i].mask)
      {
         print("%s%s", first ? "" : "|", flags[i].name);
         first = false;
      }
   if (first)
      print("none");
}

static void report_ios_configuration(void)
{
   // Report the peripheral and both pads together. A correct CFG proves nothing on its own, because the pad input
   // enable is what gates the wire through to the peripheral
   const uint32_t config = IOSLAVE->CFG;
   print("   CFG     = 0x%08X: interface %s, %s mode, I2CADDR field 0x%03X\n", config,
         (config & IOSLAVE_CFG_IFCEN_Msk) ? "ENABLED" : "DISABLED",
         (config & IOSLAVE_CFG_IFCSEL_Msk) ? "SPI" : "I2C",
         (config & IOSLAVE_CFG_I2CADDR_Msk) >> IOSLAVE_CFG_I2CADDR_Pos);
   const uint32_t pads[2] = { SCL_PIN, SDA_PIN };
   const char *names[2] = { "SCL", "SDA" };
   for (uint32_t i = 0; i < 2; ++i)
   {
      const uint32_t pad_config = (&GPIO->PINCFG0)[pads[i]];
      print("   PINCFG%u = 0x%08X: %s, FNCSEL %u, INPEN %u, PULLCFG %u\n", pads[i], pad_config, names[i],
            (pad_config & GPIO_PINCFG0_FNCSEL0_Msk) >> GPIO_PINCFG0_FNCSEL0_Pos,
            (pad_config & GPIO_PINCFG0_INPEN0_Msk) >> GPIO_PINCFG0_INPEN0_Pos,
            (pad_config & GPIO_PINCFG0_PULLCFG0_Msk) >> GPIO_PINCFG0_PULLCFG0_Pos);
   }

   // Prove that the pads can still be read now that the peripheral owns them, otherwise every capture below is
   // measuring nothing rather than measuring silence
   const uint32_t idle = GPIO->RD0 & (SCL_MASK | SDA_MASK);
   print("   GPIO_RD0 reads SCL = %u, SDA = %u with the bus idle\n", (idle & SCL_MASK) ? 1u : 0u,
         (idle & SDA_MASK) ? 1u : 0u);
   if ((idle & SCL_MASK) && (idle & SDA_MASK))
      print("   Both pads read high through the peripheral's own pin configuration, so the captures below are real\n");
   else
      print("   A pad reads low on an idle bus, so either the readback does not work in this mode or a line is held\n");
}

static void report_ios_liveness(void)
{
   // Everything so far has only proven that the IO Slave's configuration registers latch what is written to them,
   // which a peripheral will do from the bus clock alone. These three checks ask whether the block's own logic is
   // actually running, and they need no bus traffic at all
   print("   DEVPWREN.PWRENIOS = %u, DEVPWRSTATUS.PWRSTIOS = %u%s\n", (uint32_t)PWRCTRL->DEVPWREN_b.PWRENIOS,
         (uint32_t)PWRCTRL->DEVPWRSTATUS_b.PWRSTIOS,
         PWRCTRL->DEVPWRSTATUS_b.PWRSTIOS ? "" : "   <-- the power domain never came up");

   // The LRAM is the peripheral's own memory, mapped straight into the CPU address space
   const uint8_t saved = am_hal_ios_pui8LRAM[REG_LIVENESS_PROBE];
   am_hal_ios_pui8LRAM[REG_LIVENESS_PROBE] = 0xA5;
   const uint8_t first = am_hal_ios_pui8LRAM[REG_LIVENESS_PROBE];
   am_hal_ios_pui8LRAM[REG_LIVENESS_PROBE] = 0x5A;
   const uint8_t second = am_hal_ios_pui8LRAM[REG_LIVENESS_PROBE];
   am_hal_ios_pui8LRAM[REG_LIVENESS_PROBE] = saved;
   const bool lram_alive = (first == 0xA5) && (second == 0x5A);
   print("   LRAM offset 0x%02X: wrote A5 read %02X, wrote 5A read %02X -- %s\n", (uint32_t)REG_LIVENESS_PROBE,
         (uint32_t)first, (uint32_t)second, lram_alive ? "the peripheral's memory is reachable" : "LRAM IS DEAD");

   // FIFOINC is not a storage register. Writing it asks the peripheral's own logic to add that value to FIFOCTR, so
   // a FIFOCTR that moves is proof the block is clocked and executing, and one that does not is proof it is not
   const uint32_t fifo_before = IOSLAVE->FIFOCTR_b.FIFOCTR;
   IOSLAVE->FIFOINC = 5;
   am_hal_delay_us(100);
   const uint32_t fifo_after = IOSLAVE->FIFOCTR_b.FIFOCTR;
   IOSLAVE->FIFOCTR_b.FIFOCTR = fifo_before;
   const bool logic_alive = (fifo_after == (fifo_before + 5));
   print("   FIFOINC = 5 moved FIFOCTR %u -> %u -- %s\n", fifo_before, fifo_after,
         logic_alive ? "the peripheral's logic is running" : "THE PERIPHERAL'S LOGIC IS NOT RUNNING");
   if (logic_alive)
      print("   The block is alive and executing, so a dead receive path is not simply an unclocked peripheral\n");
   else
      print("   The block latches configuration but does not execute it, which is why nothing on the bus reaches it\n");
}

static void apply_address_candidate(uint32_t index)
{
   // The HAL refuses to reconfigure an enabled IO Slave, so drop the enable, move the address, and bring it back.
   // Nothing else in CFG is touched
   IOSLAVE->CFG_b.IFCEN = 0;
   IOSLAVE->CFG_b.I2CADDR = address_candidates[index].field;
   IOSLAVE->CFG_b.IFCEN = 1;
   print("[IOS] Arming CFG_I2CADDR = 0x%03X, %s\n", address_candidates[index].field,
         address_candidates[index].description);
}

static void run_ios_watch_tool(void)
{
   // Bring up the production driver untouched, so that anything proven here is true of the real firmware
   tracker_init();

   // The driver does not enable General Address detection, and it is the only way to make the peripheral react
   // without involving the address comparator at all (Programmer's Guide Section 13.8.10)
   IOSLAVE->INTEN |= AM_HAL_IOS_INT_GENAD;
   system_enable_interrupts(true);
   print("IO Slave brought up by the production tracker_init(), with GENAD additionally enabled:\n");
   report_ios_configuration();
   report_ios_liveness();
   print("[IOS] Have the master write to 0x%02X repeatedly. Each burst reports what the pads received and whether\n"
         "      the peripheral reacted, %u bursts per address encoding.\n",
         (uint32_t)EXT_HW_I2C_ADDRESS, (uint32_t)IOS_BURSTS_PER_CANDIDATE);
   print("[IOS] Also send at least one write to the general call address 0x00, with one data byte. That sets GENAD\n"
         "      regardless of CFG_I2CADDR, so it proves whether the receive path works at all.\n\n");

   uint32_t candidate = 0, bursts_on_candidate = 0, matches_on_candidate = 0;
   apply_address_candidate(candidate);
   while (true)
   {
      // Snapshot the peripheral, capture the bus, then snapshot it again. An interrupt raised during the burst stays
      // pending while the capture holds interrupts off, and is taken as soon as the capture ends
      const uint32_t interrupts_before = observer_interrupts, sticky_before = observer_sticky_status;
      const uint32_t addptr_before = IOSLAVE->ADDPTR_b.ADDPTR;
      IOSLAVE->REGACCINTCLR = IOSLAVE->REGACCINTSTAT;
      const uint32_t count = capture_burst();
      if (count < 2)
      {
         print("[IOS] Bus idle for %u ms (SCL = %u, SDA = %u), nothing to attribute\n",
               (uint32_t)SNIFFER_ARM_TIMEOUT_MS, am_hal_gpio_input_read(SCL_PIN), am_hal_gpio_input_read(SDA_PIN));
         continue;
      }
      am_hal_delay_us(2000);
      const uint32_t interrupts_after = observer_interrupts;
      const uint32_t addptr_after = IOSLAVE->ADDPTR_b.ADDPTR;
      const uint32_t register_access = IOSLAVE->REGACCINTSTAT;

      // What the pads actually received, decoded the way the peripheral is wired to interpret them
      decode_stats_t seen;
      decode_capture(count, SCL_MASK, SDA_MASK, false, &seen);
      print("[IOS] Pads saw %u transitions: %u START, %u STOP, %u bytes, address 0x%02X %s\n", count, seen.starts,
            seen.stops, seen.bytes, seen.last_address, seen.address_acks ? "ACKNOWLEDGED" : "not acknowledged");
      if (!seen.starts || !seen.bytes)
      {
         print("      That is not a decodable I2C frame, so this burst says nothing about the address encoding\n");
         continue;
      }
      const uint32_t new_flags = observer_sticky_status & ~sticky_before;
      print("      Peripheral reaction: %u interrupts (this burst ", interrupts_after - interrupts_before);
      print_interrupt_names(new_flags);
      print(", ever ");
      print_interrupt_names(observer_sticky_status);
      print("), ADDPTR 0x%02X -> 0x%02X, REGACCINTSTAT 0x%08X, GENADD 0x%02X\n", addptr_before, addptr_after,
            register_access, (uint32_t)IOSLAVE->GENADD_b.GADATA);

      // Only the peripheral's own registers are trusted here. The acknowledge bit decoded off the wire is not
      // proof: a single spurious edge shifts the whole frame by one bit and turns the read/write bit into what
      // looks like an acknowledgement, so a wire ACK with no register movement means the capture is corrupt
      const bool matched = (interrupts_after != interrupts_before) || register_access ||
                           (addptr_after != addptr_before);
      if (seen.address_acks && !matched)
         print("      The wire looked acknowledged but no register moved, so that ACK is a capture artifact.\n"
               "      Raise GLITCH_FILTER_NS if this keeps happening; a clean frame here is 28 transitions\n");
      // A general call reaches the peripheral without the address comparator, so it says something quite different
      // from an address match and must not be counted as one
      if (new_flags & AM_HAL_IOS_INT_GENAD)
      {
         print("      GENAD is set, so the I2C receive state machine works and only the address match is failing.\n"
               "      That makes CFG_I2CADDR the remaining suspect, not the peripheral or the pads\n");
         led_toggle(LED_GREEN);
         continue;
      }
      if (matched)
      {
         ++matches_on_candidate;
         print("      MATCH: CFG_I2CADDR = 0x%03X (%s) is the encoding this silicon wants for address 0x%02X\n",
               address_candidates[candidate].field, address_candidates[candidate].description, seen.last_address);
         led_toggle(LED_GREEN);
      }
      else if (seen.last_address == 0x00)
         print("      A general call reached the pads but GENAD did not set, so the receive path is not running\n");
      else
         print("      No reaction, so CFG_I2CADDR = 0x%03X does not match an address of 0x%02X on this silicon\n",
               address_candidates[candidate].field, seen.last_address);

      // Move to the next encoding once this one has had its turn. A general call is not attributable to an
      // encoding, so those bursts are skipped above rather than spending one of this candidate's turns
      if (++bursts_on_candidate >= IOS_BURSTS_PER_CANDIDATE)
      {
         if (matches_on_candidate)
            print("[IOS] Encoding 0x%03X matched %u of %u bursts; stopping here so the result stays on screen\n",
                  address_candidates[candidate].field, matches_on_candidate, bursts_on_candidate);
         else
         {
            candidate = (candidate + 1) % ADDRESS_CANDIDATE_COUNT;
            bursts_on_candidate = matches_on_candidate = 0;
            if (!candidate)
               print("[IOS] Every encoding has been tried without a match. The pads receive the frame and the\n"
                     "      peripheral ignores it, so the address comparison is not what is broken here\n");
            apply_address_candidate(candidate);
         }
      }
   }
}


// Main Application ------------------------------------------------------------------------------------------------------

int main(void)
{
   // Set up system hardware. The IO Slave is deliberately never initialized, so these two pins stay ordinary GPIOs
   setup_hardware();
   leds_init();
   timing_init();
   print("\nTracker I2C bus analyzer, version %u\n", (uint32_t)TEST_APP_VERSION);
   print("SCL is GPIO %u and SDA is GPIO %u, as declared by this board's pinout\n", (uint32_t)SCL_PIN, (uint32_t)SDA_PIN);
   if (cycle_counter_running)
      print("Cycle counter runs at %u MHz, so the sniffer samples the bus every few hundred nanoseconds\n", cycles_per_us);
   else
      print("Cycle counter is NOT running: frames still decode, but every reported time and frequency is meaningless\n");
   print("The IO Slave is disabled in this application, so this board will not acknowledge 0x%02X while it runs\n",
         (uint32_t)EXT_HW_I2C_ADDRESS);

   // Stage 3 hands both pads to the peripheral, so it cannot coexist with the pin-level stages above
   if (WATCH_THROUGH_IOS_PADS)
      run_ios_watch_tool();

   bus_configure_inputs();
   if (RUN_MASTER_PROBE)
      run_master_probe();

   // Listen to the bus for as long as the test runs, re-probing only while nothing else is using it
   print("[SNIFF] Listening. Drive a transfer from the Tracker's I2C Test Menu now.\n");
   uint32_t quiet_windows = 0;
   while (true)
   {
      const uint32_t count = capture_burst();
      if (count < 2)
      {
         // Two transitions is the least that could carry any meaning; anything less is noise or an idle bus
         ++quiet_windows;
         if (count)
            print("[SNIFF] A single transition arrived and nothing followed it, which is noise rather than a transfer\n");
         else
            print("[SNIFF] Bus idle for %u ms (SCL = %u, SDA = %u)\n", (uint32_t)SNIFFER_ARM_TIMEOUT_MS,
                  am_hal_gpio_input_read(SCL_PIN), am_hal_gpio_input_read(SDA_PIN));
         if (RUN_MASTER_PROBE && MASTER_PROBE_INTERVAL_SEC &&
             (quiet_windows >= ((MASTER_PROBE_INTERVAL_SEC * 1000u) / SNIFFER_ARM_TIMEOUT_MS)))
         {
            quiet_windows = 0;
            run_master_probe();
         }
         continue;
      }
      quiet_windows = 0;
      report_capture(count);
      led_toggle(LED_GREEN);
   }

   // Should never reach this point
   return 0;
}
