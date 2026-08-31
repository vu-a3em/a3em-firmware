// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "led.h"
#include "logging.h"
#include "system.h"
#include "tracker.h"


// Test Application Overview -------------------------------------------------------------------------------------------
//
// The other half of the Tracker I2C bring-up rig. This application turns a board into the I2C master that test_tracker2.c
// and test_tracker_bus.c have been waiting for, so that both ends of the link are running known code on known silicon.
// It exists to answer the question the A3EM hardware cannot: does the Apollo4 IO Slave work at all when everything
// around it is under our control?
//
// Run it on one board and test_tracker2.c on a second. Because the IO Slave is hard-wired to pads 0 and 1, the slave
// board has no choice of pins; this board can use any IO Master, and defaults to the one the IMU driver uses.
//
//
// Wiring, all three connections required:
//
//    this board GPIO 31 (SCL)  ->  slave board GPIO 0 (SLSCL)
//    this board GPIO 32 (SDA)  ->  slave board GPIO 1 (SLSDAWIR3)
//    this board GND            ->  slave board GND
//
// The exact pin numbers are printed at start-up, so trust the console over this comment if the defines below move.
//
// Do not add external pull-ups. This board's IOM pads carry 1.5K internal pull-ups, which is the whole bus supply:
// Table 19 of Programmer's Guide Section 9.3.1.6 requires PULLCFG cleared on the slave's pads, so the slave contributes
// nothing. Both boards must run at the same pad voltage, which is why this is an EVB-to-EVB test rather than an
// A3EM-to-EVB one.
//
//
// What it does, in two stages, either of which can be compiled out:
//
//    Conformance    Walks the bring-up sequence from the header of test_tracker2.c and checks every answer against the
//                   value that file publishes, so the result is a pass or fail count rather than a hex dump to read by
//                   eye. It covers the address scan, the magic window, both ramps, the heartbeat, a register write with
//                   read-back of the counters, a burst write with read-back of the echo area, and the FIFO port.
//
//    Stimulus       Repeats a single-byte write to the slave address forever, with a general call every few passes.
//                   This is what test_tracker_bus.c Stage 3 wants on the other end: steady, predictable traffic to
//                   attribute its captures to, without anyone having to click send.
//
// Every transfer reports whether it was acknowledged, so this is also a usable bus prodder on its own. An address scan
// runs first in both stages, and if the slave does not answer it says so and stops rather than reporting a cascade of
// failures that all have the same cause.
//
//
// For a clean conformance run, set AUTO_REFILL_FIFO to 0 in test_tracker2.c. Left at 1, that file keeps a 19-byte ramp
// permanently queued and re-queues it once a second, which is deliberate and useful when only the read path works, but
// it makes the FIFO depth checks here racy. The ramp contents are checked either way, because the automatic refill and
// the 0xA1 command queue the same pattern.


// Static Global Variables and Definitions -------------------------------------------------------------------------------

#define TEST_APP_VERSION             0x01

// This board's master pins, aliased to the digipot bus: IO Master 3 on GPIO 31 and GPIO 32, which are broken out on
// the Apollo4 Plus EVB where the IO Master 0 pair is not. Any IOM I2C pad pair works, so change these four lines to
// move the master; the slave has no such freedom because the IO Slave is hard-wired to pads 0 and 1
#define MASTER_I2C_NUMBER            DIGIPOT_I2C_NUMBER
#define PIN_MASTER_I2C_SCL           PIN_DIGIPOT_I2C_SCL
#define PIN_MASTER_I2C_SDA           PIN_DIGIPOT_I2C_SDA
#define PIN_MASTER_I2C_SCL_FUNCTION  PIN_DIGIPOT_I2C_SCL_FUNCTION
#define PIN_MASTER_I2C_SDA_FUNCTION  PIN_DIGIPOT_I2C_SDA_FUNCTION

// 100 kHz keeps a wide margin on a jumpered bus and stays under the 500 kHz floor that erratum ERR066 imposes on the
// slave's FIFO mode
#define MASTER_I2C_FREQUENCY         AM_HAL_IOM_100KHZ
#define SLAVE_ADDRESS                EXT_HW_I2C_ADDRESS

#define RUN_CONFORMANCE_SEQUENCE     1
#define RUN_STIMULUS_LOOP            1

// test_tracker2.c services its capture queue and republishes the status window from its main loop every 50 ms, so a
// read-back has to give it a few passes to catch up
#define SLAVE_PROCESS_DELAY_MS       200
#define HEARTBEAT_WAIT_MS            1500
#define STIMULUS_INTERVAL_MS         500
#define STIMULUS_GENERAL_CALL_EVERY  8
#define TRANSFER_MAX_BYTES           64

// The status window published by test_tracker2.c, which is what every conformance check below is measured against
#define REG_MAGIC                    0x40
#define REG_VERSION                  0x44
#define REG_I2C_ADDRESS              0x45
#define REG_HEARTBEAT                0x46
#define REG_WRITE_COUNT              0x47
#define REG_LAST_MESSAGE_TYPE        0x48
#define REG_COMMAND_COUNT            0x4C
#define REG_LAST_COMMAND             0x4D
#define REG_SENTINEL_A               0x4E
#define REG_SENTINEL_B               0x4F
#define REG_RAMP                     0x50
#define REG_INVERSE_RAMP             0x60
#define REG_ECHO                     0x70
#define REG_ECHO_BYTES               8
#define REG_FIFO_COUNT               0x7C
#define REG_FIFO_PORT                0x7F

#define TEST_CMD_PING                0xA0
#define TEST_CMD_SEND_RAMP           0xA1

static void *i2c_handle = NULL;
static uint32_t checks_run, checks_passed;
static union { uint32_t words[TRANSFER_MAX_BYTES / 4]; uint8_t bytes[TRANSFER_MAX_BYTES]; } rx_buffer, tx_buffer;


// Private Helper Functions ----------------------------------------------------------------------------------------------

static bool transfer(uint8_t address, uint32_t instruction_length, uint8_t offset, bool receiving, uint32_t length)
{
   // One blocking I2C transaction. A write sends the offset and the data in a single frame; a read sends the offset,
   // issues a repeated start, and reads back, which is exactly the shape the slave's address pointer expects
   am_hal_iom_transfer_t transaction = {
      .uPeerInfo.ui32I2CDevAddr = address,
      .ui32InstrLen             = instruction_length,
      .ui64Instr                = offset,
      .eDirection               = receiving ? AM_HAL_IOM_RX : AM_HAL_IOM_TX,
      .ui32NumBytes             = length,
      .pui32TxBuffer            = receiving ? NULL : tx_buffer.words,
      .pui32RxBuffer            = receiving ? rx_buffer.words : NULL,
      .bContinue                = false,
      .ui8RepeatCount           = 0,
      .ui8Priority              = 1,
      .ui32PauseCondition       = 0,
      .ui32StatusSetClr         = 0
   };
   return (am_hal_iom_blocking_transfer(i2c_handle, &transaction) == AM_HAL_STATUS_SUCCESS);
}

static bool read_at(uint8_t offset, uint32_t length)
{
   return transfer(SLAVE_ADDRESS, 1, offset, true, length);
}

static bool write_at(uint8_t offset, uint32_t length)
{
   return transfer(SLAVE_ADDRESS, 1, offset, false, length);
}

static bool probe_address(uint8_t address)
{
   // A one-byte read with no offset phase, which is the least intrusive way to find out whether anything answers
   return transfer(address, 0, 0, true, 1);
}

static void record(const char *name, bool passed)
{
   ++checks_run;
   if (passed)
      ++checks_passed;
   print("   [%s] %s\n", passed ? "PASS" : "FAIL", name);
}

static void print_hex_bytes(const uint8_t *bytes, uint32_t length)
{
   for (uint32_t i = 0; i < length; ++i)
      print("%02X ", bytes[i]);
}

static bool bytes_match(const uint8_t *actual, const uint8_t *expected, uint32_t length)
{
   for (uint32_t i = 0; i < length; ++i)
      if (actual[i] != expected[i])
         return false;
   return true;
}

static bool scan_for_slave(void)
{
   // Sweep the whole 7-bit space, skipping the reserved addresses at either end
   uint32_t found = 0;
   bool slave_present = false;
   print("Scanning the bus:\n");
   for (uint32_t address = 0x08; address <= 0x77; ++address)
      if (probe_address((uint8_t)address))
      {
         print("   Address 0x%02X answered%s\n", address, (address == SLAVE_ADDRESS) ? "   <-- the slave under test" : "");
         slave_present = slave_present || (address == SLAVE_ADDRESS);
         ++found;
      }
   if (!found)
      print("   Nothing answered anywhere between 0x08 and 0x77\n");
   record("slave acknowledges its address", slave_present);
   return slave_present;
}

static void check_status_window(void)
{
   // The fixed portion first, because it is the part that cannot be wrong for any reason other than a broken link
   if (!read_at(REG_MAGIC, 16))
   {
      record("read 16 bytes from the status window at 0x40", false);
      return;
   }
   print("   0x40: ");
   print_hex_bytes(rx_buffer.bytes, 16);
   print("\n");
   static const uint8_t magic[] = { 'A', '3', 'E', 'M' };
   record("magic value at 0x40 reads A3EM", bytes_match(rx_buffer.bytes, magic, sizeof(magic)));
   record("slave reports its own address at 0x45", rx_buffer.bytes[REG_I2C_ADDRESS - REG_MAGIC] == SLAVE_ADDRESS);
   record("sentinel 0x5A at 0x4E", rx_buffer.bytes[REG_SENTINEL_A - REG_MAGIC] == 0x5A);
   record("sentinel 0xA5 at 0x4F", rx_buffer.bytes[REG_SENTINEL_B - REG_MAGIC] == 0xA5);
   print("   Slave is running test application version %u\n", (uint32_t)rx_buffer.bytes[REG_VERSION - REG_MAGIC]);
}

static void check_ramps(void)
{
   // Two known 16-byte patterns, which together prove burst reads and byte ordering across a page boundary
   uint8_t expected[16];
   for (uint32_t i = 0; i < 16; ++i)
      expected[i] = (uint8_t)i;
   if (read_at(REG_RAMP, 16))
   {
      print("   0x50: ");
      print_hex_bytes(rx_buffer.bytes, 16);
      print("\n");
      record("ramp at 0x50 reads 00 through 0F", bytes_match(rx_buffer.bytes, expected, 16));
   }
   else
      record("ramp at 0x50 reads 00 through 0F", false);

   for (uint32_t i = 0; i < 16; ++i)
      expected[i] = (uint8_t)(0xFF - i);
   if (read_at(REG_INVERSE_RAMP, 16))
   {
      print("   0x60: ");
      print_hex_bytes(rx_buffer.bytes, 16);
      print("\n");
      record("inverse ramp at 0x60 reads FF through F0", bytes_match(rx_buffer.bytes, expected, 16));
   }
   else
      record("inverse ramp at 0x60 reads FF through F0", false);
}

static void check_heartbeat(void)
{
   // The only check here that proves the slave's CPU is alive rather than just its LRAM
   if (!read_at(REG_HEARTBEAT, 1))
   {
      record("heartbeat at 0x46 advances", false);
      return;
   }
   const uint8_t first = rx_buffer.bytes[0];
   am_hal_delay_us(HEARTBEAT_WAIT_MS * 1000u);
   if (!read_at(REG_HEARTBEAT, 1))
   {
      record("heartbeat at 0x46 advances", false);
      return;
   }
   print("   Heartbeat read %u, then %u after %u ms\n", (uint32_t)first, (uint32_t)rx_buffer.bytes[0],
         (uint32_t)HEARTBEAT_WAIT_MS);
   record("heartbeat at 0x46 advances", rx_buffer.bytes[0] != first);
}

static void check_register_write(void)
{
   // Send the ping command and confirm the slave both counted the write and decoded the command byte
   if (!read_at(REG_WRITE_COUNT, 1))
   {
      record("host write counter at 0x47 advances", false);
      return;
   }
   const uint8_t writes_before = rx_buffer.bytes[0];

   tx_buffer.bytes[0] = TEST_CMD_PING;
   if (!write_at(0x00, 1))
   {
      record("write of the ping command to offset 0x00", false);
      return;
   }
   record("write of the ping command to offset 0x00", true);
   am_hal_delay_us(SLAVE_PROCESS_DELAY_MS * 1000u);

   if (!read_at(REG_WRITE_COUNT, 1))
   {
      record("host write counter at 0x47 advances", false);
      return;
   }
   const uint8_t writes_after = rx_buffer.bytes[0];
   print("   Write counter went %u -> %u\n", (uint32_t)writes_before, (uint32_t)writes_after);
   record("host write counter at 0x47 advances", writes_after != writes_before);

   if (read_at(REG_LAST_COMMAND, 1))
      record("last command byte at 0x4D reads back 0xA0", rx_buffer.bytes[0] == TEST_CMD_PING);
   else
      record("last command byte at 0x4D reads back 0xA0", false);
   if (read_at(REG_LAST_MESSAGE_TYPE, 1))
      record("first byte of the last write at 0x48 reads back 0xA0", rx_buffer.bytes[0] == TEST_CMD_PING);
   else
      record("first byte of the last write at 0x48 reads back 0xA0", false);
}

static void check_burst_write(void)
{
   // A payload that changes every run, because the slave detects host writes by diffing the LRAM against a shadow
   // copy and would not notice an identical rewrite. The leading byte stays clear of the real message types and the
   // test command range so that the slave stores it without acting on it
   static uint8_t sequence = 0;
   uint8_t written[REG_ECHO_BYTES];
   for (uint32_t i = 0; i < REG_ECHO_BYTES; ++i)
      written[i] = (uint8_t)(0x10 + i + sequence);
   ++sequence;
   memcpy(tx_buffer.bytes, written, sizeof(written));
   if (!write_at(0x00, sizeof(written)))
   {
      record("burst write of 8 bytes to offset 0x00", false);
      return;
   }
   record("burst write of 8 bytes to offset 0x00", true);
   am_hal_delay_us(SLAVE_PROCESS_DELAY_MS * 1000u);

   if (!read_at(REG_ECHO, REG_ECHO_BYTES))
   {
      record("echo area at 0x70 returns the bytes just written", false);
      return;
   }
   print("   Wrote ");
   print_hex_bytes(written, sizeof(written));
   print("and read back ");
   print_hex_bytes(rx_buffer.bytes, REG_ECHO_BYTES);
   print("\n");
   record("echo area at 0x70 returns the bytes just written", bytes_match(rx_buffer.bytes, written, REG_ECHO_BYTES));
}

static uint32_t read_fifo_count(void)
{
   // The slave publishes its FIFO depth as a little-endian pair at 0x7C
   if (!read_at(REG_FIFO_COUNT, 2))
      return 0;
   return (uint32_t)rx_buffer.bytes[0] | ((uint32_t)rx_buffer.bytes[1] << 8);
}

static bool queue_ramp(uint32_t needed)
{
   tx_buffer.bytes[0] = TEST_CMD_SEND_RAMP;
   if (!write_at(0x00, 1))
      return false;
   am_hal_delay_us(SLAVE_PROCESS_DELAY_MS * 1000u);
   return (read_fifo_count() >= needed);
}

static bool pop_ramp(bool repeated_start, uint32_t length)
{
   // Two ways to aim the address pointer at the FIFO port before streaming from it. The first is the obvious one and
   // the one the Tracker's menu uses: a single transaction that writes the offset, issues a repeated START and reads.
   // The second parks the pointer in a transaction of its own and then does a bare read with no offset phase at all
   if (repeated_start)
      return read_at(REG_FIFO_PORT, length);
   tx_buffer.bytes[0] = REG_FIFO_PORT;
   if (!transfer(SLAVE_ADDRESS, 0, 0, false, 1))
      return false;
   return transfer(SLAVE_ADDRESS, 0, 0, true, length);
}

static bool ramp_is_intact(uint32_t length)
{
   uint8_t expected[TRANSFER_MAX_BYTES];
   for (uint32_t i = 0; i < length; ++i)
      expected[i] = (uint8_t)i;
   return bytes_match(rx_buffer.bytes, expected, length);
}

static void check_fifo_port(void)
{
   // Ask for a ramp, then pop it back out of the streaming port. Both the command and test_tracker2.c's automatic
   // refill queue the same pattern, so the contents can be checked without caring which of them supplied it
   const uint32_t ramp_length = sizeof(tracker_alert_data_t);
   if (!queue_ramp(ramp_length))
   {
      record("queue a ramp and see it counted at 0x7C", false);
      return;
   }
   record("queue a ramp and see it counted at 0x7C", true);
   const uint32_t pending = read_fifo_count();
   print("   FIFO reports %u bytes waiting, and one ramp is %u bytes\n", pending, ramp_length);

   // Style one: offset write, repeated START, burst read
   bool repeated_start_clean = false;
   if (pop_ramp(true, ramp_length))
   {
      print("   Offset write plus repeated START: ");
      print_hex_bytes(rx_buffer.bytes, ramp_length);
      print("\n");
      repeated_start_clean = ramp_is_intact(ramp_length);
   }
   print("   FIFO count is now %u\n", read_fifo_count());

   // Style two: park the pointer in its own transaction, then read with no offset phase
   bool bare_read_clean = false;
   if (queue_ramp(ramp_length) && pop_ramp(false, ramp_length))
   {
      print("   Parked pointer plus bare read:    ");
      print_hex_bytes(rx_buffer.bytes, ramp_length);
      print("\n");
      bare_read_clean = ramp_is_intact(ramp_length);
   }
   print("   FIFO count is now %u\n", read_fifo_count());

   record("FIFO port streams the ramp back intact by at least one method", repeated_start_clean || bare_read_clean);
   if (repeated_start_clean != bare_read_clean)
      print("   NOTE: only the %s method returns the ramp intact. The other loses the first byte, because aiming the\n"
            "   pointer at 0x7F pops one byte that the following read never delivers. Whoever writes the host side\n"
            "   has to use the working sequence or account for the lost byte\n",
            repeated_start_clean ? "repeated START" : "parked pointer plus bare read");
}

static void check_bare_read(void)
{
   // The connection test from the Tracker's own menu: a read with no offset phase, which returns whatever the address
   // pointer happens to be aimed at and then advances it. Informational, because its result depends on history
   if (transfer(SLAVE_ADDRESS, 0, 0, true, 1))
      print("   A bare one-byte read with no offset returned 0x%02X\n", (uint32_t)rx_buffer.bytes[0]);
   else
      print("   A bare one-byte read with no offset was not acknowledged\n");
}

static void run_conformance_sequence(void)
{
   print("\nConformance run against the status window published by test_tracker2.c:\n");
   if (!scan_for_slave())
   {
      print("   The slave never acknowledged, so every check below would fail for the same reason. Stopping.\n");
      print("   Check the three jumpers, and confirm the slave board is running test_tracker2.c.\n");
      return;
   }
   check_status_window();
   check_ramps();
   check_heartbeat();
   check_register_write();
   check_burst_write();
   check_fifo_port();
   check_bare_read();

   print("\n%u of %u checks passed. %s\n", checks_passed, checks_run,
         (checks_passed == checks_run) ? "The IO Slave works end to end on this hardware."
                                       : "See the failures above; the first one is usually the real fault.");
   led_on((checks_passed == checks_run) ? LED_GREEN : LED_RED);
}

static void run_stimulus_loop(void)
{
   // Steady, predictable traffic for test_tracker_bus.c Stage 3 to attribute its captures to
   print("\nStimulus loop: writing to 0x%02X every %u ms, with a general call every %u passes.\n",
         (uint32_t)SLAVE_ADDRESS, (uint32_t)STIMULUS_INTERVAL_MS, (uint32_t)STIMULUS_GENERAL_CALL_EVERY);
   uint32_t pass = 0, acknowledged = 0;
   while (true)
   {
      ++pass;
      bool general_call = false, succeeded;
      if (STIMULUS_GENERAL_CALL_EVERY && !(pass % STIMULUS_GENERAL_CALL_EVERY))
      {
         // A general call reaches the slave without involving its address comparator at all
         general_call = true;
         tx_buffer.bytes[0] = 0x00;
         succeeded = transfer(0x00, 0, 0, false, 1);
      }
      else
      {
         tx_buffer.bytes[0] = TEST_CMD_PING;
         succeeded = write_at(0x00, 1);
      }
      if (succeeded)
         ++acknowledged;
      print("[MASTER] Pass %u: %s %s (%u of %u acknowledged so far)\n", pass,
            general_call ? "general call to 0x00" : "write of 0xA0 to offset 0x00",
            succeeded ? "ACKNOWLEDGED" : "not acknowledged", acknowledged, pass);
      led_toggle(LED_GREEN);
      am_hal_delay_us(STIMULUS_INTERVAL_MS * 1000u);
   }
}


// Main Application ------------------------------------------------------------------------------------------------------

int main(void)
{
   setup_hardware();
   leds_init();
   print("\nTracker I2C master, version %u\n", (uint32_t)TEST_APP_VERSION);
   print("Wire this board to the slave board, all three connections:\n");
   print("   GPIO %u (SCL) -> slave GPIO %u,   GPIO %u (SDA) -> slave GPIO %u,   GND -> GND\n",
         (uint32_t)PIN_MASTER_I2C_SCL, (uint32_t)PIN_EXT_HW_I2C_SCL,
         (uint32_t)PIN_MASTER_I2C_SDA, (uint32_t)PIN_EXT_HW_I2C_SDA);
   print("This board supplies the bus pull-ups, so do not add any; the slave's pads must stay unpulled\n");

   // Bring up the IO Master. The pull-ups come from the BSP pin configuration and are the only ones on the bus
   const am_hal_iom_config_t i2c_config =
   {
      .eInterfaceMode = AM_HAL_IOM_I2C_MODE,
      .ui32ClockFreq = MASTER_I2C_FREQUENCY,
      .eSpiMode = 0,
      .pNBTxnBuf = NULL,
      .ui32NBTxnBufLength = 0
   };
   am_hal_gpio_pincfg_t scl_config = g_AM_BSP_GPIO_IOM0_SCL;
   am_hal_gpio_pincfg_t sda_config = g_AM_BSP_GPIO_IOM0_SDA;
   scl_config.GP.cfg_b.uFuncSel = PIN_MASTER_I2C_SCL_FUNCTION;
   sda_config.GP.cfg_b.uFuncSel = PIN_MASTER_I2C_SDA_FUNCTION;
   scl_config.GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
   sda_config.GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
   configASSERT0(am_hal_iom_initialize(MASTER_I2C_NUMBER, &i2c_handle));
   configASSERT0(am_hal_gpio_pinconfig(PIN_MASTER_I2C_SCL, scl_config));
   configASSERT0(am_hal_gpio_pinconfig(PIN_MASTER_I2C_SDA, sda_config));
   configASSERT0(am_hal_iom_power_ctrl(i2c_handle, AM_HAL_SYSCTRL_WAKE, false));
   configASSERT0(am_hal_iom_configure(i2c_handle, &i2c_config));
   configASSERT0(am_hal_iom_enable(i2c_handle));
   print("IO Master %u is up at %u kHz on GPIO %u and GPIO %u\n\n", (uint32_t)MASTER_I2C_NUMBER,
         (uint32_t)(MASTER_I2C_FREQUENCY / 1000u), (uint32_t)PIN_MASTER_I2C_SCL, (uint32_t)PIN_MASTER_I2C_SDA);

   if (RUN_CONFORMANCE_SEQUENCE)
      run_conformance_sequence();
   if (RUN_STIMULUS_LOOP)
      run_stimulus_loop();

   // With the stimulus loop compiled out there is nothing left to do but hold the conformance result on the LEDs
   print("\nDone.\n");
   while (true)
      am_hal_delay_us(1000000);
   return 0;
}
