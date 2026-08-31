// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "led.h"
#include "logging.h"
#include "system.h"
#include "tracker.h"


// Test Application Overview -------------------------------------------------------------------------------------------
//
// Interactive I2C bring-up aid for the GPS Tracker link. Unlike test_tracker.c, this application never initiates
// anything on its own: the Tracker board drives every exchange from its serial "I2C Test Menu", and this board simply
// reports what it saw and answers with values that are trivial to verify by eye. This makes it usable while the
// HW interrupt line (PIN_EXT_HW_INTERRUPT) is dead, since nothing here depends on the Tracker being woken up.
//
// The production tracker.c driver is used unmodified, so anything that works here works in the real firmware.
//
//
// Getting to the Tracker-side menu (see Firmware-Flashing-Guide.pdf, Section 1.3):
//
//    t <Enter>      from the main config menu, open the TEST menu
//    v <Enter>      open the I2C Test Menu
//    1 <Enter>      select device 1, "shot-detector" @ 0x58 (that's this board)
//
// The five available master operations, and what this application does with each:
//
//    1              Test Connection    -- see the note below; not the address probe it appears to be
//    2:<reg>        Read Register      -- reads one byte from the status window below
//    3:<reg>,<len>  Read burst         -- reads <len> bytes starting at <reg>
//    4:<reg>,<val>  Write Register     -- writes one byte; use <reg> = 0x00
//    5:<reg>,<data> Write burst        -- writes a whole packet; use <reg> = 0x00
//
// Observed on the bench: option 1 is not an address probe. It issues a bare one-byte read with no offset phase,
// which returns whatever the address pointer happens to point at and then advances it by one. Pressing it
// repeatedly therefore walks the pointer through the LRAM one byte per press. Options 2 through 5 all begin with
// a write phase, and on the hardware tested so far not one of them ever reached this board, so if the four
// register operations report Error while option 1 reports OK, the master's write path is the thing to chase.
//
//
// Slave-to-master status window (CPU-maintained, host-readable at these offsets):
//
//    0x40..0x43   magic 'A' '3' 'E' 'M' (41 33 45 4D)      0x4C   test commands executed
//    0x44         test application version                 0x4D   most recent test command byte
//    0x45         I2C address (0x58)                       0x4E   constant 0x5A
//    0x46         heartbeat, increments once per second    0x4F   constant 0xA5
//    0x47         host write transactions seen             0x50..0x5F   ramp 00 01 02 .. 0F
//    0x48         first byte of the most recent write      0x60..0x6F   inverse ramp FF FE .. F0
//    0x49         ADDPTR after the most recent write       0x70..0x77   first 8 bytes most recently written
//    0x4A         FIFO bytes waiting for the host
//    0x4B         of those, how many are in the HW FIFO
//
// Offsets 0x78-0x7F are the IO Slave's own host-facing registers, not RAM: 0x78-0x7B are the host interrupt
// enable/status/clear registers, 0x7C/0x7D are the low/upper bytes of the FIFO byte count, and 0x7F is the FIFO
// streaming port (reading it repeatedly pops successive bytes without advancing the address pointer).
//
//
// Master-to-slave commands, all written to offset 0x00 so that they arrive exactly where the production driver
// expects a packet. Values 0x01-0x07 are the real tracker_msg_t types and are decoded as such; 0xA0-0xA6 are
// test-only commands:
//
//    4:0x00,0xA0    ping; bumps the counters at 0x47/0x4C so a follow-up read shows the board is alive
//    4:0x00,0xA1    queue 19 ramp bytes (00 01 02 .. 12) in the FIFO -- easiest FIFO read to verify
//    4:0x00,0xA2    queue a real MSG_GPS_REQUEST (one byte, 0x06)
//    4:0x00,0xA3    queue a real MSG_STATUS packet (13 bytes)
//    4:0x00,0xA4    queue a real MSG_CRITICAL_ALERT/ALERT_GUNSHOT packet (19 bytes)
//    4:0x00,0xA5    zero the counters in the status window
//    4:0x00,0xA6    pulse PIN_EXT_HW_INTERRUPT low for 1 ms (for scoping the broken line)
//    4:0x00,0x07    real MSG_STATUS_REQUEST; tracker.c itself answers with a status packet
//
//
// Suggested bring-up sequence:
//
//    1              -> OK confirms the slave is powered, configured, and ACKing at 0x58
//    2:0x40         -> 0x41; the first byte of the magic value, proves reads reach the LRAM
//    3:0x40,16      -> 41 33 45 4D 02 58 hh ww .. proves burst reads and byte ordering
//    2:0x46         -> read twice a second apart; the heartbeat must change
//    3:0x50,16      -> 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
//    4:0x00,0xA0    -> then 2:0x47 and 2:0x48; the count increments and 0x48 reads back 0xA0
//    5:0x00,<data>  -> then 3:0x70,8; the first 8 bytes written come back verbatim
//    4:0x00,0xA1    -> then 3:0x7C,2 shows 13 00 (19 bytes), and 3:0x7F,19 yields 00 01 .. 12
//    4:0x00,0x07    -> then 3:0x7C,2 shows 0D 00 (13 bytes), and 3:0x7F,13 yields a MSG_STATUS packet
//
// If only option 1 works, the read path can still be exercised end to end without it. AUTO_REFILL_FIFO keeps a
// 19-byte ramp permanently queued, so press 1 repeatedly and watch the address pointer climb on this console. Once
// it reaches 0x7F it stops advancing, because the FIFO port does not move the pointer, and every further press
// pops one byte instead: the console then reports the FIFO draining a byte at a time. Reaching that point proves
// address matching, read transactions, LRAM addressing, and the FIFO port all work, leaving only the master's
// inability to display the bytes it received.
//
// Every host write is also decoded on the debug console, so watch that alongside the Tracker terminal. Host reads
// are not logged there: the driver's ISR discards any transaction whose address pointer lands at 0x78 or above,
// which is exactly where a FIFO read at 0x7F leaves it. Reads of the status window do surface, as a line reporting
// that no LRAM bytes changed. Use the counters at 0x47/0x4A instead to confirm that reads are landing.


// Static Global Variables and Definitions -------------------------------------------------------------------------------

#define TEST_APP_VERSION           0x03
#define HOST_WRITE_AREA_BYTES      0x40

// Set to 0 to stop the CPU touching the LRAM entirely after start-up. Erratum ERR066 warns that a CPU access to the
// LRAM concurrent with a master write can corrupt that write, so this exists to rule the status window out as a cause
#define REFRESH_STATUS_WINDOW      1

// Keeps a known packet permanently available in the FIFO so that the host can exercise the FIFO port at 0x7F without
// first having to issue a write, which is useful when only the master's read path is working
#define AUTO_REFILL_FIFO           1

// Parks the address pointer on the FIFO port at start-up so that a bare one-byte host read lands there immediately.
// Tested on Apollo4 Plus silicon and the write does not stick, so this is off; the code stays because it self-reports
#define PARK_POINTER_AT_FIFO_PORT  0
#define FIFO_PORT_OFFSET           0x7F
#define EVENT_QUEUE_LENGTH         16
#define EVENT_BYTES_CAPTURED       24

#define REG_MAGIC                  0x40
#define REG_VERSION                0x44
#define REG_I2C_ADDRESS            0x45
#define REG_HEARTBEAT              0x46
#define REG_WRITE_COUNT            0x47
#define REG_LAST_MESSAGE_TYPE      0x48
#define REG_LAST_ADDPTR            0x49
#define REG_FIFO_PENDING           0x4A
#define REG_FIFO_HW_BYTES          0x4B
#define REG_COMMAND_COUNT          0x4C
#define REG_LAST_COMMAND           0x4D
#define REG_SENTINEL_A             0x4E
#define REG_SENTINEL_B             0x4F
#define REG_RAMP                   0x50
#define REG_INVERSE_RAMP           0x60
#define REG_ECHO                   0x70
#define REG_ECHO_BYTES             8

#define TEST_CMD_PING              0xA0
#define TEST_CMD_SEND_RAMP         0xA1
#define TEST_CMD_SEND_GPS_REQUEST  0xA2
#define TEST_CMD_SEND_STATUS       0xA3
#define TEST_CMD_SEND_ALERT        0xA4
#define TEST_CMD_RESET_COUNTERS    0xA5
#define TEST_CMD_PULSE_INTERRUPT   0xA6

// A single host write transaction, captured in the ISR and decoded from the main loop
typedef struct
{
   uint8_t addptr;                         // IOSLAVE ADDPTR after the transaction completed
   uint8_t changed_offset;                 // LRAM offset of the first byte the host actually changed
   uint8_t changed_length;                 // Number of LRAM bytes changed, 0 if the host only set the address
   uint8_t captured_length;                // Number of those bytes stored in 'data' below
   uint8_t data[EVENT_BYTES_CAPTURED];
   uint32_t register_access;               // REGACCINTSTAT bitmap of which direct-area offsets were touched
} host_event_t;

static volatile host_event_t event_queue[EVENT_QUEUE_LENGTH];
static volatile uint32_t event_write_index, event_read_index, events_dropped;
static uint8_t lram_shadow[HOST_WRITE_AREA_BYTES];
static uint32_t write_count, command_count;
static volatile uint8_t last_addptr;
static uint32_t sticky_register_access, last_fifo_count, last_host_interrupts, last_interrupt_pending;
static volatile uint32_t interrupt_status_queue[EVENT_QUEUE_LENGTH];
static volatile uint32_t interrupt_write_index, interrupt_read_index, interrupt_count, sticky_interrupts;


// Private Helper Functions ----------------------------------------------------------------------------------------------

static void status_window_write(uint32_t offset, const void *data, uint32_t length)
{
   // The status window lives in the LRAM direct area, which the CPU may write at any time
   const uint8_t *bytes = (const uint8_t*)data;
   for (uint32_t i = 0; i < length; ++i)
      am_hal_ios_pui8LRAM[offset + i] = bytes[i];
}

static void status_window_set(uint32_t offset, uint8_t value)
{
   am_hal_ios_pui8LRAM[offset] = value;
}

static void status_window_refresh(uint32_t offset, uint8_t value)
{
   // Identical to status_window_set(), but suppressed when the LRAM is being kept quiet after start-up
#if REFRESH_STATUS_WINDOW
   am_hal_ios_pui8LRAM[offset] = value;
#else
   (void)offset;
   (void)value;
#endif
}

static void queue_fifo_ramp(void)
{
   // Push a byte ramp into the FIFO. The tracker driver only exposes typed senders, so an alert structure is reused
   // purely as a fixed-size byte container here
   tracker_alert_data_t ramp;
   uint8_t *bytes = (uint8_t*)&ramp;
   for (uint32_t i = 0; i < sizeof(ramp); ++i)
      bytes[i] = (uint8_t)i;
   tracker_send_alert(&ramp);
}

static void status_window_init(void)
{
   // Publish the fixed portion of the status window
   static const uint8_t magic[] = { 'A', '3', 'E', 'M' };
   status_window_write(REG_MAGIC, magic, sizeof(magic));
   status_window_set(REG_VERSION, TEST_APP_VERSION);
   status_window_set(REG_I2C_ADDRESS, EXT_HW_I2C_ADDRESS);
   status_window_set(REG_SENTINEL_A, 0x5A);
   status_window_set(REG_SENTINEL_B, 0xA5);
   for (uint32_t i = 0; i < 16; ++i)
   {
      status_window_set(REG_RAMP + i, (uint8_t)i);
      status_window_set(REG_INVERSE_RAMP + i, (uint8_t)(0xFF - i));
   }

   // Zero the dynamic portion so that stale values are never reported
   for (uint32_t i = REG_HEARTBEAT; i <= REG_LAST_COMMAND; ++i)
      if ((i != REG_SENTINEL_A) && (i != REG_SENTINEL_B))
         status_window_set(i, 0);
   for (uint32_t i = 0; i < REG_ECHO_BYTES; ++i)
      status_window_set(REG_ECHO + i, 0);
}

static void report_bus_wiring(void)
{
   // Sample both bus lines as plain inputs before the IO Slave claims them. An idle, correctly terminated bus reads
   // high on both; re-reading with an internal 50K pull-down engaged then separates a real external pull-up, which
   // easily overpowers it, from a line that was merely floating high
   uint32_t idle_level[2] = { 0, 0 }, pulled_level[2] = { 0, 0 };
   const uint32_t pins[2] = { PIN_EXT_HW_I2C_SCL, PIN_EXT_HW_I2C_SDA };
   const char *names[2] = { "SCL", "SDA" };
   am_hal_gpio_pincfg_t input_config = am_hal_gpio_pincfg_input;
   for (uint32_t i = 0; i < 2; ++i)
   {
      input_config.GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE;
      am_hal_gpio_pinconfig(pins[i], input_config);
      am_hal_delay_us(1000);
      am_hal_gpio_state_read(pins[i], AM_HAL_GPIO_INPUT_READ, &idle_level[i]);
      input_config.GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLDOWN_50K;
      am_hal_gpio_pinconfig(pins[i], input_config);
      am_hal_delay_us(1000);
      am_hal_gpio_state_read(pins[i], AM_HAL_GPIO_INPUT_READ, &pulled_level[i]);
      am_hal_gpio_pinconfig(pins[i], am_hal_gpio_pincfg_disabled);
   }

   // Report each line, then say plainly whether the bus looks usable
   for (uint32_t i = 0; i < 2; ++i)
      print("   %s (GPIO %u): idle = %u, with a 50K pull-down = %u -- %s\n", names[i], pins[i],
            idle_level[i], pulled_level[i],
            !idle_level[i] ? "STUCK LOW, the line is shorted or held down" :
            !pulled_level[i] ? "FLOATING, no external pull-up is reaching this pin" : "pulled up and idle, looks good");
   if (idle_level[0] && idle_level[1] && pulled_level[0] && pulled_level[1])
      print("   Both lines are terminated and idle, so the bus itself is wired and pulled up\n");
   else
      print("   The bus is NOT usable as wired; nothing below can succeed until this is fixed\n");
}

static void report_slave_configuration(void)
{
   // Read back what the IO Slave hardware actually ended up configured as, rather than what was asked for
   const uint32_t config = IOSLAVE->CFG, fifo_config = IOSLAVE->FIFOCFG;
   const uint32_t address_field = (config & IOSLAVE_CFG_I2CADDR_Msk) >> IOSLAVE_CFG_I2CADDR_Pos;
   print("   CFG     = 0x%08X: interface %s, %s mode, address field 0x%03X (7-bit address 0x%02X)\n",
         config, (config & IOSLAVE_CFG_IFCEN_Msk) ? "ENABLED" : "DISABLED",
         (config & IOSLAVE_CFG_IFCSEL_Msk) ? "SPI" : "I2C", address_field, (address_field >> 1) & 0x7F);
   print("   FIFOCFG = 0x%08X: read-only base 0x%02X, FIFO base 0x%02X, FIFO max 0x%03X\n", fifo_config,
         (uint32_t)(IOSLAVE->FIFOCFG_b.ROBASE << 3), (uint32_t)(IOSLAVE->FIFOCFG_b.FIFOBASE << 3),
         (uint32_t)(IOSLAVE->FIFOCFG_b.FIFOMAX << 3));
   print("   tracker_init() returned without tripping an assertion, so every HAL call in it succeeded\n");

   // Read back the two pads as well. A correct IOSLAVE CFG proves nothing on its own, because the pad input enable is
   // what actually gates the wire through to the peripheral: Programmer's Guide Section 9.2.3 says a pad with INPEN
   // clear "will always read as a 0", and Table 19 of Section 9.3.1.6 requires INPEN set and PULLCFG cleared on both
   // lines for IO Slave I2C. With INPEN clear on SDA the slave sees a constant zero and can never match its address
   const uint32_t pad_config[2] = { (&GPIO->PINCFG0)[PIN_EXT_HW_I2C_SCL], (&GPIO->PINCFG0)[PIN_EXT_HW_I2C_SDA] };
   const uint32_t pads[2] = { PIN_EXT_HW_I2C_SCL, PIN_EXT_HW_I2C_SDA };
   const char *pad_names[2] = { "SCL", "SDA" };
   bool pads_usable = true;
   for (uint32_t i = 0; i < 2; ++i)
   {
      const uint32_t function = (pad_config[i] & GPIO_PINCFG0_FNCSEL0_Msk) >> GPIO_PINCFG0_FNCSEL0_Pos;
      const uint32_t input_enabled = (pad_config[i] & GPIO_PINCFG0_INPEN0_Msk) >> GPIO_PINCFG0_INPEN0_Pos;
      const uint32_t pull_config = (pad_config[i] & GPIO_PINCFG0_PULLCFG0_Msk) >> GPIO_PINCFG0_PULLCFG0_Pos;
      const uint32_t expected_function = i ? PIN_EXT_HW_I2C_SDA_FUNCTION : PIN_EXT_HW_I2C_SCL_FUNCTION;
      pads_usable = pads_usable && input_enabled && (function == expected_function);
      print("   PINCFG%u  = 0x%08X: %s pad, FNCSEL %u (%s), INPEN %u (%s), PULLCFG %u\n", pads[i], pad_config[i],
            pad_names[i], function, (function == expected_function) ? "IO Slave" : "NOT the IO Slave",
            input_enabled, input_enabled ? "the pad reaches the peripheral" : "PAD READS AS A CONSTANT ZERO",
            pull_config);
   }
   print("   %s\n", pads_usable ? "Both pads are routed to the IO Slave with their inputs enabled" :
         "The pads are NOT usable by the IO Slave, so no address can ever match no matter what CFG says");
}

void tracker_isr_observer(uint32_t interrupt_status)
{
   // Overrides the weak no-op in the tracker driver. The driver clears the interrupt status before any other code can
   // read it, so this is the only place a read-completion interrupt can be seen at all
   ++interrupt_count;
   sticky_interrupts |= interrupt_status;
   const uint32_t next_index = (interrupt_write_index + 1) % EVENT_QUEUE_LENGTH;
   if (next_index != interrupt_read_index)
   {
      interrupt_status_queue[interrupt_write_index] = interrupt_status;
      interrupt_write_index = next_index;
   }
}

static void print_interrupt_names(uint32_t interrupt_status)
{
   // Spell out every interrupt flag present, so that a read completion is never mistaken for a write
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

static void park_address_pointer(void)
{
   // Aim the address pointer at the FIFO port so that the master's bare one-byte read pops FIFO data on the very
   // first try. Whether the pointer is writable from this side is not documented, so the result is verified
   IOSLAVE->ADDPTR_b.ADDPTR = FIFO_PORT_OFFSET;
   const uint32_t parked = IOSLAVE->ADDPTR_b.ADDPTR;
   if (parked == FIFO_PORT_OFFSET)
      print("   Address pointer parked on the FIFO port at 0x%02X, so a bare host read pops FIFO data immediately\n",
            (uint32_t)FIFO_PORT_OFFSET);
   else
      print("   Address pointer could not be parked, it reads 0x%02X; walk it up to 0x%02X with repeated host reads\n",
            parked, (uint32_t)FIFO_PORT_OFFSET);
}

static void print_hex_bytes(const uint8_t *bytes, uint32_t length)
{
   for (uint32_t i = 0; i < length; ++i)
      print("%02X ", bytes[i]);
}

static const char* message_type_name(uint8_t message_type)
{
   switch (message_type)
   {
      case MSG_CRITICAL_ALERT:  return "MSG_CRITICAL_ALERT";
      case MSG_ALERT:           return "MSG_ALERT";
      case MSG_STATUS:          return "MSG_STATUS";
      case MSG_CONFIG:          return "MSG_CONFIG";
      case MSG_GPS:             return "MSG_GPS";
      case MSG_GPS_REQUEST:     return "MSG_GPS_REQUEST";
      case MSG_STATUS_REQUEST:  return "MSG_STATUS_REQUEST";
      default:                  return "unknown";
   }
}

static void execute_test_command(uint8_t command)
{
   // Carry out a test command received at LRAM offset 0x00
   switch (command)
   {
      case TEST_CMD_PING:
         print("   Command 0xA0: ping acknowledged (counters at 0x47 and 0x4C have advanced)\n");
         break;
      case TEST_CMD_SEND_RAMP:
         queue_fifo_ramp();
         print("   Command 0xA1: queued %u ramp bytes (00 01 .. %02X); read with 3:0x7F,%u\n",
               (uint32_t)sizeof(tracker_alert_data_t), (uint32_t)(sizeof(tracker_alert_data_t) - 1),
               (uint32_t)sizeof(tracker_alert_data_t));
         break;
      case TEST_CMD_SEND_GPS_REQUEST:
         tracker_get_current_time();
         print("   Command 0xA2: queued a 1-byte MSG_GPS_REQUEST (06); read with 3:0x7F,1\n");
         break;
      case TEST_CMD_SEND_STATUS:
         tracker_update_status_data(0x11223344);
         tracker_send_status_update();
         print("   Command 0xA3: queued a %u-byte MSG_STATUS (03 44 33 22 11 ..); read with 3:0x7F,%u\n",
               (uint32_t)sizeof(tracker_status_data_t), (uint32_t)sizeof(tracker_status_data_t));
         break;
      case TEST_CMD_SEND_ALERT:
      {
         const tracker_alert_data_t alert = { .msg_type = MSG_CRITICAL_ALERT, .current_gps_data = 0,
                                              .utc_timestamp = 0x11223344, .lat = 36.1627f, .lon = -86.7816f,
                                              .height = 182.0f, .alert_type = ALERT_GUNSHOT };
         tracker_send_alert(&alert);
         print("   Command 0xA4: queued a %u-byte MSG_CRITICAL_ALERT (01 00 44 33 22 11 ..); read with 3:0x7F,%u\n",
               (uint32_t)sizeof(alert), (uint32_t)sizeof(alert));
         break;
      }
      case TEST_CMD_RESET_COUNTERS:
         write_count = command_count = 0;
         status_window_set(REG_WRITE_COUNT, 0);
         status_window_set(REG_COMMAND_COUNT, 0);
         status_window_set(REG_LAST_MESSAGE_TYPE, 0);
         status_window_set(REG_LAST_COMMAND, 0);
         status_window_set(REG_LAST_ADDPTR, 0);
         print("   Command 0xA5: counters reset\n");
         break;
      case TEST_CMD_PULSE_INTERRUPT:
         am_hal_gpio_output_clear(PIN_EXT_HW_INTERRUPT);
         am_hal_delay_us(1000);
         am_hal_gpio_output_set(PIN_EXT_HW_INTERRUPT);
         print("   Command 0xA6: pulsed PIN_EXT_HW_INTERRUPT (GPIO %u) low for 1 ms\n", (uint32_t)PIN_EXT_HW_INTERRUPT);
         break;
      default:
         break;
   }
}

static void report_host_event(const host_event_t *event)
{
   // Summarize the raw transaction before interpreting it
   print("[I2C] Host wrote: ADDPTR = 0x%02X, REGACCINTSTAT = 0x%08X\n", (uint32_t)event->addptr, event->register_access);
   if (!event->changed_length)
   {
      // Every host read starts with a write of the target offset, which lands here with no data attached
      print("   No LRAM bytes changed: an address pointer set for a read, or a rewrite of identical data\n");
      return;
   }
   print("   Offsets 0x%02X..0x%02X (%u bytes): ", (uint32_t)event->changed_offset,
         (uint32_t)(event->changed_offset + event->changed_length - 1), (uint32_t)event->changed_length);
   print_hex_bytes(event->data, event->captured_length);
   if (event->captured_length < event->changed_length)
      print("... (%u more)", (uint32_t)(event->changed_length - event->captured_length));
   print("\n");

   // Anything that does not start at offset 0x00 is a scratch write; the driver only ever looks at offset 0x00
   if (event->changed_offset)
   {
      print("   Written above offset 0x00, so the tracker driver ignored it; use 4:0x00,x or 5:0x00,x to reach it\n");
      return;
   }

   // Decode the payload as either a test command or a real tracker message
   const uint8_t message_type = event->data[0];
   if ((message_type >= TEST_CMD_PING) && (message_type <= TEST_CMD_PULSE_INTERRUPT))
   {
      ++command_count;
      status_window_set(REG_COMMAND_COUNT, (uint8_t)command_count);
      status_window_set(REG_LAST_COMMAND, message_type);
      execute_test_command(message_type);
      return;
   }
   print("   Decoded as %s (0x%02X)\n", message_type_name(message_type), (uint32_t)message_type);
   switch (message_type)
   {
      case MSG_GPS:
         if (event->changed_length >= sizeof(tracker_gps_data_t))
         {
            tracker_gps_data_t gps;
            memcpy(&gps, event->data, sizeof(gps));
            print("   Timestamp = %u, Lat = %.4f, Lon = %.4f, Ht = %.2f\n",
                  gps.utc_timestamp, gps.lat, gps.lon, gps.height);
         }
         else
            print("   Truncated: MSG_GPS needs %u bytes but only %u arrived\n",
                  (uint32_t)sizeof(tracker_gps_data_t), (uint32_t)event->changed_length);
         break;
      case MSG_STATUS_REQUEST:
         print("   The driver has already queued a status packet in response; read it with 3:0x7F,%u\n",
               (uint32_t)sizeof(tracker_status_data_t));
         break;
      default:
         break;
   }
}


// Interrupt Callback ----------------------------------------------------------------------------------------------------

static void tracker_data_available(tracker_msg_t message_type, const void *new_data)
{
   // Capture the hardware's view of the transaction before anything else can disturb it
   (void)message_type;
   (void)new_data;
   const uint8_t addptr = (uint8_t)IOSLAVE->ADDPTR_b.ADDPTR;
   const uint32_t register_access = IOSLAVE->REGACCINTSTAT;
   IOSLAVE->REGACCINTCLR = register_access;

   // Diff the host-writable area against its shadow copy to learn exactly which bytes arrived. ADDPTR alone cannot
   // distinguish a data write from the offset-only write that precedes every host read
   int32_t first_changed = -1, last_changed = -1;
   for (int32_t i = 0; i < (int32_t)HOST_WRITE_AREA_BYTES; ++i)
      if (am_hal_ios_pui8LRAM[i] != lram_shadow[i])
      {
         lram_shadow[i] = am_hal_ios_pui8LRAM[i];
         if (first_changed < 0)
            first_changed = i;
         last_changed = i;
      }

   // Hand the transaction to the main loop, which does all of the printing and responding
   const uint32_t next_index = (event_write_index + 1) % EVENT_QUEUE_LENGTH;
   if (next_index == event_read_index)
   {
      ++events_dropped;
      return;
   }
   last_addptr = addptr;
   volatile host_event_t *event = &event_queue[event_write_index];
   event->addptr = addptr;
   event->register_access = register_access;
   event->changed_offset = (first_changed < 0) ? 0 : (uint8_t)first_changed;
   event->changed_length = (first_changed < 0) ? 0 : (uint8_t)(last_changed - first_changed + 1);
   event->captured_length = (event->changed_length > EVENT_BYTES_CAPTURED) ? EVENT_BYTES_CAPTURED : event->changed_length;
   for (uint32_t i = 0; i < event->captured_length; ++i)
      event->data[i] = am_hal_ios_pui8LRAM[event->changed_offset + i];
   event_write_index = next_index;
}


// Main Application ------------------------------------------------------------------------------------------------------

int main(void)
{
   // Set up system hardware, then check the bus wiring before the IO Slave takes the pins over
   setup_hardware();
   leds_init();
   print("\nTracker I2C connectivity test, version %u\n", (uint32_t)TEST_APP_VERSION);
   print("Checking the bus lines before configuring the IO Slave:\n");
   report_bus_wiring();

   // Bring up the production tracker driver and report what the hardware ended up configured as
   tracker_init();
   system_enable_interrupts(true);
   tracker_register_data_callback(tracker_data_available);
   status_window_init();
   memset(lram_shadow, 0, sizeof(lram_shadow));
   if (AUTO_REFILL_FIFO)
      queue_fifo_ramp();
   if (PARK_POINTER_AT_FIFO_PORT)
      park_address_pointer();
   last_fifo_count = IOSLAVE->FIFOCTR_b.FIFOCTR;
   last_addptr = (uint8_t)IOSLAVE->ADDPTR_b.ADDPTR;
   IOSLAVE->REGACCINTCLR = IOSLAVE->REGACCINTSTAT;
   print("IO Slave configuration as read back from the hardware:\n");
   report_slave_configuration();
   print("   Address pointer rests at 0x%02X and INTSTAT reads 0x%08X before any host access\n",
         (uint32_t)last_addptr, IOSLAVE->INTSTAT);

   // Print the operator's quick reference to the debug console
   print("Listening as an I2C slave at address 0x%02X on IOS module %u (SCL = GPIO %u, SDA = GPIO %u)\n",
         (uint32_t)EXT_HW_I2C_ADDRESS, (uint32_t)EXT_HW_I2C_NUMBER,
         (uint32_t)PIN_EXT_HW_I2C_SCL, (uint32_t)PIN_EXT_HW_I2C_SDA);
   print("On the Tracker console, press 't' then 'v' then '1' to reach the I2C Test Menu for this device, then:\n");
   print("   1              connection test, expect OK\n");
   print("   2:0x40         expect 0x41, the first byte of the 'A3EM' magic value\n");
   print("   3:0x40,16      expect 41 33 45 4D %02X %02X followed by live counters\n",
         (uint32_t)TEST_APP_VERSION, (uint32_t)EXT_HW_I2C_ADDRESS);
   print("   3:0x50,16      expect 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
   print("   2:0x46         heartbeat, changes once per second\n");
   print("   4:0x00,0xA0    ping; 0x47 and 0x4C then advance and 0x4D reads back 0xA0\n");
   print("   4:0x00,0xA1    queue 19 ramp bytes, then 3:0x7C,2 for the count and 3:0x7F,19 for the data\n");
   print("   4:0x00,0xA2    queue MSG_GPS_REQUEST      4:0x00,0xA3    queue MSG_STATUS\n");
   print("   4:0x00,0xA4    queue MSG_CRITICAL_ALERT   4:0x00,0xA5    reset counters\n");
   print("   4:0x00,0xA6    pulse the HW interrupt     4:0x00,0x07    real MSG_STATUS_REQUEST\n");
   print("   5:0x00,<data>  send a raw packet, then 3:0x70,8 to read the first 8 bytes back\n");
   print("Waiting for the Tracker board...\n\n");

   // Service the Tracker board for as long as the test runs
   uint32_t heartbeat = 0, iteration = 0, reported_drops = 0;
   while (true)
   {
      // Report every transaction the ISR captured
      while (event_read_index != event_write_index)
      {
         host_event_t event;
         memcpy(&event, (const void*)&event_queue[event_read_index], sizeof(event));
         event_read_index = (event_read_index + 1) % EVENT_QUEUE_LENGTH;

         // Publish what was seen before decoding it, so that an immediate host read reflects this transaction
         ++write_count;
         status_window_refresh(REG_WRITE_COUNT, (uint8_t)write_count);
         status_window_refresh(REG_LAST_ADDPTR, event.addptr);
         if (event.changed_length)
         {
            status_window_refresh(REG_LAST_MESSAGE_TYPE, event.data[0]);
            const uint32_t echoed = (event.captured_length > REG_ECHO_BYTES) ? REG_ECHO_BYTES : event.captured_length;
            for (uint32_t i = 0; i < REG_ECHO_BYTES; ++i)
               status_window_refresh(REG_ECHO + i, (i < echoed) ? event.data[i] : 0);
         }
         report_host_event(&event);
         led_toggle(LED_GREEN);
      }

      // Watch the hardware directly for any bus activity the interrupt path did not report. The address pointer and
      // the register-access bitmap both move on a host access whether or not an interrupt is ever taken, so this
      // catches traffic that arrives malformed, lands outside the direct area, or is filtered by the driver's ISR
      const uint8_t addptr = (uint8_t)IOSLAVE->ADDPTR_b.ADDPTR;
      const uint32_t register_access = IOSLAVE->REGACCINTSTAT;
      if ((addptr != last_addptr) || register_access)
      {
         IOSLAVE->REGACCINTCLR = register_access;
         sticky_register_access |= register_access;
         print("[I2C] Bus activity with no interrupt: ADDPTR 0x%02X -> 0x%02X, REGACCINTSTAT = 0x%08X\n",
               (uint32_t)last_addptr, (uint32_t)addptr, register_access);
         last_addptr = addptr;
      }

      // Report every raw IOS interrupt. A read completion (XCMPRR for the direct area, XCMPRF for the FIFO port) is
      // proof that the master really did clock a read transaction, whatever the address pointer and FIFO count show
      while (interrupt_read_index != interrupt_write_index)
      {
         const uint32_t interrupt_status = interrupt_status_queue[interrupt_read_index];
         interrupt_read_index = (interrupt_read_index + 1) % EVENT_QUEUE_LENGTH;
         print("[I2C] IOS interrupt 0x%08X (", interrupt_status);
         print_interrupt_names(interrupt_status);
         print("), ADDPTR = 0x%02X, FIFOCTR = %u, FIFOSIZ = %u\n", (uint32_t)IOSLAVE->ADDPTR_b.ADDPTR,
               (uint32_t)IOSLAVE->FIFOCTR_b.FIFOCTR, (uint32_t)IOSLAVE->FIFOPTR_b.FIFOSIZ);
      }

      // Watch the raw interrupt status directly. The driver's ISR clears this the instant it runs, so anything seen
      // here is an interrupt the hardware raised that the CPU never serviced -- which separates a master that never
      // completes a transfer from a transfer that completes but whose interrupt never reaches the CPU
      const uint32_t interrupt_pending = IOSLAVE->INTSTAT;
      if (interrupt_pending != last_interrupt_pending)
      {
         print("[I2C] Unserviced INTSTAT 0x%08X -> 0x%08X (", last_interrupt_pending, interrupt_pending);
         print_interrupt_names(interrupt_pending);
         print(")\n");
         last_interrupt_pending = interrupt_pending;
      }

      // Watch the host-facing interrupt block, which the driver never clears. Bit 7 is set when the host reads the
      // FIFO port while the count is zero, so it fires even when a read moves nothing else observable
      const uint32_t host_interrupts = IOSLAVE->IOINTCTL_b.IOINT;
      if (host_interrupts != last_host_interrupts)
      {
         print("[I2C] Host interrupt status 0x%02X -> 0x%02X%s%s\n", last_host_interrupts, host_interrupts,
               (host_interrupts & 0x80) ? " (FIFO underflow: the host read an empty FIFO)" : "",
               (host_interrupts & 0x40) ? " (read error)" : "");
         last_host_interrupts = host_interrupts;
      }

      // Watch the FIFO byte count, which the hardware decrements on its own as the host reads the port at 0x7F
      const uint32_t fifo_count = IOSLAVE->FIFOCTR_b.FIFOCTR;
      if (fifo_count != last_fifo_count)
      {
         if (fifo_count < last_fifo_count)
            print("[I2C] Host read %u byte(s) from the FIFO port, %u left of %u\n",
                  last_fifo_count - fifo_count, fifo_count, last_fifo_count);
         last_fifo_count = fifo_count;
      }

      // Warn if the Tracker board is transacting faster than the console can keep up with
      if (events_dropped != reported_drops)
      {
         reported_drops = events_dropped;
         print("[I2C] Warning: %u transactions dropped, the capture queue overflowed\n", reported_drops);
      }

      // Refresh the live portion of the status window once per second
      if (++iteration >= 20)
      {
         iteration = 0;
         status_window_refresh(REG_HEARTBEAT, (uint8_t)++heartbeat);
         status_window_refresh(REG_FIFO_PENDING, (uint8_t)fifo_count);
         status_window_refresh(REG_FIFO_HW_BYTES, (uint8_t)IOSLAVE->FIFOPTR_b.FIFOSIZ);

         // Keep a known packet available so that the FIFO port can be exercised using host reads alone
         if (AUTO_REFILL_FIFO && !fifo_count)
         {
            queue_fifo_ramp();
            last_fifo_count = IOSLAVE->FIFOCTR_b.FIFOCTR;
            print("[I2C] FIFO was empty, queued %u ramp bytes (00 01 .. %02X) for the host to read at 0x7F\n",
                  (uint32_t)sizeof(tracker_alert_data_t), (uint32_t)(sizeof(tracker_alert_data_t) - 1));
         }
         if (!(heartbeat % 30))
         {
            print("[I2C] Alive: %u host writes, %u commands, %u FIFO bytes waiting, ADDPTR = 0x%02X, "
                  "REGACCINTSTAT ever set = 0x%08X\n", write_count, command_count,
                  (uint32_t)IOSLAVE->FIFOCTR_b.FIFOCTR, (uint32_t)last_addptr, sticky_register_access);
            print("      %u ISR entries, flags ever seen = 0x%08X (", interrupt_count, sticky_interrupts);
            print_interrupt_names(sticky_interrupts);
            print("), host interrupt status = 0x%02X\n", (uint32_t)IOSLAVE->IOINTCTL_b.IOINT);
            print("      INTSTAT = 0x%08X, INTEN = 0x%08X, NVIC %s and %s\n", IOSLAVE->INTSTAT, IOSLAVE->INTEN,
                  NVIC_GetEnableIRQ(IOSLAVE_IRQn) ? "enabled" : "DISABLED",
                  NVIC_GetPendingIRQ(IOSLAVE_IRQn) ? "PENDING" : "not pending");
         }
      }
      am_hal_delay_us(50000);
   }

   // Should never reach this point
   return 0;
}
