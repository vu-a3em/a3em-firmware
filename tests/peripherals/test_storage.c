#include "logging.h"
#include "storage.h"
#include "system.h"

static uint8_t read_buffer[256];
static uint8_t write_buffer[256] = "This is the content inside the TXT\nApollo4 SD Card File System Example!!!\n";

int main(void)
{
   // Set up the system hardware and initialize the SD card peripheral
   setup_hardware();
   storage_init();
   system_enable_interrupts(true);

   // Attempt to open and write a file on the SD card
   if (!storage_open("test.txt", true))
      print("ERROR: Unable to create a new file \"test.txt\" on the SD card!\n");
   else if (!storage_write(write_buffer, sizeof(write_buffer)))
      print("ERROR: Unable to write data to the SD card file!\n");
   storage_close();

   // Attempt to re-open the same file and read back its contents
   uint32_t bytes_read = 0;
   if (!storage_open("test.txt", false))
      print("ERROR: Unable to open existing SD card file \"test.txt\"!\n");
   else if (!(bytes_read = storage_read(read_buffer, sizeof(read_buffer))))
      print("ERROR: Unable to read data from the SD card file!\n");
   storage_close();

   // Verify that the contents read back from the file match what was written
   bool success = (bytes_read > 0);
   for (uint32_t i = 0; i < bytes_read; ++i)
      if (read_buffer[i] != write_buffer[i])
         success = false;
   print("Test Status: %s\n", success ? "SUCCESS" : "FAILURE");

   // Attempt to create a new directory on the SD card
   if (!storage_mkdir("TestDirectory"))
      print("ERROR: Unable to create the \"TestDirectory\" directory on the SD card!\n");
   if (!storage_chdir("TestDirectory"))
      print("ERROR: Unable to chdir to \"TestDirectory\" on the SD card!\n");

   // Attempt to write a file to the new directory on the SD card
   if (!storage_open("dirtest.txt", true))
      print("ERROR: Unable to create a new file \"test.txt\" in \"TestDirectory\" on the SD card!\n");
   else if (!storage_write(write_buffer, sizeof(write_buffer)))
      print("ERROR: Unable to write data to the SD card file!\n");
   storage_close();

   // Attempt to re-open the same file and read back its contents
   bytes_read = 0;
   if (!storage_open("dirtest.txt", false))
      print("ERROR: Unable to open existing SD card file \"test.txt\" in \"TestDirectory\"!\n");
   else if (!(bytes_read = storage_read(read_buffer, sizeof(read_buffer))))
      print("ERROR: Unable to read data from the SD card file!\n");
   storage_close();

   // Put the CPU into deep sleep forever
   while (true)
      am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

   // Should never reach this point
   return 0;
}
