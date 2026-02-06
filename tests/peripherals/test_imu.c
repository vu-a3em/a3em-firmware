#include "imu.h"
#include "logging.h"
#include "system.h"

void motion_change_callback(bool in_motion)
{
   print("Motion change detected: %s\n", in_motion ? "IN MOTION" : "STATIONARY");
}

void data_ready_callback(float accel_x_mg, float accel_y_mg, float accel_z_mg)
{
   static uint32_t count = 0;
   print("[%lu]: Received data: x = %.2f, y = %.2f, z = %.2f\n", count, accel_x_mg, accel_y_mg, accel_z_mg);
   ++count;
}

void test_motion_change_detection(void)
{
   // Enable motion change detection then go to sleep forever
   imu_enable_motion_change_detection(true, motion_change_callback);
   while (true)
      system_enter_deep_sleep_mode();
}

void test_interrupt_based_read(void)
{
   // Enable data reading then go to sleep forever
   imu_enable_raw_data_output(true, LIS2DU12_2g, 100, LIS2DU12_ODR_div_2, data_ready_callback);
   while (true)
      system_enter_deep_sleep_mode();
}

void test_polling_read(void)
{
   float accel_mg[3];
   while (true)
   {
      imu_read_accel_data(accel_mg, accel_mg + 1, accel_mg + 2);
      print("Received data: x = %.2f, y = %.2f, z = %.2f\n", accel_mg[0], accel_mg[1], accel_mg[2]);
   }
}

void test_fifo_drain(void)
{
   // Enable data reading then drain the FIFO every second
   imu_enable_raw_data_output(true, LIS2DU12_2g, 50, LIS2DU12_ODR_div_2, data_ready_callback);
   while (true)
   {
      system_delay(1000000);
      imu_drain_fifo();
   }
}

int main(void)
{
   // Set up system hardware
   setup_hardware();
   imu_init();
   system_enable_interrupts(true);

   // Choose one of the following tests to carry out
   //test_motion_change_detection();
   //test_interrupt_based_read();
   //test_polling_read();
   test_fifo_drain();

   // Should never reach this point
   return 0;
}
