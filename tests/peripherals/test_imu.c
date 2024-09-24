#include "imu.h"
#include "logging.h"
#include "system.h"

void motion_change_callback(bool in_motion)
{
   print("Motion change detected: %s\n", in_motion ? "IN MOTION" : "STATIONARY");
}

void data_ready_callback(float accel_x_mg, float accel_y_mg, float accel_z_mg)
{
   print("Received data: x = %.2f, y = %.2f, z = %.2f\n", accel_x_mg, accel_y_mg, accel_z_mg);
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
   imu_enable_raw_data_output(true, LIS2DU12_2g, 6, LIS2DU12_ODR_div_2, 6, data_ready_callback);
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

int main(void)
{
   // Set up system hardware
   setup_hardware();
   imu_init();
   system_enable_interrupts(true);

   // Choose one of the two tests to carry out
   //test_motion_change_detection();
   test_interrupt_based_read();
   //test_polling_read();

   // Should never reach this point
   return 0;
}
