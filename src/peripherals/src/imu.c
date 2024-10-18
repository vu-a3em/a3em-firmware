// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "imu.h"
#include "system.h"


// Static Global Variables and Definitions -----------------------------------------------------------------------------

#define PIN_IMU_INTERRUPT PIN_IMU_INTERRUPT2

static void *i2c_handle = NULL;
static stmdev_ctx_t imu_context;
static imu_data_ready_callback_t data_ready_callback;
static motion_change_callback_t motion_change_callback;
static lis2du12_fifo_md_t fifo_mode;
static lis2du12_md_t imu_mode;


// Private Helper Functions --------------------------------------------------------------------------------------------

static int32_t platform_read(void *handle, uint8_t reg_number, uint8_t *read_buffer, uint16_t buffer_length)
{
   am_hal_iom_transfer_t read_transaction = {
      .uPeerInfo.ui32I2CDevAddr     = IMU_I2C_ADDRESS,
      .ui32InstrLen                 = 1,
      .ui64Instr                    = reg_number,
      .eDirection                   = AM_HAL_IOM_RX,
      .ui32NumBytes                 = buffer_length,
      .pui32TxBuffer                = NULL,
      .pui32RxBuffer                = (uint32_t*)read_buffer,
      .bContinue                    = false,
      .ui8RepeatCount               = 0,
      .ui8Priority                  = 1,
      .ui32PauseCondition           = 0,
      .ui32StatusSetClr             = 0
   };
   return am_hal_iom_blocking_transfer(i2c_handle, &read_transaction);
}

static int32_t platform_write(void *handle, uint8_t reg_number, const uint8_t *write_buffer, uint16_t buffer_length)
{
   am_hal_iom_transfer_t write_transaction = {
      .uPeerInfo.ui32I2CDevAddr     = IMU_I2C_ADDRESS,
      .ui32InstrLen                 = 1,
      .ui64Instr                    = reg_number,
      .eDirection                   = AM_HAL_IOM_TX,
      .ui32NumBytes                 = buffer_length,
      .pui32TxBuffer                = (uint32_t*)write_buffer,
      .pui32RxBuffer                = NULL,
      .bContinue                    = false,
      .ui8RepeatCount               = 0,
      .ui8Priority                  = 1,
      .ui32PauseCondition           = 0,
      .ui32StatusSetClr             = 0
   };
   return am_hal_iom_blocking_transfer(i2c_handle, &write_transaction);
}

static void imu_isr(void *args)
{
   if (motion_change_callback)
   {
      static lis2du12_all_sources_t status;
      lis2du12_all_sources_get(&imu_context, &status);
      if (status.sleep_change)
         motion_change_callback(!status.sleep_state);
   }
   if (data_ready_callback)
   {
      static lis2du12_fifo_status_t fifo_status;
      lis2du12_fifo_status_get(&imu_context, &fifo_status);
      if (fifo_status.fifo_fth)
      {
         static uint8_t fifo_level;
         static lis2du12_fifo_data_t data;
         lis2du12_fifo_level_get(&imu_context, &fifo_mode, &fifo_level);
         for (uint8_t i = 0; i < fifo_level; ++i)
         {
            lis2du12_fifo_data_get(&imu_context, &imu_mode, &fifo_mode, &data);
            data_ready_callback(data.xl[0].mg[0], data.xl[0].mg[1], data.xl[0].mg[2]);
         }
      }
   }
}


// Chip-Specific API Functions -----------------------------------------------------------------------------------------

int32_t lis2du12_read_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
   if (!ctx)
      return -1;
   return ctx->read_reg(ctx->handle, reg, data, len);
}

int32_t lis2du12_write_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
   if (!ctx)
      return -1;
   return ctx->write_reg(ctx->handle, reg, data, len);
}

static void bytecpy(uint8_t *target, uint8_t *source)
{
   if (target && source)
      *target = *source;
}

float lis2du12_from_fs2g_to_mg(int16_t lsb) { return (float) lsb * 0.061f; }
float lis2du12_from_fs4g_to_mg(int16_t lsb) { return (float) lsb * 0.122f; }
float lis2du12_from_fs8g_to_mg(int16_t lsb) { return (float) lsb * 0.244f; }
float lis2du12_from_fs16g_to_mg(int16_t lsb) { return (float) lsb * 0.488f; }
float lis2du12_from_lsb_to_celsius(int16_t lsb) { return ((float) lsb / 355.5f) + 25.0f; }

int32_t lis2du12_id_get(const stmdev_ctx_t *ctx, lis2du12_id_t *val)
{
   return lis2du12_read_reg(ctx, LIS2DU12_WHO_AM_I, &val->whoami, 1);
}

int32_t lis2du12_bus_mode_set(const stmdev_ctx_t *ctx, lis2du12_bus_mode_t val)
{
   lis2du12_ctrl1_t ctrl1;
   lis2du12_if_ctrl_t if_ctrl;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_IF_CTRL, (uint8_t*)&if_ctrl, 1);
   if (!ret)
   {
      if_ctrl.i3c_disable = (uint8_t)val & 0x01U;
      if_ctrl.i2c_disable = ((uint8_t)val & 0x02U) >> 1;
      ret = lis2du12_write_reg(ctx, LIS2DU12_IF_CTRL, (uint8_t*)&if_ctrl, 1);
   }
   if (!ret)
      ret = lis2du12_read_reg(ctx, LIS2DU12_CTRL1, (uint8_t*)&ctrl1, 1);
   if (!ret)
   {
      ctrl1.sim = ((uint8_t) val & 0x04U) >> 2;
      ret = lis2du12_write_reg(ctx, LIS2DU12_CTRL1, (uint8_t*)&ctrl1, 1);
   }
   return ret;
}

int32_t lis2du12_init_set(const stmdev_ctx_t *ctx, lis2du12_init_t val)
{
   lis2du12_ctrl1_t ctrl1;
   lis2du12_ctrl4_t ctrl4;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_CTRL1, (uint8_t*)&ctrl1, 1) +
                 lis2du12_read_reg(ctx, LIS2DU12_CTRL4, (uint8_t*)&ctrl4, 1);
   switch (val)
   {
      case LIS2DU12_BOOT:
         ctrl4.boot = PROPERTY_ENABLE;
         ret += lis2du12_write_reg(ctx, LIS2DU12_CTRL4, (uint8_t*)&ctrl4, 1);
         break;
      case LIS2DU12_RESET:

         ctrl1.sw_reset = PROPERTY_ENABLE;
         ret += lis2du12_write_reg(ctx, LIS2DU12_CTRL1, (uint8_t*)&ctrl1, 1);
         break;
      case LIS2DU12_DRV_RDY:
         ctrl4.bdu = PROPERTY_ENABLE;
         ctrl1.if_add_inc = PROPERTY_ENABLE;
         ret += lis2du12_write_reg(ctx, LIS2DU12_CTRL4, (uint8_t*)&ctrl4, 1);
         ret += lis2du12_write_reg(ctx, LIS2DU12_CTRL1, (uint8_t*)&ctrl1, 1);
         break;
      default:
         ctrl1.sw_reset = PROPERTY_ENABLE;
         ret += lis2du12_write_reg(ctx, LIS2DU12_CTRL1, (uint8_t*)&ctrl1, 1);
         break;
   }
   return ret;
}

int32_t lis2du12_status_get(const stmdev_ctx_t *ctx, lis2du12_status_t *val)
{
   lis2du12_ctrl1_t ctrl1;
   lis2du12_ctrl4_t ctrl4;
   lis2du12_status_register_t status_register;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_STATUS, (uint8_t*)&status_register, 1);
   if (!ret)
      ret = lis2du12_read_reg(ctx, LIS2DU12_CTRL1, (uint8_t*)&ctrl1, 1);
   if (!ret)
      ret = lis2du12_read_reg(ctx, LIS2DU12_CTRL4, (uint8_t*)&ctrl4, 1);
   val->sw_reset = ctrl1.sw_reset;
   val->boot = ctrl4.boot;
   val->drdy_xl = status_register.drdy;
   val->power_down = status_register.pd_status;
   return ret;
}

int32_t lis2du12_pin_conf_set(const stmdev_ctx_t *ctx, lis2du12_pin_conf_t *val)
{
   lis2du12_ctrl1_t ctrl1;
   lis2du12_md2_cfg_t md2_cfg;
   lis2du12_if_ctrl_t if_ctrl;
   lis2du12_if_pu_ctrl_t if_pu_ctrl;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_IF_PU_CTRL, (uint8_t*)&if_pu_ctrl, 1);
   if (!ret)
      ret = lis2du12_read_reg(ctx, LIS2DU12_IF_CTRL, (uint8_t*)&if_ctrl, 1);
   if (!ret)
      ret = lis2du12_read_reg(ctx, LIS2DU12_CTRL1, (uint8_t*)&ctrl1, 1);
   if (!ret)
      ret = lis2du12_read_reg(ctx, LIS2DU12_MD2_CFG, (uint8_t*)&md2_cfg, 1);
   if (!ret)
   {
      if_pu_ctrl.sdo_pu_disc = ~val->sdo_pull_up;
      if_pu_ctrl.sda_pu_en = val->sda_pull_up;
      if_pu_ctrl.cs_pu_disc = ~val->cs_pull_up;
      ret = lis2du12_write_reg(ctx, LIS2DU12_IF_PU_CTRL, (uint8_t*)&if_pu_ctrl, 1);
   }
   if (!ret)
   {
      if_ctrl.pd_dis_int1 = val->int1_pull_down;
      ret = lis2du12_write_reg(ctx, LIS2DU12_IF_CTRL, (uint8_t*)&if_ctrl, 1);
   }
   if (!ret)
   {
      ctrl1.pp_od = val->int1_int2_push_pull;
      ret = lis2du12_write_reg(ctx, LIS2DU12_CTRL1, (uint8_t*)&ctrl1, 1);
   }
   if (!ret)
   {
      md2_cfg.pd_dis_int2 = val->int2_pull_down;
      ret = lis2du12_write_reg(ctx, LIS2DU12_MD2_CFG, (uint8_t*)&md2_cfg, 1);
   }
   return ret;
}

int32_t lis2du12_all_sources_get(const stmdev_ctx_t *ctx, lis2du12_all_sources_t *val)
{
   lis2du12_all_int_src_t all_int_src;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_ALL_INT_SRC, (uint8_t*)&all_int_src, 1);
   if (!ret && all_int_src.int_global == 1U)
   {
      val->free_fall = all_int_src.ff_ia_all;
      val->six_d = all_int_src.d6d_ia_all;
      val->wake_up = all_int_src.wu_ia_all;
      val->sleep_change = all_int_src.sleep_change_ia_all;
      val->single_tap = all_int_src.single_tap_all;
      val->double_tap = all_int_src.double_tap_all;

      if (all_int_src.d6d_ia_all == 1U)
      {
         lis2du12_sixd_src_t sixd_src;
         ret = lis2du12_read_reg(ctx, LIS2DU12_SIXD_SRC, (uint8_t*)&sixd_src, 1);
         val->six_d_xl = sixd_src.xl;
         val->six_d_xh = sixd_src.xh;
         val->six_d_yl = sixd_src.yl;
         val->six_d_yh = sixd_src.yh;
         val->six_d_zl = sixd_src.zl;
         val->six_d_zh = sixd_src.zh;
      }

      if (all_int_src.wu_ia_all == 1U || all_int_src.sleep_change_ia_all == 1U)
      {
         lis2du12_wake_up_src_t wu_src;
         ret = lis2du12_read_reg(ctx, LIS2DU12_WAKE_UP_SRC, (uint8_t*)&wu_src, 1);
         val->wake_up_z = wu_src.z_wu;
         val->wake_up_y = wu_src.y_wu;
         val->wake_up_x = wu_src.x_wu;
         val->sleep_state = wu_src.sleep_state;
      }

      if (all_int_src.single_tap_all == 1U || all_int_src.double_tap_all == 1U)
      {
         lis2du12_tap_src_t tap_src;
         ret = lis2du12_read_reg(ctx, LIS2DU12_TAP_SRC, (uint8_t*)&tap_src, 1);
         val->tap_z = tap_src.z_tap;
         val->tap_y = tap_src.y_tap;
         val->tap_x = tap_src.x_tap;
         val->tap_sign = tap_src.tap_sign;
      }
   }
   return ret;
}

int32_t lis2du12_mode_set(const stmdev_ctx_t *ctx, lis2du12_md_t *val)
{
   lis2du12_ctrl5_t ctrl5;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_CTRL5, (uint8_t*)&ctrl5, 1);
   ctrl5.odr = (uint8_t) val->odr;
   ctrl5.fs = (uint8_t) val->fs;
   ctrl5.bw = (uint8_t) val->bw;
   ret += lis2du12_write_reg(ctx, LIS2DU12_CTRL5, (uint8_t*)&ctrl5, 1);
   return ret;
}

int32_t lis2du12_trigger_sw(const stmdev_ctx_t *ctx, lis2du12_md_t *md)
{
   int32_t ret = 0;
   lis2du12_ctrl4_t ctrl4;
   if (md->odr == LIS2DU12_TRIG_SW)
   {
      ret = lis2du12_read_reg(ctx, LIS2DU12_CTRL4, (uint8_t*)&ctrl4, 1);
      ctrl4.soc = PROPERTY_ENABLE;
      ret += lis2du12_write_reg(ctx, LIS2DU12_CTRL4, (uint8_t*)&ctrl4, 1);
   }
   return ret;
}

int32_t lis2du12_data_get(const stmdev_ctx_t *ctx, lis2du12_md_t *md, lis2du12_data_t *data)
{
   uint8_t buff[8], j = 0U;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_OUTX_L, (uint8_t*)&buff, 8);
   for (uint8_t i = 0U; i < 3U; i++)
   {
      data->xl.raw[i] = (int16_t)buff[j + 1U];
      data->xl.raw[i] = (data->xl.raw[i] * 256) + (int16_t)buff[j];
      j += 2U;
      switch (md->fs)
      {
         case LIS2DU12_2g:
            data->xl.mg[i] = lis2du12_from_fs2g_to_mg(data->xl.raw[i]);
            break;
         case LIS2DU12_4g:
            data->xl.mg[i] = lis2du12_from_fs4g_to_mg(data->xl.raw[i]);
            break;
         case LIS2DU12_8g:
            data->xl.mg[i] = lis2du12_from_fs8g_to_mg(data->xl.raw[i]);
            break;
         case LIS2DU12_16g:
            data->xl.mg[i] = lis2du12_from_fs16g_to_mg(data->xl.raw[i]);
            break;
         default:
            data->xl.mg[i] = 0.0f;
            break;
      }
   }
   data->heat.raw = (int16_t)buff[j + 1U];
   data->heat.raw = (data->heat.raw * 256) + (int16_t)buff[j];
   data->heat.deg_c = lis2du12_from_lsb_to_celsius(data->heat.raw);
   return ret;
}

int32_t lis2du12_self_test_sign_set(const stmdev_ctx_t *ctx, lis2du12_st_t val)
{
   lis2du12_st_sign_t st_sign;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_ST_SIGN, (uint8_t*)&st_sign, 1);
   if (!ret)
   {
      st_sign.stsign = (uint8_t) val;
      ret = lis2du12_write_reg(ctx, LIS2DU12_ST_SIGN, (uint8_t*)&st_sign, 1);
   }
   return ret;
}

int32_t lis2du12_self_test_start(const stmdev_ctx_t *ctx, uint8_t val)
{
   lis2du12_ctrl3_t ctrl3;
   if (val != 1U && val != 2U)
      return -1;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_CTRL3, (uint8_t*)&ctrl3, 1);
   ctrl3.st = (uint8_t)val;
   ret += lis2du12_write_reg(ctx, LIS2DU12_CTRL3, (uint8_t*)&ctrl3, 1);
   return ret;
}

int32_t lis2du12_self_test_stop(const stmdev_ctx_t *ctx)
{
   lis2du12_ctrl3_t ctrl3;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_CTRL3, (uint8_t*)&ctrl3, 1);
   ctrl3.st = 0;
   ret += lis2du12_write_reg(ctx, LIS2DU12_CTRL3, (uint8_t*)&ctrl3, 1);
   return ret;
}

int32_t lis2du12_fifo_mode_set(const stmdev_ctx_t *ctx, lis2du12_fifo_md_t *val)
{
   uint8_t reg[2];
   lis2du12_fifo_wtm_t fifo_wtm;
   lis2du12_fifo_ctrl_t fifo_ctrl;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_FIFO_CTRL, reg, 2);
   bytecpy((uint8_t*)&fifo_ctrl, &reg[0]);
   bytecpy((uint8_t*)&fifo_wtm, &reg[1]);
   fifo_ctrl.f_mode = (uint8_t) val->operation;
   fifo_ctrl.fifo_depth = (uint8_t) val->store;
   if (val->watermark != 0x00U)
      fifo_ctrl.stop_on_fth = PROPERTY_ENABLE;
   else
      fifo_ctrl.stop_on_fth = PROPERTY_DISABLE;
   fifo_wtm.fth = val->watermark;
   bytecpy(&reg[0], (uint8_t*) &fifo_ctrl);
   bytecpy(&reg[1], (uint8_t*) &fifo_wtm);
   ret += lis2du12_write_reg(ctx, LIS2DU12_FIFO_CTRL, reg, 2);
   return ret;
}

int32_t lis2du12_fifo_status_get(const stmdev_ctx_t *ctx, lis2du12_fifo_status_t *val)
{
   lis2du12_fifo_status1_t fifo_status1;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_FIFO_STATUS1, (uint8_t*)&fifo_status1, 1);
   val->fifo_fth = fifo_status1.fth;
   val->fifo_ovr = fifo_status1.fifo_ovr;
   return ret;
}

int32_t lis2du12_fifo_level_get(const stmdev_ctx_t *ctx, lis2du12_fifo_md_t *md, uint8_t *val)
{
   lis2du12_fifo_status2_t fifo_status2;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_FIFO_STATUS2, (uint8_t*)&fifo_status2, 1);
   *val = fifo_status2.fss;
   return ret;
}

int32_t lis2du12_fifo_data_get(const stmdev_ctx_t *ctx, lis2du12_md_t *md, lis2du12_fifo_md_t *fmd, lis2du12_fifo_data_t *data)
{
   uint8_t fifo_data[8];
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_OUTX_L, fifo_data, 8);
   if (fmd->store == LIS2DU12_8_BIT)
   {
      for (int8_t i = 0; i < 3; i++)
      {
         data->xl[0].raw[i] = (int16_t)fifo_data[2 * i + 1];
         data->xl[0].raw[i] = data->xl[0].raw[i] * 256 + (int16_t)fifo_data[2 * i];
      }

      data->heat.raw = (int16_t)fifo_data[7U];
      data->heat.raw = (data->heat.raw * 256) + (int16_t)fifo_data[6U];
      data->heat.deg_c = lis2du12_from_lsb_to_celsius(data->heat.raw);
   }
   else
   {
      for (int8_t i = 0; i < 3; i++)
      {
         data->xl[0].raw[i] = (int16_t)fifo_data[i] * 256;
         data->xl[1].raw[i] = (int16_t)fifo_data[3 + i] * 256;
      }
   }

   for (int8_t i = 0; i < 3; i++)
   {
      switch (md->fs)
      {
         case LIS2DU12_2g:
            data->xl[0].mg[i] = lis2du12_from_fs2g_to_mg(data->xl[0].raw[i]);
            data->xl[1].mg[i] = lis2du12_from_fs2g_to_mg(data->xl[1].raw[i]);
            break;
         case LIS2DU12_4g:
            data->xl[0].mg[i] = lis2du12_from_fs4g_to_mg(data->xl[0].raw[i]);
            data->xl[1].mg[i] = lis2du12_from_fs4g_to_mg(data->xl[1].raw[i]);
            break;
         case LIS2DU12_8g:
            data->xl[0].mg[i] = lis2du12_from_fs8g_to_mg(data->xl[0].raw[i]);
            data->xl[1].mg[i] = lis2du12_from_fs8g_to_mg(data->xl[1].raw[i]);
            break;
         case LIS2DU12_16g:
            data->xl[0].mg[i] = lis2du12_from_fs16g_to_mg(data->xl[0].raw[i]);
            data->xl[1].mg[i] = lis2du12_from_fs16g_to_mg(data->xl[1].raw[i]);
            break;
         default:
            data->xl[0].mg[i] = 0.0f;
            data->xl[1].mg[i] = 0.0f;
            break;
      }
   }
   return ret;
}

int32_t lis2du12_interrupt_mode_set(const stmdev_ctx_t *ctx, lis2du12_int_mode_t *val)
{
   lis2du12_ctrl1_t ctrl1;
   lis2du12_interrupt_cfg_t interrupt_cfg;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_INTERRUPT_CFG, (uint8_t*)&interrupt_cfg, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_CTRL1, (uint8_t*)&ctrl1, 1);
   interrupt_cfg.int_short_en = (uint8_t) val->base_sig & 0x01U;
   interrupt_cfg.lir = ((uint8_t) val->base_sig & 0x02U) >> 1;
   interrupt_cfg.h_lactive = val->active_low;
   ctrl1.drdy_pulsed = ~val->drdy_latched;
   interrupt_cfg.interrupts_enable = val->enable;
   ret += lis2du12_write_reg(ctx, LIS2DU12_INTERRUPT_CFG, (uint8_t*)&interrupt_cfg, 1);
   ret += lis2du12_write_reg(ctx, LIS2DU12_CTRL1, (uint8_t*)&ctrl1, 1);
   return ret;
}

int32_t lis2du12_pin_int1_route_set(const stmdev_ctx_t *ctx, lis2du12_pin_int_route_t *val)
{
   lis2du12_ctrl2_t ctrl2;
   lis2du12_md1_cfg_t md1_cfg;
   lis2du12_interrupt_cfg_t interrupt_cfg;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_INTERRUPT_CFG, (uint8_t*)&interrupt_cfg, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_MD1_CFG, (uint8_t*)&md1_cfg, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_CTRL2, (uint8_t*)&ctrl2, 1);
   ctrl2.int1_boot = val->boot;
   ctrl2.int1_drdy = val->drdy_xl;
   ctrl2.int1_f_fth = val->fifo_th;
   ctrl2.int1_f_ovr = val->fifo_ovr;
   ctrl2.int1_f_full = val->fifo_full;
   md1_cfg.int1_double_tap = val->double_tap;
   md1_cfg.int1_6d = val->six_d;
   md1_cfg.int1_wu = val->wake_up;
   md1_cfg.int1_ff = val->free_fall;
   md1_cfg.int1_single_tap = val->single_tap;
   if (val->sleep_state == 1U)
   {
      interrupt_cfg.sleep_status_on_int = PROPERTY_ENABLE;
      md1_cfg.int1_sleep_change = PROPERTY_ENABLE;
   }
   if (val->sleep_change == 1U)
   {
      interrupt_cfg.sleep_status_on_int = PROPERTY_DISABLE;
      md1_cfg.int1_sleep_change = PROPERTY_ENABLE;
   }
   ret += lis2du12_write_reg(ctx, LIS2DU12_INTERRUPT_CFG, (uint8_t*)&interrupt_cfg, 1);
   ret += lis2du12_write_reg(ctx, LIS2DU12_MD1_CFG, (uint8_t*)&md1_cfg, 1);
   ret += lis2du12_write_reg(ctx, LIS2DU12_CTRL2, (uint8_t*)&ctrl2, 1);
   return ret;
}

int32_t lis2du12_pin_int1_route_get(const stmdev_ctx_t *ctx, lis2du12_pin_int_route_t *val)
{
   lis2du12_ctrl2_t ctrl2;
   lis2du12_md1_cfg_t md1_cfg;
   lis2du12_interrupt_cfg_t interrupt_cfg;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_INTERRUPT_CFG, (uint8_t*)&interrupt_cfg, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_MD1_CFG, (uint8_t*)&md1_cfg, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_CTRL2, (uint8_t*)&ctrl2, 1);
   val->boot = ctrl2.int1_boot;
   val->drdy_xl = ctrl2.int1_drdy;
   val->fifo_th = ctrl2.int1_f_fth;
   val->fifo_ovr = ctrl2.int1_f_ovr;
   val->fifo_full = ctrl2.int1_f_full;
   val->double_tap = md1_cfg.int1_double_tap;
   val->six_d = md1_cfg.int1_6d;
   val->wake_up = md1_cfg.int1_wu;
   val->free_fall = md1_cfg.int1_ff;
   val->single_tap = md1_cfg.int1_single_tap;
   val->sleep_state = interrupt_cfg.sleep_status_on_int;
   if (val->sleep_state == PROPERTY_DISABLE)
      val->sleep_change = md1_cfg.int1_sleep_change;
   else
      val->sleep_change = PROPERTY_DISABLE;
   return ret;
}

int32_t lis2du12_pin_int2_route_set(const stmdev_ctx_t *ctx, lis2du12_pin_int_route_t *val)
{
   lis2du12_ctrl3_t ctrl3;
   lis2du12_md2_cfg_t md2_cfg;
   lis2du12_interrupt_cfg_t interrupt_cfg;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_INTERRUPT_CFG, (uint8_t*)&interrupt_cfg, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_MD2_CFG, (uint8_t*)&md2_cfg, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_CTRL3, (uint8_t*)&ctrl3, 1);
   ctrl3.int2_boot = val->boot;
   ctrl3.int2_drdy = val->drdy_xl;
   ctrl3.int2_f_fth = val->fifo_th;
   ctrl3.int2_f_ovr = val->fifo_ovr;
   ctrl3.int2_f_full = val->fifo_full;
   md2_cfg.int2_double_tap = val->double_tap;
   md2_cfg.int2_6d = val->six_d;
   md2_cfg.int2_wu = val->wake_up;
   md2_cfg.int2_ff = val->free_fall;
   md2_cfg.int2_single_tap = val->single_tap;
   if (val->sleep_state == 1U)
   {
      interrupt_cfg.sleep_status_on_int = PROPERTY_ENABLE;
      md2_cfg.int2_sleep_change = PROPERTY_ENABLE;
   }
   if (val->sleep_change == 1U)
   {
      interrupt_cfg.sleep_status_on_int = PROPERTY_DISABLE;
      md2_cfg.int2_sleep_change = PROPERTY_ENABLE;
   }
   ret += lis2du12_write_reg(ctx, LIS2DU12_INTERRUPT_CFG, (uint8_t*)&interrupt_cfg, 1);
   ret += lis2du12_write_reg(ctx, LIS2DU12_MD2_CFG, (uint8_t*)&md2_cfg, 1);
   ret += lis2du12_write_reg(ctx, LIS2DU12_CTRL3, (uint8_t*)&ctrl3, 1);
   return ret;
}

int32_t lis2du12_pin_int2_route_get(const stmdev_ctx_t *ctx, lis2du12_pin_int_route_t *val)
{
   lis2du12_ctrl3_t ctrl3;
   lis2du12_md2_cfg_t md2_cfg;
   lis2du12_interrupt_cfg_t interrupt_cfg;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_INTERRUPT_CFG, (uint8_t*)&interrupt_cfg, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_MD2_CFG, (uint8_t*)&md2_cfg, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_CTRL3, (uint8_t*)&ctrl3, 1);
   val->boot = ctrl3.int2_boot;
   val->drdy_xl = ctrl3.int2_drdy;
   val->fifo_th = ctrl3.int2_f_fth;
   val->fifo_ovr = ctrl3.int2_f_ovr;
   val->fifo_full = ctrl3.int2_f_full;
   val->double_tap = md2_cfg.int2_double_tap;
   val->six_d = md2_cfg.int2_6d;
   val->wake_up = md2_cfg.int2_wu;
   val->free_fall = md2_cfg.int2_ff;
   val->single_tap = md2_cfg.int2_single_tap;
   val->sleep_state = interrupt_cfg.sleep_status_on_int;
   if (val->sleep_state == PROPERTY_DISABLE)
      val->sleep_change = md2_cfg.int2_sleep_change;
   else
      val->sleep_change = PROPERTY_DISABLE;
   return ret;
}

int32_t lis2du12_wake_up_mode_set(const stmdev_ctx_t *ctx, lis2du12_wkup_md_t *val)
{
   lis2du12_ctrl1_t ctrl1;
   lis2du12_ctrl4_t ctrl4;
   lis2du12_md1_cfg_t md1_cfg;
   lis2du12_wake_up_ths_t wake_up_ths;
   lis2du12_wake_up_dur_t wake_up_dur;
   lis2du12_interrupt_cfg_t interrupt_cfg;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_INTERRUPT_CFG, (uint8_t*)&interrupt_cfg, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_WAKE_UP_THS, (uint8_t*)&wake_up_ths, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_WAKE_UP_DUR, (uint8_t*)&wake_up_dur, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_MD1_CFG, (uint8_t*)&md1_cfg, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_CTRL1, (uint8_t*)&ctrl1, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_CTRL4, (uint8_t*)&ctrl4, 1);
   ctrl1.wu_z_en = val->z_en;
   ctrl1.wu_y_en = val->y_en;
   ctrl1.wu_x_en = val->x_en;
   if (val->threshold > 63U)
   {
      interrupt_cfg.wake_ths_w = PROPERTY_ENABLE;
      wake_up_ths.wk_ths = val->threshold / 4U;
   }
   else
   {
      interrupt_cfg.wake_ths_w = PROPERTY_DISABLE;
      wake_up_ths.wk_ths = val->threshold;
   }
   if (val->duration > 3U)
   {
      md1_cfg.wu_dur_x4 = PROPERTY_ENABLE;
      wake_up_dur.wake_dur = val->duration / 4U;
   }
   else
   {
      md1_cfg.wu_dur_x4 = PROPERTY_DISABLE;
      wake_up_dur.wake_dur = val->duration;
   }
   wake_up_ths.sleep_on = val->sleep.en;
   ctrl4.inact_odr = (uint8_t) val->sleep.odr;
   wake_up_dur.sleep_dur = val->sleep.duration;
   ret += lis2du12_write_reg(ctx, LIS2DU12_INTERRUPT_CFG, (uint8_t*)&interrupt_cfg, 1);
   ret += lis2du12_write_reg(ctx, LIS2DU12_WAKE_UP_THS, (uint8_t*)&wake_up_ths, 1);
   ret += lis2du12_write_reg(ctx, LIS2DU12_WAKE_UP_DUR, (uint8_t*)&wake_up_dur, 1);
   ret += lis2du12_write_reg(ctx, LIS2DU12_MD1_CFG, (uint8_t*)&md1_cfg, 1);
   ret += lis2du12_write_reg(ctx, LIS2DU12_CTRL1, (uint8_t*)&ctrl1, 1);
   ret += lis2du12_write_reg(ctx, LIS2DU12_CTRL4, (uint8_t*)&ctrl4, 1);
   return ret;
}

int32_t lis2du12_tap_mode_set(const stmdev_ctx_t *ctx, lis2du12_tap_md_t *val)
{
   lis2du12_int_dur_t int_dur;
   lis2du12_tap_ths_x_t tap_ths_x;
   lis2du12_tap_ths_y_t tap_ths_y;
   lis2du12_tap_ths_z_t tap_ths_z;
   lis2du12_wake_up_ths_t wake_up_ths;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_WAKE_UP_THS, (uint8_t*)&wake_up_ths, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_TAP_THS_X, (uint8_t*)&tap_ths_x, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_TAP_THS_Y, (uint8_t*)&tap_ths_y, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_TAP_THS_Z, (uint8_t*)&tap_ths_z, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_INT_DUR, (uint8_t*)&int_dur, 1);
   tap_ths_z.tap_z_en = val->z_en;
   tap_ths_z.tap_y_en = val->y_en;
   tap_ths_z.tap_x_en = val->x_en;
   tap_ths_x.tap_ths_x = val->threshold.x;
   tap_ths_y.tap_ths_y = val->threshold.y;
   tap_ths_z.tap_ths_z = val->threshold.z;
   int_dur.shock = val->shock;
   int_dur.quiet = val->quiet;
   tap_ths_y.tap_priority = (uint8_t) val->priority;
   wake_up_ths.single_double_tap = val->tap_double.en;
   int_dur.latency = val->tap_double.latency;
   ret += lis2du12_write_reg(ctx, LIS2DU12_WAKE_UP_THS, (uint8_t*)&wake_up_ths, 1);
   ret += lis2du12_write_reg(ctx, LIS2DU12_TAP_THS_X, (uint8_t*)&tap_ths_x, 1);
   ret += lis2du12_write_reg(ctx, LIS2DU12_TAP_THS_Y, (uint8_t*)&tap_ths_y, 1);
   ret += lis2du12_write_reg(ctx, LIS2DU12_TAP_THS_Z, (uint8_t*)&tap_ths_z, 1);
   ret += lis2du12_write_reg(ctx, LIS2DU12_INT_DUR, (uint8_t*)&int_dur, 1);
   return ret;
}

int32_t lis2du12_free_fall_mode_set(const stmdev_ctx_t *ctx, lis2du12_ff_md_t *val)
{
   lis2du12_free_fall_t free_fall;
   lis2du12_wake_up_dur_t wake_up_dur;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_WAKE_UP_DUR, (uint8_t*)&wake_up_dur, 1);
   ret += lis2du12_read_reg(ctx, LIS2DU12_FREE_FALL, (uint8_t*)&free_fall, 1);
   wake_up_dur.ff_dur = val->duration & 0x1FU;
   free_fall.ff_dur = (val->duration) & 0x20U >> 5;
   free_fall.ff_ths = (uint8_t) val->threshold;
   ret += lis2du12_write_reg(ctx, LIS2DU12_WAKE_UP_DUR, (uint8_t*)&wake_up_dur, 1);
   ret += lis2du12_write_reg(ctx, LIS2DU12_FREE_FALL, (uint8_t*)&free_fall, 1);
   return ret;
}

int32_t lis2du12_orientation_mode_set(const stmdev_ctx_t *ctx, lis2du12_orient_md_t *val)
{
   lis2du12_tap_ths_x_t tap_ths_x;
   int32_t ret = lis2du12_read_reg(ctx, LIS2DU12_TAP_THS_X, (uint8_t*)&tap_ths_x, 1);
   tap_ths_x.d6d_ths = (uint8_t) val->threshold;
   tap_ths_x.d4d_en = (uint8_t) val->deg_of_freedom;
   ret += lis2du12_write_reg(ctx, LIS2DU12_TAP_THS_X, (uint8_t*)&tap_ths_x, 1);
   return ret;
}


// Public API Functions ------------------------------------------------------------------------------------------------

void imu_init(void)
{
   // Initialize static variables
   data_ready_callback = NULL;
   motion_change_callback = NULL;
   imu_mode = (lis2du12_md_t){ 0 };
   fifo_mode = (lis2du12_fifo_md_t){ 0 };

   // Create an I2C configuration structure
   const am_hal_iom_config_t i2c_config =
   {
      .eInterfaceMode = AM_HAL_IOM_I2C_MODE,
      .ui32ClockFreq = AM_HAL_IOM_400KHZ,
      .eSpiMode = 0,
      .pNBTxnBuf = NULL,
      .ui32NBTxnBufLength = 0
   };

   // Initialize the IMU driver interface
   imu_context.write_reg = platform_write;
   imu_context.read_reg = platform_read;
   imu_context.handle = NULL;

   // Initialize the I2C module and enable all relevant I2C pins
   am_hal_gpio_pincfg_t scl_config = g_AM_BSP_GPIO_IOM0_SCL;
   am_hal_gpio_pincfg_t sda_config = g_AM_BSP_GPIO_IOM0_SDA;
   scl_config.GP.cfg_b.uFuncSel = PIN_IMU_I2C_SCL_FUNCTION;
   sda_config.GP.cfg_b.uFuncSel = PIN_IMU_I2C_SDA_FUNCTION;
   configASSERT0(am_hal_iom_initialize(IMU_I2C_NUMBER, &i2c_handle));
   configASSERT0(am_hal_gpio_pinconfig(PIN_IMU_I2C_SCL, scl_config));
   configASSERT0(am_hal_gpio_pinconfig(PIN_IMU_I2C_SDA, sda_config));
   am_hal_iom_power_ctrl(i2c_handle, AM_HAL_SYSCTRL_WAKE, false);
   am_hal_iom_configure(i2c_handle, &i2c_config);
   am_hal_iom_enable(i2c_handle);
   system_delay(10 * 1000);

   // Check the IMU Device ID
   int retries = 5;
   lis2du12_id_t id;
   while (--retries && (id.whoami != LIS2DU12_ID))
   {
      system_delay(100);
      lis2du12_id_get(&imu_context, &id);
   }

   // Restore the default chip configuration and set as ready for usage
   lis2du12_status_t status;
   lis2du12_init_set(&imu_context, LIS2DU12_RESET);
   do { lis2du12_status_get(&imu_context, &status); } while (status.sw_reset);
   configASSERT0(lis2du12_init_set(&imu_context, LIS2DU12_DRV_RDY));

   // Disable the I3C bus interface
   configASSERT0(lis2du12_bus_mode_set(&imu_context, LIS2DU12_I3C_DISABLE));

   // Set up incoming interrupts from the IMU
   uint32_t imu_interrupt_pin = PIN_IMU_INTERRUPT;
   configASSERT0(am_hal_gpio_pinconfig(PIN_IMU_INTERRUPT, am_hal_gpio_pincfg_input));
   configASSERT0(am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0, AM_HAL_GPIO_INT_CTRL_INDV_ENABLE, &imu_interrupt_pin));
   configASSERT0(am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, PIN_IMU_INTERRUPT, imu_isr, NULL));
   lis2du12_int_mode_t int_mode = { .enable = PROPERTY_ENABLE, .active_low = PROPERTY_DISABLE, .drdy_latched = 1, .base_sig = LIS2DU12_INT_LEVEL };
   configASSERT0(lis2du12_interrupt_mode_set(&imu_context, &int_mode));
   NVIC_SetPriority(GPIO0_001F_IRQn + GPIO_NUM2IDX(PIN_IMU_INTERRUPT), IMU_DATA_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(GPIO0_001F_IRQn + GPIO_NUM2IDX(PIN_IMU_INTERRUPT));
}

void imu_deinit(void)
{
   // Disable all interrupts and put the device into power-down mode
   if (i2c_handle)
   {
      lis2du12_int_mode_t int_mode = { .enable = PROPERTY_DISABLE, .active_low = PROPERTY_DISABLE, .drdy_latched = 1, .base_sig = LIS2DU12_INT_LEVEL };
      lis2du12_interrupt_mode_set(&imu_context, &int_mode);
      if (data_ready_callback)
         imu_enable_raw_data_output(false, 0, 0, 0, 0, NULL);
      if (motion_change_callback)
         imu_enable_motion_change_detection(false, NULL);
   }

   // Disable all IMU-based interrupts
   uint32_t imu_interrupt_pin = PIN_IMU_INTERRUPT;
   NVIC_DisableIRQ(GPIO0_001F_IRQn + GPIO_NUM2IDX(PIN_IMU_INTERRUPT));
   am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, imu_interrupt_pin, NULL, NULL);
   am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0, AM_HAL_GPIO_INT_CTRL_INDV_DISABLE, &imu_interrupt_pin);

   // Disable all I2C communications
   if (i2c_handle)
   {
      while (am_hal_iom_disable(i2c_handle) != AM_HAL_STATUS_SUCCESS);
      am_hal_iom_uninitialize(i2c_handle);
      i2c_handle = NULL;
   }
}

void imu_enable_raw_data_output(bool enable, lis2du12_fs_t measurement_range, uint32_t data_rate_hz, lis2du12_bw_t bandwidth, uint8_t fifo_depth, imu_data_ready_callback_t callback)
{
   // Enable or disable the output of raw data
   if (enable)
   {
      // Determine the data rate constant closest to the desired output data rate
      lis2du12_odr_t data_rate;
      if (data_rate_hz < 3)
         data_rate = LIS2DU12_1Hz6_ULP;
      else if (data_rate_hz < 5)
         data_rate = LIS2DU12_3Hz_ULP;
      else if (data_rate_hz < 9)
         data_rate = LIS2DU12_6Hz;
      else if (data_rate_hz < 16)
         data_rate = LIS2DU12_12Hz5;
      else if (data_rate_hz < 40)
         data_rate = LIS2DU12_25Hz;
      else if (data_rate_hz < 80)
         data_rate = LIS2DU12_50Hz;
      else if (data_rate_hz < 170)
         data_rate = LIS2DU12_100Hz;
      else if (data_rate_hz < 310)
         data_rate = LIS2DU12_200Hz;
      else if (data_rate_hz < 610)
         data_rate = LIS2DU12_400Hz;
      else
         data_rate = LIS2DU12_800Hz;

      // Configure the data FIFO settings
      fifo_mode.store = LIS2DU12_8_BIT;
      fifo_mode.watermark = MIN(fifo_depth, 100);
      fifo_mode.operation = LIS2DU12_STREAM;
      configASSERT0(lis2du12_fifo_mode_set(&imu_context, &fifo_mode));

      // Configure the output measurement range and data rate
      imu_mode.fs = measurement_range;
      imu_mode.odr = data_rate;
      imu_mode.bw = bandwidth;
      configASSERT0(lis2du12_mode_set(&imu_context, &imu_mode));

      // Enable generation of data-ready interrupts
      lis2du12_pin_int_route_t int_route;
      configASSERT0(lis2du12_pin_int2_route_get(&imu_context, &int_route));
      int_route.fifo_th = PROPERTY_ENABLE;
      configASSERT0(lis2du12_pin_int2_route_set(&imu_context, &int_route));

      // Store the user-supplied data ready callback
      data_ready_callback = callback;
   }
   else
   {
      // Disable generation of data-ready interrupts
      lis2du12_pin_int_route_t int_route;
      configASSERT0(lis2du12_pin_int2_route_get(&imu_context, &int_route));
      int_route.fifo_th = PROPERTY_DISABLE;
      configASSERT0(lis2du12_pin_int2_route_set(&imu_context, &int_route));

      // Disable all output
      imu_mode.odr = LIS2DU12_OFF;
      configASSERT0(lis2du12_mode_set(&imu_context, &imu_mode));

      // Disable the data FIFO
      fifo_mode.operation = LIS2DU12_BYPASS;
      configASSERT0(lis2du12_fifo_mode_set(&imu_context, &fifo_mode));

      // Clear the user-supplied data ready callback
      data_ready_callback = NULL;
   }
}

void imu_enable_motion_change_detection(bool enable, motion_change_callback_t callback)
{
   // Enable or disable to output of motion change notifications
   if (enable)
   {
      // Set the criteria for motion detection:
      //   [80.0 ms (0x01 * 1 / ODR_XL), 9.85 s (MAX(16, 0x01 * 16) / ODR_XL)]
      lis2du12_wkup_md_t motion_mode;
      motion_mode.duration = 0x01;
      motion_mode.threshold = 0x02;
      motion_mode.x_en = motion_mode.y_en = motion_mode.z_en = 1;
      motion_mode.sleep.en = 1;
      motion_mode.sleep.odr = LIS2DU12_SLEEP_AT_1Hz6;
      motion_mode.sleep.duration = 0;
      configASSERT0(lis2du12_wake_up_mode_set(&imu_context, &motion_mode));

      // Enable generation of motion-change interrupts
      lis2du12_pin_int_route_t int_route;
      configASSERT0(lis2du12_pin_int2_route_get(&imu_context, &int_route));
      int_route.sleep_change = PROPERTY_ENABLE;
      int_route.sleep_state = PROPERTY_ENABLE;
      configASSERT0(lis2du12_pin_int2_route_set(&imu_context, &int_route));

      imu_mode.fs =  LIS2DU12_2g;
      imu_mode.odr = LIS2DU12_25Hz;
      lis2du12_mode_set(&imu_context, &imu_mode);

      // Store the user-supplied motion change callback
      motion_change_callback = callback;
   }
   else
   {
      // Disable generation of motion-change interrupts
      lis2du12_pin_int_route_t int_route;
      configASSERT0(lis2du12_pin_int2_route_get(&imu_context, &int_route));
      int_route.sleep_change = PROPERTY_DISABLE;
      int_route.sleep_state = PROPERTY_DISABLE;
      configASSERT0(lis2du12_pin_int2_route_set(&imu_context, &int_route));

      // Disable the motion-change detection functionality
      lis2du12_wkup_md_t motion_mode;
      motion_mode.duration = 0;
      motion_mode.threshold = 0;
      motion_mode.x_en = motion_mode.y_en = motion_mode.z_en = 0;
      motion_mode.sleep.en = 0;
      motion_mode.sleep.odr = LIS2DU12_DO_NOT_CHANGE;
      motion_mode.sleep.duration = 0;
      configASSERT0(lis2du12_wake_up_mode_set(&imu_context, &motion_mode));

      // Clear the user-supplied motion change callback
      motion_change_callback = NULL;
   }
}

void imu_read_accel_data(float *accel_x_mg, float *accel_y_mg, float *accel_z_mg)
{
   static lis2du12_data_t data;
   lis2du12_data_get(&imu_context, &imu_mode, &data);
   *accel_x_mg = data.xl.mg[0];
   *accel_y_mg = data.xl.mg[1];
   *accel_z_mg = data.xl.mg[2];
}
