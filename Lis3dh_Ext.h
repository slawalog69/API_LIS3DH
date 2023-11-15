/*
 * Lis3dh_Ext.h
 *
 *  Created on: 14 нояб. 2023 г.
 *      Author: Slava
 */

#ifndef LIS3DH_EXT_H_
#define LIS3DH_EXT_H_


#include <stdbool.h>
#include "LIS3DH_regs.h"

/**
 * @brief   transport func template
 */
typedef struct {
 int (*_write_)(uint8_t StartReg, uint8_t *val,uint8_t Len );/**< template func write throw I2C(SPI) */
 int  (*_read_Arr_)(uint8_t StartReg, uint8_t* Buff, uint8_t Len);/**< template func read multibytes array throw I2C(SPI) */
 int  (*_read_One_)(uint8_t StartReg, uint8_t* Byte);/**< template func read multibytes array throw I2C(SPI) */

} lis3dh_template_func_t;

#define LIS3DH_DEFAULT_ADDRESS (0x18)  /**<   if SDO/SA0 is 3V, its 0x19  */
#define LIS3DH_SECOND_ADDRESS (0x19)  /**<   if SDO/SA0 is 3V, its 0x19  */
#define LIS3DH_ID (0x33)   /**<    REG 0x0F WHOAMI const ID   */




/** @brief A structure to represent all INTx pins polarity. **/
typedef enum {
  RISE = 0x0,
  FAIL = 0x1
} lis3dh_int_Polarity_t;


/** @brief Enable axis.**/
typedef enum {
  LIS3DH_AXSIS_Z  = 0b100, //
  LIS3DH_AXSIS_Y  = 0b010,  //
  LIS3DH_AXSIS_X = 0b001,  //
} lis3dh_Axsis_t;



/** @brief  Block data update.  flag */
typedef bool  BDU;

/** @brief  Filtered data selection. Default value: 0
(0: internal filter bypassed;
1: data from internal filter sent to output register and FIFO) flag */
typedef bool  FDS;

/** @brief High-pass filter enabled for AOI function on interrupt 2, flag */
typedef bool  HP_IA2;
/** @brief High-pass filter enabled for AOI function on interrupt 1, flag */
typedef bool  HP_IA1;



/*!
 *  @brief  Get Device ID from LIS3DH_REG_WHOAMI
 *  @return WHO AM I value
 */

uint8_t LIS3DH_getDeviceID(void); /**< check if device on line by read ID */
void LIS3DH_setDataRate(lis3dh_odr_t dataRate); /**< set DataRate  */
lis3dh_odr_t LIS3DH_getDataRate(void); /**< get DataRate  */
void LIS3DH_setRange(lis3dh_fs_t range);/**< set Range */
lis3dh_fs_t LIS3DH_getRange(void);/**< get Range */
void LIS3DH_setPolarity_On_INTs( bool Pol) ;
void LIS3DH_setIrq_On_INT_2( lis3dh_int2_md_t Irq_func) ;/**< set   func on int2 output */
void LIS3DH_setIrq_On_INT_1( lis3dh_int1_md_t Irq_func) ;/**< set   func on int1 output */
uint8_t LIS3DH_read_INT2_IRQ(void); /**< get stat on int2 output */
uint8_t LIS3DH_read_INT1_IRQ(void) ;/**< get stat on int1 output */
lis3dh_op_md_t LIS3DH_get_op_mode (void);/**< get operating mode accelerometer  */
void LIS3DH_set_op_mode (lis3dh_op_md_t op_mode);/**< set operating mode accelerometer  */
void LIS3DH_get_Values (int num_samples_read );/**< read data as  (x y z) sample from device */
bool LIS3DH_haveNewData(void);/**< read flag dataready */
void LIS3DH_ADC_sensor_enable(bool OnOff);/**< enable ADC functional */
void AccSens_Temperature_sensor_enable(bool OnOff);/**< enable Temp sensor functional */

void LIS3DH_calc_Val_g(int indx); /**< convert Raw data x y z to values of g */

void LIS3DH_get_Val_to_stream(int num_samples_read );
void LIS3DH_setConfig_On_INT1( u8 Cfg) ; /**< Set  INT1  config. */
void LIS3DH_setConfig_On_INT2( u8 Cfg); /**<  Set  INT2  config. */

void LIS3DH_setFTH_wtm( u8 wtm);/**< Set  FIFO FTH (ref watermark)  */



// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&



#endif /* LIS3DH_EXT_H_ */
