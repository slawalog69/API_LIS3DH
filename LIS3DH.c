/*!
 * @file LIS3DH.cpp

 */

#include "LIS3DH.h"
#include "Lis3dh_Ext.h"
#include "i2c.h"
#include <string.h>

#define NULL_CHECK(x,y) if(x == NULL) return y
#define WRONG_CHECK(x,vr_val,y) if(x == vr_val) return y
#define IS_IN_RANGE(x,MIN,MAX) ((x)< MIN ? false : (x) > MAX ? false : true)

static inline void simple_delay(u32 val) {
	for (int i = 0; i < val; ++i) {
		u32 t = 1000;
		while (t > 0)
			t--;
	}
}

static u16 calc_period_sampling(lis3dh_odr_t rate, bool LowPower) {
	u16 p = rate == LIS3DH_ODR_1Hz ? 10000 : rate == LIS3DH_ODR_10Hz ? 1000 :
			rate == LIS3DH_ODR_25Hz ? 400 : rate == LIS3DH_ODR_50Hz ? 200 :
			rate == LIS3DH_ODR_100Hz ? 100 : rate == LIS3DH_ODR_200Hz ? 50 :
			rate == LIS3DH_ODR_400Hz ? 25 :
			rate == LIS3DH_ODR_1kHz620_LP ? LowPower ? 6 : 0 : // only in LowPow
			rate == LIS3DH_ODR_5kHz376_LP_1kHz344_NM_HP ? LowPower ? 2 : 7 : 0; // LowPow - 5376hz other -1344Hz
	return p;
}

/*!
 *  @brief  Local variables
 */
static lis3dh_t *curr_Lis2dh_dev = NULL;
static int ini_dev(lis3dh_t *dev);
static int LIS3DH_set_TemlateFunc(lis3dh_template_func_t *func);
static i2c_t link_i2c_t = 255;

static lis3dh_fm_t fifo_mode = LIS3DH_DYNAMIC_STREAM_MODE;
static lis3dh_tr_t fifo_tr = LIS3DH_INT1_GEN;/**< Trigger selection. Default value: 0*/
static u8 fth_wtm_ref = 16;
static lis3dh_int_Polarity_t int_pin_polarity = RISE;
static BDU b_bdu = false; // true;//
static lis3dh_sim_t SPI_3wire = LIS3DH_SPI_4_WIRE;
static lis3dh_st_t Self_tst = LIS3DH_ST_DISABLE;

static u16 nominal_range_mg = 2000;
//˅˅˅˅˅˅˅˅˅˅˅˅˅ int 1 content ˅˅˅˅˅˅˅˅˅˅˅˅˅˅

static u8 Int1_duration_ms = 5; //  ms
static u8 Int1_Cfg = (_YHIE | _XHIE | _ZHIE);
static lis3dh_lir_int1_t Int1_Latch = LIS3DH_INT1_PULSED;
static u16 Int1_Threshold_mg = 400;
//˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄

//˅˅˅˅˅˅˅˅˅˅˅˅˅ int 2 content ˅˅˅˅˅˅˅˅˅˅˅˅˅˅
static u8 Int2_duration_ms = 0; //  ms
static u8 Int2_Cfg = _DIS_ALL;
static lis3dh_lir_int2_t Int2_Latch = LIS3DH_INT2_PULSED;
static u16 Int2_Threshold_mg = 0;  // mg
//˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄

//˅˅˅˅˅˅˅˅˅˅˅˅˅ CTRL_REG2 (21h) content ˅˅˅˅˅˅˅˅˅˅˅˅˅˅
static lis3dh_hpm_t tHPass = LIS3DH_NORMAL_WITH_RST; /**<  High-pass filter  mode */
static FDS fds = false;
static lis3dh_lir_click_t hpclck = LIS3DH_TAP_PULSED;
static HP_IA2 hp_ia2 = false;
static HP_IA1 hp_ia1 = true;
//˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄
static lis3dh_template_func_t lis3dh_func = { 0 };
static u16 period_sampling = 0; // ms x 0.1
static u8 *s_buff_hold = NULL;
static void *s_ext_stream = (void*)0x40004c28; //UART4->TDR // NULL;//
static int16_t xyz[35][3] = { 0 }; /**< x, y and z axis 35 values */
static float xyz_g[3] = { 0 }; /**< axis value (calculated by selected range) */
static lis3dh_ble_t ble = LIS3DH_LSB_AT_LOW_ADD; // Big/little endian data selection.(0: Data LSB; 1: Data MSB )
static lis3dh_Axsis_t axs = LIS3DH_AXSIS_X | LIS3DH_AXSIS_Y | LIS3DH_AXSIS_Z;


//˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄˄


#if defined (MODULE_LIS3DH_SPI)

#elif defined (MODULE_LIS3DH_I2C)
int send_i2C(u8 reg, u8 *val, u8 len) {
#ifdef USE_HAL_DRIVER
	if (HAL_I2C_Mem_Write(&hi2c2, (LIS3DH_DEFAULT_ADDRESS << 1), reg, 1, val, 1,
			100) != 0)
#else
	if(i2c_write_regs ( link_i2c_t , (LIS3DH_DEFAULT_ADDRESS <<1) , reg , val ,  len, 0 ) != 0)
#endif
		return LIS3DH_NOCOM;
	return LIS3DH_OK;
}
int read_i2C_arr(u8 StartReg, u8 *Buff, u8 Len) {
#ifdef USE_HAL_DRIVER
	if (HAL_I2C_Mem_Read(&hi2c2, (LIS3DH_DEFAULT_ADDRESS << 1), StartReg, 1,
			Buff, Len, 100) != 0)
#else
	if(i2c_read_regs(link_i2c_t ,  (LIS3DH_DEFAULT_ADDRESS <<1) , StartReg  ,  Buff ,Len  , 0)!= 0)
#endif
		return LIS3DH_NOCOM;
	return LIS3DH_OK;
}
int read_i2C_one(u8 StartReg, u8 *byte) {
#ifdef USE_HAL_DRIVER
	if (HAL_I2C_Mem_Read(&hi2c2, (LIS3DH_DEFAULT_ADDRESS << 1), StartReg, 1,
			byte, 1, 100) != 0)
#else
	 if(i2c_read_reg(link_i2c_t ,  (LIS3DH_DEFAULT_ADDRESS <<1) , StartReg  , byte ,  0)!= 0)
#endif
		return LIS3DH_NOCOM;
	return LIS3DH_OK;
}
#endif
/*!
 *  @brief  Instantiates a new LIS3DH 
 */

int lis3dh_init(lis3dh_t *dev, const lis3dh_params_t *params,
		lis3dh_int_cb_t cb, void *arg) {
	curr_Lis2dh_dev = dev;
	link_i2c_t = params->i2c;
	lis3dh_template_func_t t_func = { ._read_Arr_ = read_i2C_arr, ._write_ =
			send_i2C, ._read_One_ = read_i2C_one };
	LIS3DH_set_TemlateFunc(&t_func);
	//+++++++++++ ASSERTION SETTINGS ++++++++++++++++

	memcpy(&dev->params, params, sizeof(lis3dh_params_t));

	if (false
			== IS_IN_RANGE(dev->params.op_mode, LIS3DH_HR_12bit, LIS3DH_LP_8bit))
		return LIS3DH_ERROR;

	if (false
			== IS_IN_RANGE(dev->params.odr, LIS3DH_POWER_DOWN,
					LIS3DH_ODR_5kHz376_LP_1kHz344_NM_HP))
		return LIS3DH_ERROR;

	if (false == IS_IN_RANGE(dev->params.scale, LIS3DH_2g, LIS3DH_16g))
		return LIS3DH_ERROR;

	dev->params.int1_mode &= 0xFE;
	dev->params.int2_mode &= 0xF8;

#if defined (MODULE_LIS3DH_I2C)
	if (false
			== IS_IN_RANGE(dev->params.addr, LIS3DH_DEFAULT_ADDRESS,
					LIS3DH_SECOND_ADDRESS))
		return LIS3DH_ERROR;
#endif
	return ini_dev(dev);
}

static int ini_dev(lis3dh_t *dev) {

	NULL_CHECK(lis3dh_func._write_, LIS3DH_NOCOM);
	NULL_CHECK(lis3dh_func._read_Arr_, LIS3DH_NOCOM);
	NULL_CHECK(lis3dh_func._read_One_, LIS3DH_NOCOM);
	WRONG_CHECK(link_i2c_t, 255, LIS3DH_ERROR);
// ++++++++++++ assertion is devise on line +++++++++++++++

	/* Check connection */
	if (LIS3DH_getDeviceID() != LIS3DH_ID) /* No LIS3DH detected -return  */
		return LIS3DH_NODEV;
// ++++++++++++++++ registration start settings +++++++++++++++++++++

	for (int r = 0; r < dev->params.scale; r++) {/**<  convert range id to mG diapazon */
		nominal_range_mg <<= 1;
	}
//	m_LIS3DH.dataRate = ls3dh_sett->Rate_Sampling;
	period_sampling = calc_period_sampling(dev->params.odr,
			dev->params.op_mode == LIS3DH_LP_8bit);
//	m_LIS3DH.Op_Mode = ls3dh_sett->Operating_Mode;

	if (s_buff_hold == NULL)
//		m_LIS3DH.s_buff_hold = ls3dh_sett->buff_hold;
//	else
		s_buff_hold = (u8*) xyz;

//	m_LIS3DH.s_ext_stream = ls3dh_sett->ext_stream;
	//***************

	u8 init_lis3d = (dev->params.odr << 4)
			| (dev->params.op_mode == LIS3DH_LP_8bit ? 0x08 : 0) | axs;
	lis3dh_func._write_(LIS3DH_REG_CTRL1, &init_lis3d, 1);

	init_lis3d = (tHPass << 6) | (fds << 3) | (hpclck << 2) | (hp_ia2 << 1)
			| (hp_ia1 << 0);

	lis3dh_func._write_(LIS3DH_REG_CTRL2, &init_lis3d, 1);

////  int1 config
	init_lis3d = (1 << dev->params.int1_mode );
	lis3dh_func._write_(LIS3DH_REG_CTRL3, &init_lis3d , 1);
	if (dev->params.int1_mode != I1_DISABLE) {

		u8 val_Ths = Int1_Threshold_mg / (nominal_range_mg >> 7);
		lis3dh_func._write_(LIS3DH_REG_INT1THS, &val_Ths, 1);

		u8 INT1_DURATION =
				period_sampling != 0 ?
						(u32) (((float) Int1_duration_ms / period_sampling)
								* 10.0) :
						0;
		lis3dh_func._write_(LIS3DH_REG_INT1DUR, &INT1_DURATION, 1);

		//*******make hp filter reference************
		lis3dh_func._read_One_(LIS3DH_REG_REFERENCE, &val_Ths);
		//****************************

		lis3dh_func._write_( LIS3DH_REG_INT1CFG, &Int1_Cfg, 1);
	}

	init_lis3d = (b_bdu << 7) | (ble << 6) | (dev->params.scale << 4)
			| (dev->params.op_mode == LIS3DH_HR_12bit ? 0x08 : 0)
			| (Self_tst << 1) | ((u8) SPI_3wire);
	lis3dh_func._write_(LIS3DH_REG_CTRL4, &init_lis3d, 1);

	init_lis3d = (fifo_mode != LIS3DH_BYPASS_MODE ? 0x40 : 0)
			| (Int1_Latch << 3)   // latch irq
			| (Int2_Latch << 1);   // latch irq
	lis3dh_func._write_(LIS3DH_REG_CTRL5, &init_lis3d, 1);

////  int2 config
	init_lis3d = (int_pin_polarity << 1);
	if (dev->params.int2_mode != I2_DISABLE) {
		init_lis3d |= dev->params.int2_mode;
		u8 val_Ths = Int2_Threshold_mg / (nominal_range_mg >> 7);
		lis3dh_func._write_(LIS3DH_REG_INT2THS, &val_Ths, 1);
		u8 INT2_DURATION =
				period_sampling != 0 ?
						(u32) (((float) Int2_duration_ms / period_sampling)
								* 10.0) :
						0;
		lis3dh_func._write_(LIS3DH_REG_INT2DUR, &INT2_DURATION, 1);
		lis3dh_func._write_( LIS3DH_REG_INT2CFG, &Int2_Cfg, 1);
	}
	lis3dh_func._write_(LIS3DH_REG_CTRL6, &init_lis3d, 1);

	init_lis3d = (fifo_mode << 6) | (fifo_tr << 5) | (fth_wtm_ref);
	lis3dh_func._write_(LIS3DH_REG_FIFOCTRL, &init_lis3d, 1);

	u8 dumm = LIS3DH_read_INT1_IRQ();
	dumm = LIS3DH_read_INT2_IRQ();
//   LIS3DH_get_Values(1);
	lis3dh_func._read_One_( LIS3DH_REG_STATUS2, &dumm);
//   dumm =LIS3DH_read_INT2_IRQ();
	return 0;
}

/*!  @brief  Sets the data rate for the LIS3DH (controls power consumption)
 *  @param  dataRate           data rate value
 */
void LIS3DH_setDataRate(lis3dh_odr_t dataRate) {

	u8 t;
	lis3dh_func._read_One_( LIS3DH_REG_CTRL1, &t);

	t &= 0x0F;
	t |= (dataRate << 4);
	lis3dh_func._write_(LIS3DH_REG_CTRL1, &t, 1);
}

/*!
 *   @brief  Gets the data rate for the LIS3DH (ODR)
 *   @return Returns Data Rate value
 */
lis3dh_odr_t LIS3DH_getDataRate(void) {
	u8 t;
	lis3dh_func._read_One_( LIS3DH_REG_CTRL1, &t);

	return (lis3dh_odr_t) (t >> 4);
}
/*!   @brief  Sets the g range for the accelerometer (FS)
 *   @param  range  value
 */
void LIS3DH_setRange(lis3dh_fs_t range) {
	u8 t;
	lis3dh_func._read_One_( LIS3DH_REG_CTRL4, &t);
	t &= 0xCF;
	t |= (range << 4);
	lis3dh_func._write_( LIS3DH_REG_CTRL4, &t, 1);
//  delay(15); // delay to let new setting settle
	simple_delay(15);
}
/*!  @brief  Sets  operating_modes the accelerometer
 *   @param   op_mode Low_power High_res Low_power
 *
 */
void LIS3DH_set_op_mode(lis3dh_op_md_t op_mode) {
	u8 t;
	lis3dh_func._read_One_( LIS3DH_REG_CTRL1, &t);
	u8 bit = op_mode == LIS3DH_LP_8bit ? 0x08 : 0;
	t &= 0xF7;
	t |= bit;
	lis3dh_func._write_( LIS3DH_REG_CTRL1, &t, 1);

	lis3dh_func._read_One_( LIS3DH_REG_CTRL4, &t);
	bit = op_mode == LIS3DH_HR_12bit ? 0x08 : 0;
	t &= 0xF7;
	t |= bit;
	lis3dh_func._write_( LIS3DH_REG_CTRL4, &t, 1);
//  delay(15); // delay to let new setting settle
	HAL_Delay(15);
}
/*!
 *   @brief  Get  operating_mode accelerometer
 *   @return   op_mode Low_power High_res Low_power
 *
 */
lis3dh_op_md_t LIS3DH_get_op_mode(void) {

	u8 lowpwr_m, high_res_m;
	lis3dh_func._read_One_( LIS3DH_REG_CTRL1, &lowpwr_m);
	lowpwr_m >>= 3;
	lowpwr_m &= 1;
	lis3dh_func._read_One_( LIS3DH_REG_CTRL4, &high_res_m);
	high_res_m >>= 3;
	high_res_m &= 1;
	lis3dh_op_md_t ret = lowpwr_m == 1 ? LIS3DH_LP_8bit :
							high_res_m == 1 ? LIS3DH_HR_12bit : LIS3DH_NM_10bit;
	return ret;
}

/*!
 *  @brief  Gets the g range for the accelerometer
 *  @return Returns g range value
 */
lis3dh_fs_t LIS3DH_getRange(void) {

	u8 t;
	lis3dh_func._read_One_( LIS3DH_REG_CTRL4, &t);
	t >>= 4;
	t &= 0x03;
	return (lis3dh_fs_t) (t);
}

void Temperature_sensor_enable(bool OnOff) {

	u8 t;
	lis3dh_func._read_One_( LIS3DH_REG_TEMPCFG, &t);
	t &= 0xBF;
	t |= (OnOff << 6);
	lis3dh_func._write_(LIS3DH_REG_TEMPCFG, &t, 1);
}

void LIS3DH_ADC_sensor_enable(bool OnOff) {
	u8 t;
	lis3dh_func._read_One_( LIS3DH_REG_TEMPCFG, &t);
	t &= 0x7F;
	t |= (OnOff << 7);
	lis3dh_func._write_(LIS3DH_REG_TEMPCFG, &t, 1);

//  return true;
}

/*!
 *  @brief  Get Device ID from LIS3DH_REG_WHOAMI
 *  @return WHO AM I value
 */
u8 LIS3DH_getDeviceID(void) {
	u8 t;
	lis3dh_func._read_One_(LIS3DH_REG_WHOAMI, &t);
	return t;
}
/*!
 *  @brief  Check to see if new data available
 *  @return true if there is new data available, false otherwise
 */
bool LIS3DH_haveNewData(void) {
	u8 status_2;
	lis3dh_func._read_One_(LIS3DH_REG_STATUS2, &status_2);
	status_2 &= (1 << 3);
	return ((status_2 & (1 << 3)) != 0);
}

/*!
 *  @brief  Reads x y z values to external stream (example UART4)
 */

extern HAL_StatusTypeDef m_I2C_Read_to_stream(I2C_HandleTypeDef *hi2c,
		u16 DevAddress, u16 MemAddress, u8 *pData, u8 Size);

void LIS3DH_get_Val_to_stream(int num_samples_read) {
	m_I2C_Read_to_stream(&hi2c2, (LIS3DH_DEFAULT_ADDRESS << 1),
			LIS3DH_REG_READ_ALL, s_ext_stream, num_samples_read * 6);
//                                     1, m_LIS3DH.s_ext_stream , num_samples_read *6 , 100);
}

/*!
 *  @brief  Reads x y z values at once
 */

void LIS3DH_get_Values(int num_samples_read) {
	// AccSens_BusIO_Register xl_data = AccSens_BusIO_Register(
	//     i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, register_address, 6);
	if (s_ext_stream != NULL) { //LIS3DH_get_Val_to_stream( num_samples_read );
		m_I2C_Read_to_stream(&hi2c2, (LIS3DH_DEFAULT_ADDRESS << 1),
		LIS3DH_REG_READ_ALL, // 1,
				s_ext_stream, num_samples_read * 6); //, 100);

	} else
		lis3dh_func._read_Arr_(LIS3DH_REG_READ_ALL, s_buff_hold,
				num_samples_read * 6);

}

/*!
 *  @brief  Reads x y z values at once
 */

void LIS3DH_calc_Val_g(int indx) {

	u8 lsb_val;
	LIS3DH_get_Values(6);
	lis3dh_fs_t range = LIS3DH_getRange();
	u8 lowpwr_m, high_res_m;
	lis3dh_func._read_One_( LIS3DH_REG_CTRL1, &lowpwr_m);
	lowpwr_m >>= 3;
	lowpwr_m &= 1;
	lis3dh_func._read_One_( LIS3DH_REG_CTRL4, &high_res_m);
	high_res_m >>= 3;
	high_res_m &= 1;

	lsb_val = range == LIS3DH_2g ? 1 : range == LIS3DH_4g ? 2 :
				range == LIS3DH_8g ? 4 : range == LIS3DH_16g ? 12 : 1;
	u8 shft = 4;
	if (lowpwr_m == 1) {
		lsb_val <<= 4;
		shft = 8;
	} else if (high_res_m == 0) {
		lsb_val <<= 2;
		shft = 6;
	}; // normal

	xyz_g[0] = (float) ((xyz[indx][0] >> shft) * lsb_val) / 1000.0;
	xyz_g[1] = (float) ((xyz[indx][1] >> shft) * lsb_val) / 1000.0;
	xyz_g[2] = (float) ((xyz[indx][2] >> shft) * lsb_val) / 1000.0;
}

/*!
 *   @brief  Get u8 for INT1 source and clear interrupt
 *   @return register LIS3DH_REG_INT1SRC
 */
u8 LIS3DH_read_INT1_IRQ(void) {
	u8 t;
	lis3dh_func._read_One_( LIS3DH_REG_INT1SRC, &t);
	return t;
}

/*!
 *   @brief  Get u8 for INT1 source and clear interrupt
 *   @return register LIS3DH_REG_INT1SRC
 */
u8 LIS3DH_read_INT2_IRQ(void) {
	u8 t;
	lis3dh_func._read_One_( LIS3DH_REG_INT2SRC, &t);
	return t;
}

/**  @brief Enable or disable the  interupt on INT1 
 * @param int_pin which IRQ func  INT_1_PIN    
 */
void LIS3DH_setIrq_On_INT_1(lis3dh_int1_md_t Irq_func) { //
	u8 wr = Irq_func == I1_DISABLE ? 0 : (1 << Irq_func);
	lis3dh_func._write_(LIS3DH_REG_CTRL3, &wr, 1);
}

/**  @brief Enable or disable the  interupt on INT2
 * @param int_pin which IRQ func on INT_2_PIN    
 */
void LIS3DH_setIrq_On_INT_2(lis3dh_int2_md_t Irq_func) { //
	u8 wr = Irq_func == I2_DISABLE ? 0 : (1 << Irq_func);
	u8 t;
	lis3dh_func._read_One_( LIS3DH_REG_CTRL6, &t);
	t &= 0x2;
	t |= wr;
	lis3dh_func._write_(LIS3DH_REG_CTRL6, &wr, 1);

}
/**  @brief Set  INT1 and INT2 pin polarity.
 * @param Pol    0: active-high; 1: active-low 
 */
void LIS3DH_setPolarity_On_INTs( bool Pol) { //
	u8 t;
	lis3dh_func._read_One_( LIS3DH_REG_CTRL6, &t);
	t &= 0x2;
	t |= (Pol << 1);
	lis3dh_func._write_(LIS3DH_REG_CTRL6, &t, 1);
}

/**  @brief Set  INT1  config.
 * @param  Cfg
 */
void LIS3DH_setConfig_On_INT1(u8 Cfg) { //
	lis3dh_func._write_( LIS3DH_REG_INT1CFG, &Cfg, 1);
}

/**  @brief Set  INT2  config.
 * @param  Cfg
 */
void LIS3DH_setConfig_On_INT2(u8 Cfg) { //
	lis3dh_func._write_( LIS3DH_REG_INT2CFG, &Cfg, 1);
}
/**  @brief Set  FIFO FTH (ref watermark)
 * @param  Cfg
 */
void LIS3DH_setFTH_wtm(u8 wtm) { //
	u8 t;
	lis3dh_func._read_One_( LIS3DH_REG_FIFOCTRL, &t);
	t &= 0xE0;
	t |= (wtm & 0x1F);
	lis3dh_func._write_(LIS3DH_REG_FIFOCTRL, &t, 1);
}
/**  @brief Set transport func
 * @param  lis3dh_template_func_t
 */
int LIS3DH_set_TemlateFunc(lis3dh_template_func_t *func) { //
	NULL_CHECK(func->_write_, LIS3DH_NOCOM);
	NULL_CHECK(func->_read_Arr_, LIS3DH_NOCOM);
	NULL_CHECK(func->_read_One_, LIS3DH_NOCOM);
	memcpy(&lis3dh_func, func, sizeof(lis3dh_template_func_t));
	return LIS3DH_OK;
}

//+++++++++++++++++++++++++++++++++++++++++

int lis3dh_reinit(lis3dh_t *dev, u8 scale, u8 mode) {
	dev->params.op_mode = mode;
	dev->params.scale = scale;
	return ini_dev(dev);
}

int32_t lis3dh_int1_duration_set(lis3dh_t *dev, uint16_t val) {
	if (val > 127)
		val = 127;
	Int1_duration_ms = (u32) ((float) val * period_sampling / 10.0);
	lis3dh_func._write_(LIS3DH_REG_INT1DUR, (u8*) &val, 1);
	return LIS3DH_OK;
}

int32_t lis3dh_int2_duration_set(lis3dh_t *dev, uint16_t val) {
	if (val > 127)
		val = 127;
	Int2_duration_ms = (u32) ((float) val * period_sampling / 10.0);
	lis3dh_func._write_(LIS3DH_REG_INT2DUR, (u8*) &val, 1);
	return LIS3DH_OK;
}

int32_t lis3dh_int1_ths_set(lis3dh_t *dev, uint16_t val) {
	Int1_Threshold_mg = val * (nominal_range_mg >> 7);
	lis3dh_func._write_(LIS3DH_REG_INT1THS, (u8*) &val, 1);
	return LIS3DH_OK;
}

int32_t lis3dh_int1_ths_get(lis3dh_t *dev, uint16_t *val) {
	lis3dh_func._read_One_(LIS3DH_REG_INT1THS, (u8*) val);
	return LIS3DH_OK;
}

int32_t lis3dh_int1_cfg_set(lis3dh_t *dev, u8 val) {
	LIS3DH_setConfig_On_INT1(val);
	return LIS3DH_OK;
}
int32_t lis3dh_int1_src_get(lis3dh_t *dev, u8 *val) {
	*val = LIS3DH_read_INT1_IRQ();
	return LIS3DH_OK;
}
s32 lis3dh_read_reg(lis3dh_t *dev, u8 reg, u8 *data, u16 len) {
	return lis3dh_func._read_Arr_(reg, data, len);
}
s32 lis3dh_write_reg(lis3dh_t *dev, u8 reg, u8 *data, uint16_t len) {
	return lis3dh_func._write_(reg, data, len);
}
int32_t lis3dh_fifo_mode_set(lis3dh_t *dev, lis3dh_fm_t val) {
	u8 t;
	lis3dh_func._read_Arr_(LIS3DH_REG_FIFOCTRL, &t, 1);
	t &= 0x3F;
	t |= (val << 6);
	return lis3dh_func._write_(LIS3DH_REG_FIFOCTRL, &t, 1);
}

int lis3dh_read_xyz_cont(lis3dh_t *dev, lis3dh_acceleration_t *acceleration,
		u8 count) {
	return lis3dh_func._read_Arr_(LIS3DH_REG_READ_ALL, (u8*) acceleration,
			count * 6);
}

// **********        IRQ     ************

static u8 Que_int1 = 0;
static u8 stat_i1;
static u16 DataRead_cnt = 0;

lis3dh_acceleration_t arr[32];
#ifdef USE_HAL_DRIVER
#include "usart.h"
#endif
#include "CycleCount.h"
static inline void CallbckReadData() {

	stat_i1 = LIS3DH_read_INT1_IRQ();
	AddCycles(); //GetCycles();
	/// ~~~~~~~~~  FIFO OVR  ~~~~~~~~~~~~~~~~
	if (Que_int1 == 0) {
		lis3dh_read_xyz_cont(curr_Lis2dh_dev, arr, 32);
//		LIS3DH_get_Values(32);
		LIS3DH_setIrq_On_INT_1(I1_WTM);
		DataRead_cnt = 8;
		Que_int1 = 1;
		/// ~~~~~~~~~  FIFO WTM  ~~~~~~~~~~~~~~~~
	} else if (Que_int1 == 1) {
		lis3dh_read_xyz_cont(curr_Lis2dh_dev, arr, 16);  //(16);  //
//		LIS3DH_get_Values(16);
		if (DataRead_cnt)
			DataRead_cnt--;
		if (DataRead_cnt == 0) {
			Que_int1 = 0;
			LIS3DH_setIrq_On_INT_1(I1_IA1);
			lis3dh_int1_cfg_set(curr_Lis2dh_dev, (_YHIE | _XHIE | _ZHIE));
#ifdef USE_HAL_DRIVER
			HAL_UART_Transmit(&huart4, (u8*) "END\r\n", 5, 100);
#endif
			ClrCycles();
		}
	}
}

void HAL_GPIO_EXTI_Callback(u16 GPIO_Pin) {
	// ***********  int2  **********
	if (GPIO_Pin == curr_Lis2dh_dev->params.int2) {   // int2
//		osEventFlagsSet(myEvent_IRQ_LISHandle, 0xBB);
			// ***********  int1  **********
	} else if (GPIO_Pin == curr_Lis2dh_dev->params.int1) {
//		osEventFlagsSet (myEvent_IRQ_LISHandle , 0xAA);

		startCycle();
		CallbckReadData();
	}
}

