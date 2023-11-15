/**
 * @defgroup    drivers_lis3dh LIS3DH accelerometer
 * @ingroup     drivers_sensors
 * @brief       Device driver for the LIS3DH accelerometer
 *
 * @files       lis3dh.h
 * @brief       Device driver interface for the LIS3DH accelerometer
 */
 
#ifndef LIS3DH_H
#define LIS3DH_H

#include <stdint.h>

//#include "riotbuild.h"

#if defined (MODULE_LIS3DH_SPI)
#include "periph/spi.h"
#elif defined (MODULE_LIS3DH_I2C)
#include "periph/i2c.h"
#endif

#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    LIS3DH values registers enumirations
 * @{ 
 */

/**
 * @brief   SDO/SA0 pull-up 
 */
typedef enum {
    LIS3DH_PULL_UP_CONNECT     = 0,
    LIS3DH_PULL_UP_DISCONNECT  = 1
} lis3dh_sdo_pu_disc_t;

/**
 * @drief   AUX connected to temperature sensor or ADC
 */
typedef enum {
    LIS3DH_AUX_ON_TEMPERATURE   = 0x03,                 /**< Temperature sensor enable */
    LIS3DH_AUX_DISABLE          = 0x00,                 /**< AUX disable */
    LIS3DH_AUX_ON_PADS          = 0x01,                 /**< AUX pin connected to ADC */
} lis3dh_temp_en_t;

/**
 * @brief   Output data rate [Hz]
 */
typedef enum {
    LIS3DH_POWER_DOWN                      = 0x00,      /**< Power-down mode */
    LIS3DH_ODR_1Hz                         = 0x01,      /**< HR / Normal / Low-power mode (1 Hz) */
    LIS3DH_ODR_10Hz                        = 0x02,      /**< HR / Normal / Low-power mode (10 Hz) */
    LIS3DH_ODR_25Hz                        = 0x03,      /**< HR / Normal / Low-power mode (25 Hz) */
    LIS3DH_ODR_50Hz                        = 0x04,      /**< HR / Normal / Low-power mode (50 Hz) */
    LIS3DH_ODR_100Hz                       = 0x05,      /**< HR / Normal / Low-power mode (100 Hz) */
    LIS3DH_ODR_200Hz                       = 0x06,      /**< HR / Normal / Low-power mode (200 Hz) */
    LIS3DH_ODR_400Hz                       = 0x07,      /**< HR / Normal / Low-power mode (400 Hz) */
    LIS3DH_ODR_1kHz620_LP                  = 0x08,      /**< Low power mode (1.60 kHz) */
    LIS3DH_ODR_5kHz376_LP_1kHz344_NM_HP    = 0x09,      /**< HR / normal (1.344 kHz); Low-power mode (5.376 kHz) */
} lis3dh_odr_t;

/**
 * @brief   Operating mode
 */
typedef enum {
    LIS3DH_HR_12bit   = 0,                              /**< High-resolution mode */
    LIS3DH_NM_10bit   = 1,                              /**< Normal mode */
    LIS3DH_LP_8bit    = 2,                              /**< Low-power mode */
} lis3dh_op_md_t;

/**
 * @brief   Available full-scale values
 */
typedef enum {
    LIS3DH_2g   = 0,                                    /**< +- 2g */
    LIS3DH_4g   = 1,                                    /**< +- 4g */
    LIS3DH_8g   = 2,                                    /**< +- 8g */
    LIS3DH_16g  = 3,                                    /**< +- 16g */
} lis3dh_fs_t;

/**
 * @brief   Axes enables
 */
typedef enum {
    LIS3DH_AXES_ALL_DISABLE  = 0,
    LIS3DH_AXES_ALL_ENABLE   = 1,
} lis3dh_axis_t;

/**
 * High-pass filter mode
 */
typedef enum {
  LIS3DH_NORMAL_WITH_RST  = 0,                          /**< Normal mode (reset by reading REFERENCE (26h)) */
  LIS3DH_REFERENCE_MODE   = 1,                          /**< Reference signal for filtering */
  LIS3DH_NORMAL           = 2,                          /**< Normal mode */
  LIS3DH_AUTORST_ON_INT   = 3,                          /**< Autoreset on interrupt event */
} lis3dh_hpm_t;


/**
 * @brief   High-pass filter cutoff frequency selection.
 *          See section 4, subsection 4.3.1, 
 *          table 11 of the document AN3308
 */
typedef enum {
  LIS3DH_AGGRESSIVE  = 0,
  LIS3DH_STRONG      = 1,
  LIS3DH_MEDIUM      = 2,
  LIS3DH_LIGHT       = 3,
} lis3dh_hpcf_t;

/**
 * @brief   Big/little endian data mode
 */
typedef enum {
  LIS3DH_LSB_AT_LOW_ADD = 0,                            /**< Little endian */
  LIS3DH_MSB_AT_LOW_ADD = 1,                            /**< Big endian */
} lis3dh_ble_t;


/**
 * @brief   High-pass filter on interrupts/tap generator mode 
 */
typedef enum {
    LIS3DH_DISC_FROM_INT_GENERATOR  = 0,
    LIS3DH_ON_INT1_GEN              = 1,
    LIS3DH_ON_INT2_GEN              = 2,
    LIS3DH_ON_TAP_GEN               = 4,
    LIS3DH_ON_INT1_INT2_GEN         = 3,
    LIS3DH_ON_INT1_TAP_GEN          = 5,
    LIS3DH_ON_INT2_TAP_GEN          = 6,
    LIS3DH_ON_INT1_INT2_TAP_GEN     = 7,
} lis3dh_hp_t;

/**
 * @brief   Interrupt 1 pin mode
 */
typedef enum {
    I1_DISABLE  = 0,                                    /**< Disable all on INT1 */
    I1_OVERRUN  = 1,                                    /**< FIFO overrun interrupt on INT1 */
    I1_WTM      = 2,                                    /**< FIFO watermark interrupt on INT1 */
    I1_321DA    = 3,                                    /**< 321DA interrupt on INT1 */
    I1_ZYXDA    = 4,                                    /**< ZYXDA interrupt on INT1 */
    I1_IA2      = 5,                                    /**< IA2 interrupt on INT1 */
    I1_IA1      = 6,                                    /**< IA1 interrupt on INT1 */
    I1_CLICK    = 7,                                    /**< Click interrupt on INT1 */
} lis3dh_int1_md_t;

/**
 * @brief   Interrupt 1 configuration
 */
enum lis3dh_int1_cfg_t {
    _XLIE = 0x01,                                    	/**< Enable interrupt generation on X low event  */
    _XHIE = 0x02,                                    	/**< Enable interrupt generation on X high event */
    _YLIE = 0x04,                                    	/**< Enable interrupt generation on Y low event  */
    _YHIE = 0x08,                                    	/**< Enable interrupt generation on Y high event */
    _ZLIE = 0x10,                                    	/**< Enable interrupt generation on Z low event  */
    _ZHIE = 0x20,                                    	/**< Enable interrupt generation on Z high event */
    _6D   = 0x40,                                    	/**< 6 direction detection function enabled */
    _AOI  = 0x80,                                    	/**< And/Or combination of Interrupt events */
	_DIS_ALL = 0										/**< Disable all on INT1 */
};

/**
 * @brief   Interrupt 2 pin mode
 */
typedef enum {
    I2_DISABLE    = 0,                                  /**< Disable all on INT2 */
    INT_POLARITY  = 1,                                  /**< INT1 and INT2 pin polarity */
    I2_ACT        = 3,                                  /**< Enable activity interrupt on INT2 pin */
    I2_BOOT       = 4,                                  /**< Enable boot on INT2 pin */
    I2_IA2        = 5,                                  /**< Enable interrupt 2 function on INT2 pin */
    I2_IA1        = 6,                                  /**< Enable interrupt 1 function on INT2 pin */
    I2_CLICK      = 7,                                  /**< Click interrupt on INT2 pin */
} lis3dh_int2_md_t;
   
/**
 * @brief   Latch interrupt 1 mode
 */
typedef enum {
  LIS3DH_INT1_PULSED   = 0,
  LIS3DH_INT1_LATCHED  = 1,
} lis3dh_lir_int1_t;

/**
 * @brief   Latch interrupt 2 mode
 */
typedef enum {
  LIS3DH_INT2_PULSED   = 0,
  LIS3DH_INT2_LATCHED  = 1,
} lis3dh_lir_int2_t;

/**
 * @brief    Trigger for activate Stream-to-FIFO
 */
typedef enum {
  LIS3DH_INT1_GEN = 0,
  LIS3DH_INT2_GEN = 1,
} lis3dh_tr_t;

/**
 * @brief   FIFO buffer mode
 */
typedef enum {
  LIS3DH_BYPASS_MODE           = 0,
  LIS3DH_FIFO_MODE             = 1,
  LIS3DH_DYNAMIC_STREAM_MODE   = 2,
  LIS3DH_STREAM_TO_FIFO_MODE   = 3,
} lis3dh_fm_t;

/**
 * @brief   Latch click mode
 */
typedef enum {
  LIS3DH_TAP_PULSED   = 0,
  LIS3DH_TAP_LATCHED  = 1,
} lis3dh_lir_click_t;

/**
 * @brief    Self-test mode
 */
typedef enum {
  LIS3DH_ST_DISABLE   = 0,
  LIS3DH_ST_POSITIVE  = 1,
  LIS3DH_ST_NEGATIVE  = 2, 
} lis3dh_st_t;

/**
 * @brief   SPI serial interface mode
 */
typedef enum {
  LIS3DH_SPI_4_WIRE = 0,
  LIS3DH_SPI_3_WIRE = 1,
} lis3dh_sim_t;
/** @} */

/**
 * @brief   Named return values
 */
enum {
    LIS3DH_OK          =  0,                            /**< everything was fine */
    LIS3DH_NOCOM       = -1,                            /**< communication failed */
    LIS3DH_NODEV       = -2,                            /**< no LIS3DH device found on the bus */
    LIS3DH_NODATA      = -3,                            /**< no data available */
    LIS3DH_ERROR       = -4                             /**< any error */
};

/**
 * @brief   LIS3DH interrupt 1 callback
 */
typedef void (*lis3dh_int_cb_t)(void *);

/**
 * @brief   Configuration parameters for LIS3DH devices
 */
#if defined (MODULE_LIS3DH_SPI)
typedef struct {
    spi_t               spi;                            /**< SPI device the sensor is connected to */
    spi_clk_t           clk;                            /**< designated clock speed of the SPI bus */
    gpio_t              cs;                             /**< Chip select pin */
    gpio_t              int1;                           /**< INT1 pin */
    lis3dh_int1_md_t    int1_mode;                      /**< INT1 mode */
    gpio_t              int2;                           /**< INT2 pin */
    lis3dh_int1_md_t    int2_mode;                      /**< INT2 mode */
    lis3dh_fs_t         scale;                          /**< Sensor scale: 2, 4, 8, or 16 (G) */
    lis3dh_odr_t        odr;                            /**< Sensor ODR setting: LIS3DH_ODR_xxxHz */
    lis3dh_op_md_t      op_mode;                        /**< Operation mode */
    
} lis3dh_params_t;
#elif defined (MODULE_LIS3DH_I2C)
typedef struct {
    i2c_t               i2c;                            /**< I2C device */
    uint8_t             addr;                           /**< I2C address */
    gpio_t              int1;                           /**< INT1 pin */
    lis3dh_int1_md_t    int1_mode;                      /**< INT1 mode */
    gpio_t              int2;                           /**< INT2 pin */
    lis3dh_int2_md_t    int2_mode;                      /**< INT2 mode */
    lis3dh_fs_t         scale;                          /**< Ssensor scale: 2, 4, 8, or 16 (G) */
    lis3dh_odr_t        odr;                            /**< Sensor ODR setting: LIS3DH_ODR_xxxHz */
    lis3dh_op_md_t      op_mode;                        /**< Operation mode */

} lis3dh_params_t;
#endif

/**
 * @brief   Device descriptor for LIS3DH sensors
 */
typedef struct {
    lis3dh_params_t  params;                            /**< Device initialization parameters */
    lis3dh_int_cb_t  cb;                                /**< alert callback */
    void             *arg;                              /**< alert callback param */
    lis3dh_int_cb_t  cb2;                               /**< alert callback for int2*/
    void             *arg2;                             /**< alert callback param for int2 */
    lis3dh_fs_t      scale;                             /**< Internal sensor scale */
    lis3dh_op_md_t   op_mode;                           /**< Internal sensor operation mode */
} lis3dh_t;

/**
 * @brief   Result vector for accelerometer measurement
 */
typedef struct
{
    int16_t axis_x;                                     /**< Acceleration in the X direction in milli-G */
    int16_t axis_y;                                     /**< Acceleration in the Y direction in milli-G */
    int16_t axis_z;                                     /**< Acceleration in the Z direction in milli-G */
} __attribute__((packed)) lis3dh_acceleration_t;

/**
 * @brief   Initialize a LIS3DH sensor instance
 *
 * @param[in]  dev          Device descriptor of sensor to initialize
 * @param[in]  params       Configuration parameters
 * @param[in]  cb           Callback called when interrupt 1
 * @param[in]  arg          Callback argument
 *
 * @return                  Error status
 */
int lis3dh_init(lis3dh_t *dev, const lis3dh_params_t *params, lis3dh_int_cb_t cb, void *arg);

/**
 * @brief   Initialize a LIS3DH sensor instance
 *
 * @param[in]  dev          Device descriptor of sensor to initialize
 * @param[in]  cb2          Callback called when interrupt 2
 * @param[in]  arg2         Callback argument
 *
 * @return                  Error status
 */
int lis3dh_init_int2(lis3dh_t *dev, lis3dh_int_cb_t cb2, void *arg2);

/**
 * @brief   ReInitialize a LIS3DH sensor instance
 *
 * @param[in]  dev          Device descriptor of sensor to initialize
 * @param[in]  scale        Scale: from 0 to 3 mean 2,4,8,16 in G
 * @param[in]  mode         LIS3DH mode 8, 10 or 12 bits
 *
 * @return                  Error status
 */
int lis3dh_reinit(lis3dh_t *dev, uint8_t scale, uint8_t mode);

/**
 * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 1 event to be
 *         recognized.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of d in reg INT1_DURATION in millisec
 *
 * @return       Error status
 */
int32_t lis3dh_int1_duration_set(lis3dh_t *dev, uint16_t val);

/**
 * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 2 event to be
 *         recognized.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of d in reg INT2_DURATION in millisec
 *
 * @return       Error status
 */
int32_t lis3dh_int2_duration_set(lis3dh_t *dev, uint16_t val);

/**
 * @brief  User-defined threshold value for xl interrupt event on
 *         generator 1.[set]
 *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of ths in reg INT1_THS
 *
 * @return       Error status
 */
int32_t lis3dh_int1_ths_set(lis3dh_t *dev, uint16_t val);

/**
 * @brief  User-defined threshold value for xl interrupt event on
 *         generator 1.[set]
 *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
 *
 * @param  dev   Device descriptor
 * @param  val   Pointer to the values of threshold in mg
 *
 * @return       Error status
 */
int32_t lis3dh_int1_ths_get(lis3dh_t *dev, uint16_t* val);

/**
 * @brief  Interrupt generator 1 configuration register.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of register INT1_CFG
 *
 * @return       Error status
 */
int32_t lis3dh_int1_cfg_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  Interrupt generator 1 source register.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of register INT1_SRC
 *
 * @return       Error status
 */
int32_t lis3dh_int1_src_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  Read generic device register (for debug purpose)
 *
 * @param  dev    Device descriptor
 * @param  reg    Address of the register to read
 * @param  data   Pointer to buffer that store the data read
 * @param  len    Number of consecutive register to read
 * 
 * @return        Error status
 */
int32_t lis3dh_read_reg(lis3dh_t *dev, uint8_t reg, uint8_t* data, uint16_t len);

/**
 * @brief  Write generic device register (for debug purpose)
 *
 * @param  dev    Device descriptor
 * @param  reg    Address of the register to write
 * @param  data   Pointer to data to write in register reg
 * @param  len    Number of consecutive register to write
 * 
 * @return        Error status
 */
int32_t lis3dh_write_reg(lis3dh_t *dev, uint8_t reg, uint8_t* data, uint16_t len);

/**
 * @brief  FIFO mode selection.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of fm in reg FIFO_CTRL_REG
 *
 * @return       Error status
 */
int32_t lis3dh_fifo_mode_set(lis3dh_t *dev, lis3dh_fm_t val);

/**
 * @brief   Read 3D acceleration data from the accelerometer
 *
 * @param[in]  dev            Device descriptor of sensor
 * @param[out] acceleration   Accelerometer data output
 *
 * @return                    Error status
 */
int lis3dh_read_xyz_cont(lis3dh_t *dev, lis3dh_acceleration_t *acceleration, uint8_t count);

/**
 * @brief   Read temperature from the accelerometer
 * 
 * @param dev                Device descriptor of sensor
 * @param temperature_degC   Temperature output
 * 
 * @return                   Error status
 */
int lis3dh_read_temp(lis3dh_t *dev, int16_t *temperature_degC);

#ifdef __cplusplus
}
#endif

#endif /* LIS3DH_H */
/** @} */
