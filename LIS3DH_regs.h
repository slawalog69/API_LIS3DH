/*!
 *  @file LIS3DH_regs.h
 *
 */

#ifndef LIS3DH_REG_H
#define LIS3DH_REG_H

#include <stdint.h>

/*!  @brief   STATUS_REG_AUX register
 *   321OR  1, 2 and 3 axis data overrun. Default value: 0
 *          (0: no overrun has occurred; 1: a new set of data has overwritten
 * the previous ones) 3OR    3 axis data overrun. Default value: 0 (0: no
 * overrun has occurred; 1: a new data for the 3-axis has overwritten the
 * previous one) 2OR    2 axis data overrun. Default value: 0 (0: no overrun has
 * occurred; 1: a new data for the 4-axis has overwritten the previous one) 1OR
 * 1 axis data overrun. Default value: 0 (0: no overrun has occurred; 1: a new
 * data for the 1-axis has overwritten the previous one) 321DA  1, 2 and 3 axis
 * new data available. Default value: 0 (0: a new set of data is not yet
 * available; 1: a new set of data is available) 3DA:   3 axis new data
 * available. Default value: 0 (0: a new data for the 3-axis is not yet
 * available; 1: a new data for the 3-axis is available) 2DA:   2 axis new data
 * available. Default value: 0 (0: a new data for the 2-axis is not yet
 * available; 1: a new data for the 2-axis is available) 1DA    1 axis new data
 * available. Default value: 0 (0: a new data for the 1-axis is not yet
 * available; 1: a new data for the 1-axis is available)
 */
#define LIS3DH_REG_STATUS1 0x07
#define LIS3DH_REG_OUTADC1_L 0x08 /**< 1-axis acceleration data. Low value */
#define LIS3DH_REG_OUTADC1_H 0x09 /**< 1-axis acceleration data. High value */
#define LIS3DH_REG_OUTADC2_L 0x0A /**< 2-axis acceleration data. Low value */
#define LIS3DH_REG_OUTADC2_H 0x0B /**< 2-axis acceleration data. High value */
#define LIS3DH_REG_OUTADC3_L 0x0C /**< 3-axis acceleration data. Low value */
#define LIS3DH_REG_OUTADC3_H 0x0D /**< 3-axis acceleration data. High value */
#define LIS3DH_REG_INTCOUNT  0x0E /**< INT_COUNTER register [IC7, IC6, IC5, IC4, IC3, IC2, IC1, IC0] */
#define LIS3DH_REG_WHOAMI    0x0F /**< Device identification register. [0, 0, 1, 1, 0, 0, 1, 1] */
/*!  @brief TEMP_CFG_REG  Temperature configuration register.
 *   ADC_PD   ADC enable. Default value: 0
 *            (0: ADC disabled; 1: ADC enabled)
 *   TEMP_EN  Temperature sensor (T) enable. Default value: 0
 *            (0: T disabled; 1: T enabled)
 */
#define LIS3DH_REG_TEMPCFG 0x1F
/*!   @brief CTRL_REG1
 *  [ODR3, ODR2, ODR1, ODR0, LPen, Zen, Yen, Xen]
 *   ODR3-0  Data rate selection. Default value: 00
 *           (0000:50 Hz; Others: Refer to Datasheet Table 26, “Data rate
 * configuration”) LPen    Low power mode enable. Default value: 0 (0: normal
 * mode, 1: low power mode) Zen     Z axis enable. Default value: 1 (0: Z axis
 * disabled; 1: Z axis enabled) Yen     Y axis enable. Default value: 1 (0: Y
 * axis disabled; 1: Y axis enabled) Xen     X axis enable. Default value: 1 (0:
 * X axis disabled; 1: X axis enabled)
 */
#define LIS3DH_REG_CTRL1 0x20

/*!   @brief CTRL_REG2
 *  [HPM1, HPM0, HPCF2, HPCF1, FDS, HPCLICK, HPIS2, HPIS1]
 *   HPM1-0  High pass filter mode selection. Default value: 00
    0 0 Normal mode (reset by reading REFERENCE (26h))
	0 1 Reference signal for filtering
	1 0 Normal mode
	1 1 Autoreset on interrupt event

 *   HPCF 2-1 High pass filter cut off frequency selection
 *   FDS     Filtered data selection. Default value: 0
 *	 (0: internal filter bypassed; 1: data
 *from internal filter sent to output register and FIFO) HPCLICK High pass
 *filter enabled for CLICK function. (0: filter bypassed; 1: filter enabled)
 *HPIS2   X axis enable. Default value: 1 (0: X axis disabled; 1: X axis
 *enabled) HPIS1 High pass filter enabled for AOI function on interrupt 1, (0:
 *filter bypassed; 1: filter enabled)
 */
#define LIS3DH_REG_CTRL2 0x21

/*!  @brief  CTRL_REG3
 *  [I1_CLICK, I1_AOI1, I1_AOI2, I1_DRDY1, I1_DRDY2, I1_WTM, I1_OVERRUN, --]
 *   I1_CLICK    CLICK interrupt on INT1. Default value 0.
 *						   (0: Disable; 1: Enable)
 *   I1_AOI1     AOI1 interrupt on INT1. Default value 0.
 *						   (0: Disable; 1: Enable)
 *   I1_AOI2     AOI2 interrupt on INT1. Default value 0.
 *               (0: Disable; 1: Enable)
 *   I1_DRDY1    DRDY1 interrupt on INT1. Default value 0.
 *               (0: Disable; 1: Enable)
 *   I1_DRDY2    DRDY2 interrupt on INT1. Default value 0.
 *               (0: Disable; 1: Enable)
 *   I1_WTM      FIFO Watermark interrupt on INT1. Default value 0.
 *               (0: Disable; 1: Enable)
 *   I1_OVERRUN  FIFO Overrun interrupt on INT1. Default value 0.
 * 							 (0: Disable; 1: Enable)
 */
#define LIS3DH_REG_CTRL3 0x22


/*!  @brief  CTRL_REG4
 *  [BDU, BLE, FS1, FS0, HR, ST1, ST0, SIM]
 *   BDU      Block data update. Default value: 0
 *            (0: continuos update; 1: output registers not updated until MSB
 * and LSB reading) BLE      Big/little endian data selection. Default value 0.
 *            (0: Data LSB @ lower address; 1: Data MSB @ lower address)
 *   FS1-FS0  Full scale selection. default value: 00
 *            (00: +/- 2G; 01: +/- 4G; 10: +/- 8G; 11: +/- 16G)
 *   HR       High resolution output mode: Default value: 0
 *            (0: High resolution disable; 1: High resolution Enable)
 *   ST1-ST0  Self test enable. Default value: 00
 *            (00: Self test disabled; Other: See Table 34)
 *   SIM      SPI serial interface mode selection. Default value: 0
 *            (0: 4-wire interface; 1: 3-wire interface).
 */
#define LIS3DH_REG_CTRL4 0x23


/*!  @brief  CTRL_REG5
 *  [BOOT, FIFO_EN, --, --, LIR_INT1, D4D_INT1, 0, 0]
 *   BOOT     Reboot memory content. Default value: 0
 *            (0: normal mode; 1: reboot memory content)
 *   FIFO_EN  FIFO enable. Default value: 0
 *            (0: FIFO disable; 1: FIFO Enable)
 *   LIR_INT1 Latch interrupt request on INT1_SRC register, with INT1_SRC
 * register cleared by reading INT1_SRC itself. Default value: 0. (0: interrupt
 * request not latched; 1: interrupt request latched) D4D_INT1 4D enable: 4D
 * detection is enabled on INT1 when 6D bit on INT1_CFG is set to 1.
 */
#define LIS3DH_REG_CTRL5 0x24

/*!   @brief  CTRL_REG6
 *  [I2_CLICKen, I2_INT1, 0, BOOT_I1, 0, --, H_L, -]
 */
#define LIS3DH_REG_CTRL6 0x25
#define LIS3DH_REG_REFERENCE 0x26 /**< Reference value for Interrupt generation**/
/*!  @brief  STATUS_REG
 *  [ZYXOR, ZOR, YOR, XOR, ZYXDA, ZDA, YDA, XDA]
 *   ZYXOR    X, Y and Z axis data overrun. Default value: 0
 *            (0: no overrun has occurred; 1: a new set of data has overwritten
 * the previous ones) ZOR      Z axis data overrun. Default value: 0 (0: no
 * overrun has occurred; 1: a new data for the Z-axis has overwritten the
 * previous one) YOR      Y axis data overrun. Default value: 0 (0: no overrun
 * has occurred;  1: a new data for the Y-axis has overwritten the previous one)
 *   XOR      X axis data overrun. Default value: 0
 *            (0: no overrun has occurred; 1: a new data for the X-axis has
 * overwritten the previous one) ZYXDA    X, Y and Z axis new data available.
 * Default value: 0 (0: a new set of data is not yet available; 1: a new set of
 * data is available) ZDA      Z axis new data available. Default value: 0 (0: a
 * new data for the Z-axis is not yet available; 1: a new data for the Z-axis is
 * available) YDA      Y axis new data available. Default value: 0 (0: a new
 * data for the Y-axis is not yet available; 1: a new data for the Y-axis is
 * available)
 */
#define LIS3DH_REG_STATUS2 0x27

#define LIS3DH_REG_OUT_X_L 0x28 /**< X-axis acceleration data. Low value */
#define LIS3DH_REG_OUT_X_H 0x29 /**< X-axis acceleration data. High value */
#define LIS3DH_REG_OUT_Y_L 0x2A /**< Y-axis acceleration data. Low value */
#define LIS3DH_REG_OUT_Y_H 0x2B /**< Y-axis acceleration data. High value */
#define LIS3DH_REG_OUT_Z_L 0x2C /**< Z-axis acceleration data. Low value */
#define LIS3DH_REG_OUT_Z_H 0x2D /**< Z-axis acceleration data. High value */

#define LIS3DH_REG_READ_ALL  0xA8 /**<  LIS3DH_REG_OUT_X_L | 0x80 - continue flag auto inc adress*/



/*!  @brief  FIFO_CTRL_REG
 *  [FM1, FM0, TR, FTH4, FTH3, FTH2, FTH1, FTH0]
 *   FM1-FM0  FIFO mode selection. Default value: 0 0
			0 0 Bypass mode
			0 1 FIFO mode
			1 0 Stream mode
			1 1 Stream-to-FIFO
 *   TR       Trigger selection. Default value: 0
 *            0: Trigger event liked to trigger signal on INT1
 *            1: Trigger event liked to trigger signal on INT2
 *   FTH4:0   Default value: 0
 */
#define LIS3DH_REG_FIFOCTRL 0x2E
/*!< FIFO StatusRegCtl REG   
WTM, OVRN_FIFO, EMPTY, FSS4, FSS3, FSS2, FSS1, FSS0 */
#define LIS3DH_REG_FIFOSRC   0x2F 

/*!  @brief  INT1_CFG
 *  [AOI, 6D, ZHIE/ZUPE, ZLIE/ZDOWNE, YHIE/YUPE, XHIE/XUPE, XLIE/XDOWNE]
 *   AOI         And/Or combination of Interrupt events. Default value: 0. Refer
 * to Datasheet Table 54, "INT1_CFG description"
 */
#define LIS3DH_REG_INT1CFG 0x30


/*!  @brief  INT1_SRC
 *   [0, IA, ZH, ZL, YH, YL, XH, XL]
 *    IA  Interrupt active. Default value: 0
 *        (0: no interrupt has been generated; 1: one or more interrupts have
 * been generated) ZH  Z high. Default value: 0 (0: no interrupt, 1: Z High
 * event has occurred) ZL  Z low. Default value: 0 (0: no interrupt; 1: Z Low
 * event has occurred) YH  Y high. Default value: 0 (0: no interrupt, 1: Y High
 * event has occurred) YL  Y low. Default value: 0 (0: no interrupt, 1: Y Low
 * event has occurred) XH  X high. Default value: 0 (0: no interrupt, 1: X High
 * event has occurred) XL  X low. Default value: 0 (0: no interrupt, 1: X Low
 * event has occurred)
 *
 *    Interrupt 1 source register. Read only register.
 *    Reading at this address clears INT1_SRC IA bit (and the interrupt signal
 * on INT 1 pin) and allows the refreshment of data in the INT1_SRC register if
 * the latched option  was chosen.
 */
#define LIS3DH_REG_INT1SRC 0x31
/*!< Interrupt 1 threshold
register [0, THS6, THS5, THS4, THS3, THS1, THS0] */
#define LIS3DH_REG_INT1THS   0x32 
/*!< Duration value. 
D[6:0] bits set the minimum duration of the Interrupt 1 event to be recognized */
#define LIS3DH_REG_INT1DUR  0x33 
/*!  @brief  INT2_CFG

*/
#define LIS3DH_REG_INT2CFG 0x34
/*!  @brief  INT2_SRC

*/
#define LIS3DH_REG_INT2SRC 0x35
#define LIS3DH_REG_INT2THS   0x36 /**< Interrupt 2 threshold */
/*!< Duration value. Interrupt 2 
D[6:0] bits set the minimum duration of the Interrupt 2 event to be recognized */
#define LIS3DH_REG_INT2DUR  0x37 

/*!  @brief  CLICK_CFG
 *   [--, --, ZD, ZS, YD, YS, XD, XS]
 *   ZD  Enable interrupt double CLICK-CLICK on Z axis. Default value: 0
 *       (0: disable interrupt request;
 *        1: enable interrupt request on measured accel. value higher than
 * preset threshold) ZS  Enable interrupt single CLICK-CLICK on Z axis. Default
 * value: 0 (0: disable interrupt request; 1: enable interrupt request on
 * measured accel. value higher than preset threshold) YD  Enable interrupt
 * double CLICK-CLICK on Y axis. Default value: 0 (0: disable interrupt request;
 *        1: enable interrupt request on measured accel. value higher than
 * preset threshold) YS  Enable interrupt single CLICK-CLICK on Y axis. Default
 * value: 0 (0: disable interrupt request; 1: enable interrupt request on
 * measured accel. value higher than preset threshold) XD  Enable interrupt
 * double CLICK-CLICK on X axis. Default value: 0 (0: disable interrupt request;
 * 1: enable interrupt request on measured accel. value higher than preset
 * threshold) XS  Enable interrupt single CLICK-CLICK on X axis. Default value:
 * 0 (0: disable interrupt request; 1: enable interrupt request on measured
 * accel. value higher than preset threshold)
 */
#define LIS3DH_REG_CLICKCFG 0x38
/*!  @brief  CLICK_SRC
 *   [-, IA, DCLICK, SCLICK, Sign, Z, Y, X]
 *   IA  Interrupt active. Default value: 0
 *       (0: no interrupt has been generated; 1: one or more interrupts have
 * been generated) DCLICK  Double CLICK-CLICK enable. Default value: 0 (0:double
 * CLICK-CLICK detection disable, 1: double CLICK-CLICK detection enable) SCLICK
 * Single CLICK-CLICK enable. Default value: 0 (0:Single CLICK-CLICK detection
 * disable, 1: single CLICK-CLICK detection enable) Sign    CLICK-CLICK Sign.
 *           (0: positive detection, 1: negative detection)
 *   Z       Z CLICK-CLICK detection. Default value: 0
 *           (0: no interrupt, 1: Z High event has occurred)
 *   Y       Y CLICK-CLICK detection. Default value: 0
 *           (0: no interrupt, 1: Y High event has occurred)
 *   X       X CLICK-CLICK detection. Default value: 0
 *           (0: no interrupt, 1: X High event has occurred)
 */
#define LIS3DH_REG_CLICKSRC 0x39
/*!  CLICK_THS
 *   [-, Ths6, Ths5, Ths4, Ths3, Ths2, Ths1, Ths0]
 *   Ths6-Ths0  CLICK-CLICK threshold. Default value: 000 0000
 */
#define LIS3DH_REG_CLICKTHS 0x3A
/*!  TIME_LIMIT
 *   [-, TLI6, TLI5, TLI4, TLI3, TLI2, TLI1, TLI0]
 *   TLI7-TLI0  CLICK-CLICK Time Limit. Default value: 000 0000
 */
#define LIS3DH_REG_TIMELIMIT 0x3B
/*!  TIME_LATANCY
 *   [-, TLA6, TLIA5, TLA4, TLA3, TLA2, TLA1, TLA0]
 *   TLA7-TLA0  CLICK-CLICK Time Latency. Default value: 000 0000
 */
#define LIS3DH_REG_TIMELATENCY 0x3C
/*!   TIME_WINDOW
 *   [TW7, TW6, TW5, TW4, TW3, TW2, TW1, TW0]
 *   TW7-TW0  CLICK-CLICK Time window
 */
#define LIS3DH_REG_TIMEWINDOW 0x3D
/*!  ACT_THS register
 Sleep-to-wake, return-to-sleep activation threshold in low-power mode
 */
#define LIS3DH_REG_ACT_THS 0x3E
/*!  ACT_DUR register
Sleep-to-wake, return-to-sleep duration
 */
#define LIS3DH_REG_ACT_DUR 0x3F


#endif
