
#include "LIS3DH.h"
#include "Lis3dh_Ext.h"
#include "CycleCount.h"


lis3dh_params_t sett = {
	.i2c =0,
	.addr = LIS3DH_DEFAULT_ADDRESS,
	.int1 = (1<<5),
	.int1_mode = I1_IA1,
	.int2 = (1),
	.int2_mode = I2_DISABLE,
	.odr = LIS3DH_ODR_200Hz,
	.op_mode = LIS3DH_NM_10bit,
	.scale = LIS3DH_2g

};

lis3dh_t devlis3dh;
int main(void) {
	
  EnableTiming();
	lis3dh_init(&devlis3dh , &sett , NULL, NULL);
  while (1)
  {

  }
}


/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void) { 
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); 
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void) { 
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5); 
}
