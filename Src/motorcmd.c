
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */

#include "motorcmd.h"

void MotorCMD_Loop()
{
	static uint32_t led_counter;


	if (led_counter < 20)
	{
		led_counter++;
	}
	else
	{
		led_counter = 0;
		HAL_GPIO_TogglePin(GPIOD, LED1_Pin);
	}

}
