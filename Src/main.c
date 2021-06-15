/**
 ******************************************************************************
 * @file    Templates/Src/main.c
 * @author  MCD Application Team
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ssd1306.h"
#include "bmp280.h"
#include "MPU-9250.h"
#include "math.h"
#include <stdio.h>  /*rtt*/
#include <stdlib.h> /*rtt*/

/** @addtogroup STM32F0xx_HAL_Examples
 * @{
 */

/** @addtogroup Templates
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
double BME280_CalcTf(double pressure);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */

int main(void) {

    int rslt;
    uint8_t bmp280Period;
    struct bmp280_dev bmp;
    struct bmp280_config conf;
    struct bmp280_uncomp_data ucomp_data;
    struct bmp280_status bmpStatus;
    int32_t temp = 0;
    uint32_t pres = 0;
    MPU9250_gyro_val gyro;
    MPU9250_accel_val accel;
    MPU9250_magnetometer_val magn;

	HAL_Init();

	/* Configure the system clock to have a system clock = 48 Mhz */
	SystemClock_Config();

	USART_DBG_Init();
	HC05_USART_Init();
	__HAL_UART_ENABLE(&usart5);
	SSD1306_Init();
	MX_I2C1_Init();


    bmp.delay_ms = HAL_Delay;
    bmp.dev_id = 0xec;//BMP280_I2C_ADDR_SEC;
    /* Select the interface mode as I2C */
    bmp.intf = BMP280_I2C_INTF;

    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
    bmp.read = I2C_Read;
    bmp.write = I2C_Write;
    rslt = bmp280_init(&bmp);
    rslt = bmp280_get_config(&conf, &bmp);
	conf.filter = BMP280_FILTER_COEFF_16;
	conf.os_pres = BMP280_OS_16X;
	conf.os_temp = BMP280_OS_1X;
	conf.odr = BMP280_ODR_0_5_MS;
	rslt = bmp280_set_config(&conf, &bmp);

	if (rslt == BMP280_OK) {
		printf("Pressure sensor OK\r\n");
	} else {
		printf("Error by pressure sensor init\r\n");
	}

	bmp280Period = bmp280_compute_meas_time(&bmp);
	printf ("Pressure sensor period = %d\r\n",bmp280Period);
	bmp280_set_power_mode(BMP280_NORMAL_MODE,&bmp);


	while (!bmpStatus.im_update){
		bmp280Period = bmp280_get_status(&bmpStatus, &bmp);
	}


	while (1) {
		rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
		rslt = bmp280_get_comp_pres_32bit(&pres, ucomp_data.uncomp_press, &bmp);
    	rslt = bmp280_get_comp_temp_32bit(&temp,ucomp_data.uncomp_temp,&bmp);

    	char sPres[8];
		sprintf(sPres, "%x", pres);

		printf("PRS %x\n",pres);
		SSD1306_GotoXY(0,0);
		SSD1306_Puts(sPres,&Font_7x10,1);
		SSD1306_UpdateScreen();

		HAL_Delay(50);
	}
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSI48)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSI Frequency(Hz)              = 48000000
 *            PREDIV                         = 2
 *            PLLMUL                         = 2
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Select HSI48 Oscillator as PLL source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI48;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/* Select PLL as system clock source and configure the HCLK and PCLK1 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void) {
	/* User may add here some code to deal with this error */
	while (1) {
	}
}

int __io_putchar(int ch) {
	//HAL_USART_Transmit(&USART_DEBUG_HandleStruct, (uint8_t*) &ch, 1, 0x100);
	HAL_USART_Transmit(&usart5, (uint8_t*) &ch, 1, 0x100);
	return ch;
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
 * @}
 */
double BME280_CalcTf(double pressure) {
    return (44330 * (1.0 - pow(((pressure / 100) / 1013.25), 0.1903)));
}
/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
