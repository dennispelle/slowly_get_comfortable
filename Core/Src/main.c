/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define buffergroesse 64 		//wie viele speicherplätze bekommt der ringbuffer?
volatile uint8_t bufferdata[buffergroesse];
struct fifo_buffer{
	uint8_t data[buffergroesse];
	uint8_t next;
	uint8_t last;
	uint8_t changed;
} fifo_buffer = {{}, 0, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int buffer_was_raus(){
	int save_it;
	// wenn der letzte Speicherpunkt leer ist, dann wird auch nichts rausgebuffert.
    if (fifo_buffer.data[fifo_buffer.last]==0){
    	return 0;
    } else {
		save_it=fifo_buffer.data[fifo_buffer.last];
		fifo_buffer.data[fifo_buffer.last]=0;
		fifo_buffer.last++;
		if (fifo_buffer.last>=buffergroesse){
			fifo_buffer.last=0;
		}
		return save_it;
    }
}

int buffer_was_rein(uint8_t status){
	// Eine Null ist nichts, was ich reinbuffern will.
	if (status==0 || status==13){
		return 0;
	}
	// Wenn der nächste Speicherplatz nicht leer ist, dann wird auch nichts reingebuffert.
	else if	(fifo_buffer.data[fifo_buffer.next]){
		return 0;
	}
	else {
		// Wir haben freien speicher und Daten, also wird was reingebuffert.

		fifo_buffer.data[fifo_buffer.next]=status;
		fifo_buffer.next++;

		// Wenn fifo_buffer.next größer als der Buffer wird, machen wir mit null weiter.
		if(fifo_buffer.next>=buffergroesse){
			fifo_buffer.next=0;
		}
		fifo_buffer.changed=1;
		return 1;
	}
}

void check_usb_buffer(){
	// Der komplette USB-Buffer in unseren Ringbuffer übertragen
	for (uint8_t z=0;z<buffergroesse;z++){
		buffer_was_rein(bufferdata[z]);
		bufferdata[z]=0;
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void blink_green(int power){
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, power);
}
void blink_blue(int power){
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, power);
}
void blink_red(int power){
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, power);
}
int button1_pressed;
void  HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin_irgendwas) {
	button1_pressed=1;
}

void show_lifesigns(){
	static uint32_t time;
	if((HAL_GetTick()-time)>1000){
		time=HAL_GetTick();
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	}
}
uint8_t get_fifo_buffer_length(){
uint8_t n;
        if (fifo_buffer.next<fifo_buffer.last){
        	n = fifo_buffer.next + buffergroesse - fifo_buffer.last +1;
	}else{
		n = fifo_buffer.next - fifo_buffer.last + 1;
	}
	return n;
}
uint8_t check_command(uint8_t length){
	uint8_t answer[length];
	for (uint8_t i=0;i<length;i++){
		answer[i]=buffer_was_raus();
	}

	if (strcasecmp(answer,"doit")){
		return 1;
	}else{
		return 0;
	}


}
void answer_command(){
	uint8_t n=get_fifo_buffer_length();
	uint8_t cmd=check_command(n);
	if(fifo_buffer.changed){
		switch (cmd){
			case 1:
				CDC_Transmit_FS("\n\r I will do it \n\r what next? \n\r", strlen("\n\r I will do it \n\r what next? \n\r"));
				break;
			case 0:
				CDC_Transmit_FS("\n\r I am confused \n\r try again \n\r", strlen("\n\r I am confused \n\r try again \n\r"));
				break;
		}
	}
			
	if((fifo_buffer.next==fifo_buffer.last) && (fifo_buffer.changed)){
		fifo_buffer.changed=0;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Lebenszeichen durch LED3 und ". . . . . ." über USB to serial
	  show_lifesigns();

	  // USB_buffer ind Ringbuffer übertragen
	  check_usb_buffer();

	  // Eingegangene Befehle beantworten
	  answer_command();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

