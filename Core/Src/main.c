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
struct fifo_buffer {
    uint8_t data[buffergroesse];
    uint8_t next;
    uint8_t last;
    uint8_t changed;
} fifo_buffer = {{}, 0, 0, 0};


#define clk_time 1 // in millisekunden
#define wait_time 500 // in millisekunden
struct datenpaket{
	char bit[32];
	uint32_t clk;
	uint32_t wait;
} datenpaket = {{}, clk_time, wait_time};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int buffer_was_raus() {
    int save_it;
    // wenn der letzte Speicherpunkt leer ist, dann wird auch nichts rausgebuffert.
    if (fifo_buffer.data[fifo_buffer.last]==0) {
        return 0;
    } else {
        save_it=fifo_buffer.data[fifo_buffer.last];
        fifo_buffer.data[fifo_buffer.last]=0;
        fifo_buffer.last++;
        if (fifo_buffer.last>=buffergroesse) {
            fifo_buffer.last=0;
        }
        return save_it;
    }
}

int buffer_was_rein(uint8_t status) {
    // Eine Null ist nichts, was ich reinbuffern will.
    if (status==0 || status==13) {
        return 0;
    }
    // Wenn der nächste Speicherplatz nicht leer ist, dann wird auch nichts reingebuffert.
    else if	(fifo_buffer.data[fifo_buffer.next]) {
        return 0;
    }
    else {
        // Wir haben freien speicher und Daten, also wird was reingebuffert.

        fifo_buffer.data[fifo_buffer.next]=status;
        fifo_buffer.next++;

        // Wenn fifo_buffer.next größer als der Buffer wird, machen wir mit null weiter.
        if(fifo_buffer.next>=buffergroesse) {
            fifo_buffer.next=0;
        }
        fifo_buffer.changed=1;
        return 1;
    }
}

void check_usb_buffer() {
    // Der komplette USB-Buffer in unseren Ringbuffer übertragen
    for (uint8_t z=0; z<buffergroesse; z++) {
        buffer_was_rein(bufferdata[z]);
        bufferdata[z]=0;
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void blink_green(int power) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, power);
}
void blink_blue(int power) {
	if (power==2){
		HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
	}else{	
    	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, power);
	}
}
void blink_red(int power) {
    	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, power);
}
uint8_t read_input(){
    return HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin);
}
void show_lifesigns() {
    static uint32_t time;
    if((HAL_GetTick()-time)>1000) {
        time=HAL_GetTick();
        HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    }
}
uint8_t get_fifo_buffer_length() {
    uint8_t n;
    if (fifo_buffer.next<fifo_buffer.last) {
        n = fifo_buffer.next + buffergroesse - fifo_buffer.last +1;
    } else {
        n = fifo_buffer.next - fifo_buffer.last + 1;
    }
    return n;
}
void print_usb(char string[64]){
	uint16_t stringlength = strlen(string);
	CDC_Transmit_FS(string,stringlength);
}

#define input_help "help"  	// 1
#define input_calc "calc" 	// 2
#define input_hclk "hclk" 	// 3
#define input_dclk "dclk" // 4
#define input_read "read" 	// 5
#define input_show "show" 	// 6

uint8_t check_command() {

    uint8_t length=get_fifo_buffer_length();
    char answer[length];
    for (uint8_t i=0; i<length; i++) {
        answer[i]=buffer_was_raus();
    }
	// vergleich der eingaben mit den vordefinierten befehlen
    if 	 (!strcmp(answer,input_help)) {
        return 1;
    } else if (!strcmp(answer,input_calc)) {
        return 2;
    } else if (!strcmp(answer,input_hclk)) {
        return 3;
    } else if (!strcmp(answer,input_dclk)) {
        return 4;
    } else if (!strcmp(answer,input_read)) {
        return 5;
    } else if (!strcmp(answer,input_show)) {
        return 6;
    } else {
        return 0;
    }


}
uint8_t half_clktime(){
	// diese funktion prüft, ob der clk durch 2 teilbar ist und wenn ja, teilt es ihn durch 2;
	if (datenpaket.clk%2){
		return 0;
	} else {
		datenpaket.clk/=2;
		return 1;
	}
	
}
uint8_t double_clktime(){
	if (datenpaket.clk>(1<<30)){
		return 0;
	} else {
		datenpaket.clk*=2;
		return 1;
	}
} 

void calculate_data(){
	// hier werden die eingelesenen datenbits zu chars konvertriert
	for (uint8_t j=0;j<32;j++){
		// mach aus True and False '1' und '0'
		if (datenpaket.bit[j]<2){
			datenpaket.bit[j]+=48;
		}
	}
}


void start_reading_Spi(){
	//Diese Funktion soll den SPI_sensor auslesen;
	uint32_t tick_time=HAL_GetTick();
	blink_green(1);
	blink_blue(0);
	while(HAL_GetTick()-tick_time<datenpaket.wait){/*do nothing*/}
	tick_time=HAL_GetTick();
	blink_green(0);
	// timer interrupt!
	for (uint8_t i=0;i<32;i++){
		// 0=aus,1=an,2=switch on/off
		blink_blue(1);
		while(HAL_GetTick()-tick_time<datenpaket.clk){}
		tick_time=HAL_GetTick();
		// schreibe den input an die stelle i von datenpaket;
		datenpaket.bit[i]=read_input();
		while(HAL_GetTick()-tick_time<datenpaket.clk){}
		tick_time=HAL_GetTick();
		blink_blue(0);
		while(HAL_GetTick()-tick_time<datenpaket.clk*2){}
		tick_time=HAL_GetTick();
		
	}
	blink_green(1);
}
#define usb_delay 1
void write_new_line_usb(uint8_t wait){
	// springt in nächste zeile und an den Anfang. bei 0 
	if (wait){	
	HAL_Delay(usb_delay);
	}
	print_usb("\n\r");
	if (wait){	
	HAL_Delay(usb_delay);
	}
}

void show_commands(){
	// diese Funktion listet alle möglichen Befehlseingaben auf
	print_usb(input_help);
	write_new_line_usb(1);
	print_usb(input_read);//umcasten auf irgerndwas
	write_new_line_usb(1);
	print_usb(input_calc);
	write_new_line_usb(1);
	print_usb(input_show);
	write_new_line_usb(1);
	print_usb(input_hclk);
	write_new_line_usb(1);
	print_usb(input_dclk);
	write_new_line_usb(1);
}




#define response_1 "\n\r these commands are implemented:\n\r"
#define response_2 "\n\r calculate measurement\n\r"
#define response_hclk "\n\r half clocktime \n\r" 		// case 3
#define response_no_hclk "\n\r can't half clocktime \n\r" 	// case 3
#define response_dclk "\n\r double clocktime \n\r"
#define response_no_dclk "\n\r can't double clocktime \n\r"
#define response_show_bits "\n\r received bits:\n\r"

#define response_read "\n\r starting reading routine\n\r"
#define response_done "\n\r done reading\n\r"
#define response_d "\n\r invalid input \n\r try help\n\r"
void show_data(){
	    // Diese funktion soll die gemessenen Daten anzeigen
	    print_usb(response_show_bits);
	    HAL_Delay(usb_delay);
	    print_usb(datenpaket.bit);
	    HAL_Delay(usb_delay);
	    write_new_line_usb(0);
}

void answer_command() {
    uint8_t cmd=check_command();
    if(fifo_buffer.changed) {
        switch (cmd) {
        case 1:
        	print_usb(response_1);
	    show_commands();
            break;
        case 2:
        	print_usb(response_2);
	    calculate_data();
            break;
        case 3:
	    // clk halbieren wenn möglich 
	    if (half_clktime()){
	    	print_usb(response_hclk);
	    } else {
	    	print_usb(response_no_hclk);
	    }
	    break;
        case 4:
	    // clk verdoppeln wenn möglich
	    if (double_clktime()){
	    	print_usb(response_dclk);
	    } else {
	    	print_usb(response_no_dclk);
	    }
            break;
        case 5:
        	print_usb(response_read);
            start_reading_Spi();
            print_usb(response_done);
            calculate_data();
            show_data();
            break;
        case 6:
        	show_data();
	    break;
	case 0:
		print_usb(response_d);
            break;
        default:
        	print_usb(response_d);
            break;
        }
    }

    if((fifo_buffer.next==fifo_buffer.last) && (fifo_buffer.changed)) {
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
    	//wait for interrupt (stromsparen)

        // Lebenszeichen durch blinken(rot) der LED3
        show_lifesigns();

        // USB_buffer in Ringbuffer übertragen
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

