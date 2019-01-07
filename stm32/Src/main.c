
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "lwip.h"

/* USER CODE BEGIN Includes */
#include  <errno.h>
#include  <sys/unistd.h>

#include "stm32746g_discovery_lcd.h"
#include "Utilities/Fonts/fonts.h"
#include "stm32746g_discovery_ts.h"
#include "term_io.h"
#include "dbgu.h"
#include "ansi.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/apps/fs.h"
#include "lwip/dhcp.h"
#include "lwip/tcpip.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "lwip.h"

#include "wm8994/wm8994.h"

//
#include "lwip/apps/mqtt.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

CRC_HandleTypeDef hcrc;

DCMI_HandleTypeDef hdcmi;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

QSPI_HandleTypeDef hqspi;

RTC_HandleTypeDef hrtc;

SAI_HandleTypeDef hsai_BlockA2;
SAI_HandleTypeDef hsai_BlockB2;

SD_HandleTypeDef hsd1;

SPDIFRX_HandleTypeDef hspdif;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

SDRAM_HandleTypeDef hsdram1;


osThreadId defaultTaskHandle;

float buffer[BUFFER_SIZE];
int first_index = 0;
int current_pos = 0;
int is_full = 0;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);





/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void mainTask(void* p);

osThreadId netconn_thread_handle;

#define LCD_X_SIZE			RK043FN48H_WIDTH
#define LCD_Y_SIZE			RK043FN48H_HEIGHT

#define PRINTF_USES_HAL_TX		0

int __io_putchar(int ch)
{
	uint8_t data = ch;
	#if PRINTF_USES_HAL_TX
		HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)&data, len, 100);
	#else
		while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET) { ; }
		huart1.Instance->TDR = (uint16_t)data;
	#endif
	return 0;
}

char inkey(void)
{
	uint32_t flags = huart1.Instance->ISR;

	if((flags & UART_FLAG_RXNE) || (flags & UART_FLAG_ORE))
	{
		__HAL_UART_CLEAR_OREFLAG(&huart1);
		return (huart1.Instance->RDR);
	}
	else
		return 0;
}

//partially based on available code examples
static void lcd_start(void)
{
  /* LCD Initialization */
  BSP_LCD_Init();

  /* LCD Initialization */
  BSP_LCD_LayerDefaultInit(0, (unsigned int)0xC0000000);
  //BSP_LCD_LayerDefaultInit(1, (unsigned int)lcd_image_bg+(LCD_X_SIZE*LCD_Y_SIZE*4));
  BSP_LCD_LayerDefaultInit(1, (unsigned int)0xC0000000+(LCD_X_SIZE*LCD_Y_SIZE*4));

  /* Enable the LCD */
  BSP_LCD_DisplayOn();

  /* Select the LCD Background Layer  */
  BSP_LCD_SelectLayer(0);

  /* Clear the Background Layer */
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);

  BSP_LCD_SetColorKeying(1,LCD_COLOR_WHITE);

  /* Select the LCD Foreground Layer  */
  BSP_LCD_SelectLayer(1);

  /* Clear the Foreground Layer */
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);

  /* Configure the transparency for foreground and background :
     Increase the transparency */
  BSP_LCD_SetTransparency(0, 255);
  BSP_LCD_SetTransparency(1, 255);
}

//[rmv]
void draw_button(int xPos, int yPos, int width, int height, uint32_t color, uint8_t* text);
void draw_background(void)
{
	/* Select the LCD Background Layer  */
	BSP_LCD_SelectLayer(0);
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_FillRect(0,0,LCD_X_SIZE,LCD_Y_SIZE);

	//select Foreground Layer
	BSP_LCD_SelectLayer(1);
	char greeting[] = "Hello";
	draw_button(50,50,100,100,LCD_COLOR_MAGENTA,&greeting);
}

void draw_button(int xPos, int yPos, int width, int height, uint32_t color, uint8_t* text){

	BSP_LCD_SelectLayer(0);
	BSP_LCD_SetTextColor(color);
	BSP_LCD_FillRect(xPos,yPos,width,height);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
	BSP_LCD_DisplayStringAt(xPos+5, yPos+5, text, 0);

}

void draw_chart(){

    int chart_height = 200;
	int chart_height_coord = 230;
    int chart_length = 450;
    int chart_y_pos = 250;
    int chart_x_pos = 20;
	int chart_y_pos_coord = 20;

//drawing coordinates system
    BSP_LCD_SelectLayer(0);
    BSP_LCD_Clear(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DrawHLine(chart_x_pos,chart_y_pos,chart_length);
    BSP_LCD_DrawVLine(chart_x_pos,chart_y_pos_coord,chart_height_coord);
    
//    BSP_LCD_FillTriangle(chart_length-10,chart_length-10,chart_length,chart_y_pos+5,chart_y_pos-5,chart_y_pos);

    //check amount of data
    int n = 0;
    for(int i = 0; i < BUFFER_SIZE; i++){
        if(buffer[i] != INT32_MIN)
            n++;
    }

    //int n = 90; //sizeof(data)/sizeof(data[0]);
    float max = buffer[0];
    float min = buffer[0];//0;//data[0];

    for(int i=1; i<n; i++){
      if(buffer[i] < min)
          min = buffer[i];
      if(buffer[i] > max)
          max = buffer[i];
    }
    int min_i = min;//0;//min;
    int max_i = max;
    float coordinates_scale = chart_height/(max_i - min_i);
    float amplitude = max - min; // ?
    float scale = chart_height/amplitude; // ?

    // draw scale
    int y = chart_y_pos;
    BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
    for(int i=1;i<amplitude;i++){
      BSP_LCD_DrawHLine(chart_x_pos,chart_y_pos-i*coordinates_scale,chart_length);
    }

// draw data
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

  int sample_len = chart_length/n;
  int stop_cond = (first_index == 0) ? BUFFER_SIZE -1 : first_index -1;
  for(int i= first_index; i != stop_cond; i = (i+1)%BUFFER_SIZE){
      if(buffer[i] != INT32_MIN) {
          int x1, y1, x2, y2;
          x1 = chart_x_pos + i * sample_len;
          x2 = chart_x_pos + (i + 1) * sample_len;
          y1 = chart_y_pos - (buffer[i] - min_i) * coordinates_scale;
          y2 = chart_y_pos - (buffer[i + 1] - min_i) * coordinates_scale;
          BSP_LCD_DrawLine(x1, y1, x2, y2);
      }
  }
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	Point points[3];
	points[0].X=chart_x_pos+5;
	points[1].X=chart_x_pos-5;
	points[2].X=chart_x_pos;
	
	points[0].Y=chart_y_pos_coord+10;
	points[1].Y=chart_y_pos_coord+10;
	points[2].Y=chart_y_pos_coord;
	BSP_LCD_FillPolygon(points,3);

	points[0].X=chart_x_pos+chart_length-10;
	points[1].X=chart_x_pos+chart_length;
	points[2].X=chart_x_pos+chart_length-10;

	points[0].Y=chart_y_pos-5;
	points[1].Y=chart_y_pos;
	points[2].Y=chart_y_pos+5;
	BSP_LCD_FillPolygon(points,3);
	//BSP_LCD_FillPolygon(chart_x_pos-5,chart_x_pos-5,chart_x_pos+5,chart_height-10,chart_height,chart_height-10);


	BSP_LCD_SetFont(&Font12);
	
	int j = 0;
	for(int i = min; i <=max ; i+=5){
		int someInt = i;
		char str[3];
		sprintf(str, "%d", someInt);
		BSP_LCD_DisplayStringAt(chart_x_pos-15,chart_y_pos-5-j*coordinates_scale, (uint8_t*)str, LEFT_MODE);
		j+=5;
	}
	/*
	BSP_LCD_DisplayStringAt(chart_x_pos-15,chart_y_pos-5, (uint8_t*)"0", LEFT_MODE);
	BSP_LCD_DisplayStringAt(chart_x_pos-15,chart_y_pos-5-5*coordinates_scale, (uint8_t*)"5", LEFT_MODE);
	BSP_LCD_DisplayStringAt(chart_x_pos-15,chart_y_pos-5-10*coordinates_scale, (uint8_t*)"10", LEFT_MODE);
	BSP_LCD_DisplayStringAt(chart_x_pos-15,chart_y_pos-5-15*coordinates_scale, (uint8_t*)"15", LEFT_MODE);
	*/
	}

static TS_StateTypeDef  TS_State;


int initialize_touchscreen(void)
{
	uint8_t  status = 0;
	status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
	if(status != TS_OK) return -1;
	return 0;
}

void add_data_to_buffer(char* new_item)
{
    int buff_size = sizeof(buffer);
    float new_item_f = atoi(new_item);
    buffer[current_pos] = new_item_f;

    if( is_full == 0){
        current_pos = (current_pos++)%buff_size;
    }

    if(is_full == 1){
        first_index = (first_index++)%buff_size;
        current_pos = (current_pos++)%buff_size;
    }

    if(current_pos == buff_size - 1 && is_full == 0)
    {
        is_full = 1;
        first_index = 1;
    }
}



 int print_data(char* data, int client_id, int len){
	BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
	if(client_id == 1){
		printf("%s\n",data);
		int i;
		for(i = 10; i < 70;i++)
			BSP_LCD_ClearStringLine(i);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_FillRect(0.3*LCD_X_SIZE,0.1*LCD_Y_SIZE,10,50);
		BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
		BSP_LCD_ClearStringLine(10);
		BSP_LCD_ClearStringLine(50);
		char target[100];

		strncpy(target, data, len);
		target[len] = '\0';
		BSP_LCD_DisplayStringAt(0, 10, (uint8_t*)"Temperature1 is:", CENTER_MODE);
		BSP_LCD_DisplayStringAt(0, 50, (uint8_t*)target, CENTER_MODE);
	}
	else{
	printf("%s\n",data);
	char target[100];
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_FillRect(0.3*LCD_X_SIZE,0.1*LCD_Y_SIZE,10,50);
		BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
	strncpy(target, data, len);
	target[len] = '\0';
	BSP_LCD_DisplayStringAt(0, 70, (uint8_t*)"Temperature2 is:", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, 110, (uint8_t*)target, CENTER_MODE);
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{


  float data[90] = {20.34,21.39,20.05,25.07,21.97,24.27,26.34,26.39,26.32,22.01,20.34,27.39,22.05,25.07,20.97,24.27,26.34,20.39,26.32,22.01,20.34,21.39,22.05,21.07,26.97,20.23,26.34,26.39,26.32,22.01,20.34,21.39,22.05,25.07,26.97,24.27,26.34,26.39,26.32,22.01,20.34,21.39,22.05,25.07,26.97,24.27,26.34,26.39,26.32,22.01,20.34,21.39,22.05,25.07,26.97,24.27,26.34,26.39,26.32,22.01,20.34,21.39,22.05,25.07,26.97,24.27,26.34,26.39,26.32,22.01,20.34,21.39,22.05,25.07,26.97,24.27,26.34,26.39,26.32,22.01,20.34,21.39,22.05,25.07,26.97,24.27,26.34,26.39,26.32,22.01};

// fill buffer
for(int i = 0; i < BUFFER_SIZE; i++)
    buffer[i] = INT32_MIN;


  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  debug_init(&huart1);

  xprintf(ANSI_FG_GREEN "STM32F746 Discovery Project" ANSI_FG_DEFAULT "\n");

  printf("Regular printf\n");

lcd_start();
draw_background();
initialize_touchscreen();
draw_chart(data);

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 4096);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);


  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPDIFRX|RCC_PERIPHCLK_LTDC
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART6|RCC_PERIPHCLK_SAI2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C3
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 100;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 4;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 4;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}




/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}



/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PB5   ------> USB_OTG_HS_ULPI_D7
     PH4   ------> USB_OTG_HS_ULPI_NXT
     PB13   ------> USB_OTG_HS_ULPI_D6
     PB12   ------> USB_OTG_HS_ULPI_D5
     PC0   ------> USB_OTG_HS_ULPI_STP
     PC2   ------> USB_OTG_HS_ULPI_DIR
     PA5   ------> USB_OTG_HS_ULPI_CK
     PB10   ------> USB_OTG_HS_ULPI_D3
     PA3   ------> USB_OTG_HS_ULPI_D0
     PB1   ------> USB_OTG_HS_ULPI_D2
     PB0   ------> USB_OTG_HS_ULPI_D1
     PB11   ------> USB_OTG_HS_ULPI_D4
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, ARDUINO_D7_Pin|ARDUINO_D8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMI_PWR_EN_GPIO_Port, DCMI_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin
                           ULPI_D2_Pin ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D3_Pin
                          |ULPI_D2_Pin|ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_INT_Pin */
  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D7_Pin ARDUINO_D8_Pin LCD_DISP_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TP3_Pin NC2_Pin */
  GPIO_InitStruct.Pin = TP3_Pin|NC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_PWR_EN_Pin */
  GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DCMI_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_RXER_Pin */
  GPIO_InitStruct.Pin = RMII_RXER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);
static void mqtt_sub_request_cb(void *arg, err_t result);
static void mqtt_pub_request_cb(void *arg, err_t result);
int example_do_connect(mqtt_client_t *client)
{
  struct mqtt_connect_client_info_t ci;
  err_t err;

  /* Setup an empty client info structure */
  memset(&ci, 0, sizeof(ci));

  /* Minimal amount of information required is client identifier, so set it here */
  ci.client_id = "lwip_test";

  /* Initiate client and connect to server, if this fails immediately an error code is returned
     otherwise mqtt_connection_cb will be called with connection result after attempting
     to establish a connection with the server.
     For now MQTT version 3.1.1 is always used */
  ip_addr_t ip_addr;
  if(1 == ipaddr_aton( "192.168.0.101",&ip_addr)){
	printf("Created ip");
  }
  err = mqtt_client_connect(client, &ip_addr, 1883, mqtt_connection_cb, 0, &ci);

  /* For now just print the result code if something goes wrong */
  if(err != ERR_OK) {
    printf("mqtt_connect return tutaj %d\n", err);
	return 1;

  }
  else{
	printf("mqtt_connectbvgfdbgfdgfd\n");
	return 0;
  }
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{

  err_t err;
  err_t err2;
  if(status == MQTT_CONNECT_ACCEPTED) {
    printf("mqtt_connection_cb: Successfully connected\n");

    /* Setup callback for incoming publish requests */
    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, arg);

    /* Subscribe to a topic named "subtopic" with QoS level 1, call mqtt_sub_request_cb with result */
    err = mqtt_subscribe(client, "esp/temperature/client2", 1, mqtt_sub_request_cb, arg);
	err2 = mqtt_subscribe(client, "esp/temperature/client1", 1, mqtt_sub_request_cb, arg);

    if(err != ERR_OK || err2 != ERR_OK) {
      printf("mqtt_subscribe return: %d\n", err);
    }
  } else {
    printf("mqtt_connection_cb: Disconnected, reason: %d\n", status);
    /* Its more nice to be connected, so try to reconnect */
    example_do_connect(client);
  }

}

static void mqtt_sub_request_cb(void *arg, err_t result)
{
  /* Just print the result code here for simplicity,
     normal behaviour would be to take some action if subscribe fails like
     notifying user, retry subscribe or disconnect from server */
  printf("Subscribe result: %d\n", result);
  if(result == ERR_OK){
  printf("Subscribe result is OK!\n");
  }
}

//-----------------------------------------------------------------
//3. Implementing callbacks for incoming publish and data

/* The idea is to demultiplex topic and create some reference to be used in data callbacks
   Example here uses a global variable, better would be to use a member in arg
   If RAM and CPU budget allows it, the easiest implementation might be to just take a copy of
   the topic string and use it in mqtt_incoming_data_cb
*/
static int inpub_id;
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) // callback invoked when publish starts, contain topic and total length of payload
{
  printf("Incoming publish at topic %s with total length %u\n", topic, (unsigned int)tot_len);

  /* Decode topic string into a user defined reference */
  if(strcmp(topic, "print_payload") == 0) {
    inpub_id = 0;
  } else if(topic[0] == 'A') {
    /* All topics starting with 'A' might be handled at the same way */
    inpub_id = 1;
  } else if(strcmp(topic,"esp/temperature/client1") == 0){
	inpub_id = 2;
  }
  else {
    /* For all other topics */
    inpub_id = 3;
  }
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) // callback for each fragment of payload that arrives
{
  printf("Incoming publish payload with length %d, flags %u\n", len, (unsigned int)flags);

  if(flags & MQTT_DATA_FLAG_LAST) {
    /* Last fragment of payload received (or whole part if payload fits receive buffer
       See MQTT_VAR_HEADER_BUFFER_LEN)  */

    /* Call function or do action depending on reference, in this case inpub_id */
    if(inpub_id == 0) {
      /* Don't trust the publisher, check zero termination */
      if(data[len-1] == 0) {
        printf("mqtt_incoming_data_cb: %s\n", (const char *)data);
      }
    } else if(inpub_id == 1) {
      /* Call an 'A' function... */

	  xprintf("Call an A function ...\n");

    } else if(inpub_id == 2){
		print_data((const char *)data, 1,len);
		add_data_to_buffer((const char *)data);
	}
	else {
      //printf("mqtt_incoming_data_cb: Ignoring payload...\n");
	  printf("mqtt_incoming_data_cb: %s\n", (const char *)data);
	  print_data((const char *)data,2,len);
    }
  } else {
    /* Handle fragmented payload, store in buffer, write to file or whatever */
  }
  //data = NULL;
}

//-----------------------------------------------------------------
//4. Using outgoing publish
 static void mqtt_pub_request_cb(void *arg, err_t result)
{
  if(result != ERR_OK) {
    printf("Publish result: %d\n", result);
  }
}

void example_publish(mqtt_client_t *client, void *arg)
{
  const char *pub_payload= "give_temperature";
  err_t err;
  u8_t qos = 2; /* 0 1 or 2, see MQTT specification */
  u8_t retain = 0; /* No don't retain such crappy payload... */
  err = mqtt_publish(client, "esp/temperature", pub_payload, strlen(pub_payload), qos, retain, mqtt_pub_request_cb, arg);
  if(err != ERR_OK) {
    printf("Publish err: %d\n", err);
  }
}

void create_menu(){


}
void StartDefaultTask(void const * argument)
{
  MX_LWIP_Init();

  /*MQTT */

  int tmp = 1; //1
  mqtt_client_t static_client;
  while(tmp == 1){
	xprintf("Trying to connect\n");
	vTaskDelay(1000);
	tmp = example_do_connect(&static_client);
	vTaskDelay(1000);
  //xprintf("tutaj koniecx");
  }

  for(;;)
  {

	BSP_TS_GetState(&TS_State);
	if(TS_State.touchDetected)
	{
		BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_SelectLayer(0);
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		BSP_LCD_FillRect(0,0,LCD_X_SIZE,LCD_Y_SIZE);

		//select Foreground Layer
		BSP_LCD_SelectLayer(1);
		char greeting[] = "Hello";
		draw_button(50,50,100,100,LCD_COLOR_MAGENTA,&greeting);

	}

	char key = inkey();
	if(key == 'p'){
		xprintf("Send message \n");
		example_publish(&static_client,2);
	}


	vTaskDelay(2);

  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
