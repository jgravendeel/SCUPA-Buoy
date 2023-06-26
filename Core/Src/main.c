/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "nmea_parse.h"
#include "LoRa.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void try_parse_wearable_message();
void try_parse_gps_data();
void try_parse_observer_message();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define WEARABLE_TO_BUOY_BUFF_SIZE 255 // limited to uint8_t
#define GPS_DATA_BUFF_SIZE 128
#define OBSERVER_TO_BUOY_BUFF_SIZE 255 // limited to uint8_t

uint8_t wearable_to_buoy_buff[WEARABLE_TO_BUOY_BUFF_SIZE] = { 0 };
uint8_t gps_data_buff[GPS_DATA_BUFF_SIZE] = { 0 };
uint8_t observer_to_buoy_buff[OBSERVER_TO_BUOY_BUFF_SIZE] = { 0 };
uint8_t last_coords[64] = { 0 };

GPS gps_parser;
LoRa lora_parser;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_RTC_Init();
    MX_SPI2_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_UART5_Init();
    /* USER CODE BEGIN 2 */
    lora_parser = newLoRa();
    lora_parser.hSPIx = &hspi2;
    lora_parser.CS_port = LORA_NSS_GPIO_Port;
    lora_parser.CS_pin = LORA_NSS_Pin;
    lora_parser.reset_port = LORA_RESET_GPIO_Port;
    lora_parser.reset_pin = LORA_RESET_Pin;
    lora_parser.DIO0_port = LORA_IRQ_GPIO_Port;
    lora_parser.DIO0_pin = LORA_IRQ_Pin;

    lora_parser.frequency = 433; // legal frequency in NL and curacao, do research before changing this
    lora_parser.spredingFactor = SF_7; // higher sf means lower throughput
    lora_parser.bandWidth = BW_125KHz; // can go up to 500 kHz, have to see what is best
    lora_parser.crcRate = CR_4_5; // ratio of error correcting bits vs data bits, have to see what is best
    lora_parser.power = POWER_20db; // default value is max so probably just for energy savings
    lora_parser.overCurrentProtection = 100; // there are some legal limits to this, more research required, leave at default for now
    lora_parser.preamble = 8; // bits for synchronizing transmitter with receiver, default is probably fine

    LoRa_reset(&lora_parser);
    uint16_t lora_status = LoRa_init(&lora_parser);
    if (lora_status != LORA_OK) {
        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
        Error_Handler();
    }
    LoRa_startReceiving(&lora_parser);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        try_parse_wearable_message();
        try_parse_gps_data();
        try_parse_observer_message();
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE
            | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Activate the Over-Drive mode
     */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void send_to_observer(uint8_t *buff) {
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
    LoRa_transmit(&lora_parser, buff, strlen((char*) buff), 5000);
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
}

void send_to_wearable(uint8_t *buff) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(USART1_RTS_GPIO_Port, USART1_RTS_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, buff, strlen((char*) buff), 100);
    HAL_GPIO_WritePin(USART1_RTS_GPIO_Port, USART1_RTS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
}

void try_parse_wearable_message() {
    memset(wearable_to_buoy_buff, 0, WEARABLE_TO_BUOY_BUFF_SIZE);
    HAL_UART_Receive(&huart1, wearable_to_buoy_buff, WEARABLE_TO_BUOY_BUFF_SIZE,
            100);

    // basic filter for valid messages
    if (wearable_to_buoy_buff[0] != '|') {
        return;
    }

    send_to_observer(wearable_to_buoy_buff);
}

void try_parse_gps_data() {
    memset(gps_data_buff, 0, GPS_DATA_BUFF_SIZE);
    HAL_UART_Receive(&huart5, gps_data_buff, GPS_DATA_BUFF_SIZE, 100);

    nmea_parse(&gps_parser, gps_data_buff);
    if (!gps_parser.fix) {
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
        return;
    }
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

    // if the coordinate is invalid it defaults to 0.0,0.0
    if (gps_parser.latitude == 0 && gps_parser.longitude == 0) {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
        return;
    }
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

    // no support for float formatting so use int instead
    int before_dot_lat = (int) gps_parser.latitude;
    int after_dot_lat = (int) ((gps_parser.latitude - before_dot_lat) * 1e6);
    int before_dot_lng = (int) gps_parser.longitude;
    int after_dot_lng = (int) ((gps_parser.longitude - before_dot_lng) * 1e6);

    uint8_t latlng[64] = { 0 };
    sprintf((char*) latlng, "|GPB|%d.%d|%d.%d|", before_dot_lat, after_dot_lat,
            before_dot_lng, after_dot_lng);

    // prevent spamming the wearable with identical coordinates
    if (strcmp((char*) latlng, (char*) last_coords) == 0) {
        return;
    }

    strcpy((char*) last_coords, (char*) latlng);
    send_to_wearable(latlng);
}

void try_parse_observer_message() {
    LoRa_receive(&lora_parser, observer_to_buoy_buff,
    OBSERVER_TO_BUOY_BUFF_SIZE);

    // basic filter for valid messages
    if (observer_to_buoy_buff[0] != '|') {
        return;
    }

    send_to_wearable(observer_to_buoy_buff);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
