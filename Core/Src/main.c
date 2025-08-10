/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "wifi.h"
#include "stdio.h"
#include <string.h>
#include <math.h>
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_exti.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_tests.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern void initialise_monitor_handles(void); // for printf semi-hosting support

#define MAX_LENGTH 1024
#define WIFI_READ_TIMEOUT 10000
#define WIFI_WRITE_TIMEOUT 10000

const char* WiFi_SSID = "waao"; // Replace with your WiFi SSID
const char* WiFi_password = "12345678"; // Replace with your WiFi password
const WIFI_Ecn_t WiFi_security = WIFI_ECN_WPA2_PSK;
const uint16_t SOURCE_PORT = 1234;
uint8_t ipaddr[4] = {192, 168, 43, 1};

const char* SERVER_NAME = "api.seniverse.com";
const uint16_t DEST_PORT = 80;

SPI_HandleTypeDef hspi3;

#define TOTAL_BUFFER_SIZE 2048
uint8_t totalResp[TOTAL_BUFFER_SIZE];
uint16_t totalLen = 0;
uint8_t tempBuffer[256]; // 每次接收的临时缓冲区
uint16_t len;
WIFI_Status_t ret;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UART1_Init(void);
UART_HandleTypeDef huart1;
/* USER CODE END 0 */
#define TEMP_THRESHOLD 37.5f
#define ACC_THRESHOLD 9.0f
#define GYRO_THRESHOLD 5.0f
#define MAG_THRESHOLD 3000000.0f
float baseline_gyro_norm = 0.0f;
float baseline_mag_norm = 0.0f;

volatile uint8_t emergency_mode = 0;
volatile uint8_t fever_detected = 0, fall_detected = 0, abnormal_detected = 0, mag_detected = 0;
volatile uint32_t last_emergency = 0;
volatile uint32_t last_led_toggle = 0;
volatile uint32_t last_button_press = 0;
volatile uint8_t count_button = 0;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  initialise_monitor_handles();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  UART1_Init();
  int seconds_count = 0;
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();

  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  /* USER CODE END 2 */


  BSP_LED_Init(LED2);

     BSP_TSENSOR_Init();
     BSP_ACCELERO_Init();
     BSP_GYRO_Init();
     BSP_MAGNETO_Init();
     BSP_HSENSOR_Init();
     BSP_PSENSOR_Init();
     //PC13用来EXTI interrupt
     __HAL_RCC_GPIOC_CLK_ENABLE();
     GPIO_InitTypeDef GPIO_InitStruct = {0};
     GPIO_InitStruct.Pin = GPIO_PIN_13;
     GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
     HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
     //设置EXTI interrupt优先级，只要按下按钮就会触发HAL_GPIO_EXTI_Callback()
     HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
     //PB14控制LED2输出
     __HAL_RCC_GPIOB_CLK_ENABLE();
     GPIO_InitStruct.Pin = GPIO_PIN_14;
     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

     __HAL_RCC_GPIOD_CLK_ENABLE();
     GPIO_InitStruct.Pin = GPIO_PIN_11;
     GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
     __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
     HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);  // EXTI for pins 10 to 15
     HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


     uint32_t previousTick = HAL_GetTick();
     uint32_t previousWReq = HAL_GetTick();
     static uint16_t counter=0;

     SENSOR_IO_Write(0xD4, 0x10, 0x20);
     SENSOR_IO_Write(0xD4, 0x19, 0x0C);
     SENSOR_IO_Write(0xD4, 0x5E, 0x02);
     SENSOR_IO_Write(0xD4, 0x58, 0x01);

     uint8_t md1_cfg;
  	 uint8_t func_src1;
     md1_cfg = SENSOR_IO_Read(0xD4, 0x5E);
     printf("MD1_CFG = 0x%02X\n", md1_cfg);






       char uartBuffer[256];
       char city[32];
       char temperature[8];
       char weather[32];
       char last_update[64];
       char time[6];

       uint8_t req[MAX_LENGTH];
       //uint8_t resp[MAX_LENGTH];
       uint16_t Datalen;

       WIFI_Status_t WiFi_Stat;
           uint32_t retryStart;

           while (1) {
               WiFi_Stat = WIFI_Init();
               HAL_UART_Transmit(&huart1, (uint8_t*)"Initializing WiFi...\r\n",
                                 strlen("Initializing WiFi...\r\n"), HAL_MAX_DELAY);

               if (WiFi_Stat == WIFI_STATUS_OK) break;

               HAL_UART_Transmit(&huart1, (uint8_t*)"WiFi Init Failed. Retrying...\r\n",
                                 strlen("WiFi Init Failed. Retrying...\r\n"), HAL_MAX_DELAY);

               // 记录当前时间
               retryStart = HAL_GetTick();
               while (HAL_GetTick() - retryStart < 2000);
           }

           while (1) {
               WiFi_Stat = WIFI_Connect(WiFi_SSID, WiFi_password, WiFi_security);
               if (WiFi_Stat == WIFI_STATUS_OK) break;

               HAL_UART_Transmit(&huart1, (uint8_t*)"WiFi Connect Failed. Retrying...\r\n",
                                 strlen("WiFi Connect Failed. Retrying...\r\n"), HAL_MAX_DELAY);

               retryStart = HAL_GetTick();
               while (HAL_GetTick() - retryStart < 2000);
           }

           HAL_UART_Transmit(&huart1, (uint8_t*)"Connected to WiFi. Resolving host...\r\n",
                             strlen("Connected to WiFi. Resolving host...\r\n"), HAL_MAX_DELAY);

           while (1) {
               WiFi_Stat = WIFI_GetHostAddress(SERVER_NAME, ipaddr);
               if (WiFi_Stat == WIFI_STATUS_OK) break;

               HAL_UART_Transmit(&huart1, (uint8_t*)"Host Resolve Failed. Retrying...\r\n",
                                 strlen("Host Resolve Failed. Retrying...\r\n"), HAL_MAX_DELAY);

               retryStart = HAL_GetTick();
               while (HAL_GetTick() - retryStart < 2000);
           }

           while (1) {
               WiFi_Stat = WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, DEST_PORT, SOURCE_PORT);
               if (WiFi_Stat == WIFI_STATUS_OK) break;

               HAL_UART_Transmit(&huart1, (uint8_t*)"Open Connection Failed. Retrying...\r\n",
                                 strlen("Open Connection Failed. Retrying...\r\n"), HAL_MAX_DELAY);

               retryStart = HAL_GetTick();
               while (HAL_GetTick() - retryStart < 2000);
           }

           HAL_UART_Transmit(&huart1, (uint8_t*)"WiFi Connected Successfully!\r\n",
                             strlen("WiFi Connected Successfully!\r\n"), HAL_MAX_DELAY);


           char rxBuffer[80];
           uint8_t rxChar;
           uint16_t index = 0;
           memset(rxBuffer, 0, 80);
           HAL_UART_Transmit(&huart1, (uint8_t*)"Key in city\r\n",
                                        strlen("Key in city\r\n"), HAL_MAX_DELAY);



           while (1) {
           HAL_UART_Receive(&huart1, &rxChar, 1, HAL_MAX_DELAY);
           if (rxChar == '\r' || rxChar == '\n')
           {
               HAL_UART_Transmit(&huart1, (uint8_t *)"Received: ", 6, HAL_MAX_DELAY);
               HAL_UART_Transmit(&huart1, rxBuffer, strlen((char *)rxBuffer), HAL_MAX_DELAY);
               HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
               break;
           }
           if (index < 79)
           {
        	   rxBuffer[index++] = rxChar;
           }
           }
           rxBuffer[index] = '\0';


		   const char* reqUserKey = "STEK6421IRBVnzBZ_";
		   const char* reqLocation = rxBuffer;


		   printf("city: %s\n", reqLocation);

		   const char* reqUnit = "c";

		   char reqWeather[256];
		   sprintf(reqWeather, "/v3/weather/now.json?key=%s&location=%s&language=en&unit=%s",
				   reqUserKey, reqLocation, reqUnit);

		   sprintf((char*)req, "GET %s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n",
				   reqWeather, SERVER_NAME);
		   HAL_UART_Transmit(&huart1, (uint8_t*)"Initialization Complete!\r\n Requesting...\r\n",
		                     strlen("Initialization Complete!\r\n Requesting...\r\n"), HAL_MAX_DELAY);

		   WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);

		   while ((ret = WIFI_ReceiveData(1, tempBuffer, sizeof(tempBuffer) - 1, &len, WIFI_READ_TIMEOUT)) == WIFI_STATUS_OK && len > 0) {
			   if (totalLen + len >= TOTAL_BUFFER_SIZE - 1) {
				   HAL_UART_Transmit(&huart1, (uint8_t*)"Buffer too small",
				   		                     strlen("Buffer too small"), HAL_MAX_DELAY);
				   break;
			   }
			   memcpy(totalResp + totalLen, tempBuffer, len);
			   totalLen += len;
		   }

		   totalResp[totalLen] = '\0';

		   extract_value(totalResp, "name", city, sizeof(city));
		   extract_value(totalResp, "temperature", temperature, sizeof(temperature));
		   extract_value(totalResp, "text", weather, sizeof(weather));
		   extract_value(totalResp, "last_update", last_update, sizeof(last_update));
		   strncpy(time, last_update + 11, 5);
		   time[5] = '\0';


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		   printf("%s",totalResp);
		          printf("city: %s\n", city);
		          printf("temperature: %s°C\n", temperature);
		          printf("weather: %s\n", weather);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


		   snprintf(uartBuffer, sizeof(uartBuffer),
					"City: %s\r\nTemperature: %s°C\r\nWeather: %s\r\nTime: %s\r\n",
					city, temperature, weather, time);
		   HAL_UART_Transmit(&huart1, (uint8_t*)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);

		   HAL_UART_Transmit(&huart1, (uint8_t*)"Entering Standard Mode.\r\n", 26, HAL_MAX_DELAY);


     while (1) {
     	uint32_t now = HAL_GetTick();
     	uint32_t W_now = HAL_GetTick();
      	func_src1 = SENSOR_IO_Read(0xD4, 0x53);
      	//printf("tti = 0x%02X\n", func_src1);

         if (emergency_mode == 0) {
        	 ssd1306_WeatherDisp (city, temperature, weather, time);
             if (now - previousTick >= 1000) {
                 previousTick = now;
                 //用SysTick interrupt每秒对sensors进行一次data记录
                 BSP_LED_Off(LED2);

                 float temp = BSP_TSENSOR_ReadTemp();
                 float humidity = BSP_HSENSOR_ReadHumidity();
                 float pressure = BSP_PSENSOR_ReadPressure();

                 float acc[3];
                 int16_t acc_i16[3]={0};
                 BSP_ACCELERO_AccGetXYZ(acc_i16);
                 acc[0]=acc_i16[0]*9.8f/1000.0f;
                 acc[1]=acc_i16[1]*9.8f/1000.0f;
                 acc[2]=acc_i16[2]*9.8f/1000.0f;
                 float gyro[3]={0};
                 BSP_GYRO_GetXYZ(gyro);
                 gyro[0]=(float)gyro[0]/1000.0f;
                 gyro[1]=(float)gyro[1]/1000.0f;
                 gyro[2]=(float)gyro[2]/1000.0f;
                 float mag[3];
                 int16_t mag_i16[3]={0};
                 BSP_MAGNETO_GetXYZ(mag_i16);
                 mag[0]=mag_i16[0]*1000.0f;
                 mag[1]=mag_i16[1]*1000.0f;
                 mag[2]=mag_i16[2]*1000.0f;

                 static uint8_t baseline_captured = 0;
                 if (!baseline_captured) {
                     baseline_gyro_norm = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
                     baseline_mag_norm  = sqrtf(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
                     baseline_captured = 1;
                 }//只会记录一次板子静止时的 gyro norm

                 char line1[256], line2[256], line3[256];
                 snprintf(line1,sizeof(line1),
                 		"%03d TEMP_%.2f ACC_%.2f_%.2f_%.2f\r\n",
                 		counter,temp,acc[0]/9.8f,acc[1]/9.8f,acc[2]/9.8f);
                 //题目要求把m/s^2转化成g
                 snprintf(line2, sizeof(line2),
                 		"%03d GYRO_%.2f_%.2f_%.2f MAGNETO_%.2f_%.2f_%.2f\r\n",
                         counter, gyro[0],gyro[1],gyro[2], mag[0], mag[1], mag[2]);
                 snprintf(line3, sizeof(line3),
                         "%03d HUMIDITY_%.2f BARO_%.2f\r\n",
                         counter, humidity, pressure);

                 HAL_UART_Transmit(&huart1, (uint8_t*)line1, strlen(line1), HAL_MAX_DELAY);
                 HAL_UART_Transmit(&huart1, (uint8_t*)line2, strlen(line2), HAL_MAX_DELAY);
                 HAL_UART_Transmit(&huart1, (uint8_t*)line3, strlen(line3), HAL_MAX_DELAY);
                 counter++;
             }
             if (W_now - previousWReq >= 100*1000) {
            	 previousWReq = W_now;
            	 WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);

            	 		   while ((ret = WIFI_ReceiveData(1, tempBuffer, sizeof(tempBuffer) - 1, &len, WIFI_READ_TIMEOUT)) == WIFI_STATUS_OK && len > 0) {
            	 			   if (totalLen + len >= TOTAL_BUFFER_SIZE - 1) {
            	 				   HAL_UART_Transmit(&huart1, (uint8_t*)"Buffer too small",
            	 				   		                     strlen("Buffer too small"), HAL_MAX_DELAY);
            	 				   break;
            	 			   }
            	 			   memcpy(totalResp + totalLen, tempBuffer, len);
            	 			   totalLen += len;
            	 		   }

            	 		   totalResp[totalLen] = '\0';

            	 		   extract_value(totalResp, "name", city, sizeof(city));
            	 		   extract_value(totalResp, "temperature", temperature, sizeof(temperature));
            	 		   extract_value(totalResp, "text", weather, sizeof(weather));
            	 		   extract_value(totalResp, "last_update", last_update, sizeof(last_update));
            	 		   strncpy(time, last_update + 11, 5);
            	 		   time[5] = '\0';

            	 		  snprintf(uartBuffer, sizeof(uartBuffer),
            	 		  					"City: %s\r\nTemperature: %s°C\r\nWeather: %s\r\nTime: %s\r\n",
            	 		  					city, temperature, weather, time);
            	 		  HAL_UART_Transmit(&huart1, (uint8_t*)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
             }
         }else {

        	 ssd1306_Alarm();
        	 func_src1 = SENSOR_IO_Read(0xD4, 0x53);
             float temp = BSP_TSENSOR_ReadTemp();
             float acc[3];
             int16_t acc_i16[3]={0};
             BSP_ACCELERO_AccGetXYZ(acc_i16);
             acc[0]=acc_i16[0]*9.8f/1000.0f;
             acc[1]=acc_i16[1]*9.8f/1000.0f;
             acc[2]=acc_i16[2]*9.8f/1000.0f;
             float gyro[3]={0};
             BSP_GYRO_GetXYZ(gyro);
             gyro[0]=(float)gyro[0]/1000.0f;
             gyro[1]=(float)gyro[1]/1000.0f;
             gyro[2]=(float)gyro[2]/1000.0f;
             float mag[3];
             int16_t mag_i16[3]={0};
             BSP_MAGNETO_GetXYZ(mag_i16);
             mag[0]=mag_i16[0]*1000.0f;
             mag[1]=mag_i16[1]*1000.0f;
             mag[2]=mag_i16[2]*1000.0f;

             float gyro_norm=0.0f;
             gyro_norm=sqrt(pow(gyro[0],2)+pow(gyro[1],2)+pow(gyro[2],2));
             float mag_norm=0.0f;
             mag_norm=sqrt(pow(mag[0],2)+pow(mag[1],2)+pow(mag[2],2));

             fever_detected = (temp > TEMP_THRESHOLD);
             //比37.5高就发烧了
             fall_detected = fabsf(acc[2]) < ACC_THRESHOLD;
             //比较在z方向的重力加速度来判断是否摔倒
             abnormal_detected =fabsf(gyro_norm - baseline_gyro_norm) > GYRO_THRESHOLD;
             //比较水平静止时的gyro norm和现在的值的偏差是否大于5dps
             mag_detected = fabsf(mag_norm-baseline_mag_norm) > MAG_THRESHOLD;

             uint32_t blink_interval=1000;
             if (fever_detected){
             	blink_interval=333;//如果fever了是3Hz亮LED
             } else if (fall_detected||abnormal_detected||mag_detected){
             	blink_interval=500;//别的毛病2Hz亮LED
             }

             if (now - last_led_toggle >= blink_interval) {
                 last_led_toggle = now;
                 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
             }
             //all sensors data 5s传一次
             if (now - last_emergency >= 5000) {
                 last_emergency = now;

                 if (fever_detected)
                     HAL_UART_Transmit(&huart1, (uint8_t*)"Fever detected!\r\n", 18, HAL_MAX_DELAY);
                 if (fall_detected)
                     HAL_UART_Transmit(&huart1, (uint8_t*)"Fall detected!\r\n", 17, HAL_MAX_DELAY);
                 if (abnormal_detected)
                     HAL_UART_Transmit(&huart1, (uint8_t*)"Abnormal movement detected!\r\n", 30, HAL_MAX_DELAY);
                 if (mag_detected)
                     HAL_UART_Transmit(&huart1, (uint8_t*)"Posture anomaly detected!\r\n", 28, HAL_MAX_DELAY);

                 char status[100];
                 snprintf(status, sizeof(status), "TEMP=%.2f ACC_Z=%.2f GYRO=%.2f MAG=%.2f\r\n",
                         temp, acc[2], gyro[0], mag[0]);
                 HAL_UART_Transmit(&huart1, (uint8_t*)status, strlen(status), HAL_MAX_DELAY);

             if (fever_detected||fall_detected||abnormal_detected||mag_detected){
            	 ssd1306_Fill(White);
            	 ssd1306_Scroll();
             }
             }
         }
     }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void extract_value(const char* json, const char* key, char* dest, size_t dest_size) {
    char searchKey[32];
    sprintf(searchKey, "\"%s\":\"", key);
    char* ptr = strstr(json, searchKey);
    if(ptr) {
        ptr += strlen(searchKey);
        char* end = strchr(ptr, '\"');
        if(end) {
            size_t len = end - ptr;
            if(len >= dest_size)
                len = dest_size - 1;
            strncpy(dest, ptr, len);
            dest[len] = '\0';
        } else {
            dest[0] = '\0';
        }
    } else {
        dest[0] = '\0';
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_1) {
        SPI_WIFI_ISR();
    }
        if (GPIO_Pin == BUTTON_EXTI13_Pin) {//如果elderly按了button则进入EXTI interrupt
            uint32_t now = HAL_GetTick();
            if (!emergency_mode) {//如果在标准模式
                if (now - last_button_press < 1000) {//判断是否在一秒内双击
                    count_button++;
                    if (count_button >= 2) {//如果是的话就进入到emergency mode
                        emergency_mode = 1;
                        count_button = 0;
                        HAL_UART_Transmit(&huart1, (uint8_t*)"Entering Emergency Mode.\r\n", 27, HAL_MAX_DELAY);
                    }
                } else {
                    count_button = 1;//在标准模式但没能1秒内完成两次双击
                }
                last_button_press = now;
            } else {
                emergency_mode = 0;//如果不在标准模式，则从emergency回到标准模式
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);//且关掉LED2，因为要回standard mode了
                HAL_UART_Transmit(&huart1, (uint8_t*)"Returning to Standard Mode.\r\n", 29, HAL_MAX_DELAY);
            }
        }
        if (GPIO_Pin == GPIO_PIN_11)
        {
          //printf("falllllllll\n");
          HAL_UART_Transmit(&huart1, (uint8_t*)"Tilt detected.\r\n", 29, HAL_MAX_DELAY);
        }
}

void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi3);
}

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */




static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00F12981;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void UART1_Init(void)
{
/* Pin configuration for UART. BSP_COM_Init() can do this automatically
*/
__HAL_RCC_GPIOB_CLK_ENABLE();
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
/* Configuring UART1 */
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
while(1);
}
}
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
