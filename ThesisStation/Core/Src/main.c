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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "Autofox_INA226_c.h"
#include "hx711.h"
#include "hx711Config.h"
#define hx711_delay(x)    HAL_Delay(x)
#include "VL6180X.h"
#define TCAADDR  0xE0
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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
#define BUFFER_LEN  1
uint8_t RX_BUFFER[BUFFER_LEN] = { 0 };
uint8_t TX_BUFFER[BUFFER_LEN] = { 0 };
int16_t inputchar;
int sclk[2] = { 0 };
int led = 0;
int sclk2[2] = { 0 };
int sclk3[2] = { 0 };
uint64_t val_0 = 0;
uint64_t val_1 = 0;
uint64_t val_2 = 0;
uint64_t _micros = -4294967295;
uint64_t timestamp = 0;
int trig = 0;
int distance = 0;
int in1 = 0;
int in2 = 0;
int pwm1 = 0;
int in3 = 0;
int in4 = 0;
int pwm2 = 0;
int in5 = 0;
int in6 = 0;
int pwm3 = 0;
hx711_t loadcell;
hx711_t loadcell2;
float weight1;
float weight2;
int hx711_channel = 1;
uint64_t hx_711timestamp = 0;
uint64_t hx_711readtimestamp[2] = { 0, 1 };
uint64_t hx_711readtimestamp2[2] = { 0, 1 };
int timecheck[2] = { 0 };
uint8_t loadcellc1 = 0;
uint8_t loadcellc2 = 0;
uint8_t loadcellc3 = 0;
uint8_t loadcellc4 = 0;
float tareweight1[5] = { 0, 0, 0, 0, 0 };
float tareweight2[5] = { 0, 0, 0, 0, 0 };
float tareweight3[5] = { 0, 0, 0, 0, 0 };
float tareweight4[5] = { 0, 0, 0, 0, 0 };
int ave1 = 0;
int ave2 = 0;
int ave3 = 0;
int ave4 = 0;
int check = 0;
int answer = 0;
int tare = 0;
int start = 1;
int start2 = 1;
int cal = 0;
int distance_avg = 0;
int countultra = 0;
float Kp1 = 30;
float Kd1 = 0;
float Ki1 = 0;
float sumpid1 = 0;
float errorpid1[2] = { 0, 0 };
float require1 = 0.0;
float Kp2 = 30;
float Kd2 = 0;
float Ki2 = 0;
float sumpid2 = 0;
float errorpid2[2] = { 0, 0 };
float require2 = 0.0;
float Kp3 = 6;
float Kd3 = 0;
float Ki3 = 0;
float sumpid3 = 0;
float errorpid3[2] = { 0, 0 };
float require3 = 0.0;
int limitswitch1 = 0;
int limitswitch2 = 0;
int limitswitch3 = 0;
int limitswitchc1 = 0;
int limitswitchc2 = 0;
int limitswitchc3 = 0;
float distancemetre1 = 0;
float distancemetre2 = 0;
float distancemetre3 = 0;
float velocitymetre1 = 0;
float velocitymetre2 = 0;
float velocitymetre3 = 0;
int distancepulse1 = 0;
int distancepulse2 = 0;
int distancepulse3 = 0;
float velocitypulse1 = 0;
float velocitypulse2 = 0;
float velocitypulse3 = 0;
int distancestamp1 = 0;
int distancestamp2 = -65535;
int distancestamp3 = -65535;
int olddistance1 = 0;
int distance1tozero = 0;
int distance3tozero = 0;
int error1 = 0;
int error2 = 0;
int error3 = 0;
int32_t EncoderPositionDiff1;
uint64_t EncoderTimeDiff1;
int32_t EncoderPositionDiff2;
uint64_t EncoderTimeDiff2;
int32_t EncoderPositionDiff3;
uint64_t EncoderTimeDiff3;
int emergencyswitch = 0;
double theVoltage_V;
double theCurrent_mA;
double thePower_mA;
const uint8_t INA226_IC2_ADDRESS = 0x40;
const double SHUNT_RESISTOR_OHMS = 0.15;
AutoFox_INA226 ina226;
float lux1;
float range1;
float lux2;
float range2;
int zerostate = 0;
int oldzerostate = 0;
int activate1 = 0;
int activate2 = 0;
int activate3 = 0;
float ptg1 = 0;
float ptg2 = 0;
float ptg3 = 0;
uint64_t Timestamp_Encoder = 0;
int state = -1;
int emergency = 0;
uint64_t chance = 0;
float rscale1[4] = {-0.155, -0.153,0.05,0.280};
float rscale2[4] = {-0.33 , -0.33 , 0.08 , 0.53};
const float x[6] = {13085,10713,32300,38650,567.3,431};
const float y[6] = {11768,8600,35100,48300,585,512.5};
const float z[6] = {12850,5600,61200,31200,369.52,561.9};
const float p[6] = {8100,6500,30500,70721,724.3,651.8};
const float w = 1170;
const float h = 1065;
const float m = 1.56;
float xr[6];
float yr[6];
float zr[6];
float pr[6];
float xc[4];
float yc[4];
float zc[4];
float pc[4];
float rc[4];
uint64_t initial_time = 0;
uint64_t weight_time = 0;
uint64_t bluetooth_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

void hx711_delay_us();
void hx711_lock(hx711_t *hx711);
void hx711_unlock(hx711_t *hx711);
void hx711_init(hx711_t *hx711, GPIO_TypeDef *clk_gpio, uint16_t clk_pin,
		GPIO_TypeDef *dat_gpio, uint16_t dat_pin);
int32_t hx711_value(hx711_t *hx711);
int32_t hx711_value_ave(hx711_t *hx711, uint16_t sample);
void hx711_tare(hx711_t *hx711, uint16_t sample, int unit);
void hx711_calibration(hx711_t *hx711, int32_t channel, float scale);
float hx711_weight(hx711_t *hx711, uint16_t sample, uint8_t num);
void hx711_coef_set(hx711_t *hx711, float coefA, float coefB);
float hx711_coef_get(hx711_t *hx711);
void hx711_power_down(hx711_t *hx711);
void hx711_power_up(hx711_t *hx711);
uint64_t micros();
void tcaselect(uint8_t i);
void setResistance(int percent);
float EncoderVelocity_Update(int unit);
void limitswitchlowpass();
void distancemeasurement();
void velocitymeasurement();
void gotoposition(int unit);
void setzero();
void pwmdrive();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_USART6_UART_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */
//  HAL_UART_Receive_IT(&huart1, RX_BUFFER, BUFFER_LEN);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	hx711_init(&loadcell, GPIOC, GPIO_PIN_0, GPIOC, GPIO_PIN_1);
	hx711_coef_set(&loadcell, 1, 1); // read after calibration
	hx711_init(&loadcell2, GPIOC, GPIO_PIN_2, GPIOC, GPIO_PIN_3);
	hx711_coef_set(&loadcell2, 1, 1); // read after calibration
//  AutoFox_INA226_Constructor(&ina226);
//  AutoFox_INA226_Init(&ina226,INA226_IC2_ADDRESS,SHUNT_RESISTOR_OHMS,10);
//  AutoFox_INA226_setupCalibration(&ina226,SHUNT_RESISTOR_OHMS,10);
	tcaselect(0);
	HAL_Delay(1);
	VL6180X_Init(&hi2c2);
	HAL_Delay(1);
	tcaselect(1);
	HAL_Delay(1);
	VL6180X_Init(&hi2c2);
	xr[0] = (m*(h-x[5])*(w-x[4]))/(h*w);
	xr[1] = (m*(h-x[5])*x[4])/(h*w);
	xr[2] = (m*x[5]*(w-x[4]))/(h*w);
	xr[3] = (m*x[5]*x[4])/(h*w);
	yr[0] = (m*(h-y[5])*(w-y[4]))/(h*w);
	yr[1] = (m*(h-y[5])*y[4])/(h*w);
	yr[2] = (m*y[5]*(w-y[4]))/(h*w);
	yr[3] = (m*y[5]*y[4])/(h*w);
	zr[0] = (m*(h-z[5])*(w-z[4]))/(h*w);
	zr[1] = (m*(h-z[5])*z[4])/(h*w);
	zr[2] = (m*z[5]*(w-z[4]))/(h*w);
	zr[3] = (m*z[5]*z[4])/(h*w);
	pr[0] = (m*(h-p[5])*(w-p[4]))/(h*w);
	pr[1] = (m*(h-p[5])*p[4])/(h*w);
	pr[2] = (m*p[5]*(w-p[4]))/(h*w);
	pr[3] = (m*p[5]*p[4])/(h*w);
	xc[0] = x[0]/xr[0];
	xc[1] = x[1]/xr[1];
	xc[2] = x[2]/xr[2];
	xc[3] = x[3]/xr[3];
	yc[0] = y[0]/yr[0];
	yc[1] = y[1]/yr[1];
	yc[2] = y[2]/yr[2];
	yc[3] = y[3]/yr[3];
	zc[0] = z[0]/zr[0];
	zc[1] = z[1]/zr[1];
	zc[2] = z[2]/zr[2];
	zc[3] = z[3]/zr[3];
	pc[0] = p[0]/pr[0];
	pc[1] = p[1]/pr[1];
	pc[2] = p[2]/pr[2];
	pc[3] = p[3]/pr[3];
	rc[0] = 27306.9453;
	rc[1] = 23035.0215;
	rc[2] = 100504.188;
	rc[3] = 124988.211;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		switch (state) {
		case -1: // initial set zero
			zerostate = 1;
			state = -2;
			break;
		case -2: // set zero finish wait for 5 sec
			if(zerostate == 0){
				initial_time = micros();
				state = -3;
			}
			break;
		case -3: // after w8 for 10 sec tare loadcell
			if(micros() - initial_time >= 10000000){
				hx711_tare(&loadcell, 4, 1);
				hx711_tare(&loadcell2, 4, 2);
				hx711_calibration(&loadcell, 2, rc[0]);
				hx711_calibration(&loadcell2, 2,  rc[1]);
				hx711_calibration(&loadcell, 1,  rc[2]);
				hx711_calibration(&loadcell2, 1,  rc[3]);
				state = 0;
			}
			break;
		case 0: // idle
			if(loadcell.weightB + loadcell2.weightB + loadcell.weightA + loadcell2.weightA >= 1.80){
				weight_time = micros();
				state = 1;
			}
			break;
		case 1: // detect drone weight for 5 sec
			if(loadcell.weightB + loadcell2.weightB + loadcell.weightA + loadcell2.weightA <= 1.80){
				state = 0;
			}
			if(micros() - weight_time >= 5000000){
				state = 2;
				bluetooth_time = micros();
			}
			break;
		case 2:  // check bluetooth connection
			TX_BUFFER[0] = '1';
			HAL_UART_Transmit(&huart1, TX_BUFFER, 1, 10);
			HAL_UART_Receive_IT(&huart1, RX_BUFFER, BUFFER_LEN);
			if(inputchar == '1'){
				state = 11;
			}
			if(micros() - bluetooth_time >= 5000000){
				state = 0;
			}
			break;
		case 11: // push panel1
			ptg1 = 588;
			activate1 = 1;
			state = 12;
			break;
		case 12:  //  push panel2
			if (activate1 == 0) {
				activate2 = 1;
				ptg2 = 615;
				state = 13;
			}
			break;
		case 13: // check orientation
			if (activate2 == 0) {
				if (range1 <= 180 && range2 >= 200) {
					state = 142;
				}
				if (range2 <= 180 && range1 >= 200) {
					state = 141;
				}else{
					if(chance == 0){
						chance = micros();
					}
					if(micros() - chance >= 3000000){
						state = 132;
						chance = 0;
					}
				}
			}
			break;
		case 132:  //error
			zerostate = 1;
			state = 0;
		    break;
		case 141: // push panel2 more
			ptg2 = 680;
			activate2 = 1;
			state = 1412;
			break;
		case 1412: // push finish drone in charging area
			if (activate2 == 0) {
				state = 15;
			}
			break;
		case 142: // push panel1 more
			ptg1 = 655;
			activate1 = 1;
			state = 1422;
			break;
		case 1422: // push finish drone in charging area
			if (activate1 == 0) {
				state = 15;
			}
			break;
		case 15: // Charging state
			break;
		case 16: // push panel1 back
			ptg1 = 0;
			activate1 = 1;
			state = 17;
			break;
		case 17: // after panel1 back to 50 cm panel2 will go back too
			if (distancemetre1 <= 500) {
				ptg2 = 0;
				activate2 = 1;
				state = 18;
			}
			break;
		case 18: // after panel1 and panel 2 go back slider will move done back to center
			if (distancemetre1 <= 500 && distancemetre2 <= 250) {
				ptg3 = 635;
				in5 = 1;
				in6 = 0;
				activate3 = 1;
				state = 19;
			}
			break;
		case 19: // after drone slide to center set zero
			if (activate3 == 0) {
				zerostate = 1;
				state = 0;
			}
			break;
		default:
			break;
		}
		if (emergency == 1) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
		} else {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		}
//	  Limit switch
		limitswitchlowpass();
//	  set zero
		setzero();
//	  Distance measurement
		distancemeasurement();
//	  velocity measurement
		if (micros() - Timestamp_Encoder >= 100) {
			velocitymeasurement();
		}
//	  go to position
		if (activate1 == 1 && zerostate == 0) {
			gotoposition(1);
		} else if (activate1 == 1 && zerostate == 0) {
			require1 = 0;
		}
		if (activate2 == 1 && zerostate == 0) {
			gotoposition(2);
		} else if (activate2 == 1 && zerostate == 0) {
			require2 = 0;
		}
		if (activate3 == 1 && zerostate == 0) {
			gotoposition(3);
		} else if (activate3 == 1 && zerostate == 0) {
			require3 = 0;
		}

//	  PWM drive
		pwmdrive();
		//	  Load cell
		if (micros() - hx_711timestamp >= 500000) {
			if (loadcellc1 == 10 && loadcellc2 == 10 && loadcellc3 == 10
					&& loadcellc4 == 10) {
				hx_711timestamp = micros();
				loadcellc1 = 4;
				loadcellc2 = 4;
				loadcellc3 = 4;
				loadcellc4 = 4;
				ave1 = 0;
				ave2 = 0;
				ave3 = 0;
				ave4 = 0;
			}
		}
		hx711_weight(&loadcell, 4, 1);
		hx711_weight(&loadcell2, 4, 2);
		sclk[0] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		if (sclk[0] == 0 && sclk[1] == 1) {
			hx711_tare(&loadcell, 4, 1);
			hx711_tare(&loadcell2, 4, 2);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
		}
//	  Bluetooth
//	  HAL_UART_Receive_IT(&huart1, RX_BUFFER, BUFFER_LEN);
//	  if(inputchar == '1')
//	      {
//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
//	      }
//	  else if(inputchar == '0')
//	      {
//	      	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
//	      }
//	  sclk[0] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
//	  if(sclk[0] == 0 && sclk[1] == 1){
//		  if(led == 0){
//			  led = 1;
//			  TX_BUFFER[0] = '1';
//		  }
//		  else if(led == 1){
//			  led = 0;
//			  TX_BUFFER[0] = '0';
//		  }
//	HAL_UART_Transmit(&huart1, TX_BUFFER, 1, 10);
//	  }
//	sclk[1] = sclk[0];

		//current and voltage sensor
//	 theVoltage_V = AutoFox_INA226_GetShuntVoltage_uV(&ina226) ;
//	 HAL_Delay (100);
//	 theCurrent_mA = AutoFox_INA226_GetCurrent_uA(&ina226) ;
//	 HAL_Delay (100);
//	 thePower_mA = AutoFox_INA226_GetPower_uW(&ina226);
//	 HAL_Delay (100);

		//laser sensor
//
		tcaselect(0);
		HAL_Delay(1);
//		lux1 = VL6180X_readLux(VL6180X_ALS_GAIN_5);
		range1 = VL6180X_readRange();
		tcaselect(1);
		HAL_Delay(1);
//		lux2 = VL6180X_readLux(VL6180X_ALS_GAIN_5);
		range2 = VL6180X_readRange();
//	 setResistance(check);
//	 HAL_Delay (100);
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 5000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 99;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_3 | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_15 | GPIO_PIN_3
					| GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC0 PC2 PC10 PC11
	 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11
			| GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC1 PC3 PC7 PC8
	 PC9 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_7 | GPIO_PIN_8
			| GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA2 PA3 LD2_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB15
	 PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_15
			| GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB14 */
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PC6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB3 PB4 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	inputchar = *RX_BUFFER;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_6) {
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 1) {
			emergency = 1;
		} else {
			emergency = 0;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim5) {
		_micros += 4294967295;
	}
	if (htim == &htim2) {
		if ((TIM2->CNT) <= 32000) {
			distancestamp2 += 65535;
		} else {
			distancestamp2 -= 65535;
		}
	}
	if (htim == &htim3) {
		if ((TIM3->CNT) <= 32000) {
			distancestamp3 += 65535;
		} else {
			distancestamp3 -= 65535;
		}
	}
	if (htim == &htim4) {
		if (require1 == 0) {
			errorpid1[0] = 0;
		} else if (require1 <= 0) {
			errorpid1[0] = velocitypulse1
					- require1 * 12.0 * 64.0 * 4.0 * 20.0 / 15.0 / 8.0;
		} else if (require1 > 0) {
			errorpid1[0] = require1 * 12.0 * 64.0 * 4.0 * 20.0 / 15.0 / 8.0
					- velocitypulse1;
		}
		sumpid1 = sumpid1 + errorpid1[0];
		pwm1 = (Kp1 * errorpid1[0] + Ki1 * sumpid1
				+ Kd1 * (errorpid1[0] - errorpid1[1]));
		errorpid1[1] = errorpid1[0];
		if (pwm1 < 0) {
			pwm1 = -pwm1;
			in1 = !in1;
			in2 = !in2;
		}
		if (pwm1 > 5000)
			pwm1 = 5000;
		if ((limitswitch1 == 1 && in1 == 0 && in2 == 1) || emergency == 1) {
			pwm1 = 0;
			in1 = 0;
			in2 = 0;
		}
		if (require2 == 0) {
			errorpid2[0] = 0;
		} else if (require2 <= 0) {
			errorpid2[0] = velocitypulse2
					- require2 * 12.0 * 64.0 * 4.0 * 20.0 / 15.0 / 8.0;
		} else if (require2 > 0) {
			errorpid2[0] = require2 * 12.0 * 64.0 * 4.0 * 20.0 / 15.0 / 8.0
					- velocitypulse2;
		}
		sumpid2 = sumpid2 + errorpid2[0];
		pwm2 = (Kp2 * errorpid2[0] + Ki2 * sumpid2
				+ Kd2 * (errorpid2[0] - errorpid2[1]));
		errorpid2[1] = errorpid2[0];
		if (pwm2 < 0) {
			pwm2 = -pwm2;
			in3 = !in3;
			in4 = !in4;
		}
		if (pwm2 > 5000)
			pwm2 = 5000;
		if ((limitswitch2 == 1 && in3 == 0 && in4 == 1) || emergency == 1) {
			pwm2 = 0;
		}
		if (require3 == 0) {
			errorpid3[0] = 0;
		} else if (require3 <= 0) {
			errorpid3[0] = velocitypulse3
					- require3 * 12.0 * 64.0 * 4.0 * 15.0 / 10.0 / 12.0 / 5.08;
		} else if (require3 > 0) {
			errorpid3[0] = require3 * 12.0 * 64.0 * 4.0 * 15.0 / 10.0 / 12.0/5.08
					- velocitypulse3;
		}
		sumpid3 = sumpid3 + errorpid3[0];
		pwm3 = (Kp3 * errorpid3[0] + Ki3 * sumpid3
				+ Kd3 * (errorpid3[0] - errorpid3[1]));
		errorpid3[1] = errorpid3[0];
		if (pwm3 < 0) {
							pwm3 = -pwm3;
							in5 = !in5;
							in6 = !in6;
						}
		if (pwm3 > 2300)
			pwm3 = 2300;
		if ((limitswitch3 == 1 && in5 == 0 && in6 == 1) || emergency == 1) {
			pwm3 = 0;
		}
	}
}

uint64_t micros() {
	return _micros + htim5.Instance->CNT;
}
//#############################################################################################
void hx711_delay_us(void) {
	uint64_t delay = micros();
	while (delay - micros() <= 1)
		;
}
//#############################################################################################
void hx711_lock(hx711_t *hx711) {
	while (hx711->lock)
		;
	hx711->lock = 1;
}
//#############################################################################################
void hx711_unlock(hx711_t *hx711) {
	hx711->lock = 0;
}
//#############################################################################################
void hx711_init(hx711_t *hx711, GPIO_TypeDef *clk_gpio, uint16_t clk_pin,
		GPIO_TypeDef *dat_gpio, uint16_t dat_pin) {
	hx711_lock(hx711);
	hx711->clk_gpio = clk_gpio;
	hx711->clk_pin = clk_pin;
	hx711->dat_gpio = dat_gpio;
	hx711->dat_pin = dat_pin;

	GPIO_InitTypeDef gpio = { 0 };
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio.Pin = clk_pin;
	HAL_GPIO_Init(clk_gpio, &gpio);
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio.Pin = dat_pin;
	HAL_GPIO_Init(dat_gpio, &gpio);
	HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_SET);
	hx711_delay(10);
	HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET);
	hx711_delay(10);
}
//#############################################################################################
int32_t hx711_value(hx711_t *hx711) {
	uint32_t data = 0;
	if (tare == 1) {
		uint32_t startTime = HAL_GetTick();
		while (HAL_GPIO_ReadPin(hx711->dat_gpio, hx711->dat_pin) == GPIO_PIN_SET) {
			if (HAL_GetTick() - startTime > 150)
				return 0;
		}
		tare = 0;
	}
	if (HAL_GPIO_ReadPin(hx711->dat_gpio, hx711->dat_pin) == GPIO_PIN_RESET) {
		for (int8_t i = 0; i < 24; i++) {
			HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_SET);
			hx711_delay_us();
			HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET);
			hx711_delay_us();
			data = data << 1;
			if (HAL_GPIO_ReadPin(hx711->dat_gpio, hx711->dat_pin)
					== GPIO_PIN_SET)
				data++;
		}
		for (int8_t i = 0; i < hx711_channel; i++) {
			HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_SET);
			hx711_delay_us();
			HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET);
			hx711_delay_us();
		}
		data = data ^ 0x800000;
		return data;
	} else {
		return 100000;
	}
}
//#############################################################################################
int32_t hx711_value_ave(hx711_t *hx711, uint16_t sample) {
	hx711_lock(hx711);
	int64_t ave = 0;
	for (uint16_t i = 0; i < sample; i++) {
		ave += hx711_value(hx711);
//    hx711_delay(5);
	}
	int32_t answer = (int32_t) (ave / sample);
	hx711_unlock(hx711);
	return answer;
}
//#############################################################################################
void hx711_tare(hx711_t *hx711, uint16_t sample, int unit) {
	if (unit == 1) {
		float w1 = 0;
		float w2 = 0;
		for (int i = 1; i < 5; i++) {
			w1 += tareweight1[i];
			w2 += tareweight2[i];
		}
		hx711->offsetA = (w1 * (hx711->coefA)) / 4 + (hx711->offsetA);
		hx711->offsetB = (w2 * (hx711->coefB)) / 4 + (hx711->offsetB);
	}
	if (unit == 2) {
		float w3 = 0;
		float w4 = 0;
		for (int i = 1; i < 5; i++) {
			w3 += tareweight3[i];
			w4 += tareweight4[i];
		}
		hx711->offsetA = (w3 * (hx711->coefA)) / 4 + (hx711->offsetA);
		hx711->offsetB = (w4 * (hx711->coefB)) / 4 + (hx711->offsetB);
	}

}
//#############################################################################################
void hx711_calibration(hx711_t *hx711, int32_t channel, float scale) {

	if (channel == 1) {
		hx711->coefA = scale;
	}
	if (channel == 2) {
		hx711->coefB = scale;
	}

}
//#############################################################################################
float hx711_weight(hx711_t *hx711, uint16_t sample, uint8_t num) {
	if (hx_711readtimestamp[1] == 1 && num == 1) {
		if (loadcellc1 > 0 && loadcellc1 < 10) {
			hx711_channel = 1;
			int value = hx711_value(hx711);
			if (value != 100000) {
				if (start == 1)
					start = 0;
				else {
					loadcellc1 -= 1;
					timecheck[1] += 1;
					ave1 += value;
					if (loadcellc1 == 0) {
						hx_711readtimestamp[1] = 2;
						hx_711readtimestamp[0] = micros();
						start = 1;

					}
				}
			}
		}
	}
	if (hx_711readtimestamp[1] == 2 && num == 1) {
		if (loadcellc2 > 0 && loadcellc2 < 10) {
			hx711_channel = 2;
			int value2 = hx711_value(hx711);
			if (value2 != 100000) {
				if (start == 1) {
					start = 0;
					timecheck[0] = micros();
				} else {
					timecheck[1] = micros() - timecheck[0];
					loadcellc2 -= 1;
					ave2 += value2;
					if (loadcellc2 == 0) {
						hx_711readtimestamp[1] = 1;
						hx_711readtimestamp[0] = micros();
						start = 1;
					}
				}
			}
		}
	}
	if (hx_711readtimestamp2[1] == 1 && num == 2) {
		if (loadcellc3 > 0 && loadcellc3 < 10) {
			hx711_channel = 1;
			int value3 = hx711_value(hx711);
			if (value3 != 100000) {
				if (start2 == 1)
					start2 = 0;
				else {
					loadcellc3 -= 1;
					ave3 += value3;
					if (loadcellc3 == 0) {
						hx_711readtimestamp2[1] = 2;
						hx_711readtimestamp2[0] = micros();
						start2 = 1;
					}
				}
			}
		}
	}
	if (hx_711readtimestamp2[1] == 2 && num == 2) {
		if (loadcellc4 > 0 && loadcellc4 < 10) {
			hx711_channel = 2;
			int value4 = hx711_value(hx711);
			if (value4 != 100000) {
				if (start2 == 1)
					start2 = 0;
				else {
					loadcellc4 -= 1;
					ave4 += value4;
					if (loadcellc4 == 0) {
						hx_711readtimestamp2[1] = 1;
						hx_711readtimestamp2[0] = micros();
						start2 = 1;
					}
				}
			}
		}
	}

	if ((loadcellc1 <= 0 || loadcellc1 >= 5) && loadcellc1 != 10 && num == 1) {
		int32_t data = (int32_t) (ave1 / sample);
		hx711->weightA = (data - hx711->offsetA) / hx711->coefA;
		loadcellc1 = 10;
		tareweight1[(int) tareweight1[0] + 1] = hx711->weightA;
		tareweight1[0] = fabs(((int) tareweight1[0] + 1) % 4);
	}
	if ((loadcellc2 <= 0 || loadcellc2 >= 5) && loadcellc2 != 10 && num == 1) {
		int32_t data = (int32_t) (ave2 / sample);
		hx711->weightB = (data - hx711->offsetB) / hx711->coefB;
		loadcellc2 = 10;
		tareweight2[(int) tareweight2[0] + 1] = hx711->weightB;
		tareweight2[0] = fabs(((int) tareweight2[0] + 1) % 4);
	}
	if ((loadcellc3 <= 0 || loadcellc3 >= 5) && loadcellc3 != 10 && num == 2) {
		int32_t data = (int32_t) (ave3 / sample);
		hx711->weightA = (data - hx711->offsetA) / hx711->coefA;
		loadcellc3 = 10;
		tareweight3[(int) tareweight3[0] + 1] = hx711->weightA;
		tareweight3[0] = fabs(((int) tareweight3[0] + 1) % 4);
	}
	if ((loadcellc4 <= 0 || loadcellc4 >= 5) && loadcellc4 != 10 && num == 2) {
		int32_t data = (int32_t) (ave4 / sample);
		hx711->weightB = (data - hx711->offsetB) / hx711->coefB;
		loadcellc4 = 10;
		tareweight4[(int) tareweight4[0] + 1] = hx711->weightB;
		tareweight4[0] = fabs(((int) tareweight4[0] + 1) % 4);
	}
}
//#############################################################################################
void hx711_coef_set(hx711_t *hx711, float coefA, float coefB) {
	hx711->coefA = coefA;
	hx711->coefB = coefB;
}
//#############################################################################################
float hx711_coef_get(hx711_t *hx711) {
	if (hx711_channel == 1) {
		return hx711->coefA;
	} else if (hx711_channel == 2) {
		return hx711->coefB;
	}
	return 0;
}
//#############################################################################################
void hx711_power_down(hx711_t *hx711) {
	HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_SET);
	hx711_delay(1);
}
//#############################################################################################
void hx711_power_up(hx711_t *hx711) {
	HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET);
}
//#############################################################################################
void tcaselect(uint8_t i) {
	if (i > 7)
		return;
	uint8_t pData[1] = { 1 << i };
	HAL_I2C_Master_Transmit(&hi2c2, TCAADDR, pData, 1, 10);
}

void setResistance(int percent) {
//  digitalWrite(UD, LOW);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
//  digitalWrite(CS, LOW);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	for (int i = 0; i < 100; i++) {
//    digitalWrite(INC, LOW);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		hx711_delay_us();
//    digitalWrite(INC, HIGH);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		hx711_delay_us();
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	for (int i = 0; i < percent; i++) {
		//    digitalWrite(INC, LOW);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		hx711_delay_us();
		//    digitalWrite(INC, HIGH);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		hx711_delay_us();
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

float EncoderVelocity_Update(int unit) {
	//Save Last state
	static uint32_t EncoderLastPosition1 = 0;
	static uint64_t EncoderLastTimestamp1 = 0;
	static uint32_t EncoderLastPosition2 = 0;
	static uint64_t EncoderLastTimestamp2 = 0;
	static uint32_t EncoderLastPosition3 = 0;
	static uint64_t EncoderLastTimestamp3 = 0;
	//read data
	if (unit == 1) {
		uint32_t EncoderNowPosition1 = distancepulse1;
		uint64_t EncoderNowTimestamp1 = micros() / 10;
		EncoderTimeDiff1 = EncoderNowTimestamp1 - EncoderLastTimestamp1;
		EncoderPositionDiff1 = EncoderNowPosition1 - EncoderLastPosition1;
		//Update Position and time
		EncoderLastPosition1 = EncoderNowPosition1;
		EncoderLastTimestamp1 = EncoderNowTimestamp1;
		return (EncoderPositionDiff1 * 100000) / (float) EncoderTimeDiff1;
	}
	if (unit == 2) {
		uint32_t EncoderNowPosition2 = distancepulse2;
		uint64_t EncoderNowTimestamp2 = micros() / 10;
		EncoderTimeDiff2 = EncoderNowTimestamp2 - EncoderLastTimestamp2;
		EncoderPositionDiff2 = EncoderNowPosition2 - EncoderLastPosition2;
		//Update Position and time
		EncoderLastPosition2 = EncoderNowPosition2;
		EncoderLastTimestamp2 = EncoderNowTimestamp2;
		return (EncoderPositionDiff2 * 100000) / (float) EncoderTimeDiff2;
	}
	if (unit == 3) {
		uint32_t EncoderNowPosition3 = distancepulse3;
		uint64_t EncoderNowTimestamp3 = micros() / 10;
		EncoderTimeDiff3 = EncoderNowTimestamp3 - EncoderLastTimestamp3;
		EncoderPositionDiff3 = EncoderNowPosition3 - EncoderLastPosition3;
		//Update Position and time
		EncoderLastPosition3 = EncoderNowPosition3;
		EncoderLastTimestamp3 = EncoderNowTimestamp3;
		return (EncoderPositionDiff3 * 100000) / (float) EncoderTimeDiff3;
	}
}
void limitswitchlowpass() {
	if (limitswitch1 == 1 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == 0) {
		limitswitchc1 += 1;
	} else if (limitswitch1 == 0 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == 1) {
		limitswitchc1 -= 1;
	} else {
		limitswitchc1 = 0;
	}
	if (limitswitchc1 >= 10)
		limitswitch1 = 0;
	if (limitswitchc1 <= -10)
		limitswitch1 = 1;

	if (limitswitch2 == 1 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 0) {
		limitswitchc2 += 1;
	} else if (limitswitch2 == 0 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1) {
		limitswitchc2 -= 1;
	} else {
		limitswitchc2 = 0;
	}
	if (limitswitchc2 >= 10)
		limitswitch2 = 0;
	if (limitswitchc2 <= -10)
		limitswitch2 = 1;

	if (limitswitch3 == 1 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 0) {
		limitswitchc3 += 1;
	} else if (limitswitch3 == 0 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 1) {
		limitswitchc3 -= 1;
	} else {
		limitswitchc3 = 0;
	}
	if (limitswitchc3 >= 10)
		limitswitch3 = 0;
	if (limitswitchc3 <= -10)
		limitswitch3 = 1;

}
void distancemeasurement() {
	distancepulse1 = TIM1->CNT - error1 + distancestamp1;
	if (olddistance1 - distancepulse1 >= 32500) {
		distancestamp1 += 65535;
		olddistance1 = distancepulse1 + 65535;
		distancemetre1 = (distancepulse1 + 65535) / 10.0 / 64.0 / 4.0 / 20.0
				* 15.0 * 8.0;
	} else if (distancepulse1 - olddistance1 >= 32500) {
		distancestamp1 -= 65535;
		olddistance1 = distancepulse1 - 65535;
		distancemetre1 = (distancepulse1 - 65535) / 10.0 / 64.0 / 4.0 / 20.0
				* 15.0 * 8.0;
	} else {
		olddistance1 = distancepulse1;
		distancemetre1 = distancepulse1 / 10.0 / 64.0 / 4.0 / 20.0 * 15.0 * 8.0;
	}
	distancepulse2 = TIM2->CNT - error2 + distancestamp2;
	distancemetre2 = distancepulse2 / 12.0 / 64.0 / 4.0 / 20.0 * 15.0 * 8.0;
	distancepulse3 = TIM3->CNT - error3 + distancestamp3;
	distancemetre3 = distancepulse3 / 12.0 / 64.0 / 4.0 / 15.0 * 10.0 * 12.0
			* 5.08;
}
void velocitymeasurement() {
	Timestamp_Encoder = micros();
	velocitypulse1 = ((99 * velocitypulse1 + EncoderVelocity_Update(1)) / 100);
	velocitypulse2 = ((99 * velocitypulse2 + EncoderVelocity_Update(2)) / 100);
	velocitypulse3 = ((9 * velocitypulse3 + EncoderVelocity_Update(3)) / 10);
	velocitymetre1 = velocitypulse1 / 12.0 / 64.0 / 4.0 / 20.0 * 15.0 * 8.0;
	velocitymetre2 = velocitypulse2 / 12.0 / 64.0 / 4.0 / 20.0 * 15.0 * 8.0;
	velocitymetre3 = velocitypulse3 / 12.0 / 64.0 / 4.0 / 15.0 * 10.0 * 12.0
			* 5.08;
}
void gotoposition(int unit) {
	if (unit == 1) {
		if (ptg1 - distancemetre1 <= 0.1 && ptg1 - distancemetre1 >= -0.3) {
			activate1 = 0;
			in1 = 0;
			in2 = 0;
		} else if (ptg1 - distancemetre1 >= 0) {
			in1 = 1;
			in2 = 0;
			require1 = 10;
		} else if (ptg1 - distancemetre1 <= 0) {
			in1 = 0;
			in2 = 1;
			require1 = -10;
		}
	}
	if (unit == 2) {
		if (ptg2 - distancemetre2 <= 0.1 && ptg2 - distancemetre2 >= -0.3) {
			activate2 = 0;
			in3 = 0;
			in4 = 0;
		} else if (ptg2 - distancemetre2 >= 0) {
			in3 = 1;
			in4 = 0;
			require2 = 10;
		} else if (ptg2 - distancemetre2 <= 0) {
			in3 = 0;
			in4 = 1;
			require2 = -10;
		}
	}
	if (unit == 3) {
		if (ptg3 - distancemetre3 <= 1.5 && ptg3 - distancemetre3 >= -1.5) {
			activate3 = 0;
			in5 = 0;
			in6 = 0;
		} else if (ptg3 - distancemetre3 >= 0) {
			in5 = 1;
			in6 = 0;
			require3 = 40;
		} else if (ptg3 - distancemetre3 <= 0) {
			in5 = 0;
			in6 = 1;
			require3 = -40;
		}
	}
}
void setzero() {
	if (zerostate == 1) {
		if (oldzerostate == 0 && zerostate == 1) {
			distance1tozero = distancemetre1;
			distance3tozero = distancemetre3;
		}
		if (distancemetre3 - distance3tozero <= -300 || limitswitch3 == 1) {
			in1 = 0;
			in2 = 1;
			require1 = -10;
		}
		if (distancemetre1 - distance1tozero <= -100 || limitswitch1 == 1) {
			in3 = 0;
			in4 = 1;
			require2 = -10;
		}
		in5 = 0;
		in6 = 1;
		if (distancemetre3 <= 100){
			require3 = -20;
		}
		else require3 = -40;

		if (limitswitch1 == 1) {
			error1 = TIM1->CNT;
			distancestamp1 = 0;
			olddistance1 = 0;
			activate1 = 0;
		}
		if (limitswitch2 == 1) {
			error2 = TIM2->CNT;
			distancestamp2 = 0;
			activate2 = 0;
		}
		if (limitswitch3 == 1) {
			error3 = TIM3->CNT;
			distancestamp3 = 0;
			activate3 = 0;
		}
		if (limitswitch1 == 1 && limitswitch2 == 1 && limitswitch3 == 1) {
			zerostate = 0;
			require1 = 0;
			require2 = 0;
			require3 = 0;
		}
	}
	oldzerostate = zerostate;
}
void pwmdrive() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, in1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, in2);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, in3);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, in4);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm2);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, in5);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, in6);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm3);
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

