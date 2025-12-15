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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHATTER 100

// 制御シンボル
#define UP true
#define DOWN false
#define HIGH 1
#define LOW 0

// 制御するドライバ数
#define CHCOUNT 4

// 動作範囲
#define LENGTH 8.2  // アクチュエータのストローク(mm)
#define MAXPT  2600 // 最大パルス数
#define INITCT 2700 // 初期化時に下げる数(強制脱調)

// 時間定数
#define STBYTIME 1000 // 不動時間がこれ以上あるとスタンバイ(mS)
#define STBYDLY  100  // スタンバイ復帰時の待ち時間(uS)

// 等加速度運動パラメータ
#define INITVEL  7.5 // 初期速度(mm/S)
#define MAXVEL  50.0 // 最大速度(mm/S)
#define ACCEL  750.0 // 加速度(mm/S2)
#define MARGIN 10    // 減速余裕(pulse)

// 簡易数学関数
#define abs(x) (x<0 ? x*-1 : x)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint32_t tch=0;
TIM_HandleTypeDef* timarray[] = {
	&htim1,
	&htim2,
	&htim3,
	&htim4
};

typedef struct PORTPIN {
	GPIO_TypeDef *port;
    uint16_t bit;
} portpin;

typedef struct CH {
  portpin dir;
  portpin clk;
  portpin stby;
} ch;

ch ports[] = {
  {{ CH0DIR_GPIO_Port, CH0DIR_Pin },{ CH0CLK_GPIO_Port, CH0CLK_Pin },{ CH0EN_GPIO_Port, CH0EN_Pin }},
  {{ CH1DIR_GPIO_Port, CH1DIR_Pin },{ CH1CLK_GPIO_Port, CH1CLK_Pin },{ CH1EN_GPIO_Port, CH1EN_Pin }},
  {{ CH2DIR_GPIO_Port, CH2DIR_Pin },{ CH2CLK_GPIO_Port, CH2CLK_Pin },{ CH2EN_GPIO_Port, CH2EN_Pin }},
  {{ CH3DIR_GPIO_Port, CH3DIR_Pin },{ CH3CLK_GPIO_Port, CH3CLK_Pin },{ CH3EN_GPIO_Port, CH3EN_Pin }}
};

int8_t chstat[CHCOUNT] = {0, 0, 0, 0};

float lperstep;
long  initvel;
long  maxvel;
long  accel; // 加速度 steps/S2
uint32_t	period[CHCOUNT]={0,0,0,0};
int32_t		dest[CHCOUNT] = {2600,2600,2600,2600};
int32_t		currentpt[CHCOUNT] = {MAXPT,MAXPT,MAXPT,MAXPT}; //初期化の為暫定的に上限にする。
int32_t		velocity[CHCOUNT]={};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void digitalWrite(portpin pp , int level) {
        HAL_GPIO_WritePin(pp.port, pp.bit, level);
}

// RTC_ReadTimeCounter()からパクった。(RTC_ReadTimeCounter()は外に公開するヘッダに載ってないので）。
static uint32_t ticktime()
{
  uint16_t high1 = 0U, high2 = 0U, low = 0U;
  uint32_t timecounter = 0U;

  high1 = READ_REG(hrtc.Instance->CNTH & RTC_CNTH_RTC_CNT);
  low   = READ_REG(hrtc.Instance->CNTL & RTC_CNTL_RTC_CNT);
  high2 = READ_REG(hrtc.Instance->CNTH & RTC_CNTH_RTC_CNT);
  if (high1 != high2)
    timecounter = (((uint32_t) high2 << 16U) | READ_REG(hrtc.Instance->CNTL & RTC_CNTL_RTC_CNT));
  else
    timecounter = (((uint32_t) high1 << 16U) | low);
  return timecounter;
}

void reservation( uint32_t t, int8_t i, portpin clkpin, bool hl ) {
	__HAL_TIM_SET_COUNTER(timarray[tch], t);
	HAL_TIM_Base_Start_IT(timarray[tch]);
	chstat[i]=1;
}

void onestep(int8_t i, ch* channel, bool ud, uint32_t steptime) {
  // DIRとCLKを設定する。
  digitalWrite(channel->dir, ud ? HIGH : LOW);
  digitalWrite(channel->clk, HIGH);

  // CLK立下りとL期間終了イベントを予約する。
  reservation(steptime, i, channel->clk, LOW );
}

// 等加速度運動による現在速度と周期を計算しモーターを１ステップ進める。。
void kinematics(int8_t i) {
  int ap;
  int ud;

  // velocity[]から次回迄のperiod[]を算出。
  period[i] = abs(1e6 / velocity[i]); // 周期[uS]

  // currentpt[]とdest[]を比べてUP/DOWN/=を判断する。それに応じud[]を設定する。
  //      ud[] -1:DOWN 0:STOP +1:UP
  // ud[]に応じてDIRを設定。
  if (currentpt[i] < dest[i]) {
    ud = 1;
  } else if (dest[i] < currentpt[i]) {
    ud = -1;
  } else {
    ud = 0;
  }

  // === 次回のvelocity[]を算出。===
  // 減速点apを計算
  ap = dest[i] - (velocity[i]
		  + (0 <= velocity[i] ? initvel : -1 * initvel)) / 2 * (abs(velocity[i]) - initvel) / accel
    - ud*MARGIN;
  // 場合分けして次回の速度velociy[]を計算
  if (0 < ud) { // 上昇
    if (currentpt[i] < ap) {
      velocity[i] = velocity[i] + accel * period[i] / 1e6; // 上向きに対し加速
      if (maxvel < velocity[i]) velocity[i] = maxvel; // 上限に揃える
      if (-1 * initvel < velocity[i] && velocity[i] < initvel) velocity[i] = initvel; // ±initvalの間はすっ飛ばす。
    } else {
      velocity[i] = velocity[i] - accel * period[i] / 1e6; // 上向きに対し減速
      if (velocity[i] < initvel) velocity[i] = initvel; // 下限に揃える
    }
  } else if (ud < 0) { // 下降
    if (ap < currentpt[i]) {
      velocity[i] = velocity[i] - accel * period[i] / 1e6; // 上向きに対し減速(下に加速)
      if (velocity[i] < -1 * maxvel) velocity[i] = -1 * maxvel;
      if (-1 * initvel < velocity[i] && velocity[i] < initvel) velocity[i] = -1 * initvel; // ±initvalの間はすっ飛ばす。
    } else {
      velocity[i] = velocity[i] + accel * period[i] / 1e6; // 上向きに対し加速(下に減速)
      if (-1 * initvel < velocity[i]) velocity[i] = -1 * initvel;
    }
  }

  // 移動/現在地更新
  if (currentpt[i] != dest[i]) {
    // 指示に従って移動
    if (0 <= velocity[i]) {
      currentpt[i]++;
      onestep(i, &(ports[i]), UP, period[i] / 2);
    } else {
      currentpt[i]--;
      onestep(i, &(ports[i]), DOWN, period[i] / 2);
    }
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
	bool step1f=false;

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
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // その他変数の準備
  lperstep = LENGTH / MAXPT; // length/step(mm)
  initvel = INITVEL / lperstep; // step/S
  maxvel = MAXVEL / lperstep; // step/S
  accel = ACCEL / lperstep; // step/S2

  // 初期化
  for(int8_t i=0; i<CHCOUNT; i++) {
      dest[i]=0;          // そして目標値を0。
      kinematics(i);      // 動作開始。
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_TIM_Base_Start_IT(&htim1);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  for (int i=0; i<CHCOUNT; i++ ) {
		  if (chstat[i] == 2) {
			  chstat[i]=0;
			  kinematics(i);
		  }
	  }

	  if ( 1000 < ticktime(&hrtc) && ! step1f ) {
		  step1f=true;
		  for(int8_t i=0; i<CHCOUNT; i++) {
			  dest[i]=2500;
			  kinematics(i);
		  }
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = 40;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim1.Init.Period = 200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
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
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 250;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
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
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim3.Init.Period = 333;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
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
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 64000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim4.Init.Period = 500;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CH0DIR_Pin|CH0CLK_Pin|CH0EN_Pin|CH1DIR_Pin
                          |CH1CLK_Pin|CH1EN_Pin|CH2DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CH2CLK_Pin|CH2EN_Pin|CH3DIR_Pin|CH3CLK_Pin
                          |CH3EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CH0DIR_Pin CH0CLK_Pin CH0EN_Pin CH1DIR_Pin
                           CH1CLK_Pin CH1EN_Pin CH2DIR_Pin */
  GPIO_InitStruct.Pin = CH0DIR_Pin|CH0CLK_Pin|CH0EN_Pin|CH1DIR_Pin
                          |CH1CLK_Pin|CH1EN_Pin|CH2DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CH2CLK_Pin CH2EN_Pin CH3DIR_Pin CH3CLK_Pin
                           CH3EN_Pin */
  GPIO_InitStruct.Pin = CH2CLK_Pin|CH2EN_Pin|CH3DIR_Pin|CH3CLK_Pin
                          |CH3EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// ### タイマー割込み処理 ###
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if ( htim->Instance == htim1.Instance ) {
		HAL_TIM_Base_Stop_IT(&htim1);
		if ( chstat[0]==1 ) {
			chstat[0]=0;
			digitalWrite(ports->clk, HIGH);
			HAL_TIM_Base_Start_IT(timarray[0]);
		} else {
			chstat[0]=2;
		}
	} else	if ( htim->Instance == htim2.Instance ) {
		HAL_TIM_Base_Stop_IT(&htim2);
		if ( chstat[1]==1 ) {
			chstat[1]=0;
			digitalWrite(ports->clk, HIGH);
			HAL_TIM_Base_Start_IT(timarray[1]);
		} else {
			chstat[1]=2;
		}
	} else	if ( htim->Instance == htim3.Instance ) {
		HAL_TIM_Base_Stop_IT(&htim3);
		if ( chstat[2]==1 ) {
			chstat[2]=0;
			digitalWrite(ports->clk, HIGH);
			HAL_TIM_Base_Start_IT(timarray[2]);
		} else {
			chstat[2]=2;
		}
	} else	if ( htim->Instance == htim4.Instance ) {
		HAL_TIM_Base_Stop_IT(&htim4);
		if ( chstat[3]==1 ) {
			chstat[3]=0;
			digitalWrite(ports->clk, HIGH);
			HAL_TIM_Base_Start_IT(timarray[3]);
		} else {
			chstat[3]=2;
		}
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
#ifdef USE_FULL_ASSERT
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
