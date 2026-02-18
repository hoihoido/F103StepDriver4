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

  2026-01-04 テーブル方式作成開始
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>		// for DEBUG
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHATTER 100

// I2Cコマンド
#define CMDMOVE 0x00
#define CMDRST  0x01

// 制御シンボル
#define UP true
#define DOWN false
#define HIGH 1
#define LOW 0
#define IDOFFSET 0x10

// 制御するドライバ数
#define CHCOUNT 4

// 動作範囲
#define LENGTH 8.2  // アクチュエータのストローク(mm)
#define MAXPT  2600 // 最大パルス数

// 時間定数
#define STBYTIME 1000 // 不動時間がこれ以上あるとスタンバイ(mS)
#define STBYDLY  100  // スタンバイ復帰時の待ち時間(uS)

// 等加速度運動パラメータ
#define INITVEL  5.0 // 初期速度(mm/S)
#define MAXVEL  31.5 // 最大速度(mm/S)
#define ACCEL  20.0 //500.0 // 加速度(mm/S2)
#define MARGIN 5    // 減速余裕(pulse)

#define MAXTBL 1500

// EN復帰待ち時間(x1uS)
#define ENWTIME 1000 // 1mS

// 絶対値マクロ
#define abs(x) (x<0 ? x*-1 : x)

// IIC用
#define IICBUFSIZE 32

#define digitalRead(pp) (HAL_GPIO_ReadPin(pp.port, pp.bit ))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

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
  portpin enbl;
} ch;

ch ports[] = {
  {{ CH0DIR_GPIO_Port, CH0DIR_Pin },{ CH0CLK_GPIO_Port, CH0CLK_Pin },{ CH0EN_GPIO_Port, CH0EN_Pin }},
  {{ CH1DIR_GPIO_Port, CH1DIR_Pin },{ CH1CLK_GPIO_Port, CH1CLK_Pin },{ CH1EN_GPIO_Port, CH1EN_Pin }},
  {{ CH2DIR_GPIO_Port, CH2DIR_Pin },{ CH2CLK_GPIO_Port, CH2CLK_Pin },{ CH2EN_GPIO_Port, CH2EN_Pin }},
  {{ CH3DIR_GPIO_Port, CH3DIR_Pin },{ CH3CLK_GPIO_Port, CH3CLK_Pin },{ CH3EN_GPIO_Port, CH3EN_Pin }}
};

// ID0:PB13 ID1:PB14 ID2:PB15 ID3:PA8 (ID=BINARY+1)
portpin ids[] = {
	{ID0_GPIO_Port, ID0_Pin},{ID1_GPIO_Port, ID1_Pin},
	{ID2_GPIO_Port, ID2_Pin},{ID3_GPIO_Port, ID3_Pin}
};

portpin marker={MARKER_GPIO_Port, MARKER_Pin};

//int8_t chstat[CHCOUNT] = {0, 0, 0, 0}; // 0:未動作 1:CLK=Hタイマ動作中 2:CLK=Lタイマ動作中
enum timstat{
	TNON, // タイマー未動作
	TCLKH,// CLKHタイマー動作中
	TCLKL,// CLKLタイマー動作中
	TENW // ENABLE WAIT タイマー動作中
} chstat[CHCOUNT] = {TNON, TNON, TNON, TNON};

float lperstep;
long  initvel;
long  maxvel;
long  accel; // 加速度 steps/S2
uint32_t	period[CHCOUNT]={0,0,0,0};
int32_t		dest[CHCOUNT] = {2600,2600,2600,2600};
//int32_t		currentpt[CHCOUNT] = {MAXPT,MAXPT,MAXPT,MAXPT}; //初期化の為暫定的に上限にする。
int32_t		currentpt[CHCOUNT] = {2500,2500,2500,2500};
int32_t		velocity[CHCOUNT]={};
uint8_t		stepstartf[CHCOUNT]={false,false,false,false};
uint32_t	stbytime[CHCOUNT]={0,0,0,0};
bool		stbyf[CHCOUNT];

uint16_t acctable[MAXTBL]; // 加減速周期テーブル
int16_t velocipt[CHCOUNT]={0,0,0,0};

bool pulseendf=false;
bool i2crcvf = false;
uint8_t buf[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void I2C_RESET( void ){
  HAL_I2C_DeInit(&hi2c1) ;
  HAL_I2C_Init(&hi2c1) ;
}

void digitalWrite(portpin pp , int level) {
        HAL_GPIO_WritePin(pp.port, pp.bit, level);
}
/*
GPIO_PinState digitalRead(portpin pp) {
	return(HAL_GPIO_ReadPin(pp.port, pp.bit ));
}
*/
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

// 加速度テーブル作成
// 2026-01-29 acctable[]は[1]始まりで[0]は±切り替え時のみ使用する様に変更。
void maketable() {
	int i;
	float velocity;
	float period;
	bool endf=false;

	velocity=INITVEL;    // 初期速度[steps/S]
	for ( i=1; i<MAXTBL; i++) {
		period=lperstep/velocity; // 周期[S]
		acctable[i]=(uint16_t)(period*1e6)/2; // 半周期[uS]
		if ( endf ) {
			acctable[i+1]=0;
			break;
		}
		if ( i==MAXTBL-1) {
			acctable[MAXTBL-1]=0;
			break;
		}
		velocity = velocity + ACCEL * period; // 次回の速度
		if (MAXVEL <= velocity) {
			velocity = MAXVEL; // 上限に揃える
			endf = true;
		}
	}
	acctable[0]=acctable[1]*2;    // [0]は±切替時のみ実行。
}

void onestep(int8_t i, ch* channel, bool ud, uint32_t steptime) {
	// EN復帰期間の保存用
	static ch*  channelback[CHCOUNT];
	static bool udback[CHCOUNT];
	static uint32_t steptimeback[CHCOUNT];

	// スタンバイモードからの復帰
	if ( chstat[i] != TENW ) { // 復帰タイマー明けではない
		if ( stbyf[i] ) { // 今までスタンバイ中だった。
			digitalWrite(ports[i].enbl, HIGH);
			stbyf[i]=false;

			// 状態を保存(復帰タイマー明けに続きをやるため）。
			channelback[i]=channel;
			udback[i]=ud;
			steptimeback[i]=steptime;

			// 復帰タイマー起動
			chstat[i]=TENW;
			__HAL_TIM_SET_COUNTER(timarray[i], ENWTIME);
			HAL_TIM_Base_Start_IT(timarray[i]);

			stbytime[i] = ticktime(&hrtc);
			return; // 一旦抜ける（続きはタイマー明けで・・）

		}
	} else { // 復帰タイマー明け
		channel=channelback[i];
		ud=udback[i];
		steptime=steptimeback[i];
	}
	stbytime[i] = ticktime(&hrtc);

	// DIRとCLKを設定する。
	digitalWrite(channel->dir, ud ? HIGH : LOW);
	digitalWrite(channel->clk, HIGH);

	// タイマースタート。
	chstat[i]=TCLKH;
	__HAL_TIM_SET_COUNTER(timarray[i], steptime);
	HAL_TIM_Base_Start_IT(timarray[i]);
}

// 等加速度運動による現在速度と周期を計算しモーターを１ステップ進める。。
void kinematics(int8_t i) {
  int tablept;
  int16_t way;
#ifdef DEBUG
  printf("kinematics %d %ld %ld %d\n",i,dest[i],currentpt[i],velocipt[i]);
#endif

  way = dest[i] - currentpt[i]; // 目的地までの道のり（±）
  tablept=abs(velocipt[i]);

  if ( 0 < way)  {
	  // 上り
	  // onestep(i, &(ports[i]), 0<=velocipt[i]? UP:DOWN, acctable[tablept]);
	  if ( ((way-MARGIN) < velocipt[i]) && (1 < velocipt[i]) )
		  velocipt[i]--; // 上方向に減速
	  else if ( velocipt[i] < (way-MARGIN) && (0 != acctable[tablept+1]) )
		  velocipt[i]++; // 上方向に加速(下方向に減速も含む)

	  tablept=abs(velocipt[i]);
	  if (0<velocipt[i]) {
		  onestep(i, &(ports[i]), UP, acctable[tablept]);
		  currentpt[i]++;
	  } else {
		  onestep(i, &(ports[i]), DOWN, acctable[tablept]);
		  currentpt[i]--;
	  }
  } else if ( way < 0 ) {
	  // 下り
	  // onestep(i, &(ports[i]), 0<=velocipt[i]? UP:DOWN, acctable[tablept]);
	  if ( (velocipt[i] < (way+MARGIN))  && (velocipt[i]) < -1 )
		  velocipt[i]++; // 下方向に減速
	  else if ( (way+MARGIN) < velocipt[i] && (0 != acctable[tablept+1]) )
		  velocipt[i]--; // 下方向に加速(上方向に減速も含む)

	  tablept=abs(velocipt[i]);
	  if (0<velocipt[i]) {
		  onestep(i, &(ports[i]), UP, acctable[tablept]);
	      currentpt[i]++;
	  } else {
	      onestep(i, &(ports[i]), DOWN, acctable[tablept]);
	      currentpt[i]--;
	  }
  } else {
	  velocipt[i]=0;
#ifdef DEBUG
	  printf("STOP %d\n",i);
#endif
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
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //tickstart = ticktime(&hrtc);

  // その他変数の準備
  lperstep = LENGTH / MAXPT; // length/step(mm)
  initvel = INITVEL / lperstep; // step/S
  maxvel = MAXVEL / lperstep; // step/S
  accel = ACCEL / lperstep; // step/S2

  maketable();

  // 初期化
  for(int8_t i=0; i<CHCOUNT; i++) {
	  digitalWrite(ports[i].enbl, HIGH);
	  velocity[i] = initvel;
      dest[i]=0;          // そして目標値を0。
      kinematics(i);      // 動作開始。
  }

  if ( HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t *)buf, IICBUFSIZE) != HAL_OK) {
	  Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_TIM_Base_Start_IT(&htim1);

  // ============================== MAIN LOOP ===============================
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // I2C受信データあり
		if ( i2crcvf ) {
			if ( buf[0]==CMDMOVE ) { // モーター移動
				for (int i=0 ; i < 4 ; i++ ) {
					int val = buf[i*2+1]*256+buf[i*2+2];
					if ( dest[i] != val) {
						dest[i]=val;
						#ifdef DEBUG
							printf( "%d:",val );
						#endif
						kinematics(i);
					}
				}
				#ifdef DEBUG
					putchar('\n');
				#endif
			} else if ( buf[0] == CMDRST ) { // 原点復帰
				for (int i=0 ; i < 4 ; i++ ) {
					int val = buf[i*2+1]*256+buf[i*2+2];
					if ( val ) {
						currentpt[i]=2500;
						dest[i]=0;
						kinematics(i);
					}
				}
			}
			i2crcvf = false;
			if ( HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t *)buf, 128) != HAL_OK) {
			  Error_Handler();
			}
			pulseendf=true;
		}

	  // 一定時間動作が無ければイネーブル端子をLにする。
	  for ( int i=0 ; i<CHCOUNT; i++ ) {
//		  if ( (stbyf[i] == false ) &&  (STBYTIME < (ticktime(&hrtc) - stbytime[i])) ) {

		  uint32_t stbytimebak = stbytime[i]; // stbytime[i]が割込みで変る可能性があるのでバックアップ
		  if ( (stbyf[i] == false ) &&  (STBYTIME < (ticktime(&hrtc) - stbytimebak)) ) {
			  digitalWrite(ports[i].enbl, LOW);
			  stbyf[i] = true;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
	//MX注意：MXで再生成したら下の「hi2c1.Init.OwnAddress1 = 2;」行をコメントアウトする事。
	int id = (digitalRead(ids[3])<<3) + (digitalRead(ids[2])<<2)
			+ (digitalRead(ids[1])<<1) + digitalRead(ids[0]);
	hi2c1.Init.OwnAddress1 = (id+IDOFFSET)<<1;
#ifdef DEBUG
	printf("I2C ID=%d\n",id);
#endif
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  //hi2c1.Init.OwnAddress1 = 2;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 64;
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
  htim2.Init.Prescaler = 64;
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
  htim3.Init.Prescaler = 64;
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
  htim4.Init.Prescaler = 64;
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
                          |CH1CLK_Pin|CH1EN_Pin|CH2DIR_Pin|MARKER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CH2CLK_Pin|CH2EN_Pin|CH3DIR_Pin|CH3CLK_Pin
                          |CH3EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CH0DIR_Pin CH0CLK_Pin CH0EN_Pin CH1DIR_Pin
                           CH1CLK_Pin CH1EN_Pin CH2DIR_Pin MARKER_Pin */
  GPIO_InitStruct.Pin = CH0DIR_Pin|CH0CLK_Pin|CH0EN_Pin|CH1DIR_Pin
                          |CH1CLK_Pin|CH1EN_Pin|CH2DIR_Pin|MARKER_Pin;
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

  /*Configure GPIO pins : ID0_Pin ID1_Pin ID2_Pin */
  GPIO_InitStruct.Pin = ID0_Pin|ID1_Pin|ID2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ID3_Pin */
  GPIO_InitStruct.Pin = ID3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ID3_GPIO_Port, &GPIO_InitStruct);

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
// digitalWrite()
// kinematics()
// └onestep()
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// どのCH(どのタイマー)の割込みかループで探す。
	for ( int i=0; i<CHCOUNT; i++) {
		if ( htim->Instance == timarray[i]->Instance) {
			HAL_TIM_Base_Stop_IT(timarray[i]);
			if ( chstat[i]==TCLKH ) { // H期間終了ならここ。CLKをLに下げて同じ時間でタイマー起動。
				chstat[i]=TCLKL;
				digitalWrite(ports[i].clk, LOW);
				__HAL_TIM_SET_COUNTER(timarray[i], acctable[abs(velocipt[i])]);
				HAL_TIM_Base_Start_IT(timarray[i]);
			} else if (chstat[i]==TCLKL) {			// L期間終了ならここ
				chstat[i]=TNON;
				kinematics(i);
			} else if (chstat[i]==TENW){	// ENABLE端子復帰待ち時間終了ならここ
				onestep(i,NULL,false,0);	// 先に中断したonestep()の続きを実行。
			}
			break;
		}
	}
}

// for DEBUG (printf...)
int _write(int file, char *ptr, int len)
{
  int DataIdx;
  for(DataIdx=0; DataIdx<len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef * hi2c) {
	i2crcvf=true;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c ) {
  if(hi2c->Instance == I2C1) {

	  if ( HAL_I2C_ERROR_AF == HAL_I2C_GetError(hi2c)) {
		i2crcvf=true;
	    return;
	  }
	  Error_Handler();
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
