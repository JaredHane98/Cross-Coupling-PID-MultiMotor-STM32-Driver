/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "Motor.h"
#include "I2C_Controller.h"
#include <string.h>
#include "Motor_Controller.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
Motor_Handle motor_handle = { 0 };
I2C_Handle i2c_handle = { 0 };
GPIO_Pin left_direction_pin = { 0 };
GPIO_Pin right_direction_pin = { 0 };
GPIO_Pin motor_brake_pin = { 0 };
GPIO_Pin motor_enable_pin = { 0 };
I2C_Motor_Command command_handle = { .data = { 0 }, .current_command = NO_COMMAND, .next_command = NO_COMMAND };

PID_Parameters f_left_motor_pos_pid_params  = { .kd = 0.0, .ki = 0.6, .kp = 1.75, .min = 0, .max = 60, .dac_default =  682};
PID_Parameters f_right_motor_pos_pid_params = { .kd = 0.0, .ki = 0.6, .kp = 1.75, .min = 0, .max = 60, .dac_default =  674};
PID_Parameters b_left_motor_pos_pid_params  = { .kd = 0.0, .ki = 0.6, .kp = 1.75, .min = 0, .max = 60, .dac_default =  679};
PID_Parameters b_right_motor_pos_pid_params = { .kd = 0.0, .ki = 0.6, .kp = 1.75, .min = 0, .max = 60, .dac_default =  682};

PID_Parameters pid_parameters_velocity = { .kd = 0.001, .ki = 2, .kp = 0.2, .min = 0, .max = 60, .dac_default =  682};

volatile bool motor_ready = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	I2C_ListenCpltCallback(&i2c_handle, hi2c);
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	I2C_ErrorCallback(&i2c_handle, hi2c);
}
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
		uint16_t AddrMatchCode)
{
	I2C_AddrCallback(&i2c_handle, hi2c, TransferDirection, AddrMatchCode);
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	I2C_SlaveRxCpltCallback(&motor_handle, &i2c_handle, &command_handle, hi2c);
}
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	I2C_SlaveTxCpltCallback(&i2c_handle, hi2c);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(motor_ready)
	{
        Update_Motor_Period_Elapsed(&motor_handle.motors[0], htim);
        Update_Motor_Period_Elapsed(&motor_handle.motors[1], htim);
        Update_Motor_Period_Elapsed(&motor_handle.motors[2], htim);
        Update_Motor_Period_Elapsed(&motor_handle.motors[3], htim);
	}
}

void Get_Command_Data(uint8_t *data, const uint8_t buffer_size)
{
	if(sizeof(int8_t) * 2 + sizeof(int32_t) * 2 > buffer_size)
		Error_Handler();
	const int32_t current_command = command_handle.current_command;
	const int32_t next_command = command_handle.next_command;

	data[0] = MOTOR_CURRENT_STATE;
	memcpy(&data[1], &current_command, sizeof(int32_t));
	data[5] = MOTOR_NEXT_STATE;
	memcpy(&data[6], &next_command, sizeof(int32_t));
}

void Get_Motor_State(I2C_Read_Message *read_message)
{
	uint8_t offset = 0;
	for(uint8_t i = 0; i < NUM_MOTORS ; i++)
		offset += Get_Motor_Data(&motor_handle.motors[i],
				read_message->message + offset, I2C_MESSAGE_DATA_SIZE - offset,
				i);

	Get_Command_Data(read_message->message + offset,
			I2C_MESSAGE_DATA_SIZE - offset);
	read_message->crc = I2C_Compute_CRC(read_message->message,
			I2C_MESSAGE_DATA_SIZE);
}

void Update_I2C_Read_State()
{
	I2C_Read_Message next_message =
	{ 0 };
	Get_Motor_State(&next_message);

	while(i2c_handle.next_read_message.in_use == true)
	{
	}

	i2c_handle.next_read_message.in_use = true;
	memcpy(i2c_handle.next_read_message.message, next_message.message,
	I2C_MESSAGE_SIZE);
	i2c_handle.next_read_message.updated = true;
	i2c_handle.next_read_message.in_use = false;
}

void Update_Motors(Motor_Handle *handle, I2C_Motor_Command *motor_command)
{
	switch(motor_command->current_command)
	{
	case FORWARD_POSITION_COMMAND:
		Run_Forward_Position_Command(handle, motor_command);
		break;
	case BACKWARDS_POSITION_COMMAND:
		Run_Backward_Position_Command(handle, motor_command);
		break;
	case LEFT_TURN_COMMAND:
		// haven't implemented
		break;
	case RIGHT_TURN_COMMAND:
		// haven't implemented
		break;
	case STOP_COMMAND:
		Run_Stop_Command(handle, motor_command);
		break;
	default:
		break;

	};
}

void Update_Next_Motor_Command(I2C_Motor_Command *motor_command)
{
	const float target = *(float*)motor_command->data;

	switch(motor_command->next_command)
	{
	case FORWARD_POSITION_COMMAND:
	case BACKWARDS_POSITION_COMMAND:
		Set_Motor_Command(&motor_handle.front_left_motor, &motor_handle.front_right_motor, false, target);
		Set_Motor_Command(&motor_handle.front_right_motor, &motor_handle.front_left_motor, true, target);
		Set_Motor_Command(&motor_handle.back_left_motor, &motor_handle.back_right_motor, false, target);
		Set_Motor_Command(&motor_handle.back_right_motor, &motor_handle.back_left_motor, true, target);
		break;
	case LEFT_TURN_COMMAND:
	case RIGHT_TURN_COMMAND:    // fix me
		Set_Motor_Command(&motor_handle.front_left_motor, &motor_handle.back_left_motor, false, target);
		Set_Motor_Command(&motor_handle.back_left_motor, &motor_handle.front_left_motor, true, target);
		Set_Motor_Command(&motor_handle.front_right_motor, &motor_handle.back_right_motor, false, target);
		Set_Motor_Command(&motor_handle.back_right_motor, &motor_handle.front_right_motor, true, target);
		break;
	case STOP_COMMAND:
		break;
	default:
		break;
	}
	motor_command->current_command = motor_command->next_command;
	motor_command->next_command = NO_COMMAND;
}

/// @brief 1 == front_pos_sync, 2 == back_pos_sync, 3 == left_pos_sync, 4 == right_pos_sync, 5 sync_all_pos, 6 == front_velo_sync, 7 = back_velo_sync, 8 == left_velo_sync, 9 == right_velo_sync, 10 == sync_all_velo 
/// @param command To run
/// @param motor_command Pointer to I2c_Motor_Command structure
void Run_Motor_Command_Calibrate(const int32_t command, I2C_Motor_Command *motor_command)
{
	switch(command)
	{
	case 1:
		Run_Front_Position_Sync(&motor_handle, motor_command);
		break;
	case 2:
		Run_Back_Position_Sync(&motor_handle, motor_command);
		break;
	case 3:
		Run_Left_Position_Sync(&motor_handle, motor_command);
		break;
	case 4:
		Run_Right_Position_Sync(&motor_handle, motor_command);
		break;
	case 5:
		Run_Sync_All_Position(&motor_handle, motor_command);
		break;
	case 6:
		Run_Front_Velocity_Sync(&motor_handle, motor_command);
		break;
	case 7:
		Run_Back_Velocity_Sync(&motor_handle, motor_command);
		break;
	case 8:
		Run_Left_Velocity_Sync(&motor_handle, motor_command);
		break;
	case 9:
		Run_Right_Velocity_Sync(&motor_handle, motor_command);
		break;
	case 10:
		Run_Sync_All_Velocity(&motor_handle, motor_command);
		break;
	default:
		break;
	}
}


#define CALIBRATE_MODE 1



void Init_Motor_Controller()
{
    GPIO_Output_Init(&left_direction_pin, LEFT_MOTORS_DIR_PIN_GPIO_Port, LEFT_MOTORS_DIR_PIN_Pin, GPIO_PIN_RESET);
	GPIO_Output_Init(&right_direction_pin, RIGHT_MOTORS_DIR_PIN_GPIO_Port, RIGHT_MOTORS_DIR_PIN_Pin, GPIO_PIN_RESET);
	GPIO_Output_Init(&motor_brake_pin, ALL_MOTORS_BRAKE_PIN_GPIO_Port, ALL_MOTORS_BRAKE_PIN_Pin, GPIO_PIN_SET);
	GPIO_Output_Init(&motor_enable_pin, ALL_MOTORS_ENABLE_PIN_GPIO_Port, ALL_MOTORS_ENABLE_PIN_Pin, GPIO_PIN_RESET);


    Init_Motor_Pins(&motor_handle.front_left_motor, 0, 0, MOTOR1_U_GPIO_Port, MOTOR1_U_Pin,
					MOTOR1_V_GPIO_Port, MOTOR1_V_Pin, MOTOR1_W_GPIO_Port, MOTOR1_W_Pin,
					&left_direction_pin, &motor_brake_pin, &motor_enable_pin);

	Init_Motor_Pins(&motor_handle.front_right_motor, 1, 0, MOTOR2_U_GPIO_Port, MOTOR2_U_Pin,
					MOTOR2_V_GPIO_Port, MOTOR2_V_Pin, MOTOR2_W_GPIO_Port, MOTOR2_W_Pin,
					&left_direction_pin, &motor_brake_pin, &motor_enable_pin);

	Init_Motor_Pins(&motor_handle.back_left_motor, 2, 0, MOTOR3_U_GPIO_Port, MOTOR3_U_Pin,
					MOTOR3_V_GPIO_Port, MOTOR3_V_Pin, MOTOR3_W_GPIO_Port, MOTOR3_W_Pin,
					&left_direction_pin, &motor_brake_pin, &motor_enable_pin);

	Init_Motor_Pins(&motor_handle.back_right_motor, 3, 0, MOTOR4_U_GPIO_Port, MOTOR4_U_Pin,
					MOTOR4_V_GPIO_Port, MOTOR4_V_Pin, MOTOR4_W_GPIO_Port, MOTOR4_W_Pin,
					&left_direction_pin, &motor_brake_pin, &motor_enable_pin);


    Init_Motor_Pid_Position(&motor_handle.front_left_motor, &f_left_motor_pos_pid_params);
	Init_Motor_Pid_Position(&motor_handle.front_right_motor, &f_right_motor_pos_pid_params);
	Init_Motor_Pid_Position(&motor_handle.back_left_motor, &b_left_motor_pos_pid_params);
	Init_Motor_Pid_Position(&motor_handle.back_right_motor, &b_right_motor_pos_pid_params);

	Init_Motor_Pid_Velocity(&motor_handle.front_left_motor, &pid_parameters_velocity);
	Init_Motor_Pid_Velocity(&motor_handle.front_right_motor, &pid_parameters_velocity);
	Init_Motor_Pid_Velocity(&motor_handle.back_left_motor, &pid_parameters_velocity);
	Init_Motor_Pid_Velocity(&motor_handle.back_right_motor, &pid_parameters_velocity);

    Set_I2C_Handle(&motor_handle, &hi2c2);
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  Init_Motor_Timer();
  Init_Motor_Controller();
  Run_Stop_Command(&motor_handle, &command_handle);

  if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
	  Error_Handler();

    motor_ready = true;

    Init_I2C_Handle(&i2c_handle, &hi2c1); 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	while(1)
	{
        
		int32_t motor_command = 0;
        Run_Motor_Command_Calibrate(motor_command, &command_handle);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 36;
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
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
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LEFT_MOTORS_DIR_PIN_Pin|RIGHT_MOTORS_DIR_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ALL_MOTORS_ENABLE_PIN_Pin|ALL_MOTORS_BRAKE_PIN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LEFT_MOTORS_DIR_PIN_Pin RIGHT_MOTORS_DIR_PIN_Pin */
  GPIO_InitStruct.Pin = LEFT_MOTORS_DIR_PIN_Pin|RIGHT_MOTORS_DIR_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ALL_MOTORS_ENABLE_PIN_Pin ALL_MOTORS_BRAKE_PIN_Pin */
  GPIO_InitStruct.Pin = ALL_MOTORS_ENABLE_PIN_Pin|ALL_MOTORS_BRAKE_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR4_W_Pin MOTOR4_V_Pin MOTOR4_U_Pin */
  GPIO_InitStruct.Pin = MOTOR4_W_Pin|MOTOR4_V_Pin|MOTOR4_U_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR3_W_Pin MOTOR3_V_Pin MOTOR3_U_Pin MOTOR2_W_Pin */
  GPIO_InitStruct.Pin = MOTOR3_W_Pin|MOTOR3_V_Pin|MOTOR3_U_Pin|MOTOR2_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR2_V_Pin MOTOR2_U_Pin MOTOR1_W_Pin MOTOR1_V_Pin
                           MOTOR1_U_Pin */
  GPIO_InitStruct.Pin = MOTOR2_V_Pin|MOTOR2_U_Pin|MOTOR1_W_Pin|MOTOR1_V_Pin
                          |MOTOR1_U_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	Run_Stop_Command(&motor_handle, &command_handle);
	__disable_irq();
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
