/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file : main.c
 * @brief : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define COMM_MODE_UART
#if !defined(COMM_MODE_UART) && !defined(COMM_MODE_CAN)
#define COMM_MODE_CAN
#endif
#if defined(COMM_MODE_UART) && defined(COMM_MODE_CAN)
#error Select only one communication mode
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId MotorTaskHandle;
osThreadId ControlTaskHandle;
/* USER CODE BEGIN PV */
#ifdef COMM_MODE_CAN
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
const uint32_t CAN_ID_COMMANDS = 0x010;
const uint32_t CAN_ID_TELEMETRY = 0x020;
#endif
uint8_t rx_buffer[33];
#ifdef COMM_MODE_UART
uint8_t uart_rx_buffer[32];
volatile uint8_t uart_restart_needed = 0;
static StaticSemaphore_t uart_tx_mutex_buffer;
static SemaphoreHandle_t uart_tx_mutex;
#endif
volatile uint32_t command_timestamp = 0;
volatile uint32_t command_fault_generation = 0;
volatile uint32_t fault_generation = 0;
volatile uint32_t motor_task_heartbeat = 0;
uint8_t adc_sample_valid = 0;
int8_t last_motor_direction = 0;
uint32_t motor_stop_time = 0;
volatile uint16_t command_length=0;
uint32_t adc_value = 0;
volatile uint8_t command_ready=0;
float motor_current = 0.0f;
const float CURRENT_TRESHOLD = 3.5f;
uint16_t encoder_count = 0;
uint16_t encoder_last_count = 0;
int16_t speed_rpm = 0;
uint32_t last_time = 0;
uint32_t last_pid_time;

typedef struct{
    float Kd;
    float Kp;
    float Ki;
    float integral_sum;
    float prev_error;
    float out_min;
    float out_max;
}PID_Controller;
PID_Controller motor1_pid;
float target_speed_rpm;
int8_t motor_direction = 0;

typedef enum{
	Waiting_command_State,
	Fault_State,
	Motor_logic_Handling_State,
}SystemState_t;
SystemState_t current_state = Waiting_command_State;
typedef enum {
	FAULT_NONE = 0x00,
	FAULT_ESTOP = 0x01,
	FAULT_OVERCURRENT = 0x02,
	FAULT_TIMEOUT = 0x04,
	FAULT_STALL = 0x08,
	FAULT_ADC = 0x10,
	FAULT_TASK = 0x20,
	FAULT_COMM = 0x40
} FaultCode_t;
volatile uint32_t system_faults = FAULT_NONE;
volatile uint32_t last_comm_time = 0;
uint32_t stall_time_ms = 0;
const uint32_t COMM_TIMEOUT_MS = 500;
typedef struct{
uint32_t timestamp;
int16_t current_rpm;
int16_t target_rpm;
float current_amps;
uint32_t active_faults;
}TelemetryPacket_t;
TelemetryPacket_t telemetry_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_IWDG_Init(void);
static void MX_CAN_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void StopMotorOutputs(void)
{
    if (htim2.Instance != NULL) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        htim2.Instance->EGR = TIM_EGR_UG;
    }
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
}

static void AddFault(uint32_t faults)
{
    if (faults != FAULT_NONE) {
        system_faults |= faults;
        fault_generation++;
    }
}

static void ResetPID(void)
{
    motor1_pid.integral_sum = 0.0f;
    motor1_pid.prev_error = 0.0f;
    stall_time_ms = 0;
}

static void StopMotor(void)
{
    if (motor_direction != 0 || target_speed_rpm > 0.0f) {
        motor_stop_time = HAL_GetTick();
    }
    StopMotorOutputs();
    motor_direction = 0;
    target_speed_rpm = 0.0f;
    ResetPID();
    current_state = system_faults == FAULT_NONE ? Waiting_command_State : Fault_State;
}

float PID_Compute(PID_Controller *pid, float target_rpm, float current_rpm, float dt)
{
    if (!isfinite(dt) || dt <= 0.0f) {
        return 0.0f;
    }
    float error = target_rpm - current_rpm;
    float integral = pid->integral_sum + pid->Ki * error * dt;
    if (integral > pid->out_max) integral = pid->out_max;
    if (integral < pid->out_min) integral = pid->out_min;
    float d_term = pid->Kd * (error - pid->prev_error) / dt;
    float result = pid->Kp * error + integral + d_term;
    if ((result <= pid->out_max || error < 0.0f) &&
        (result >= pid->out_min || error > 0.0f)) {
        pid->integral_sum = integral;
    }
    result = pid->Kp * error + pid->integral_sum + d_term;
    pid->prev_error = error;
    last_pid_time = HAL_GetTick();
    if (result > pid->out_max) result = pid->out_max;
    if (result < pid->out_min) result = pid->out_min;
    return result;
}

static void StoreCommandFromISR(const uint8_t *data, uint16_t length)
{
    BaseType_t task_woken = pdFALSE;
    UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
    if (length == 0 || length >= sizeof(rx_buffer) || command_ready) {
        AddFault(FAULT_COMM);
        command_ready = 0;
        command_length = 0;
        StopMotorOutputs();
    } else {
        memcpy(rx_buffer, data, length);
        rx_buffer[length] = '\0';
        command_length = length;
        command_timestamp = HAL_GetTick();
        command_fault_generation = fault_generation;
        command_ready = 1;
    }
    taskEXIT_CRITICAL_FROM_ISR(mask);
    vTaskNotifyGiveFromISR(MotorTaskHandle, &task_woken);
    portYIELD_FROM_ISR(task_woken);
}

static uint8_t TakeCommand(uint8_t *data, uint16_t *length,
                           uint32_t *timestamp, uint32_t *generation)
{
    uint8_t ready;
    taskENTER_CRITICAL();
    ready = command_ready;
    if (ready) {
        *length = command_length;
        memcpy(data, rx_buffer, *length + 1U);
        *timestamp = command_timestamp;
        *generation = command_fault_generation;
        command_ready = 0;
    }
    taskEXIT_CRITICAL();
    return ready;
}

static uint8_t ProcessCommand(uint8_t *data, uint16_t length, uint32_t timestamp,
                              uint32_t generation, char *reply, size_t reply_size)
{
    while (length > 0 && (data[length - 1U] == '\r' ||
           data[length - 1U] == '\n' || data[length - 1U] == '\0')) {
        length--;
    }
    data[length] = '\0';
    for (uint16_t i = 0; i < length; i++) {
        if (data[i] == '\0' || data[i] == '\r' || data[i] == '\n') {
            snprintf(reply, reply_size, "Unknown command\r\n");
            return 0;
        }
    }
    uint8_t stop = strcmp((char *)data, "STOP") == 0;
    uint8_t reset = strcmp((char *)data, "RESET") == 0;
    uint8_t crash = strcmp((char *)data, "CRASH") == 0;
    int speed = -1;
    int8_t direction = 0;
    if (length >= 2 && length <= 4 && (data[0] == 'F' || data[0] == 'R')) {
        speed = 0;
        direction = data[0] == 'F' ? 1 : -1;
        for (uint16_t i = 1; i < length; i++) {
            if (data[i] < '0' || data[i] > '9') {
                speed = -1;
                break;
            }
            speed = speed * 10 + data[i] - '0';
        }
    }
    if (!stop && !reset && !crash && speed < 0) {
        snprintf(reply, reply_size, "Unknown command\r\n");
        return 0;
    }
    uint32_t now = HAL_GetTick();
    if (!stop && speed != 0 && now - timestamp > COMM_TIMEOUT_MS) {
        snprintf(reply, reply_size, "Command expired\r\n");
        return 0;
    }

    uint8_t do_crash = 0;
    uint32_t faults;
    int16_t rpm;
    uint8_t result = 0;
    taskENTER_CRITICAL();
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) {
        AddFault(FAULT_ESTOP);
        StopMotor();
    }
    if (stop || speed == 0) {
        StopMotor();
        last_comm_time = now;
        result = 1;
    } else if (reset) {
        if (generation == fault_generation && adc_sample_valid &&
            motor_current <= CURRENT_TRESHOLD &&
            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET &&
            (system_faults & FAULT_TASK) == 0
#ifdef COMM_MODE_UART
            && !uart_restart_needed
#endif
#ifdef COMM_MODE_CAN
            && (hcan.Instance->ESR & CAN_ESR_BOFF) == 0
#endif
            ) {
            system_faults = FAULT_NONE;
            StopMotor();
            motor_stop_time = now;
            last_comm_time = now;
            result = 2;
        } else {
            result = 3;
        }
    } else if (system_faults == FAULT_NONE) {
        if (crash) {
            StopMotor();
            do_crash = 1;
            result = 4;
        } else if (last_motor_direction != 0 && direction != last_motor_direction &&
                   (motor_direction != 0 || abs(speed_rpm) > 10 ||
                    now - motor_stop_time < 100U)) {
            if (motor_direction != 0) StopMotor();
            last_comm_time = timestamp;
            result = 5;
        } else {
            if (motor_direction == 0) ResetPID();
            motor_direction = direction;
            last_motor_direction = direction;
            target_speed_rpm = (float)speed;
            current_state = Motor_logic_Handling_State;
            last_comm_time = timestamp;
            result = 6;
        }
    }
    faults = system_faults;
    rpm = speed_rpm;
    taskEXIT_CRITICAL();

    if (result == 1) {
        snprintf(reply, reply_size, "Status: Motor stopped, RPM=%d, FAULTS:0x%02lX\r\n",
                 rpm, (unsigned long)faults);
    } else if (result == 2) {
        snprintf(reply, reply_size, "System faults cleared. Ready\r\n");
    } else if (result == 3) {
        snprintf(reply, reply_size, "RESET refused. FAULTS:0x%02lX\r\n", (unsigned long)faults);
    } else if (result == 4) {
        snprintf(reply, reply_size, "Simulating fatal crash\r\n");
    } else if (result == 5) {
        snprintf(reply, reply_size, "Motor stopped. Wait before reversing\r\n");
    } else if (result == 6) {
        snprintf(reply, reply_size, "Status: %s, Target=%d RPM, RPM=%d\r\n",
                 direction > 0 ? "Forward" : "Reverse", speed, rpm);
    } else {
        snprintf(reply, reply_size, "Error! Active Faults: 0x%02lX. Send RESET.\r\n",
                 (unsigned long)faults);
    }
    return do_crash;
}

static void UART_Send(const char *data)
{
#ifdef COMM_MODE_UART
    if (xSemaphoreTake(uart_tx_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
#endif
    HAL_UART_Transmit(&huart1, (uint8_t *)data, (uint16_t)strlen(data), 20);
#ifdef COMM_MODE_UART
    xSemaphoreGive(uart_tx_mutex);
#endif
}

static void ControlMotor(uint32_t now, uint8_t adc_valid)
{
    encoder_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint16_t difference = (uint16_t)(encoder_count - encoder_last_count);
    int32_t delta = difference <= 32767U ? (int32_t)difference : (int32_t)difference - 65536;
    uint32_t elapsed_ms = now - last_time;
    if (elapsed_ms != 0) {
        int32_t rpm = (int32_t)((int64_t)delta * 60000 / ((int64_t)96 * elapsed_ms));
        if (rpm > 32767) rpm = 32767;
        if (rpm < -32768) rpm = -32768;
        speed_rpm = (int16_t)rpm;
        encoder_last_count = encoder_count;
        last_time = now;
    }

    taskENTER_CRITICAL();
    adc_sample_valid = adc_valid;
    uint32_t faults = FAULT_NONE;
    if (!adc_valid) faults |= FAULT_ADC;
    if (adc_valid && motor_current > CURRENT_TRESHOLD) faults |= FAULT_OVERCURRENT;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) faults |= FAULT_ESTOP;
    if (now - motor_task_heartbeat > 300U || elapsed_ms > 300U) faults |= FAULT_TASK;
#ifdef COMM_MODE_CAN
    if ((hcan.Instance->ESR & CAN_ESR_BOFF) != 0) faults |= FAULT_COMM;
#endif
    if (motor_direction != 0 && now - last_comm_time > COMM_TIMEOUT_MS) faults |= FAULT_TIMEOUT;
    AddFault(faults);

    float pwm = 0.0f;
    if (system_faults == FAULT_NONE && motor_direction != 0 && target_speed_rpm > 0.0f) {
        float current_speed_abs = (float)abs(speed_rpm);
        if (elapsed_ms != 0) {
            pwm = PID_Compute(&motor1_pid, target_speed_rpm, current_speed_abs,
                              (float)elapsed_ms / 1000.0f);
        }
        if (target_speed_rpm >= 10.0f && current_speed_abs < 10.0f && pwm > 0.0f) {
            stall_time_ms += elapsed_ms;
            if (stall_time_ms >= 1000U) AddFault(FAULT_STALL);
        } else {
            stall_time_ms = 0;
        }
    }
    if (system_faults != FAULT_NONE || motor_direction == 0 || target_speed_rpm <= 0.0f) {
        StopMotor();
    } else {
        HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin,
                         motor_direction > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin,
                         motor_direction < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)pwm);
    }
    telemetry_data.timestamp = now;
    telemetry_data.current_rpm = speed_rpm;
    telemetry_data.target_rpm = (int16_t)(target_speed_rpm * motor_direction);
    telemetry_data.current_amps = motor_current;
    telemetry_data.active_faults = system_faults;
    uint8_t watchdog_healthy = (system_faults & FAULT_TASK) == 0;
    taskEXIT_CRITICAL();
    if (watchdog_healthy && HAL_IWDG_Refresh(&hiwdg) != HAL_OK) Error_Handler();
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  StopMotorOutputs();
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) Error_Handler();
#ifdef COMM_MODE_UART
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
#endif
#ifdef COMM_MODE_CAN
  CAN_FilterTypeDef canfilterconfig = {0};
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = (CAN_ID_COMMANDS << 5);
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = (0x7FF << 5);
  canfilterconfig.FilterMaskIdLow = 0x0006;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK ||
      HAL_CAN_Start(&hcan) != HAL_OK) Error_Handler();
  TxHeader.StdId = CAN_ID_TELEMETRY;
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;
#endif
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
  StopMotorOutputs();
  last_time = HAL_GetTick();
  motor_task_heartbeat = last_time;
  motor_stop_time = last_time;
  encoder_last_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  motor1_pid.Kp = 40.0f;
  motor1_pid.Ki = 10.0f;
  motor1_pid.Kd = 1.0f;
  motor1_pid.out_max = 65535.0f;
  motor1_pid.out_min = 0.0f;
  ResetPID();
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) {
    AddFault(FAULT_ESTOP);
    current_state = Fault_State;
  }
  HAL_IWDG_Refresh(&hiwdg);
/* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
#ifdef COMM_MODE_UART
  uart_tx_mutex = xSemaphoreCreateMutexStatic(&uart_tx_mutex_buffer);
  if (uart_tx_mutex == NULL) Error_Handler();
#endif
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, StartDefaultTask, osPriorityNormal, 0, 256);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, StartTask02, osPriorityHigh, 0, 256);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  if (MotorTaskHandle == NULL || ControlTaskHandle == NULL) Error_Handler();
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  Error_Handler();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_Pin|MOTOR_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_IN1_Pin MOTOR_IN2_Pin */
  GPIO_InitStruct.Pin = MOTOR_IN1_Pin|MOTOR_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_1) {
        UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
        AddFault(FAULT_ESTOP);
        StopMotorOutputs();
        taskEXIT_CRITICAL_FROM_ISR(mask);
    }
}

#ifdef COMM_MODE_UART
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) return;
        StoreCommandFromISR(uart_rx_buffer, Size);
        if (HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_rx_buffer, sizeof(uart_rx_buffer)) != HAL_OK) {
            UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
            AddFault(FAULT_COMM);
            StopMotorOutputs();
            uart_restart_needed = 1;
            taskEXIT_CRITICAL_FROM_ISR(mask);
        }
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
        AddFault(FAULT_COMM);
        StopMotorOutputs();
        uart_restart_needed = 1;
        command_ready = 0;
        taskEXIT_CRITICAL_FROM_ISR(mask);
    }
}
#endif

#ifdef COMM_MODE_CAN
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can)
{
    if (can->Instance != CAN1) return;
    if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
        UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
        AddFault(FAULT_COMM);
        StopMotorOutputs();
        taskEXIT_CRITICAL_FROM_ISR(mask);
        return;
    }
    if (RxHeader.IDE == CAN_ID_STD && RxHeader.RTR == CAN_RTR_DATA &&
        RxHeader.StdId == CAN_ID_COMMANDS && RxHeader.DLC > 0 && RxHeader.DLC <= 8) {
        StoreCommandFromISR(RxData, (uint16_t)RxHeader.DLC);
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *can)
{
    if (can->Instance == CAN1) {
        UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
        AddFault(FAULT_COMM);
        StopMotorOutputs();
        command_ready = 0;
        taskEXIT_CRITICAL_FROM_ISR(mask);
    }
}
#endif
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the MotorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  (void)argument;
  char tx_buffer[100];
  uint8_t command[sizeof(rx_buffer)];
#ifdef COMM_MODE_UART
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buffer, sizeof(uart_rx_buffer)) != HAL_OK) {
    taskENTER_CRITICAL();
    AddFault(FAULT_COMM);
    uart_restart_needed = 1;
    taskEXIT_CRITICAL();
  }
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  UART_Send("\r\n--- SYSTEM BOOT / RESET ---\r\n");
#endif
#ifdef COMM_MODE_CAN
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK) {
    Error_Handler();
  }
#endif
  for (;;) {
    motor_task_heartbeat = HAL_GetTick();
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
    motor_task_heartbeat = HAL_GetTick();
#ifdef COMM_MODE_UART
    taskENTER_CRITICAL();
    uint8_t restart = uart_restart_needed;
    uart_restart_needed = 0;
    taskEXIT_CRITICAL();
    if (restart) {
      HAL_NVIC_DisableIRQ(USART1_IRQn);
      HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);
      HAL_StatusTypeDef status = HAL_UART_AbortReceive(&huart1);
      if (status == HAL_OK) {
        status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buffer, sizeof(uart_rx_buffer));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
      }
      HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
      HAL_NVIC_EnableIRQ(USART1_IRQn);
      if (status != HAL_OK) {
        taskENTER_CRITICAL();
        AddFault(FAULT_COMM);
        StopMotorOutputs();
        uart_restart_needed = 1;
        taskEXIT_CRITICAL();
      }
    }
#endif
    uint16_t length = 0;
    uint32_t timestamp = 0;
    uint32_t generation = 0;
    if (TakeCommand(command, &length, &timestamp, &generation)) {
      uint8_t crash = ProcessCommand(command, length, timestamp, generation, tx_buffer, sizeof(tx_buffer));
      UART_Send(tx_buffer);
      if (crash) {
        for (;;) {}
      }
    }
  }
/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */

  /* Infinite loop */
  (void)argument;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);
#ifdef COMM_MODE_UART
  char tele_buffer[128];
#endif
  for (;;) {
    uint8_t adc_valid = 0;
    if (HAL_ADC_Start(&hadc1) == HAL_OK && HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
      adc_value = HAL_ADC_GetValue(&hadc1);
      adc_valid = adc_value <= 4095U;
    }
    if (HAL_ADC_Stop(&hadc1) != HAL_OK) adc_valid = 0;
    taskENTER_CRITICAL();
    if (adc_valid) motor_current = (float)adc_value * 5.0f / 4095.0f;
    taskEXIT_CRITICAL();
    ControlMotor(HAL_GetTick(), adc_valid);
#ifdef COMM_MODE_UART
    int amps_int = (int)telemetry_data.current_amps;
    int amps_frac = (int)((telemetry_data.current_amps - amps_int) * 100.0f);
    snprintf(tele_buffer, sizeof(tele_buffer), "[TELEMETRY] T:%lu RPM:%d TGT:%d I:%d.%02dA FAULTS:0x%02lX\r\n",
             (unsigned long)telemetry_data.timestamp, telemetry_data.current_rpm,
             telemetry_data.target_rpm, amps_int, amps_frac,
             (unsigned long)telemetry_data.active_faults);
    UART_Send(tele_buffer);
#endif
#ifdef COMM_MODE_CAN
    uint16_t rpm = (uint16_t)telemetry_data.current_rpm;
    uint16_t target = (uint16_t)telemetry_data.target_rpm;
    TxData[0] = (uint8_t)(rpm >> 8);
    TxData[1] = (uint8_t)rpm;
    TxData[2] = (uint8_t)(target >> 8);
    TxData[3] = (uint8_t)target;
    TxData[4] = (uint8_t)(telemetry_data.current_amps * 10.0f);
    TxData[5] = (uint8_t)telemetry_data.active_faults;
    TxData[6] = 0;
    TxData[7] = 0;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
      if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
        taskENTER_CRITICAL();
        AddFault(FAULT_COMM);
        StopMotor();
        taskEXIT_CRITICAL();
      }
    }
#endif
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
/* USER CODE END StartTask02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  StopMotorOutputs();
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
