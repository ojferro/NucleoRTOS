#include "main.h"

extern "C" {
#include "cmsis_os.h"
}

#include "string.h"
#include <stdarg.h>
#include <stdio.h>
#include "peripheral_config.h"
#include "Logger.h"
#include "mpu6050.hpp"
#include "ComplementaryFilter.h"
#include "mcp2515.hpp"
#include "ODrive.hpp"
#include "Controller.h"
#include "LQR.h"

#include <stdio.h>
#include <cstring>
#include <cmath>

// Define CAN IDs of the Axes. Not to be confused with the encoder SPI CS ports.
#define ODRV_AXIS0_CAN_ID 0x4 // ID of the Left axis
#define ODRV_AXIS1_CAN_ID 0x3 // ID of the Right axis



#define RxBuf_SIZE 9 // This needs to be EXACTLY the size of the msg being received.
#define MainBuf_SIZE 18

static float enc_pos_0 = 0.0f;
static float enc_vel_0 = 0.0f;
static float enc_pos_1 = 0.0f;
static float enc_vel_1 = 0.0f;

uint8_t RxBuf[RxBuf_SIZE];
uint8_t MainBuf[MainBuf_SIZE];
bool receivedUartData = false;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size){
    memcpy(MainBuf, RxBuf, RxBuf_SIZE);

    // TODO: Without these 2 lines, it goes crazy. Investigate why.
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

    receivedUartData = true;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __NOP();
  }
}

void handleCANRx(CANFrame& rxMsg){
    uint32_t odrvID  = rxMsg.id >> 5;
    uint32_t odrvCmd = rxMsg.id & 0b11111;

    if (odrvCmd == ODrive::AxisCommand::ENCODER_ESTIMATES)
    {
        const auto encoderPos = can_getSignal<float>(rxMsg, 0, 32, true, 1, 0);
        const auto encoderVel = can_getSignal<float>(rxMsg, 4, 32, true, 1, 0);

        if (odrvID == ODRV_AXIS0_CAN_ID)
        {
            // Update state variables
            enc_pos_0 = encoderPos;
            enc_vel_0 = encoderVel;

            // Transmit for visualization
            const float wheelRadius = 0.03f;
            debugLogFmt("enc_pos_0:%.3f\n", encoderPos * M_TWOPI * wheelRadius);
            debugLogFmt("enc_vel_0:%f\n", encoderVel * M_TWOPI * wheelRadius);
        }
        else if (odrvID == ODRV_AXIS1_CAN_ID)
        {
            // Update state variables
            enc_pos_1 = encoderPos;
            enc_vel_1 = encoderVel;

            // Transmit for visualization
            const float wheelRadius = 0.03f;
            debugLogFmt("enc_pos_1:%.3f\n", -encoderPos * M_TWOPI * wheelRadius);
            debugLogFmt("enc_vel_1:%f\n", -encoderVel * M_TWOPI * wheelRadius);
        }
    }

    if (odrvCmd == ODrive::AxisCommand::GET_BUS_VOLTAGE_CURRENT)
    {
        const auto busVoltage  = can_getSignal<float>(rxMsg, 0, 32, true, 1, 0);
        const auto busCurrent  = can_getSignal<float>(rxMsg, 4, 32, true, 1, 0);
        debugLogFmt("bus_voltage:%.3f\n", busVoltage);
        debugLogFmt("bus_current:%.3f\n", busCurrent);
    }

    if (odrvCmd == ODrive::AxisCommand::ODRIVE_HEARTBEAT_MESSAGE)
    {
        const auto axisError = can_getSignal<uint32_t>(rxMsg, 0, 32, true);
        if (axisError != 0)
        {
            if (odrvID == ODRV_AXIS0_CAN_ID)
                debugLogFmt("dbg_msg:Axis0 Err Code: %d\n", axisError);

            if (odrvID == ODRV_AXIS1_CAN_ID)
                debugLogFmt("dbg_msg:Axis1 Err Code: %d\n", axisError);
        }
    }
}

struct CommandHandler{

    CommandHandler(): m_controlMode(ODrive::ControlMode::POSITION_CONTROL){}

    ODrive::ControlMode m_controlMode;

    void handleMasterCmd(const std::string& strRx, ODrive::Axis& axis0, ODrive::Axis& axis1){
        // Control State
        if (strRx.compare("calib_rtn") == 0)
        {
            // axis.clearErrors();
            axis0.setRequestedState(ODrive::AxisState::FULL_CALIBRATION_SEQUENCE);
            axis1.setRequestedState(ODrive::AxisState::FULL_CALIBRATION_SEQUENCE);

            const auto dbg_msg = "dbg_msg: Dispatched "+strRx+" to ODrive\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("clear_err") == 0)
        {
            axis0.clearErrors();
            axis1.clearErrors();

            const auto dbg_msg = "dbg_msg: Dispatched "+strRx+" to ODrive\n";
            debugLog(dbg_msg.c_str());
        }
        // else if (strRx.compare("ClLp_ctrl") == 0)
        // {
        //     axis0.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);
        //     axis1.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);

        //     const auto dbg_msg = "dbg_msg: Dispatched "+strRx+" to ODrive\n";
        //     debugLog(dbg_msg.c_str());
        // }
        else if (strRx.compare("idle_ctrl") == 0)
        {
            axis0.setRequestedState(ODrive::AxisState::IDLE);
            axis1.setRequestedState(ODrive::AxisState::IDLE);

            const auto dbg_msg = "dbg_msg: Dispatched "+strRx+" to ODrive\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("auto_ctrl") == 0)
        {
            // Initialize wheels to 0
            axis0.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);
            axis0.setControllerModes(ODrive::ControlMode::POSITION_CONTROL);
            axis0.setInputPos(0, 0, 0);

            axis1.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);
            axis1.setControllerModes(ODrive::ControlMode::POSITION_CONTROL);
            axis1.setInputPos(0, 0, 0);

            osDelay(1000);
            debugLog("dbg_msg: Zeroing wheels done.\n");

            axis0.setRequestedState(ODrive::AxisState::IDLE);
            axis1.setRequestedState(ODrive::AxisState::IDLE);

            debugLog("dbg_msg: Starting closed loop torque control.\n");
            osDelay(500);

            m_controlMode = ODrive::ControlMode::TORQUE_CONTROL;
            axis0.setControllerModes(m_controlMode);
            axis0.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);

            axis1.setControllerModes(m_controlMode);
            axis1.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);

            axis0.setInputTorque(0);
            axis1.setInputTorque(0);

            debugLog("dbg_msg: Torque control active.\n");
        }

        // Control Modes
        else if (strRx.compare("posn_ctrl") == 0)
        {
            m_controlMode = ODrive::ControlMode::POSITION_CONTROL;
            axis0.setControllerModes(m_controlMode);
            axis1.setControllerModes(m_controlMode);

            axis0.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);
            axis1.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);

            const auto dbg_msg = "dbg_msg: Dispatched "+strRx+" to ODrive\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("velo_ctrl") == 0)
        {
            m_controlMode = ODrive::ControlMode::VELOCITY_CONTROL;
            axis0.setControllerModes(m_controlMode);
            axis1.setControllerModes(m_controlMode);

            axis0.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);
            axis1.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);

            const auto dbg_msg = "dbg_msg: Dispatched "+strRx+" to ODrive\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("torq_ctrl") == 0)
        {
            m_controlMode = ODrive::ControlMode::TORQUE_CONTROL;
            axis0.setControllerModes(m_controlMode);
            axis1.setControllerModes(m_controlMode);

            axis0.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);
            axis1.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);

            const auto dbg_msg = "dbg_msg: Dispatched "+strRx+" to ODrive\n";
            debugLog(dbg_msg.c_str());
        }
        else if (strRx.compare("volt_ctrl") == 0)
        {
            m_controlMode = ODrive::ControlMode::VOLTAGE_CONTROL;
            axis0.setControllerModes(m_controlMode);
            axis1.setControllerModes(m_controlMode);

            axis0.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);
            axis1.setRequestedState(ODrive::AxisState::CLOSED_LOOP_CONTROL);

            const auto dbg_msg = "dbg_msg: Dispatched "+strRx+" to ODrive\n";
            debugLog(dbg_msg.c_str());
        }
        
        // Setpoint
        else if (strRx.find("sp:")!=std::string::npos)
        {
            const auto sp = strRx.substr(3);
            auto setpoint = std::stof(sp);

            if (m_controlMode == ODrive::ControlMode::POSITION_CONTROL)
            {
                debugLogFmt("dbg_msg: Setting to POSITION_CONTROL\n");
                axis0.setInputPos(setpoint, 0, 0);
                axis1.setInputPos(-setpoint, 0, 0);
            }
            else if (m_controlMode == ODrive::ControlMode::VELOCITY_CONTROL)
            {
                debugLogFmt("dbg_msg: Setting to VELOCITY_CONTROL\n");
                axis0.setInputVel(setpoint, 0.0f);
                axis1.setInputVel(-setpoint, 0.0f);
            }
            else if (m_controlMode == ODrive::ControlMode::TORQUE_CONTROL)
            {
                debugLogFmt("dbg_msg: Setting to TORQUE_CONTROL\n");
                axis0.setInputTorque(setpoint);
                axis1.setInputTorque(-setpoint);
            }
            else if (m_controlMode == ODrive::ControlMode::VOLTAGE_CONTROL)
            {
                debugLogFmt("dbg_msg: Voltage control not implemented yet\n");
            }

            debugLogFmt("dbg_msg: Setpoint %f command received\n", setpoint);
        }
        
        else
        {
            std::string feedback = "dbg_msg:Unknown cmd " + strRx + "\n";
            debugLog(feedback.c_str());
        }
    }
};


osThreadId_t controlTaskHandle;

const  size_t CONTROL_TASK_STACK_SIZE = 1024*8;                 // Stack size (in words)
static StackType_t control_task_stack[CONTROL_TASK_STACK_SIZE]; // Stack array
static StaticTask_t control_task_tcb;                           // Task control block

const osThreadAttr_t controlTask_attributes = 
  {
    .name = "controlTask",
    .attr_bits = 0,
    .cb_mem = &control_task_tcb,
    .cb_size = sizeof(StaticTask_t),
    .stack_mem = control_task_stack,
    .stack_size = CONTROL_TASK_STACK_SIZE * sizeof(StackType_t), // number of bytes = thisValue * sizeof(StackType_t)
    .priority = (osPriority_t) osPriorityNormal,
    .tz_module = 0,
    .reserved = 0
  };

osThreadId_t instructionReceiverTaskHandle;

const  size_t INSTRUCTION_RECEIVER_TASK_STACK_SIZE = 1024*4;                 // Stack size (in words)
static StackType_t instruction_receiver_task_stack[INSTRUCTION_RECEIVER_TASK_STACK_SIZE]; // Stack array
static StaticTask_t instruction_receiver_task_tcb;                           // Task control block

struct InstructionReceiverContext{
    ODrive::Axis& axis0;
    ODrive::Axis& axis1;
    CommandHandler& cmdHandler;
};

struct ControlTaskContext{
    ODrive::Axis& axis0;
    ODrive::Axis& axis1;
};

const osThreadAttr_t instructionReceiverTask_attributes = 
  {
    .name = "instructionReceiverTask",
    .attr_bits = 0,
    .cb_mem = &instruction_receiver_task_tcb,
    .cb_size = sizeof(StaticTask_t),
    .stack_mem = instruction_receiver_task_stack,
    .stack_size = INSTRUCTION_RECEIVER_TASK_STACK_SIZE * sizeof(StackType_t), // number of bytes = thisValue * sizeof(StackType_t)
    .priority = (osPriority_t) osPriorityBelowNormal,
    .tz_module = 0,
    .reserved = 0
  };

static_assert(sizeof(StackType_t) == 4, "StackType_t size is not 4 bytes");

osThreadId_t canCallbackHandlerTaskHandle;

const  size_t canCallbackTaskStackSize = 1024*4;                 // Stack size (in words)
static StackType_t can_callback_task_stack[canCallbackTaskStackSize]; // Stack array
static StaticTask_t can_callback_task_tcb;                           // Task control block

const osThreadAttr_t canCallbackHandlerTask_attributes = 
  {
    .name = "canCallbackHandlerTask",
    .attr_bits = 0,
    .cb_mem = &can_callback_task_tcb,
    .cb_size = sizeof(StaticTask_t),
    .stack_mem = can_callback_task_stack,
    .stack_size = canCallbackTaskStackSize * sizeof(StackType_t), // number of bytes = thisValue * sizeof(StackType_t)
    .priority = (osPriority_t) osPriorityNormal,
    .tz_module = 0,
    .reserved = 0
  };

void SystemClock_Config(void);
void ControlTask(void *argument);
void InstructionReceiverTask(void *argument);
void CanCallbackHandlerTask(void *argument);


int main(void)
{
  HAL_Init();

  SystemClock_Config();

  // Initialize all configured peripherals
  if (!InitializeHardware())
  {
      Error_Handler();
  }

  debugLog("dbg_msg:STM32 online\n");

  // Ensure MPU6050 is up and running
    debugLog("dbg_msg:Initializing MPU6050\n");
    while (MPU6050_Init(&hi2c1) == 1)
    {
        debugLog("dbg_msg:MPU6050 offline\n");
        HAL_Delay(200);
    }
    debugLog("dbg_msg:MPU6050 online\n");

    // Initialize MCP2515
    // Important note: Anything referenced by the other tasks needs to be marked as static
    static auto mcp2515 = MCP2515(&hspi1);

    MCP2515::ERROR _e;
    _e = mcp2515.reset();
    _e = mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    _e = mcp2515.setNormalMode();

    if (_e != MCP2515::ERROR_OK)
    {
        debugLog("Error!\n");
    }
    debugLog("dbg_msg:MCP2515 online\n");

    // Set up safely, to be idle and in position control mode
    // Important note: Anything referenced by the other tasks needs to be marked as static
    static ODrive::Axis axis0(ODRV_AXIS0_CAN_ID, mcp2515);
    axis0.setRequestedState(ODrive::AxisState::IDLE);
    axis0.setControllerModes(ODrive::ControlMode::POSITION_CONTROL);
    axis0.getBusVoltageCurrent();
    axis0.setLimits(200.0f, 10.0f);
    axis0.clearErrors();
    axis0.setPositionGain(20.0f);
    axis0.setVelGains(0.05f, 0.001f);

    static ODrive::Axis axis1(ODRV_AXIS1_CAN_ID, mcp2515);
    axis1.setRequestedState(ODrive::AxisState::IDLE);
    axis1.setControllerModes(ODrive::ControlMode::POSITION_CONTROL);
    axis1.getBusVoltageCurrent();
    axis1.setLimits(200.0f, 10.0f);
    axis1.clearErrors();
    axis1.setPositionGain(20.0f);
    axis1.setVelGains(0.05f, 0.001f);

    debugLog("dbg_msg:ODrive online\n");

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);

    // Set up context for Instruction Receiving Task
    // Important note: Anything referenced by the other tasks needs to be marked as static
    static auto cmdHandler = CommandHandler();
    static InstructionReceiverContext context{axis0, axis1, cmdHandler};
    static ControlTaskContext controllerContext{axis0, axis1};

  // Init scheduler
  osKernelInitialize();

  // Create the threads
  controlTaskHandle = osThreadNew(ControlTask, &controllerContext, &controlTask_attributes);
  instructionReceiverTaskHandle = osThreadNew(InstructionReceiverTask, &context, &instructionReceiverTask_attributes);
  canCallbackHandlerTaskHandle = osThreadNew(CanCallbackHandlerTask, &mcp2515, &canCallbackHandlerTask_attributes);

  if (controlTaskHandle == NULL || instructionReceiverTaskHandle == NULL || canCallbackHandlerTaskHandle == NULL)
  {
    debugLog("dbg_msg: ERROR: Failed to start RTOS threads.\n");
    Error_Handler();
  }

  // Start scheduler
  osKernelStart();

  // We should never get here as control is now taken by the scheduler
  while (1)
  {}
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
// static void MX_GPIO_Init(void)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};
// /* USER CODE BEGIN MX_GPIO_Init_1 */
// /* USER CODE END MX_GPIO_Init_1 */

//   /* GPIO Ports Clock Enable */
//   __HAL_RCC_GPIOC_CLK_ENABLE();
//   __HAL_RCC_GPIOH_CLK_ENABLE();
//   __HAL_RCC_GPIOA_CLK_ENABLE();
//   __HAL_RCC_GPIOB_CLK_ENABLE();

//   /*Configure GPIO pin Output Level */
//   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

//   /*Configure GPIO pin : B1_Pin */
//   GPIO_InitStruct.Pin = B1_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

//   /*Configure GPIO pin : LD2_Pin */
//   GPIO_InitStruct.Pin = LD2_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

// }

float ToDeg(float rad)
{
    constexpr float toDeg = 180.0f / M_PI;
    return rad * toDeg;
}

// template <typename T>
// T clamp(const T& value, const T& lower, const T& upper) {
//     if (value < lower) return lower;
//     if (value > upper) return upper;
//     return value;
// }

void ControlTask(void *argument)
{
  ControlTaskContext *c = (ControlTaskContext *)argument;

  auto cf = ComplementaryFilter(&hi2c1);

  // Control
  auto controller = LQR();
  Controller::U u = {0.0f, 0.0f};
  Controller::State state = {0.0f, 0.0f, 0.0f, 0.0f};

  while (true)
  {
    cf.Step();
    // debugLogFmt("imu_p:%.6f\n", cf.GetPitchAngle() * 180.0f / M_PI);
    // debugLogFmt("imu_p_dot:%.6f\n", cf.GetPitchAngleDot() * 180.0f / M_PI);

    // TODO: Populate state and run controller!
    state[Controller::StateIndex::X] = enc_pos_0;
    state[Controller::StateIndex::THETA] = cf.GetPitchAngle();
    state[Controller::StateIndex::X_DOT] = enc_vel_0;
    state[Controller::StateIndex::THETA_DOT] = cf.GetPitchAngleDot();

    // debugLogFmt("x:%.6f\n", state[Controller::StateIndex::X]);
    // debugLogFmt("theta:%.6f\n", state[Controller::StateIndex::THETA]);
    // debugLogFmt("x_dot:%.6f\n", state[Controller::StateIndex::X_DOT]);
    // debugLogFmt("theta_dot:%.6f\n", state[Controller::StateIndex::THETA_DOT]);

    controller.iterate(u, state);

    // Send the control signal to the ODrive
    const auto maxTorque = 0.05f;
    u[0] = std::min(std::max(u[0], -maxTorque), maxTorque);
    // u[1] = std::min(std::max(u[1], -maxTorque), maxTorque);
    c->axis0.setInputTorque(u[0]);
    c->axis1.setInputTorque(-u[0]);
    // c->axis0.setInputVel(u[0]*5, 0.0f);
    // c->axis1.setInputVel(-u[1]*5, 0.0f);

    // debugLogFmt("ctrl_u_0:%.6f\n", u[0]);

    osDelay(10);
  }
}

void InstructionReceiverTask(void *argument)
{  
  InstructionReceiverContext *c = (InstructionReceiverContext *)argument;
  while (true)
  {
    if (receivedUartData)
    {
        receivedUartData = false; // TODO: There's likely a race condition here! This might take precedence over the IRQ for DMA on Uart RX...
        std::string strRx = ""; // TODO: Remove heap allocation, size is known beforehand.
        for (const auto& element : RxBuf) {
            // Convert each element to its ASCII representation and append to string
            strRx += static_cast<char>(element);
        }

        debugLogFmt("dbg_msg: Received %s\n", strRx.c_str());
        c->cmdHandler.handleMasterCmd(strRx, c->axis0, c->axis1);
    }
    osDelay(100);
  }
}

// TODO: Handle this with an interrupt.
void CanCallbackHandlerTask(void *argument)
{
  MCP2515 *mcp2515 = (MCP2515 *)argument;
  while (true)
  {
    CANFrame rxMsg;
    if (mcp2515->readMessage(&rxMsg) == MCP2515::ERROR_OK)
    {
        handleCANRx(rxMsg);
    }
    osDelay(10);
  }
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
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  const auto err_msg = "NUCLEO IN ERROR STATE";
  HAL_UART_Transmit(&huart2, (uint8_t*)err_msg, strlen(err_msg), 10);
  __disable_irq();
  while (1)
  {
  }
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
