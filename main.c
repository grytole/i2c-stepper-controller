//
// I2C 2-channel stepper controller
//

// This I2C slave device has 6 registers:
//  [0][1] - int16_t value of motor A speed in steps per second ([0] - hi, [1] - lo)
//  [2][3] - int16_t value of motor B speed in steps per second ([0] - hi, [1] - lo)
//  [4]    - uint8_t value of motors enabled flag (non-zero is 'enabled')
//  [5]    - uint8_t value of microstepping configuration (1, 2, 4, 8 or 16)

#include "stm8s.h"

#define STEPPER_ENABLE_PORT (GPIOC)
#define STEPPER_ENABLE_PIN  (GPIO_PIN_3)

#define STEPPER_DIR_A_PORT  (GPIOC)
#define STEPPER_DIR_A_PIN   (GPIO_PIN_4)

#define STEPPER_DIR_B_PORT  (GPIOC)
#define STEPPER_DIR_B_PIN   (GPIO_PIN_5)

#define STEPPER_STEP_A_PORT (GPIOC)
#define STEPPER_STEP_A_PIN  (GPIO_PIN_6)

#define STEPPER_STEP_B_PORT (GPIOC)
#define STEPPER_STEP_B_PIN  (GPIO_PIN_7)

#define STEPPER_MS1_PORT    (GPIOD)
#define STEPPER_MS1_PIN     (GPIO_PIN_1)

#define STEPPER_MS2_PORT    (GPIOD)
#define STEPPER_MS2_PIN     (GPIO_PIN_2)

#define STEPPER_MS3_PORT    (GPIOD)
#define STEPPER_MS3_PIN     (GPIO_PIN_3)

#define STEPPER_CONTROLLER_DATA_SIZE (6)

__IO uint16_t m_timerDeltaA = 0;
__IO uint16_t m_timerDeltaB = 0;

__IO uint8_t stepperControllerData[STEPPER_CONTROLLER_DATA_SIZE] = { 0 };
__IO uint8_t i2c_dataSize = 0;

__IO int16_t stepsPerSecondA = 0;
__IO int16_t stepsPerSecondB = 0;

INTERRUPT_HANDLER(TIM1_CAP_COM_IRQHandler, ITC_IRQ_TIM1_CAPCOM);
void stepper_Initialize(void);
void stepper_SetSpeed(int16_t stepsPerSecondA, int16_t stepsPerSecondB);
void stepper_SetEnabled(bool isEnabled);
void stepper_SetMicroStepping(uint8_t microStepDivider);

void i2c_Initialize(void);
uint8_t i2c_Update(void);

//----------------------------------------------------------------------------------- STEPPER

INTERRUPT_HANDLER(TIM1_CAP_COM_IRQHandler, ITC_IRQ_TIM1_CAPCOM) {
  disableInterrupts();
  if (SET == TIM1_GetITStatus(TIM1_IT_CC1)) {
    TIM1_ClearITPendingBit(TIM1_IT_CC1);
    GPIO_WriteHigh(STEPPER_STEP_A_PORT, STEPPER_STEP_A_PIN);
    TIM1_SetCompare1(m_timerDeltaA + TIM1_GetCounter());
    GPIO_WriteLow(STEPPER_STEP_A_PORT, STEPPER_STEP_A_PIN);
  }
  if (SET == TIM1_GetITStatus(TIM1_IT_CC2)) {
    TIM1_ClearITPendingBit(TIM1_IT_CC2);
    GPIO_WriteHigh(STEPPER_STEP_B_PORT, STEPPER_STEP_B_PIN);
    TIM1_SetCompare2(m_timerDeltaB + TIM1_GetCounter());
    GPIO_WriteLow(STEPPER_STEP_B_PORT, STEPPER_STEP_B_PIN);
  }
  enableInterrupts();
}

void stepper_Initialize(void) {
  GPIO_Init(STEPPER_ENABLE_PORT, STEPPER_ENABLE_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(STEPPER_DIR_A_PORT, STEPPER_DIR_A_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(STEPPER_DIR_B_PORT, STEPPER_DIR_B_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(STEPPER_STEP_A_PORT, STEPPER_STEP_A_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(STEPPER_STEP_B_PORT, STEPPER_STEP_B_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(STEPPER_MS1_PORT, STEPPER_MS1_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(STEPPER_MS2_PORT, STEPPER_MS2_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(STEPPER_MS3_PORT, STEPPER_MS3_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

  TIM1_DeInit();
  TIM1_TimeBaseInit(0, TIM1_COUNTERMODE_UP, UINT16_MAX, 0);
  TIM1_OC1Init(TIM1_OCMODE_TIMING, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE, UINT16_MAX,
               TIM1_OCPOLARITY_HIGH, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_RESET, TIM1_OCNIDLESTATE_RESET);
  TIM1_OC2Init(TIM1_OCMODE_TIMING, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE, UINT16_MAX,
               TIM1_OCPOLARITY_HIGH, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_RESET, TIM1_OCNIDLESTATE_RESET);
  TIM1_ITConfig(TIM1_IT_CC1, ENABLE);
  TIM1_ITConfig(TIM1_IT_CC2, ENABLE);

  stepper_SetSpeed(0, 0);
  stepper_SetEnabled(FALSE);
  stepper_SetMicroStepping(1);

  TIM1_Cmd(ENABLE);
}

void stepper_SetSpeed(int16_t stepsPerSecondA, int16_t stepsPerSecondB) {
  uint32_t timerPeriod = CLK_GetClockFreq() / (TIM1_GetPrescaler() + 1);

  if (0 == stepsPerSecondA) {
    TIM1_CCxCmd(TIM1_CHANNEL_1, DISABLE);
  } else {
    if (0 < stepsPerSecondA) {
      m_timerDeltaA = timerPeriod / stepsPerSecondA;
      GPIO_WriteHigh(STEPPER_DIR_A_PORT, STEPPER_DIR_A_PIN);
    } else {
      m_timerDeltaA = timerPeriod / (-stepsPerSecondA);
      GPIO_WriteLow(STEPPER_DIR_A_PORT, STEPPER_DIR_A_PIN);
    }
    TIM1_CCxCmd(TIM1_CHANNEL_1, ENABLE);
  }

  if (0 == stepsPerSecondB) {
    TIM1_CCxCmd(TIM1_CHANNEL_2, DISABLE);
  } else {
    if (0 < stepsPerSecondB) {
      m_timerDeltaB = timerPeriod / stepsPerSecondB;
      GPIO_WriteHigh(STEPPER_DIR_B_PORT, STEPPER_DIR_B_PIN);
    } else {
      m_timerDeltaB = timerPeriod / (-stepsPerSecondB);
      GPIO_WriteLow(STEPPER_DIR_B_PORT, STEPPER_DIR_B_PIN);
    }
    TIM1_CCxCmd(TIM1_CHANNEL_2, ENABLE);
  }
}

void stepper_SetEnabled(bool isEnabled) {
  if (FALSE == isEnabled) {
    GPIO_WriteHigh(STEPPER_ENABLE_PORT, STEPPER_ENABLE_PIN);
  } else {
    GPIO_WriteLow(STEPPER_ENABLE_PORT, STEPPER_ENABLE_PIN);
  }
}

void stepper_SetMicroStepping(uint8_t microStepDivider) {
  switch (microStepDivider) {
    case 1:
      GPIO_WriteLow(STEPPER_MS1_PORT, STEPPER_MS1_PIN);
      GPIO_WriteLow(STEPPER_MS2_PORT, STEPPER_MS2_PIN);
      GPIO_WriteLow(STEPPER_MS3_PORT, STEPPER_MS3_PIN);
      break;
    case 2:
      GPIO_WriteHigh(STEPPER_MS1_PORT, STEPPER_MS1_PIN);
      GPIO_WriteLow(STEPPER_MS2_PORT, STEPPER_MS2_PIN);
      GPIO_WriteLow(STEPPER_MS3_PORT, STEPPER_MS3_PIN);
      break;
    case 4:
      GPIO_WriteLow(STEPPER_MS1_PORT, STEPPER_MS1_PIN);
      GPIO_WriteHigh(STEPPER_MS2_PORT, STEPPER_MS2_PIN);
      GPIO_WriteLow(STEPPER_MS3_PORT, STEPPER_MS3_PIN);
      break;
    case 8:
      GPIO_WriteHigh(STEPPER_MS1_PORT, STEPPER_MS1_PIN);
      GPIO_WriteHigh(STEPPER_MS2_PORT, STEPPER_MS2_PIN);
      GPIO_WriteLow(STEPPER_MS3_PORT, STEPPER_MS3_PIN);
      break;
    case 16:
      GPIO_WriteHigh(STEPPER_MS1_PORT, STEPPER_MS1_PIN);
      GPIO_WriteHigh(STEPPER_MS2_PORT, STEPPER_MS2_PIN);
      GPIO_WriteHigh(STEPPER_MS3_PORT, STEPPER_MS3_PIN);
      break;
    default:
      break;
  }
}

//----------------------------------------------------------------------------------- I2C

void i2c_Initialize(void) {
  I2C_DeInit();
  I2C_Init(I2C_MAX_STANDARD_FREQ, (I2C_ADDRESS << 1), I2C_DUTYCYCLE_2,
           I2C_ACK_CURR, I2C_ADDMODE_7BIT, (CLK_GetClockFreq() / 1000000));
  I2C_StretchClockCmd(ENABLE);
  I2C_Cmd(ENABLE);
}

uint8_t i2c_Update(void) {
  uint16_t event = 0;
  uint8_t newByte = 0;
  bool addrAcquired = FALSE;
  uint8_t dataIndex = 0;
  uint8_t dataBytesReceived = 0;

  event = I2C_GetLastEvent();
  if (I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED == event) {
    while (TRUE) {
      disableInterrupts();
      event = I2C_GetLastEvent();
      if (I2C_EVENT_SLAVE_STOP_DETECTED == event) {
        I2C_AcknowledgeConfig(I2C_ACK_CURR);
        enableInterrupts();
        return dataBytesReceived;
      } else if (I2C_EVENT_SLAVE_BYTE_RECEIVED == event) {
        newByte = I2C_ReceiveData();
        enableInterrupts();
        if (FALSE == addrAcquired) {
          dataIndex = newByte;
          addrAcquired = TRUE;
        } else if (STEPPER_CONTROLLER_DATA_SIZE > (dataIndex + dataBytesReceived)) {
          stepperControllerData[dataIndex + dataBytesReceived] = newByte;
          dataBytesReceived++;
        }
      }
    }
  }

  return dataBytesReceived;
}

//----------------------------------------------------------------------------------- MAIN


void main(void) {
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

  i2c_Initialize();
  stepper_Initialize();

  enableInterrupts();

  while (TRUE) {
    i2c_dataSize = i2c_Update();
    if (i2c_dataSize) {
      stepsPerSecondA = (stepperControllerData[0] << 8) | stepperControllerData[1];
      stepsPerSecondB = (stepperControllerData[2] << 8) | stepperControllerData[3];

      stepper_SetSpeed(stepsPerSecondA, stepsPerSecondB);
      stepper_SetEnabled(stepperControllerData[4]);
      stepper_SetMicroStepping(stepperControllerData[5]);
    }
  }
}

