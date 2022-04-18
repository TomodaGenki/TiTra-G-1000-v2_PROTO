/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#ifndef TRUE
	#define TRUE		(1)
#endif
#ifndef FALSE
	#define FALSE		(0)
#endif

#define	FILTER_ENABLE	1
#define	FILTER_DISABLE	0
#define	WHEEL_ENC_FILTER	FILTER_ENABLE
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void int_1msCallback(void);
uint8_t get_fw_version(uint8_t);
uint16_t get_turn_encoder(void);
uint16_t get_l_wheel_enc_cnt(void);
uint16_t get_r_wheel_enc_cnt(void);
uint32_t get_turn_enc_cnt(void);
void enc_pulse_tim_cnt_reset(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define O_OptIn4_Pin GPIO_PIN_2
#define O_OptIn4_GPIO_Port GPIOE
#define O_OptIn3_Pin GPIO_PIN_3
#define O_OptIn3_GPIO_Port GPIOE
#define O_OptIn2_Pin GPIO_PIN_4
#define O_OptIn2_GPIO_Port GPIOE
#define O_OptIn1_Pin GPIO_PIN_5
#define O_OptIn1_GPIO_Port GPIOE
#define O_Wheel2Speed_Pin GPIO_PIN_6
#define O_Wheel2Speed_GPIO_Port GPIOE
#define O_BlueLed_R_Pin GPIO_PIN_8
#define O_BlueLed_R_GPIO_Port GPIOI
#define INT_Wheel1Pulse_Pin GPIO_PIN_13
#define INT_Wheel1Pulse_GPIO_Port GPIOC
#define INT_Wheel1Pulse_EXTI_IRQn EXTI15_10_IRQn
#define I_Wheel1Dir_Pin GPIO_PIN_14
#define I_Wheel1Dir_GPIO_Port GPIOC
#define INT_Wheel2Pulse_Pin GPIO_PIN_15
#define INT_Wheel2Pulse_GPIO_Port GPIOC
#define INT_Wheel2Pulse_EXTI_IRQn EXTI15_10_IRQn
#define O_Wheel1Reset_Pin GPIO_PIN_2
#define O_Wheel1Reset_GPIO_Port GPIOF
#define O_WheelDCLock_Pin GPIO_PIN_3
#define O_WheelDCLock_GPIO_Port GPIOF
#define O_Wheel2CW_Pin GPIO_PIN_4
#define O_Wheel2CW_GPIO_Port GPIOF
#define O_Wheel2CCW_Pin GPIO_PIN_5
#define O_Wheel2CCW_GPIO_Port GPIOF
#define O_Wheel2Reset_Pin GPIO_PIN_6
#define O_Wheel2Reset_GPIO_Port GPIOF
#define I_TurnBrakeSw_F_Pin GPIO_PIN_7
#define I_TurnBrakeSw_F_GPIO_Port GPIOF
#define O_Wheel1Load_Pin GPIO_PIN_8
#define O_Wheel1Load_GPIO_Port GPIOF
#define I_Wheel1Error_Pin GPIO_PIN_9
#define I_Wheel1Error_GPIO_Port GPIOF
#define O_BlueLed_L_Pin GPIO_PIN_10
#define O_BlueLed_L_GPIO_Port GPIOF
#define I_BrakeReleSw_F_Pin GPIO_PIN_0
#define I_BrakeReleSw_F_GPIO_Port GPIOC
#define I_ResetSw_F_Pin GPIO_PIN_1
#define I_ResetSw_F_GPIO_Port GPIOC
#define I_LiftSw_F_Pin GPIO_PIN_0
#define I_LiftSw_F_GPIO_Port GPIOA
#define O_Wheel1Speed_Pin GPIO_PIN_1
#define O_Wheel1Speed_GPIO_Port GPIOA
#define O_AlarmCh1_Pin GPIO_PIN_2
#define O_AlarmCh1_GPIO_Port GPIOH
#define O_Wheel_STO_Pin GPIO_PIN_3
#define O_Wheel_STO_GPIO_Port GPIOH
#define O_AlarmCh3_Pin GPIO_PIN_4
#define O_AlarmCh3_GPIO_Port GPIOH
#define O_AlarmCh4_Pin GPIO_PIN_5
#define O_AlarmCh4_GPIO_Port GPIOH
#define DA2_TurnSpeed_Pin GPIO_PIN_5
#define DA2_TurnSpeed_GPIO_Port GPIOA
#define I_TurnBrakeSw_R_Pin GPIO_PIN_6
#define I_TurnBrakeSw_R_GPIO_Port GPIOA
#define I_Bumper1_Pin GPIO_PIN_7
#define I_Bumper1_GPIO_Port GPIOA
#define I_Bumber2_Pin GPIO_PIN_4
#define I_Bumber2_GPIO_Port GPIOC
#define O_TurnStopMode_Pin GPIO_PIN_5
#define O_TurnStopMode_GPIO_Port GPIOC
#define O_TableLed_Pin GPIO_PIN_0
#define O_TableLed_GPIO_Port GPIOB
#define O_LiftBrake_Pin GPIO_PIN_1
#define O_LiftBrake_GPIO_Port GPIOB
#define O_RedLed_L_Pin GPIO_PIN_11
#define O_RedLed_L_GPIO_Port GPIOF
#define O_GreenLed_L_Pin GPIO_PIN_12
#define O_GreenLed_L_GPIO_Port GPIOF
#define O_Lift1AlarmReset_Pin GPIO_PIN_13
#define O_Lift1AlarmReset_GPIO_Port GPIOF
#define I_Wheel2Dir_Pin GPIO_PIN_14
#define I_Wheel2Dir_GPIO_Port GPIOF
#define O_LiftStopMode_Pin GPIO_PIN_15
#define O_LiftStopMode_GPIO_Port GPIOF
#define O_Lift1CCW_Pin GPIO_PIN_0
#define O_Lift1CCW_GPIO_Port GPIOG
#define O_Lift1CW_Pin GPIO_PIN_1
#define O_Lift1CW_GPIO_Port GPIOG
#define I_BrakeReleSw_R_Pin GPIO_PIN_7
#define I_BrakeReleSw_R_GPIO_Port GPIOE
#define I_Turn0degSens_Pin GPIO_PIN_8
#define I_Turn0degSens_GPIO_Port GPIOE
#define I_Turn90degSens_Pin GPIO_PIN_9
#define I_Turn90degSens_GPIO_Port GPIOE
#define I_Turn180degSens_Pin GPIO_PIN_10
#define I_Turn180degSens_GPIO_Port GPIOE
#define I_Turn270degSens_Pin GPIO_PIN_11
#define I_Turn270degSens_GPIO_Port GPIOE
#define I_Lift2UpLimitSens_Pin GPIO_PIN_12
#define I_Lift2UpLimitSens_GPIO_Port GPIOE
#define I_Lift2UpSens_Pin GPIO_PIN_13
#define I_Lift2UpSens_GPIO_Port GPIOE
#define I_Lift2DownSens_Pin GPIO_PIN_14
#define I_Lift2DownSens_GPIO_Port GPIOE
#define I_Lift2DownLimitSens_Pin GPIO_PIN_15
#define I_Lift2DownLimitSens_GPIO_Port GPIOE
#define I_Lift1UpLimitSens_Pin GPIO_PIN_6
#define I_Lift1UpLimitSens_GPIO_Port GPIOH
#define I_Lift1UpSens_Pin GPIO_PIN_7
#define I_Lift1UpSens_GPIO_Port GPIOH
#define I_Lift1DownSens_Pin GPIO_PIN_8
#define I_Lift1DownSens_GPIO_Port GPIOH
#define I_Lift1DownLimitSens_Pin GPIO_PIN_9
#define I_Lift1DownLimitSens_GPIO_Port GPIOH
#define O_AddRelay_Pin GPIO_PIN_10
#define O_AddRelay_GPIO_Port GPIOH
#define O_Lidar1In4_Pin GPIO_PIN_11
#define O_Lidar1In4_GPIO_Port GPIOH
#define O_Lidar1In5_Pin GPIO_PIN_12
#define O_Lidar1In5_GPIO_Port GPIOH
#define I_Lift1Alm_Pin GPIO_PIN_12
#define I_Lift1Alm_GPIO_Port GPIOB
#define O_Lift2AlarmReset_Pin GPIO_PIN_14
#define O_Lift2AlarmReset_GPIO_Port GPIOB
#define I_EmergencyMoni_Pin GPIO_PIN_15
#define I_EmergencyMoni_GPIO_Port GPIOB
#define INT_TurnEncorder_Pin GPIO_PIN_9
#define INT_TurnEncorder_GPIO_Port GPIOD
#define INT_TurnEncorder_EXTI_IRQn EXTI9_5_IRQn
#define I_Wheel2Error_Pin GPIO_PIN_10
#define I_Wheel2Error_GPIO_Port GPIOD
#define I_TurnAlm_Pin GPIO_PIN_11
#define I_TurnAlm_GPIO_Port GPIOD
#define O_TurnAlarmReset_Pin GPIO_PIN_12
#define O_TurnAlarmReset_GPIO_Port GPIOD
#define O_TurnBrake_Pin GPIO_PIN_13
#define O_TurnBrake_GPIO_Port GPIOD
#define O_TurnCCW_Pin GPIO_PIN_14
#define O_TurnCCW_GPIO_Port GPIOD
#define O_TurnCW_Pin GPIO_PIN_15
#define O_TurnCW_GPIO_Port GPIOD
#define O_BrakeRelay_Pin GPIO_PIN_2
#define O_BrakeRelay_GPIO_Port GPIOG
#define O_LiftRelay_Pin GPIO_PIN_3
#define O_LiftRelay_GPIO_Port GPIOG
#define O_TurnRelay_Pin GPIO_PIN_4
#define O_TurnRelay_GPIO_Port GPIOG
#define O_DriveRelay_Pin GPIO_PIN_5
#define O_DriveRelay_GPIO_Port GPIOG
#define O_ChargeRelay_Pin GPIO_PIN_6
#define O_ChargeRelay_GPIO_Port GPIOG
#define I_Lift1Tim_Pin GPIO_PIN_7
#define I_Lift1Tim_GPIO_Port GPIOG
#define I_Lift1Tim_EXTI_IRQn EXTI9_5_IRQn
#define I_Lift2Tim_Pin GPIO_PIN_8
#define I_Lift2Tim_GPIO_Port GPIOG
#define I_Lift2Tim_EXTI_IRQn EXTI9_5_IRQn
#define I_Lift2Alm_Pin GPIO_PIN_6
#define I_Lift2Alm_GPIO_Port GPIOC
#define O_GreenLed_R_Pin GPIO_PIN_7
#define O_GreenLed_R_GPIO_Port GPIOC
#define O_RedLed_R_Pin GPIO_PIN_8
#define O_RedLed_R_GPIO_Port GPIOC
#define O_Wheel1CCW_Pin GPIO_PIN_9
#define O_Wheel1CCW_GPIO_Port GPIOA
#define O_Wheel1CW_Pin GPIO_PIN_10
#define O_Wheel1CW_GPIO_Port GPIOA
#define O_Lift2CCW_Pin GPIO_PIN_11
#define O_Lift2CCW_GPIO_Port GPIOA
#define O_Lift2CW_Pin GPIO_PIN_12
#define O_Lift2CW_GPIO_Port GPIOA
#define O_Lidar1In3_Pin GPIO_PIN_13
#define O_Lidar1In3_GPIO_Port GPIOH
#define O_Lidar1In2_Pin GPIO_PIN_14
#define O_Lidar1In2_GPIO_Port GPIOH
#define I_Lidar1Trouble_Pin GPIO_PIN_15
#define I_Lidar1Trouble_GPIO_Port GPIOH
#define I_Lidar1Out3_Pin GPIO_PIN_0
#define I_Lidar1Out3_GPIO_Port GPIOI
#define O_Wheel2Load_Pin GPIO_PIN_15
#define O_Wheel2Load_GPIO_Port GPIOA
#define I_TurnEncoderBch_Pin GPIO_PIN_12
#define I_TurnEncoderBch_GPIO_Port GPIOC
#define O_Lidar1In1_Pin GPIO_PIN_2
#define O_Lidar1In1_GPIO_Port GPIOD
#define O_Lidar2In5_Pin GPIO_PIN_3
#define O_Lidar2In5_GPIO_Port GPIOD
#define O_Lidar2In4_Pin GPIO_PIN_4
#define O_Lidar2In4_GPIO_Port GPIOD
#define O_Lidar2In3_Pin GPIO_PIN_5
#define O_Lidar2In3_GPIO_Port GPIOD
#define O_Lidar2In2_Pin GPIO_PIN_6
#define O_Lidar2In2_GPIO_Port GPIOD
#define O_Lidar2In1_Pin GPIO_PIN_7
#define O_Lidar2In1_GPIO_Port GPIOD
#define I_Lidar1Out2_Pin GPIO_PIN_10
#define I_Lidar1Out2_GPIO_Port GPIOG
#define I_Lidar1Out1_Pin GPIO_PIN_11
#define I_Lidar1Out1_GPIO_Port GPIOG
#define I_Lidar2Trouble_Pin GPIO_PIN_12
#define I_Lidar2Trouble_GPIO_Port GPIOG
#define I_Lidar2Out3_Pin GPIO_PIN_13
#define I_Lidar2Out3_GPIO_Port GPIOG
#define I_Lidar2Out1_Pin GPIO_PIN_15
#define I_Lidar2Out1_GPIO_Port GPIOG
#define I_LiftSw_R_Pin GPIO_PIN_3
#define I_LiftSw_R_GPIO_Port GPIOB
#define I_ResetSw_R_Pin GPIO_PIN_4
#define I_ResetSw_R_GPIO_Port GPIOB
#define I_OptGo_Pin GPIO_PIN_5
#define I_OptGo_GPIO_Port GPIOB
#define I_OptOut4_Pin GPIO_PIN_6
#define I_OptOut4_GPIO_Port GPIOB
#define I_OptOut3_Pin GPIO_PIN_7
#define I_OptOut3_GPIO_Port GPIOB
#define I_OptOut2_Pin GPIO_PIN_8
#define I_OptOut2_GPIO_Port GPIOB
#define I_OptOut1_Pin GPIO_PIN_9
#define I_OptOut1_GPIO_Port GPIOB
#define O_OptSel_Pin GPIO_PIN_0
#define O_OptSel_GPIO_Port GPIOE
#define O_OptMode_Pin GPIO_PIN_1
#define O_OptMode_GPIO_Port GPIOE
#define I_Lidar2Out2_Pin GPIO_PIN_4
#define I_Lidar2Out2_GPIO_Port GPIOI
#define O_USART6_TR_Pin GPIO_PIN_5
#define O_USART6_TR_GPIO_Port GPIOI
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
