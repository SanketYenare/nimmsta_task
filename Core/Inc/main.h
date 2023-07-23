/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

// defines for boolean keywords
#define TRUE  0x00
#define FALSE 0x01

/*
 * defines for different tone frequencies
 * */
#define TIM15_PWM_FREQ_880   18181		//startup tone
#define TIM15_PWM_FREQ_C6    15281		//C6 - 1047 Hz
#define TIM15_PWM_FREQ_D6    13617		//D6 - 1175 Hz
#define TIM15_PWM_FREQ_E6    12130		//E6 - 1319 Hz

/* Test tone function declaration */
void TIMER_TEST_TONE();

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


#define CPU_BUZZER_DISABLE_OUT_Pin GPIO_PIN_1
#define CPU_BUZZER_DISABLE_OUT_GPIO_Port GPIOE
#define CPU_BUZZER_SLEEP_OUT_Pin GPIO_PIN_14
#define CPU_BUZZER_SLEEP_OUT_GPIO_Port GPIOC
#define Push_Buttom_Pin GPIO_PIN_5
#define Push_Buttom_GPIO_Port GPIOC
#define Push_Buttom_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
