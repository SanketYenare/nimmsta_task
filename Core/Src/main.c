/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @description	: Programming task to play buzzer tone at startup and consecutive
  * 				  three musical notes C6, D6 and E6 after Push button interrupt
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

// Timer base handler structure object
TIM_HandleTypeDef htim15;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM15_Init(void);

// PUSH BUTTON INTERRUPT FLAG
static unsigned char PUSH_BUTTON_INTERRUPT = FALSE;


/**
  * @brief  : Application entry point.
  * Initialization of System clock, GPIOs, Timer. A test tone is played after the initialization.
  *
  * @retval int
  */
int main(void)
{

  // Initialize system peripherals
  SYSTEM_INIT();
  // Issue test tone
  TIMER_TEST_TONE();


  /*
   * PUSH BUTTON is treated as an external hardware interrupt connected to GPIO - PC5.
   * As the button in pressed, interrupt is caught at the RISING EDGE.
   * In the while() a PUSH_BUTTON_INTERRUPT flag is polled.
   * */
  while (1)
  {
		//Enter Low-power mode - STOP mode.
		STM_ENTER_LPM();


	    if(PUSH_BUTTON_INTERRUPT)
	    {
		    //Load timer auto-reload register with note frequency
		    TIM15->ARR = TIM15_PWM_FREQ_C6;				//C6 Note frequency

			// Enable Buzzer-logic circuit
			HAL_GPIO_WritePin(CPU_BUZZER_DISABLE_OUT_GPIO_Port, CPU_BUZZER_DISABLE_OUT_Pin, GPIO_PIN_SET);
			// Enable H-bridge functioning
			HAL_GPIO_WritePin(CPU_BUZZER_SLEEP_OUT_GPIO_Port, CPU_BUZZER_SLEEP_OUT_Pin, GPIO_PIN_SET);

			// start Timer PWM
			HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
			HAL_Delay(500);								//wait 500ms

			TIM15->ARR = TIM15_PWM_FREQ_D6;				//D6 Note frequency

			HAL_Delay(500);								//wait 500ms

			TIM15->ARR = TIM15_PWM_FREQ_E6;				//E6 Note frequency

			HAL_Delay(500);								//wait 500ms

			// Disable H-bridge
			HAL_GPIO_WritePin(CPU_BUZZER_SLEEP_OUT_GPIO_Port, CPU_BUZZER_SLEEP_OUT_Pin, GPIO_PIN_SET);

			//Stop Timer PWM
			HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim15, TIM_CHANNEL_1);

			// Disable Buzzer-logic circuit
			HAL_GPIO_WritePin(CPU_BUZZER_DISABLE_OUT_GPIO_Port, CPU_BUZZER_DISABLE_OUT_Pin, GPIO_PIN_SET);

			PUSH_BUTTON_INTERRUPT = FALSE;		//RESET push button flag

	    }//END if

  }//END while()

} //END main()



/*
 * brief@  Function to setup controller modules
 * 			- System Clock setup
 * 			- GPIO setup
 * 			- Timer Setup
 * */
void SYSTEM_INIT()
{

	HAL_Init();

	//Initialize clock
	SystemClock_Config();

	// Initialize GPIOs
	MX_GPIO_Init();

	//Initialize Timer
	MX_TIM15_Init();

}

/*
 * @brief:	Enter Low-power mode - STOP mode
 * 			- Low power regulator ON
 * 			- CPU - OFF
 * 			- GPIO enabled for Interrupt
 * 			- WAKE-UP by External interrupt
 *
 * */
void STM_ENTER_LPM()
{

	//Enter STOP_MODE 1 - LOW POWR REGULATOR for Lower power consumption. Trade-off is Delayed Wake-up time
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}



/**
  * @brief System Clock Configuration
  * @return value - None
  */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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

  /*
   * Disable Low-speed External clock to use PC14 as a standard GPIO pin
   */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_LSE_CONFIG(RCC_LSE_OFF);			//Low-speed External Oscillator Switched OFF

}

/**
	* @brief TIM15 Initialization Function
	* Configuring TIMER 15 as PWM generator.
	* Generating Complimentary PWM on CHANNEL 1 Pin: PG9 and PG10
	* PWM frequency = CLK FREQ / PERIOD = 16 MHz / 10000 = 16000 Hz
	* AutoRelode Register = ENABLE - do not have to reset the timer to zero during runtime
	*
	* @param None
	* @retval None
  */
static void MX_TIM15_Init(void)
{


  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 10000;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 1;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;								//Duty cycle = 50%
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 10;								// Dead time insertion for complementary PWM
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CPU_BUZZER_DISABLE_OUT_GPIO_Port, CPU_BUZZER_DISABLE_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CPU_BUZZER_SLEEP_OUT_GPIO_Port, CPU_BUZZER_SLEEP_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CPU_BUZZER_DISABLE_OUT_Pin */
  GPIO_InitStruct.Pin = CPU_BUZZER_DISABLE_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CPU_BUZZER_DISABLE_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CPU_BUZZER_SLEEP_OUT_Pin */

  // Configure GPIO pin output level
  HAL_GPIO_WritePin(GPIOC, CPU_BUZZER_DISABLE_OUT_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = CPU_BUZZER_SLEEP_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CPU_BUZZER_SLEEP_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Push_Buttom_Pin */
  GPIO_InitStruct.Pin = Push_Buttom_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Push_Buttom_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}



/**
 * @brief Function to generate test tone for 880 Hz on startup
  * @param None
  * @retval None
  */
void TIMER_TEST_TONE()
{
	//Load timer auto-reload register with test-tone frequency
	TIM15->ARR = TIM15_PWM_FREQ_880;

	// Enable Buzzer-logic circuit
	HAL_GPIO_WritePin(CPU_BUZZER_DISABLE_OUT_GPIO_Port, CPU_BUZZER_DISABLE_OUT_Pin, GPIO_PIN_SET);
	// Enable H-bridge functioning
	HAL_GPIO_WritePin(CPU_BUZZER_SLEEP_OUT_GPIO_Port, CPU_BUZZER_SLEEP_OUT_Pin, GPIO_PIN_SET);


	// start Timer PWM
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);

	HAL_Delay(1000); 								//wait 1sec

	// Disable H-bridge
	HAL_GPIO_WritePin(CPU_BUZZER_SLEEP_OUT_GPIO_Port, CPU_BUZZER_SLEEP_OUT_Pin, GPIO_PIN_SET);

	// stop Timer PWM
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim15, TIM_CHANNEL_1);

	// Disable Buzzer-logic circuit
	HAL_GPIO_WritePin(CPU_BUZZER_DISABLE_OUT_GPIO_Port, CPU_BUZZER_DISABLE_OUT_Pin, GPIO_PIN_SET);


	return;
}


/*
 * brief@  GPIO_PUSH_BUTTON_PIN interrupt handler
 * retval None
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  //Check if interrupt occurred at the right PIN
  if(GPIO_Pin == Push_Buttom_Pin)
  {
	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);  //clears wake-up flag

	  //Re-initialize system peripherals
	  SYSTEM_INIT();

	  //Set button interrupt flag
	  PUSH_BUTTON_INTERRUPT = TRUE;
  }

}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
	  break;
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
