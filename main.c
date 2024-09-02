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
#include "liquidcrystal_i2c.h"
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

/* USER CODE BEGIN PV */
unsigned int parking_occupied_PC5 = 0;
unsigned int parking_occupied_PC7 = 0;
unsigned int parking_occupied_PC9 = 0;
unsigned int parking_occupied_PC10 = 0;
unsigned int parking_occupied_PC13 = 0;
unsigned int parking_occupied_PC12 = 0;

unsigned int new_detection_change = 0;

unsigned int spots_ammount_left = 0;
unsigned int spots_ammount_right = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void EXTI15_10_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void calculate_guidance_direction();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void EXTI9_5_IRQHandler(void) {

    new_detection_change = 1;  // Flag to indicate a state change was detected

    // Handle interrupt for PC5
    if (EXTI->PR & (1 << 5)) {  // Check if the interrupt is for PC5
        if (GPIOC->IDR & (1 << 5)) {  // Rising edge detected (car leaving)
            parking_occupied_PC5 = 0;  // Mark parking space at PC5 as available
        } else {  // Falling edge detected (car approaching)
            parking_occupied_PC5 = 1;  // Mark parking space at PC5 as occupied
        }
        EXTI->PR |= (1 << 5);  // Clear the interrupt flag for PC5
    }

    // Handle interrupt for PC7
    if (EXTI->PR & (1 << 7)) {  // Check if the interrupt is for PC7
        if (GPIOC->IDR & (1 << 7)) {  // Rising edge detected (car leaving)
            parking_occupied_PC7 = 0;  // Mark parking space at PC7 as available
        } else {  // Falling edge detected (car approaching)
            parking_occupied_PC7 = 1;  // Mark parking space at PC7 as occupied
        }
        EXTI->PR |= (1 << 7);  // Clear the interrupt flag for PC7
    }

    // Handle interrupt for PC9
    if (EXTI->PR & (1 << 9)) {  // Check if the interrupt is for PC9
        if (GPIOC->IDR & (1 << 9)) {  // Rising edge detected (car leaving)
            parking_occupied_PC9 = 0;  // Mark parking space at PC9 as available
        } else {  // Falling edge detected (car approaching)
            parking_occupied_PC9 = 1;  // Mark parking space at PC9 as occupied
        }
        EXTI->PR |= (1 << 9);  // Clear the interrupt flag for PC9
    }
}





void EXTI15_10_IRQHandler(void) {

    // Flag a change in detection status
    new_detection_change = 1;

    // Handle interrupt for PC10
    if (EXTI->PR & (1 << 10)) {  // Check if the interrupt is triggered for PC10
        if (GPIOC->IDR & (1 << 10)) {  // Rising edge detected (car departing)
            parking_occupied_PC10 = 0;  // Mark parking space at PC10 as unoccupied
        } else {  // Falling edge detected (car arriving)
            parking_occupied_PC10 = 1;  // Mark parking space at PC10 as occupied
        }
        EXTI->PR |= (1 << 10);  // Clear the interrupt flag for PC10
    }

    // Handle interrupt for PC12
    if (EXTI->PR & (1 << 12)) {  // Check if the interrupt is triggered for PC12
        if (GPIOC->IDR & (1 << 12)) {  // Rising edge detected (car departing)
            parking_occupied_PC12 = 0;  // Mark parking space at PC12 as unoccupied
        } else {  // Falling edge detected (car arriving)
            parking_occupied_PC12 = 1;  // Mark parking space at PC12 as occupied
        }
        EXTI->PR |= (1 << 12);  // Clear the interrupt flag for PC12
    }

    // Handle interrupt for PC13
    if (EXTI->PR & (1 << 13)) {  // Check if the interrupt is triggered for PC13
        if (GPIOC->IDR & (1 << 13)) {  // Rising edge detected (car departing)
            parking_occupied_PC13 = 0;  // Mark parking space at PC13 as unoccupied
        } else {  // Falling edge detected (car arriving)
            parking_occupied_PC13 = 1;  // Mark parking space at PC13 as occupied
        }
        EXTI->PR |= (1 << 13);  // Clear the interrupt flag for PC13
    }
}

void calculate_guidance_direction(){
	spots_ammount_left = parking_occupied_PC9 + parking_occupied_PC7 + parking_occupied_PC5;
	spots_ammount_right = parking_occupied_PC10 + parking_occupied_PC12 + parking_occupied_PC13;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize all configured peripherals */  /***********************************************************************************************************/

  //********************* Configure pins as output (LEDs) *************************

  // Configure PA0 (LED) as output (01)
  GPIOA->MODER &= ~(3 << (0*2));  // Clear bits 0 and 1
  GPIOA->MODER |= (1 << (0*2));   // Set PA0 as output

  // Configure PA1 (LED) as output (01)
  GPIOA->MODER &= ~(3 << (1*2));  // Clear bits 2 and 3
  GPIOA->MODER |= (1 << (1*2));   // Set PA1 as output

  // Configure PA4 (LED) as output (01)
  GPIOA->MODER &= ~(1 << (4*2 + 1)); // Clear bit 9
  GPIOA->MODER |= (1 << (4*2));     // Set bit 8 to configure PA4 as output

  // Configure PA5 (LED) as output (01)
  GPIOA->MODER &= ~(3 << (5*2));   // Clear bits 10 and 11
  GPIOA->MODER |= (1 << (5*2));    // Set bit 10 to configure PA5 as output

  // Configure PA8 (LED) as output (01)
  GPIOA->MODER &= ~(1 << (8*2 + 1)); // Clear bit 17
  GPIOA->MODER |= (1 << (8*2));     // Set bit 16 to configure PA8 as output

  // Configure PA9 (LED) as output (01)
  GPIOA->MODER &= ~(3 << (9*2));   // Clear bits 18 and 19
  GPIOA->MODER |= (1 << (9*2));    // Set bit 18 to configure PA9 as output

  //********************* Configure pins as input (Flying Fish sensors) *************************

  // Configure PC5 (Sensor Flying Fish) as input (00)
  GPIOC->MODER &= ~(3 << (5*2));  // Clear bits 10 and 11 to set PC5 as input

  // Configure PC7 (Sensor Flying Fish) as input (00)
  GPIOC->MODER &= ~(3 << (7*2));  // Clear bits 14 and 15 to set PC7 as input

  // Configure PC9 (Sensor Flying Fish) as input (00)
  GPIOC->MODER &= ~(3 << (9*2));  // Clear bits 18 and 19 to set PC9 as input

  // Configure PC10 (Sensor Flying Fish) as input (00)
  GPIOC->MODER &= ~(3 << (10*2));  // Clear bits 20 and 21 to set PC10 as input

  // Configure PC12 (Sensor Flying Fish) as input (00)
  GPIOC->MODER &= ~(1 << (12*2 + 1)); // Clear bit 25
  GPIOC->MODER &= ~(1 << (12*2));    // Clear bit 24 to configure PC12 as input

  // Configure PC13 (Sensor Flying Fish) as input (00)
  GPIOC->MODER &= ~(3 << (13*2));  // Clear bits 26 and 27 to configure PC13 as input

  // EXTI5 configuration for both rising and falling edges
  EXTI->RTSR |= (1 << 5);  // Enable rising edge trigger for EXTI5 (PC5)
  EXTI->FTSR |= (1 << 5);  // Enable falling edge trigger for EXTI5 (PC5)

  // EXTI7 configuration for both rising and falling edges
  EXTI->RTSR |= (1 << 7);  // Enable rising edge trigger for EXTI7 (PC7)
  EXTI->FTSR |= (1 << 7);  // Enable falling edge trigger for EXTI7 (PC7)

  // EXTI9 configuration for both rising and falling edges
  EXTI->RTSR |= (1 << 9);  // Enable rising edge trigger for EXTI9 (PC9)
  EXTI->FTSR |= (1 << 9);  // Enable falling edge trigger for EXTI9 (PC9)

  // EXTI10 configuration for both rising and falling edges
  EXTI->RTSR |= (1 << 10);  // Enable rising edge trigger for EXTI10 (PC10)
  EXTI->FTSR |= (1 << 10);  // Enable falling edge trigger for EXTI10 (PC10)

  // EXTI12 configuration for both rising and falling edges
  EXTI->RTSR |= (1 << 12);  // Enable rising edge trigger for EXTI12 (PC12)
  EXTI->FTSR |= (1 << 12);  // Enable falling edge trigger for EXTI12 (PC12)

  // EXTI13 configuration for both rising and falling edges
  EXTI->RTSR |= (1 << 13);  // Enable rising edge trigger for EXTI13 (PC13)
  EXTI->FTSR |= (1 << 13);  // Enable falling edge trigger for EXTI13 (PC13)

  // SYSCFG configuration to associate EXTI5 with GPIOC
  SYSCFG->EXTICR[1] &= ~(0xF << 4);  // Clear bits [7:4] (EXTI5 field)
  SYSCFG->EXTICR[1] |= (0x2 << 4);   // Set bits [7:4] to 0010 to associate EXTI5 with GPIOC

  // SYSCFG configuration to associate EXTI7 with GPIOC
  SYSCFG->EXTICR[1] &= ~(0xF << 12);  // Clear bits [15:12] (EXTI7 field)
  SYSCFG->EXTICR[1] |= (0x2 << 12);   // Set bits [15:12] to 0010 to associate EXTI7 with GPIOC

  // SYSCFG configuration to associate EXTI9 with GPIOC
  SYSCFG->EXTICR[2] &= ~(0xF << 4);  // Clear bits [7:4] (EXTI9 field)
  SYSCFG->EXTICR[2] |= (0x2 << 4);   // Set bits [7:4] to 0010 to associate EXTI9 with GPIOC

  // SYSCFG configuration to associate EXTI10 with GPIOC
  SYSCFG->EXTICR[2] &= ~(0xF << 8);  // Clear bits [11:8] (EXTI10 field)
  SYSCFG->EXTICR[2] |= (0x2 << 8);   // Set bits [11:8] to 0010 to associate EXTI10 with GPIOC

  // SYSCFG configuration to associate EXTI12 with GPIOC
  SYSCFG->EXTICR[3] &= ~(0xF << 0);  // Clear bits [3:0] (EXTI12 field)
  SYSCFG->EXTICR[3] |= (0x2 << 0);   // Set bits [3:0] to 0010 to associate EXTI12 with GPIOC

  // SYSCFG configuration to associate EXTI13 with GPIOC
  SYSCFG->EXTICR[3] &= ~(0xF << 4);  // Clear bits [7:4] (EXTI13 field)
  SYSCFG->EXTICR[3] |= (0x2 << 4);   // Set bits [7:4] to 0010 to associate EXTI13 with GPIOC

  // Unmask EXTI interrupts
  EXTI->IMR |= (1 << 5);  // Unmask EXTI5
  EXTI->IMR |= (1 << 7);  // Unmask EXTI7
  EXTI->IMR |= (1 << 9);  // Unmask EXTI9
  EXTI->IMR |= (1 << 10); // Unmask EXTI10
  EXTI->IMR |= (1 << 12);  // Unmask EXTI12
  EXTI->IMR |= (1 << 13);  // Unmask EXTI13

  // NVIC configuration to enable EXTI interrupts
  NVIC->ISER[0] |= (1 << 23);  // Enable EXTI9_5 interrupt in NVIC (position 23)
  NVIC->ISER[1] |= (1 << (40-32));  // Enable EXTI15_10 interrupt in NVIC (position 40)

  MX_GPIO_Init();
  MX_I2C1_Init();



  HD44780_Init(2);
  HD44780_Clear();
  HD44780_Backlight();
  HD44780_Clear();
  HD44780_SetBacklight(255);


   spots_ammount_left = 0;
   spots_ammount_right = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {

    	if(new_detection_change == 1){
    		calculate_guidance_direction();
    	}


    	if(parking_occupied_PC5){
    		GPIOA->BSRR = (1 << 0); // LED PA0 ON
    		HD44780_SetCursor(0,0);
    		HD44780_PrintStr("P1");
    	} else {
    		GPIOA->BSRR = (1 << 0) << 16; // LED PA0 OFF
    		HD44780_SetCursor(0,0);
    		HD44780_PrintStr("  ");
    	}

    	if(parking_occupied_PC7){
    		GPIOA->BSRR = (1 << 1); // LED PA1 ON
    		HD44780_SetCursor(4,0);
    		HD44780_PrintStr("P2");
    	} else {
    		GPIOA->BSRR = (1 << 1) << 16; // LED PA1 OFF
    		HD44780_SetCursor(4,0);
    		HD44780_PrintStr("  ");
    	}

    	if(parking_occupied_PC9){
    		GPIOA->BSRR = (1 << 9); // LED PA9 ON
    		HD44780_SetCursor(8,0);
    		HD44780_PrintStr("P3");
    	} else {
    		GPIOA->BSRR = (1 << 9) << 16; // LED PA9 OFF
    		HD44780_SetCursor(8,0);
    		HD44780_PrintStr("  ");
    	}

    	if(parking_occupied_PC10){
    		GPIOA->BSRR = (1 << 8); // LED PA8 ON
    		HD44780_SetCursor(0,1);
    		HD44780_PrintStr("P4");
    	} else {
    		GPIOA->BSRR = (1 << 8) << 16; // LED PA8 OFF
    		HD44780_SetCursor(0,1);
    		HD44780_PrintStr("  ");
    	}

    	if(parking_occupied_PC12){
    		GPIOA->BSRR = (1 << 4); // LED PA4 ON
    		HD44780_SetCursor(4,1);
    		HD44780_PrintStr("P5");
    	} else {
    		GPIOA->BSRR = (1 << 4) << 16; // LED PA4 OFF
    		HD44780_SetCursor(4,1);
    		HD44780_PrintStr("  ");
    	}


    	if(parking_occupied_PC13) {
    		GPIOA->BSRR = (1 << 5); // LED PA5 ON

    		HD44780_SetCursor(8,1);
    		HD44780_PrintStr("P6");
    	} else {
    		GPIOA->BSRR = (1 << 5) << 16; // LED PA5 OFF
    		HD44780_SetCursor(8,1);
    		HD44780_PrintStr("  ");
    	}



    	if(spots_ammount_left>spots_ammount_right){
    		HD44780_SetCursor(10,0);
    		HD44780_PrintStr("---->");
    		HD44780_SetCursor(10,1);
    		HD44780_PrintStr("---->");
    	}else if(spots_ammount_left<spots_ammount_right){
    		HD44780_SetCursor(10,0);
    		HD44780_PrintStr("<----");
    		HD44780_SetCursor(10,1);
    		HD44780_PrintStr("<----");
    	}else{
    		HD44780_SetCursor(10,0);
    		HD44780_PrintStr("- - -");
    		HD44780_SetCursor(10,1);
    		HD44780_PrintStr("- - -");
    	}



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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.OwnAddress1 = 0;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
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
