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
#define DEBOUNCE_DELAY 50  // Retardo de debounce en milisegundos
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/***********************************************************************************************************/
int time_measurement_1;              // Stores the start time of the event for time measurement
int time_measurement_2;              // Stores the end time or duration of the event for time measurement
int trigger_pulse_value;             // Value indicating the status of the trigger pulse
int new_distance_is_measured;        // Flag indicating whether a new distance has been measured
unsigned int entrance_gate_distance; // Distance measured at the entrance gate

unsigned int exit_gate_car = 0;      // Indicates the status of the car at the exit gate (0 for absent, 1 for present)
short DutyCycle;                     // Duty cycle for PWM control, used with timers for servo control

unsigned int available_parking_spaces = 6; // Number of available parking spaces

unsigned int entrance_button_pressed = 0;  // Flag to check if the entrance button has been pressed
unsigned int exit_button_pressed = 0;      // Flag to check if the exit button has been pressed

unsigned int current_time = 0;             // Variable to store the current time

/***********************************************************************************************************/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TS_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
/***********************************************************************************************************/
void servo_1_00_degrees();
void servo_1_45_degrees();
void servo_1_90_degrees();

void buzzer_toggleing();
void buzzer_sound_on();
void buzzer_sound_off();


void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);

char* int_to_string(unsigned int number);
/***********************************************************************************************************/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/***********************************************************************************************************/


void servo_1_00_degrees(){
	DutyCycle= 50;
	TIM4->CCR1 = DutyCycle;
}

void servo_1_45_degrees(){
	DutyCycle = 75;
	TIM4->CCR1 = DutyCycle;
}

void servo_1_90_degrees(){
	DutyCycle = 100;
	TIM4->CCR1 = DutyCycle;
}
void servo_2_00_degrees(){
	DutyCycle= 50;
	TIM4->CCR2 = DutyCycle;
}

void servo_2_45_degrees(){
	DutyCycle = 75;
	TIM4->CCR2 = DutyCycle;
}

void servo_2_90_degrees(){
	DutyCycle = 100;
	TIM4->CCR2 = DutyCycle;
}


void buzzer_toggleing(){
    TIM2->CCMR1 = 0x3000;     // Set CCMR1 for toggle mode to enable buzzer toggling
}

void buzzer_sound_on(){
    TIM2->CCMR1 = 0x4000;    // Configure CCMR1 to force output to 0, turning on the buzzer
}

void buzzer_sound_off(){
    TIM2->CCMR1 = 0x5000;  // Configure CCMR1 to force output to 1, turning off the buzzer
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * TIMER 2 INTERRUPT HANDLER
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void TIM2_IRQHandler(void) {
    // Handle Channel 1 interrupt for distance measurement
    if ((TIM2->SR & 0x02) == 0x02) {
        if ((GPIOA->IDR & 0x00000020) != 0) {  // Check for rising edge
            time_measurement_1 = TIM2->CCR1;  // Capture time at the start of the event
        } else {  // Falling edge detected
            time_measurement_2 = TIM2->CCR1 - time_measurement_1;  // Calculate time difference
            if (time_measurement_2 < 0) {
                time_measurement_2 += 0xFFFF;  // Adjust for timer overflow
            }
            entrance_gate_distance = time_measurement_2 / 58;  // Calculate distance
            new_distance_is_measured = 1;  // Signal that new distance is measured
        }
        TIM2->SR &= ~0x02;  // Clear the interrupt flag for Channel 1
    }

    // Handle Channel 2 interrupt for the intermittent buzzer
    if ((TIM2->SR & 0x04) == 0x04) {
        TIM2->CCR2 += 64000;  // Schedule next buzzer activation after 250 ms
        TIM2->SR &= ~0x04;  // Clear the interrupt flag for Channel 2
    }

    // Handle Channel 3 interrupt for the ultrasound trigger
    if ((TIM2->SR & 0x08) == 0x08) {
        GPIOD->BSRR |= (1 << (2 + 16));  // Deactivate the trigger (set PD2 to low)
        trigger_pulse_value = 0;  // Reset trigger pulse value
        TIM2->SR &= ~0x08;  // Clear the interrupt flag for Channel 3
    }
}




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * TIMER 3 INTERRUPT HANDLER
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void TIM3_IRQHandler(void) {
    // Check for interrupt from CHANNEL 1
    if ((TIM3->SR & 0x0002) != 0) { // Channel 1 interrupt check
        GPIOD -> BSRR |= (1 << 2);  // Trigger output on pin PD2
        trigger_pulse_value = 1;    // Set trigger pulse status to active
        TIM3->CCR1 = TIM3->CNT + 300; // Schedule next trigger after 300 ms
        TIM2->CCR3 = TIM2->CNT + 13;  // Schedule trigger end after 13 us
    }
    TIM3->SR = 0x0000;  // Clear the interrupt flag
}


void EXTI9_5_IRQHandler(void) {
    // Handle the interrupt for PC7
    if (EXTI->PR & (1 << 7)) {  // Check if the interrupt was triggered by PC7
        if (GPIOC->IDR & (1 << 7)) {  // Check for rising edge (car leaving)
            exit_gate_car = 0;  // Space is now free
        } else {  // Check for falling edge (car approaching)
            exit_gate_car = 1;  // Space is occupied
        }
        EXTI->PR |= (1 << 7);  // Clear the interrupt flag
    }
}



void EXTI15_10_IRQHandler(void) {
	// Handle interrupt for PA11
	if (EXTI->PR & (1 << 11)) {  // Check if the interrupt is triggered by PA11
		if (GPIOA->IDR & (1 << 11)) {  // Rising edge detected (button released)
			entrance_button_pressed = 0;
			GPIOA->BSRR = (1<<4)<<16; // Turn off LED
		} else {  // Falling edge detected (button pressed)
			entrance_button_pressed = 1;
			available_parking_spaces--;
			GPIOA->BSRR = (1<<4); // Turn on LED
		}
		EXTI->PR |= (1 << 11);  // Clear the interrupt flag
	}

	// Handle interrupt for PA12
	if (EXTI->PR & (1 << 12)) {  // Check if the interrupt is triggered by PA12
		if (GPIOA->IDR & (1 << 12)) {  // Rising edge detected (button released)
			exit_button_pressed = 0;
			GPIOA->BSRR = (1<<4)<<16; // Turn off LED
		} else {  // Falling edge detected (button pressed)
			exit_button_pressed = 1;
			available_parking_spaces++;
			GPIOA->BSRR = (1<<4); // Turn on LED
		}
		EXTI->PR |= (1 << 12);  // Clear the interrupt flag
	}
}


char* int_to_string(unsigned int number) {
    // Allocate memory for a string of 3 characters (two digits plus a null terminator)
    char *result = (char *)malloc(3 * sizeof(char));
    if (result == NULL) {
        // Handle the error if memory allocation fails
        return NULL;
    }

    // Format the number to two digits and store it in 'result'
    snprintf(result, 3, "%02d", number);

    return result;
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
  MX_ADC_Init();
  MX_TS_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  // * * * * * * * * * * * * * * * * * * * PIN CONFIGURATION * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

  /* * * * * * * * * * * * * * * * * * * * LED (PA4): Configured as Output * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  // Configure PA4 as output
  GPIOA->MODER |= (1 << (4*2));        // Set MODER4 to '01' to enable general purpose output mode
  GPIOA->MODER &= ~(1 << (4*2 + 1));   // Clear the adjacent bit to confirm output mode

  /* * * * * * * * * * * * * * * * * * * BUTTON (PA12): Configured as Input * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  // Configure PA12 as input
  GPIOA->MODER &= ~(1 << (12*2));      // Clear bit 24 to set MODER12 to '00' (input mode)
  GPIOA->MODER &= ~(1 << (12*2 + 1));  // Clear bit 25 to ensure the input mode is set

  // Enable pull-up resistor for PA12
  GPIOA->PUPDR |= (1 << (12*2));       // Set bit 24 to enable the pull-up resistor
  GPIOA->PUPDR &= ~(1 << (12*2 + 1));  // Clear bit 25 to configure it as pull-up

  // Enable both rising and falling edge triggers for EXTI12
  EXTI->RTSR |= (1 << 12);  // Enable rising edge trigger for EXTI12 (PA12)
  EXTI->FTSR |= (1 << 12);  // Enable falling edge trigger for EXTI12 (PA12)

  // System configuration to map EXTI12 to GPIOA
  SYSCFG->EXTICR[3] &= ~(0xF << 0);   // Clear bits [3:0] in EXTICR3 to reset previous settings
  SYSCFG->EXTICR[3] |= (0x0 << 0);    // Set bits [3:0] to map EXTI12 to GPIOA

  // Unmask EXTI12 to enable interrupts
  EXTI->IMR |= (1 << 12);  // Unmask EXTI12 to permit handling of interrupts





  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * BUTTON (PA11): Configured as Input
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  // Set PA11 as input by clearing MODER bits
  GPIOA->MODER &= ~(1 << (11*2));
  GPIOA->MODER &= ~(1 << (11*2 + 1));

  // Configure pull-up resistor for PA11
  GPIOA->PUPDR |= (1 << (11*2));
  GPIOA->PUPDR &= ~(1 << (11*2 + 1));

  // Enable rising and falling edge triggers for EXTI11
  EXTI->RTSR |= (1 << 11);  // Enable rising edge trigger for EXTI11 (PA11)
  EXTI->FTSR |= (1 << 11);  // Enable falling edge trigger for EXTI11 (PA11)

  // System configuration to associate EXTI11 with GPIOA
  SYSCFG->EXTICR[2] &= ~(0xF << 12);  // Clear bits [15:12] for EXTI11
  SYSCFG->EXTICR[2] |= (0x0 << 12);   // Set bits [15:12] to link EXTI11 with GPIOA

  // Unmask EXTI11 to enable interrupts
  EXTI->IMR |= (1 << 11);  // Unmask EXTI11 to enable its interrupts

  // NVIC configuration to enable EXTI15_10 interrupt, which includes EXTI11
  NVIC->ISER[1] |= (1 << (40-32));  // Enable EXTI15_10 in NVIC (position 40)



  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * TIMER 4 Configuration for Output on PB7 and PB6
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  // Configuring PB7 as alternate function (AF) for Timer 4 output
  short DC=75;
  GPIOB->MODER |= 0x00000001 << (2*7 + 1);  // Set PB7 to alternate function mode
  GPIOB->MODER &= ~(0x00000001 << (2*7));   // Clear previous mode setting for PB7
  GPIOB->MODER |= 0x00000001 << (2*6 + 1);  // Set PB6 to alternate function mode
  GPIOB->MODER &= ~(0x00000001 << (2*6));   // Clear previous mode setting for PB6
  GPIOB->AFR[0] |= (0x02 << (7*4));         // Link PB7 to AF2 (TIM4 Channel 2)
  GPIOB->AFR[0] |= (0x02 << (6*4));         // Link PB6 to AF2 (TIM4 Channel 1)

  // Timer base configuration (no clock source or slave mode configuration required here)
  TIM4->CR1 = 0x0080;
  TIM4->CR2 = 0x0000;
  TIM4->SMCR = 0x0000;

  // Timer counter configuration for PWM frequency
  TIM4->PSC = 639;                          // Set prescaler to 639, achieving a frequency of 50 kHz for timer operations
  TIM4->CNT = 0;                            // Start counting from 0
  TIM4->ARR = 999;                          // Set auto-reload to 999 for a 100 Hz PWM frequency

  // PWM duty cycle setup for channels
  TIM4->CCR2 = DC;                          // Initialize duty cycle for Channel 2 to 75 (7.5% of ARR)
  TIM4->CCR1 = DC;                          // Initialize duty cycle for Channel 1 to 75 (7.5% of ARR)

  // Interrupt and output mode configuration
  TIM4->DIER = 0x0000;                      // Disable all interrupts for Timer 4
  TIM4->CCMR1 = 0x6868;                     // Configure output mode for PWM on both channels, starting high with preload
  TIM4->CCER = 0x0011;                      // Enable output for channels, active high

  // Enable and initialize the timer
  TIM4->CR1 |= 0x0001;                      // Start the timer
  TIM4->EGR |= 0x0001;
  TIM4->SR = 0;                             // Clear any pending interrupt flags


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * Buzzer Configuration
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  GPIOA->MODER&=~(0x03 <<(2*1)); //00
  GPIOA->MODER|=(1<<3); //1
  GPIOA->AFR[0]|=(0x01<<(1*4));

  //* * * PD2 Ultrasounds Trigger * * * * * * * * * * * * * * * * * * * * * *
  GPIOD->MODER &=~(0x03<<(2*2)); //00
  GPIOD->MODER |=(1<<(4));

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * PC6 TIM3 Ultrasound Trigger Configuration
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  GPIOC->MODER&=~(0x03 <<(2*6)); //00
  GPIOC->MODER|=(1<<(13)); //1
  GPIOC->AFR[1]|=(0x02 <<(6*4));


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * PA5 Ultrasound Echo Pin Configuration
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  GPIOA->MODER&=~(0x03 <<(2*5));//00
  GPIOA->MODER|=(1<<(11));
  GPIOA->AFR[0]|=(0x01 <<(5*4));

  /* * * * * * * * TIMER 2 CONFIGURATION * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  TIM2->CR1 |= 0x0001;
  TIM2->CR2 = 0;
  TIM2->SMCR = 0;
  TIM2->PSC = 31; // Set prescaler to 31, resulting in a timer frequency of 1 MHz (32 MHz clock divided by 32)
  TIM2->CNT = 0;
  TIM2->ARR = 0xFFFF;
  TIM2->DIER = 0x000E;
  TIM2->CCMR1 = 0x5001;
  TIM2->CCMR2 = 0x0000;
  TIM2->CCER = 0x011B; // Set capture/compare enable register for rising and falling edge
  TIM2->CCR1=10;
  TIM2->CCR2=64000;
  TIM2->CCR3=13;
  TIM4->EGR |= 0x01;
  TIM2->SR = 0;
  NVIC->ISER[0] |= (1 << 28);// Enable TIM2 interrupt in NVIC


  /* * * * * * * * TIMER 3 CONFIGURATION * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  TIM3->CR1 = 0;
  TIM3->CR2 = 0;
  TIM3->SMCR = 0;
  TIM3->CCMR1=0;
  TIM3->CCMR2=0;
  TIM3->CCER = 0;
  TIM3->PSC = 31999; // Set prescaler to 31999 for 1ms time base
  TIM3->EGR=0;//TOC
  TIM3->ARR = 0xFFFF;// Set auto-reload register to max value
  TIM3->CCR1 += 300; // Set capture/compare register 1 for 300ms interval
  TIM3-> CNT = 0; // Reset the counter
  TIM3->DIER |= 2;
  TIM3->CR1|=1;  // Enable the counter
  NVIC->ISER[0] |= (1 << 29);// Enable interrupt from TIM3 in NVIC

  // Counter enabling
   TIM3->CR1 |= 0x0001;   // CEN = 1 -> Start counter
   TIM3->EGR |= 0x0001;   // UG = 1 -> Generate update event
   TIM3->SR = 0;          // Counter flags cleared

   // PC7 (Sensor Flying Fish) set as input (00)
  GPIOC->MODER &= ~(3 << (7*2)); // Clear bits 14 and 15 to configure PC7 as input
  // EXTI7 configuration for both rising and falling edges
  EXTI->RTSR |= (1 << 7);  // Enable rising edge trigger for EXTI7 (PC7)
  EXTI->FTSR |= (1 << 7);  // Enable falling edge trigger for EXTI7 (PC7)
  // SYSCFG to associate EXTI7 with GPIOC
  SYSCFG->EXTICR[1] &= ~(0xF << 12);  /// Clear bits [15:12] (EXTI7 field)
  SYSCFG->EXTICR[1] |= (0x2 << 12);   // Set bits [15:12] to associate EXTI7 with GPIOC
  EXTI->IMR |= (1 << 7);  // Unmask EXTI7
  NVIC->ISER[0] |= (1 << (23));   // Enable EXTI9_5 in NVIC (position 23)




  servo_2_00_degrees();
  servo_1_00_degrees();


  time_measurement_1 = 0;
  time_measurement_2 = 0;
  new_distance_is_measured = 0;
  entrance_gate_distance = 0;
  trigger_pulse_value = 0;

  buzzer_toggleing();


  HD44780_Init(2);
  HD44780_Clear();
  HD44780_SetCursor(0,0);
  HD44780_PrintStr("NUMERO DE PLAZAS");
  HD44780_SetCursor(3,1);
  HD44780_PrintStr("LIBRES:");
 //******************************************************************************************************/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(exit_gate_car==1){
		  if(exit_button_pressed == 1){
		  				 servo_1_00_degrees();

		  			  }else{servo_1_90_degrees();}
	  }else{servo_1_90_degrees();}


	  if(new_distance_is_measured==1){                   //minimum distance
		  if(entrance_gate_distance<=10  ){
			  buzzer_sound_on();
			  new_distance_is_measured=0;

			  if(entrance_button_pressed == 1){
				  servo_2_00_degrees();
			  }

		  }
		  else if(entrance_gate_distance>10 && entrance_gate_distance<=20){// Case 2: Average Distance
			  buzzer_toggleing();
			  new_distance_is_measured=0;
			  if(entrance_button_pressed == 1){
				  servo_2_45_degrees();
			  }

		  }

		  else{// Case 3: Maximum Distance
			  buzzer_sound_off();
			  servo_2_90_degrees();

			  new_distance_is_measured=0;  // reset this variable to zero for new measurements

		  }

		   HD44780_SetCursor(13,1);
		   HD44780_PrintStr(int_to_string(available_parking_spaces));

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
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC3;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  * @brief TS Initialization Function
  * @param None
  * @retval None
  */
static void MX_TS_Init(void)
{

  /* USER CODE BEGIN TS_Init 0 */

  /* USER CODE END TS_Init 0 */

  /* USER CODE BEGIN TS_Init 1 */

  /* USER CODE END TS_Init 1 */
  /* USER CODE BEGIN TS_Init 2 */

  /* USER CODE END TS_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SEG12_Pin */
  GPIO_InitStruct.Pin = SEG12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(SEG12_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG22_Pin SEG23_Pin */
  GPIO_InitStruct.Pin = SEG22_Pin|SEG23_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG3_Pin SEG4_Pin */
  GPIO_InitStruct.Pin = SEG3_Pin|SEG4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
