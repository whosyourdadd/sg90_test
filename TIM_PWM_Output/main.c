/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 
/* Includes ------------------------------------------------------------------*/
#include "main.h"

//Library config for this project!!!!!!!!!!!
#include "stm32f4xx_conf.h"
//#include "stm32f4xx_tim.h"
/** @addtogroup STM32F4-Discovery_PWM_Output_Demo
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void TIM_Configuration(void);
void GPIO_Configuration(void);;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  volatile int i;
  int n = 1;
  uint16_t brightness = 0;      
  uint16_t who_run = 0;

  RCC_Configuration();
  TIM_Configuration();
  GPIO_Configuration();

  /*
  while(1)  // Do not exit
  {
  
    if(brightness + n <= 0)
        who_run = (who_run + 1) % 4; 

    if (((brightness + n) >= 3000) || ((brightness + n) <= 0))
      n = -n; // if  brightness maximum/maximum change direction
    
    brightness += n;
    // TIM4->CCR1 = brightness - 1;
    // TIM4->CCR2 = brightness - 1;
    // TIM4->CCR3 = brightness - 1;
    // TIM4->CCR4 = brightness - 1;
    
    //Light LEDs in turn
    switch(who_run){
        case 0:
            TIM4->CCR1 = brightness - 1; // set brightness
            break;
        case 1:
            TIM4->CCR2 = brightness - 1; // set brightness
            break;
        case 2:
            TIM4->CCR3 = brightness - 1; // set brightness
            break;
        case 3:
            TIM4->CCR4 = brightness - 1; // set brightness
            break;
    }
    for(i=0;i<10000;i++);  // delay
  }
  */
  while(1){
    switch(who_run){
	case 0:
           TIM_SetCompare1(TIM4,5);
           break;
        case 1:
           TIM_SetCompare1(TIM4,10);
           break;
        case 2:
           TIM_SetCompare1(TIM4,15);
           break;
        case 3:
           TIM_SetCompare1(TIM4,20);
           break;
        case 4:
           TIM_SetCompare1(TIM4,15);
           break;
        case 5:
           TIM_SetCompare1(TIM4,10);
           break; 
        default:
           break;
    }
   for(i=0;i<1000000;i++);
   who_run++;
   who_run%=6;

  }
  return(0); // System will implode
}   
  
  

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
   RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOD , ENABLE );
   RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );
   RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOB, ENABLE );
}

/**
  * @brief  configure the PD12~15 to Timers
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure); // Reset init structure
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
    //GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
      

    // Setup Blue & Green LED on STM32-Discovery Board to use PWM.
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_12; //PD12->LED3 PD13->LED4 PD14->LED5 PD15->LED6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init( GPIOD, &GPIO_InitStructure );  
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 |  GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init( GPIOB, &GPIO_InitStructure);
    
	
}

/**
  * @brief  configure the TIM4 for PWM mode
  * @param  None
  * @retval None
  */
void TIM_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    // Let PWM frequency equal 100Hz.
    // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
    // Solving for prescaler gives 240.
    TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
    //TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;//for sg90 
    //TIM_TimeBaseInitStruct.TIM_Period = 31600;   
    TIM_TimeBaseInitStruct.TIM_Period = 200 -1;//for sg90
    //TIM_TimeBaseInitStruct.TIM_Prescaler = SystemCoreClock/5000000; 
    TIM_TimeBaseInitStruct.TIM_Prescaler = 8400 -1;//for sg90
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;    
    TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStruct );
    
    TIM_OCStructInit( &TIM_OCInitStruct );
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    //TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
    // Initial duty cycle equals 0%. Value can range from zero to 65535.
    //TIM_Pulse = TIM4_CCR1 register (16 bits)
    //TIM_OCInitStruct.TIM_Pulse = 65535; //(0=Always Off, 65535=Always On)
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low; 
    TIM_OC1Init( TIM4, &TIM_OCInitStruct ); // Channel 1  FOR PB6 PWM
    TIM_OC2Init( TIM4, &TIM_OCInitStruct ); // Channel 2  LED
    TIM_OC3Init( TIM4, &TIM_OCInitStruct ); // Channel 3  LED FOR PB8 PWM
    TIM_OC4Init( TIM4, &TIM_OCInitStruct ); // Channel 4  LED
    //TIM_OCInitStruct.TIM_Pulse = 0;
    //TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
    //TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High; 
  
    //TIM_OC2Init( TIM4, &TIM_OCInitStruct );  // cH 1 Led
    //TIM_OC4Init( TIM4, &TIM_OCInitStruct ); 
    TIM_Cmd( TIM4, ENABLE );
    //TIM_SetCompare1(TIM4,2370);
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
