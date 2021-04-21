#include "stm32f10x.h"                  // Device header
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stdio.h"

float distance = 0 ; // for distance value 
char message[20] ; // for warning message in tera term console 
uint16_t flag = 0 ; // for permanent button's flag 

void gpioConfig(){
 
	GPIO_InitTypeDef GPIO_InitValues ;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE) ;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE ) ;
	
	// for trig input 
	
	GPIO_InitValues.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_InitValues.GPIO_Pin=GPIO_Pin_1;
	GPIO_InitValues.GPIO_Speed = GPIO_Speed_50MHz ;
	
	GPIO_Init(GPIOA , &GPIO_InitValues) ;
	
	// for echo output 
	
  GPIO_InitValues.GPIO_Mode = GPIO_Mode_IPD ;
	GPIO_InitValues.GPIO_Pin =GPIO_Pin_0 ;
	GPIO_InitValues.GPIO_Speed = GPIO_Speed_50MHz ;
	
	GPIO_Init(GPIOA , &GPIO_InitValues) ;
	
	// for usart pins TX 
	
	GPIO_InitValues.GPIO_Mode = GPIO_Mode_AF_PP ;
	GPIO_InitValues.GPIO_Pin = GPIO_Pin_9 ;
	GPIO_InitValues.GPIO_Speed = GPIO_Speed_50MHz ;
	
	GPIO_Init(GPIOA, &GPIO_InitValues) ;
	
	// for leds
	
	GPIO_InitValues.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_InitValues.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
	GPIO_InitValues.GPIO_Speed = GPIO_Speed_50MHz ;
	
	GPIO_Init(GPIOB,&GPIO_InitValues) ;
	
	// for DC motor 
	
	GPIO_InitValues.GPIO_Mode = GPIO_Mode_AF_PP ;
	GPIO_InitValues.GPIO_Pin = GPIO_Pin_6 ; // PA6 Pins - > TIM_3_CH1
	GPIO_InitValues.GPIO_Speed = GPIO_Speed_50MHz ;

	GPIO_Init(GPIOA , &GPIO_InitValues ) ;
	
	// for stop button 
	
	GPIO_InitValues.GPIO_Mode = GPIO_Mode_IPD ;
	GPIO_InitValues.GPIO_Pin = GPIO_Pin_7 ;
	
	GPIO_Init(GPIOA, &GPIO_InitValues) ;
	
}


void usartConfig(){

	USART_InitTypeDef UART_InitValues ;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE) ;
	
	UART_InitValues.USART_BaudRate = 9600 ;
	UART_InitValues.USART_HardwareFlowControl=USART_HardwareFlowControl_None ;
	UART_InitValues.USART_Mode = USART_Mode_Tx ;
	UART_InitValues.USART_Parity=USART_Parity_No ;
	UART_InitValues.USART_StopBits=USART_StopBits_1 ;
	UART_InitValues.USART_WordLength = USART_WordLength_8b ;
	
	USART_Init(USART1 , &UART_InitValues) ;
	USART_Cmd(USART1 , ENABLE ) ;
	
}

void uartTransmit(char *string){
   while(*string){
      while(!(USART1->SR & 0x00000040));
      USART_SendData(USART1,*string);
      *string++;

   }
}

void delayUS(uint32_t time) {

	uint32_t NewTime = time * 24 ; // for 1 us cycle 
	
	while(NewTime -- ) ;

}

float HC_SR04Config(){
	
	uint32_t time = 0 ;
	float tempDistance = 0 ;
	
  // for start to high 
	GPIO_ResetBits(GPIOA,GPIO_Pin_1) ;
	delayUS(10) ;
	GPIO_SetBits(GPIOA , GPIO_Pin_1) ;
	delayUS(10) ;
	GPIO_ResetBits(GPIOA , GPIO_Pin_1) ;
	delayUS(10) ;
	
	while(!(GPIO_ReadInputDataBit(GPIOA , GPIO_Pin_0))) ; // wait for the reading 
	
	while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)){
	
		time ++ ;
		delayUS(1) ;
		
	}
	
	tempDistance= (float)time * 0.034 ; // for cm values 
	delayUS(1000) ;
	
	return tempDistance ;

}

void timerConfig(){
	
	TIM_TimeBaseInitTypeDef TIM_InitValues ;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE) ;
	
  TIM_InitValues.TIM_ClockDivision = TIM_CKD_DIV1 ;
	TIM_InitValues.TIM_Period = 2399 ;
	TIM_InitValues.TIM_Prescaler = 10 ;
	TIM_InitValues.TIM_RepetitionCounter = 0 ;
	
	TIM_TimeBaseInit(TIM3 , &TIM_InitValues) ;
	TIM_Cmd(TIM3, ENABLE ) ;
	
}

void pwmConfig(uint16_t timePulse){
	
	TIM_OCInitTypeDef PWM_InitValues ;
	
	PWM_InitValues.TIM_OCMode = TIM_OCMode_PWM1 ;
	PWM_InitValues.TIM_OCPolarity = TIM_OCPolarity_High ;
	PWM_InitValues.TIM_OutputState = TIM_OutputState_Enable ;
	PWM_InitValues.TIM_Pulse = timePulse ;
	
	TIM_OC1Init(TIM3 , &PWM_InitValues) ;
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable) ;

}

void extiConfig(){

	// obtaiden for interrup
	EXTI_InitTypeDef EXTI_InitValues ;
	NVIC_InitTypeDef NVIC_InitValues ;
	
	// opened clock for interrup with AFIO mode 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE) ;
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA , GPIO_PinSource7) ;
	
	EXTI_InitValues.EXTI_Line = EXTI_Line7 ;
	EXTI_InitValues.EXTI_LineCmd= ENABLE ;
	EXTI_InitValues.EXTI_Mode = EXTI_Mode_Interrupt ;
	EXTI_InitValues.EXTI_Trigger = EXTI_Trigger_Rising ;
	
	EXTI_Init(&EXTI_InitValues) ;
	
	// obtaining to nvic 
	
	NVIC_InitValues.NVIC_IRQChannel = EXTI9_5_IRQn ;
	NVIC_InitValues.NVIC_IRQChannelCmd = ENABLE ;
	NVIC_InitValues.NVIC_IRQChannelPreemptionPriority = 1 ;
	NVIC_InitValues.NVIC_IRQChannelSubPriority = 1 ;
	
	NVIC_Init(&NVIC_InitValues) ;
	
}

void EXTI9_5_IRQHandler(){
  
	// reset the dc motor with interrup
	if(EXTI_GetITStatus(EXTI_Line7)!= RESET){
		
		pwmConfig(0) ;
	
	}
	
	EXTI_ClearITPendingBit(EXTI_Line7) ;

}

int main(){

	gpioConfig() ;
	usartConfig() ;
	extiConfig() ;
	
	while(1){
		
		distance = HC_SR04Config() ;
		
		if(distance >= 400){
		
			distance = 400 ;
		
		}
		
		if(distance <= 2){
		
			distance = 2 ;
		}
		
		sprintf(message,"distance : %f cm \n",distance) ;
    uartTransmit(message) ; 
		delayUS(1000) ;
		
		if(distance <= 400 ){
		
			GPIO_ResetBits(GPIOB , GPIO_Pin_0) ; // red led close 
			delayUS(1000) ;
			GPIO_SetBits(GPIOB , GPIO_Pin_1) ; // green led open  
			delayUS(1000) ;
			pwmConfig(2399) ;
			
		}   
		
		if(distance <= 200 ){
		
			GPIO_ResetBits(GPIOB , GPIO_Pin_0) ; // red close 
			delayUS(1000) ;
			GPIO_SetBits(GPIOB , GPIO_Pin_1) ; // green led open 
			delayUS(1000) ;
			pwmConfig(1399) ;
		
		}
		
		if(distance <= 100){
			
			GPIO_ResetBits(GPIOB , GPIO_Pin_0) ; // red close 
			delayUS(1000) ;
			GPIO_SetBits(GPIOB , GPIO_Pin_1) ; // green led open 
			delayUS(1000) ;
			pwmConfig(399) ;
		
		}
		
		if(distance <= 50){
		
			GPIO_ResetBits(GPIOB , GPIO_Pin_1) ; // green led close 
			delayUS(1000) ;
			GPIO_SetBits(GPIOB , GPIO_Pin_0) ; // red led open 
			delayUS(1000) ;
			pwmConfig(99) ;
		
		}
	
 }

}
