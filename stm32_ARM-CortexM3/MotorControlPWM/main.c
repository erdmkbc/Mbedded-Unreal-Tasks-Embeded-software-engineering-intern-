#include "stm32f10x.h"                  // Device header
#include "stm32f10x_adc.h"              // Keil::Device:StdPeriph Drivers:ADC
#include "stm32f10x_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM

uint32_t buttonState_1= 0; // see the for state buttun from STM_Studio
uint32_t buttonState_2= 0;
uint16_t flag_G = 0; // for permanent button's flag
uint16_t flag_R = 0;
uint32_t adcValue = 0 ; 
uint32_t mapValue = 0 ;

void gpioConfig(){

	GPIO_InitTypeDef GPIOInitValues ;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); // Motor
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); // button
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE); // pot
	// Opened to pins's clocks
	
	GPIOInitValues.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIOInitValues.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_6;
	GPIOInitValues.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA,&GPIOInitValues);
	// Opened to motor pins from "PA" pins
	
	GPIOInitValues.GPIO_Mode=GPIO_Mode_IPD;
	GPIOInitValues.GPIO_Pin=GPIO_Pin_0 |GPIO_Pin_1|GPIO_Pin_2; // GPIO Pin 2 is our interup for reset button 
	// opened to three leds button for leds
	
	
	GPIO_Init(GPIOB,&GPIOInitValues);
	// Opened to button pins from "PB" pins
	
	GPIOInitValues.GPIO_Mode=GPIO_Mode_AIN;
	GPIOInitValues.GPIO_Pin=GPIO_Pin_0;
	
	GPIO_Init(GPIOC,&GPIOInitValues);
  // Opened to pots pin from "PC" Pins

}

void adcConfig(){

	ADC_InitTypeDef ADC_InitValues;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	// Opened to ADC channel's clock
	
	ADC_InitValues.ADC_ContinuousConvMode=ENABLE;
	ADC_InitValues.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_InitValues.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	ADC_InitValues.ADC_Mode=ADC_Mode_Independent;
	ADC_InitValues.ADC_NbrOfChannel=1;
	ADC_InitValues.ADC_ScanConvMode=DISABLE;
	
	ADC_Init(ADC1,&ADC_InitValues);
	ADC_Cmd(ADC1,ENABLE);

}

uint16_t readAdc(){
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_55Cycles5);
	
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	
	return ADC_GetConversionValue(ADC1);
	
}

float map(float adcValue , float max , float min , float conMax , float conMin ){

	return adcValue * ((conMax - conMin )/( max - min )) ;
	// 4050 to 0 discirited to 2399 to 0 
}

void timerConfig(){

	TIM_TimeBaseInitTypeDef TIM_InitValues;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	// opened to cloks for green led's timers
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	// opened to clocks red led's timers
	
	TIM_InitValues.TIM_ClockDivision =  TIM_CKD_DIV1; 
	TIM_InitValues.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitValues.TIM_Period = 2399;
	TIM_InitValues.TIM_Prescaler = 10;
	TIM_InitValues.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM2,&TIM_InitValues);
	TIM_Cmd(TIM2,ENABLE);
	// for green led
	
	TIM_InitValues.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_InitValues.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_InitValues.TIM_Period=2399;
	TIM_InitValues.TIM_Prescaler=10;
	TIM_InitValues.TIM_RepetitionCounter=0;
	
  TIM_TimeBaseInit(TIM3,&TIM_InitValues);
	TIM_Cmd(TIM3,ENABLE);
	// for red led 
	
}

void pwmConfig_G(uint16_t timePulse_G){

	TIM_OCInitTypeDef PWM_InitValues;
	// for green led's obtained to pwm signals. 
	PWM_InitValues.TIM_OCMode=TIM_OCMode_PWM1;
	PWM_InitValues.TIM_OCPolarity=TIM_OCPolarity_High;
	PWM_InitValues.TIM_OutputState=TIM_OutputState_Enable;
	PWM_InitValues.TIM_Pulse=timePulse_G;
	
	TIM_OC1Init(TIM2,&PWM_InitValues);
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
	
}

void pwmConfig_R(uint16_t timePulse_R){

	TIM_OCInitTypeDef PWM_InitValues_R;
	// for red led's obtained to pwm signals
	PWM_InitValues_R.TIM_OCMode=TIM_OCMode_PWM1;
	PWM_InitValues_R.TIM_OCPolarity=TIM_OCPolarity_High;
	PWM_InitValues_R.TIM_OutputState=TIM_OutputState_Enable;
	PWM_InitValues_R.TIM_Pulse=timePulse_R;

	TIM_OC1Init(TIM3,&PWM_InitValues_R);
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);

}

void interupConfig(){

	 EXTI_InitTypeDef EXTI_InitValues;
	 NVIC_InitTypeDef NVIC_InitValues;
	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource2);
	 //opened to external interup's clock
	
	 EXTI_InitValues.EXTI_Line=EXTI_Line2;
	 EXTI_InitValues.EXTI_LineCmd=ENABLE;
	 EXTI_InitValues.EXTI_Mode=EXTI_Mode_Interrupt;
	 EXTI_InitValues.EXTI_Trigger=EXTI_Trigger_Rising;
	
	 EXTI_Init(&EXTI_InitValues);
	 // determine to nvic modes
	 NVIC_InitValues.NVIC_IRQChannel=EXTI2_IRQn;
	 NVIC_InitValues.NVIC_IRQChannelCmd=ENABLE;
	 NVIC_InitValues.NVIC_IRQChannelPreemptionPriority=1;
	 NVIC_InitValues.NVIC_IRQChannelSubPriority=1;
	
	 NVIC_Init(&NVIC_InitValues);

}

void EXTI2_IRQHandler(){

	if(EXTI_GetITStatus(EXTI_Line2)!=RESET){
	
		pwmConfig_G(2399);
		pwmConfig_R(2399);
	
	}
  
	EXTI_ClearITPendingBit(EXTI_Line2);

}
// control the interup's button statement
void delay(uint32_t time){

	while(time--);

}

int main(){
	
	gpioConfig();
	timerConfig();
	adcConfig() ;
	interupConfig();
	while(1){
		
		buttonState_1=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
		buttonState_2=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1);
		adcValue=readAdc();
		mapValue=map(adcValue,4050,0,2399,9);

		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)!=RESET){
		
		if(flag_G == 0 ){	
		
			flag_G=1;
			pwmConfig_G(mapValue);
		
		}
    else{
		
			flag_G=0;
			pwmConfig_G(9) ;
		
		}		
	 // obtained to permanent button
	}
		
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)!=RESET){
		
		if(flag_R == 0 ){	
		
			flag_R=1;
			pwmConfig_R(mapValue);
		
		}
    else{
		
			flag_R=0;
			pwmConfig_R(9) ;
		
		}		
	
	}
	
  }		
}

	 



