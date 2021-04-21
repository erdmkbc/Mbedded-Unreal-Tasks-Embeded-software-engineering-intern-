#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_adc.h"              // Keil::Device:StdPeriph Drivers:ADC
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "math.h"
#include "stdio.h"
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART

uint16_t adcValue = 0 ;
char message[20] = "\nThe degree is : \n" ;
uint16_t temprature = 0 ;

void gpioConfig(){

	GPIO_InitTypeDef GPIO_InitValues ;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE) ;
	
	// for thermistor
	
	GPIO_InitValues.GPIO_Mode = GPIO_Mode_AIN ;
	GPIO_InitValues.GPIO_Pin = GPIO_Pin_0 ;
	
	GPIO_Init(GPIOA , &GPIO_InitValues) ;
	
	// for uart com using USART1_TX 
	
	GPIO_InitValues.GPIO_Mode = GPIO_Mode_AF_PP ;
	GPIO_InitValues.GPIO_Pin = GPIO_Pin_9 ;
	GPIO_InitValues.GPIO_Speed= GPIO_Speed_50MHz ;
	
	GPIO_Init(GPIOA,&GPIO_InitValues) ;

}

void adcConfig(){

	ADC_InitTypeDef ADC_InitValues ;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE) ;
	
	ADC_InitValues.ADC_ContinuousConvMode = ENABLE ;
	ADC_InitValues.ADC_DataAlign = ADC_DataAlign_Right ;
	ADC_InitValues.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None ;
	ADC_InitValues.ADC_Mode = ADC_Mode_Independent ;
	ADC_InitValues.ADC_NbrOfChannel = 1 ;
	ADC_InitValues.ADC_ScanConvMode = DISABLE ;
	
	ADC_Init(ADC1 , &ADC_InitValues) ;
	ADC_Cmd(ADC1 , ENABLE) ;
	
}

uint16_t readADC(){
	
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_0 , 1 , ADC_SampleTime_55Cycles5) ;
	
	ADC_SoftwareStartConvCmd(ADC1 , ENABLE) ;
	
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != RESET ) ;
	return ADC_GetConversionValue(ADC1) ;

}

double termistor(uint16_t adcValue){
 
 double temp;
 temp = log(((40950000 / adcValue) - 10000));
 temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temp * temp)) * temp);
 temp= temp - 273.15;
 return temp;
}

void uartConfig(){

	USART_InitTypeDef USART_InitValues ;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE) ;
	
	USART_InitValues.USART_BaudRate = 9600 ;
	USART_InitValues.USART_HardwareFlowControl = USART_HardwareFlowControl_None ;
	USART_InitValues.USART_Mode=USART_Mode_Tx ;
	USART_InitValues.USART_Parity=USART_Parity_No ;
	USART_InitValues.USART_StopBits=USART_StopBits_1 ;
	USART_InitValues.USART_WordLength=USART_WordLength_8b ;
	
	USART_Init(USART1 , &USART_InitValues) ;
	USART_Cmd(USART1,ENABLE) ;

}

void uartTransmit(char *string){

	while(*string){
	
		while(!(USART1 ->SR & 0x00000040)){
		
			USART_SendData(USART1,*string) ;
			*string ;
			
		}
	
	}

}

void delay(uint32_t time) {

	while(time -- ) ;

}

int main(){
	
	gpioConfig() ;
	adcConfig() ;
	uartConfig() ;

	while(1){
		
		adcValue = readADC() ;
		temprature = termistor(adcValue) ;
		sprintf(message, "\n The degree is : %d \n" , temprature) ;
		uartTransmit(message) ;
		
		delay(7200000) ;
		
	}

}

