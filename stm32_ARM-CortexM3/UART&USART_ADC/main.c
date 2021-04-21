#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_adc.h"              // Keil::Device:StdPeriph Drivers:ADC
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART
#include "stdio.h"

uint32_t data = 0 ;
char message[20] ;

void gpioConfig(){

	// openinig to microphone gpio pins -> PA0 for ADC 
	
	GPIO_InitTypeDef GPIO_InitValues ;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE) ; // Opened to baz line for GPIOA
	
	GPIO_InitValues.GPIO_Mode=GPIO_Mode_AIN ;
	GPIO_InitValues.GPIO_Pin=GPIO_Pin_0 ; // ADC1_Channel 0
	
	GPIO_Init(GPIOA,&GPIO_InitValues);
	
	// USART pins opening from PA9 (USART_1_TX) 
	
	GPIO_InitValues.GPIO_Mode=GPIO_Mode_AF_PP ; 
	GPIO_InitValues.GPIO_Pin=GPIO_Pin_9;
	
	GPIO_Init(GPIOA,&GPIO_InitValues);
	
}

void adcConfig(){

  ADC_InitTypeDef ADC_InitValues ;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE) ; // Opened baz line for ADC1
	
	ADC_InitValues.ADC_ContinuousConvMode=ENABLE ; // this mode that it continously display ADC value 
	ADC_InitValues.ADC_DataAlign=ADC_DataAlign_Right; // our data start to from right 
	ADC_InitValues.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None ; // this projecth no need to trigger
	ADC_InitValues.ADC_Mode=ADC_Mode_Independent;
	ADC_InitValues.ADC_NbrOfChannel=1;
	ADC_InitValues.ADC_ScanConvMode=DISABLE; // using one channes 
	
	ADC_Init(ADC1,&ADC_InitValues);
	ADC_Cmd(ADC1,ENABLE);

}

uint32_t readADC(){

	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_13Cycles5);
	
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)!=RESET);
	
		return ADC_GetConversionValue(ADC1);

}

void uartConfig(){

	USART_InitTypeDef UART_InitValues ;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE) ; // Opened baz line for USART1
	
	UART_InitValues.USART_BaudRate=9600; // our baudrate 9600
	UART_InitValues.USART_HardwareFlowControl=USART_HardwareFlowControl_None; // this projecth has no RS232 protocols
	UART_InitValues.USART_Parity=USART_Parity_No; // parity bit control that data values even or odd this projecth has no need.
	UART_InitValues.USART_StopBits=USART_StopBits_1; 
	UART_InitValues.USART_WordLength=USART_WordLength_8b; 
	
	USART_Init(USART1,&UART_InitValues);
	USART_Cmd(USART1,ENABLE);

}

void uartTransmit(char *string){

	while(*string){
	
		while(!(USART1->SR & 0x00000040)){
		
			USART_SendData(USART1,*string);
			*string++ ;
			
		}
	
	}

}

void delay(uint32_t time){

	while(time--);

}

int main(){
	
	gpioConfig();
	adcConfig() ;
	uartConfig() ;

	while(1){
		
		data = readADC() ;
		sprintf(message,"adcValue=%d",data);
		uartTransmit(message) ;
		delay(50) ;
	
	}


}

