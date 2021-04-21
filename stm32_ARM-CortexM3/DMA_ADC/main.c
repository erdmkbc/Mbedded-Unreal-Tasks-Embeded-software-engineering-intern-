#include "stm32f10x.h"                  // Device header
#include "stm32f10x_adc.h"              // Keil::Device:StdPeriph Drivers:ADC
#include "stm32f10x_dma.h"              // Keil::Device:StdPeriph Drivers:DMA
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC

uint16_t adcValue [2];

void gpioConfig(){

	GPIO_InitTypeDef GPIO_InitValues ;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE) ;
	
	// for adc 
	
	GPIO_InitValues.GPIO_Mode = GPIO_Mode_AIN ;
	GPIO_InitValues.GPIO_Pin=GPIO_Pin_0 ;
	
	GPIO_Init(GPIOA , &GPIO_InitValues) ;
	
	GPIO_InitValues.GPIO_Mode = GPIO_Mode_AIN ;
	GPIO_InitValues.GPIO_Pin=GPIO_Pin_1 ;
	
	GPIO_Init(GPIOA , &GPIO_InitValues) ;

}

void adcConfig(){

	ADC_InitTypeDef ADC_InitValues ;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE) ;
	
	ADC_InitValues.ADC_ContinuousConvMode = ENABLE ; 
	ADC_InitValues.ADC_DataAlign = ADC_DataAlign_Right ;
	ADC_InitValues.ADC_ExternalTrigConv = DISABLE ;
	ADC_InitValues.ADC_Mode =ADC_Mode_Independent ;
	ADC_InitValues.ADC_NbrOfChannel = 2 ;
	ADC_InitValues.ADC_ScanConvMode = DISABLE ;
	
	ADC_Init(ADC1 , &ADC_InitValues) ;
	ADC_Cmd(ADC1 , ENABLE ) ;
	
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_0 , 1 , ADC_SampleTime_55Cycles5) ;
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_1 , 2 , ADC_SampleTime_55Cycles5) ;
	ADC_SoftwareStartConvCmd(ADC1,ENABLE) ;
	
	ADC_DMACmd(ADC1,ENABLE) ;

}

void dmaConfig(){

	DMA_InitTypeDef DMA_InitValues ;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE) ;
	
	DMA_Cmd(DMA1_Channel1 , DISABLE) ;
	DMA_DeInit(DMA1_Channel1) ;
	
	DMA_InitValues.DMA_BufferSize = 2 ;
	DMA_InitValues.DMA_DIR = DMA_DIR_PeripheralSRC ;
	DMA_InitValues.DMA_M2M = DMA_M2M_Disable ;
	DMA_InitValues.DMA_MemoryBaseAddr = (uint32_t)adcValue ;
	DMA_InitValues.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord ;
	DMA_InitValues.DMA_MemoryInc = DMA_MemoryInc_Enable ;
	DMA_InitValues.DMA_Mode = DMA_Mode_Circular ;
	DMA_InitValues.DMA_PeripheralBaseAddr = (uint32_t) & ADC1 ->DR ;
	DMA_InitValues.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord ;
	DMA_InitValues.DMA_PeripheralInc = DMA_PeripheralInc_Disable ;
	DMA_InitValues.DMA_Priority = DMA_Priority_High ;
	
	DMA_Cmd(DMA1_Channel1 , ENABLE) ;
	DMA_DeInit(DMA1_Channel1) ;

}

int main(){
	
	gpioConfig() ;
	adcConfig() ;
	dmaConfig() ;
	

	while(1){
	
	
	
	}


}


