//--------------------------------------------------------------
// File     : stm32_ub_dig_out.c
// Datum    : 01.02.2013
// Version  : 1.2
// Autor    : UB
// EMail    : mc-4u(@)t-online.de
// Web      : www.mikrocontroller-4u.de
// CPU      : STM32F4
// IDE      : CooCox CoIDE 1.7.0
// Module   : GPIO
// Funktion : Digital_Out Funktionen
//--------------------------------------------------------------

//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32_ub_dig_out.h"


//--------------------------------------------------------------
// Definition aller Digital-Out Pins
// Reihenfolge wie bei DOUT_NAME_t
//
// Init : [Bit_SET,Bit_RESET]
//--------------------------------------------------------------
DOUT_PIN_t DOUT_PIN[] = {
		// Name    ,PORT , PIN       , CLOCK              , Init
		{DOUT_ST1, GPIOA,GPIO_Pin_0 ,RCC_AHB1Periph_GPIOA,Bit_RESET},
		{DOUT_DIR1, GPIOA,GPIO_Pin_1 ,RCC_AHB1Periph_GPIOA,Bit_RESET},
		{DOUT_ST2, GPIOA,GPIO_Pin_2 ,RCC_AHB1Periph_GPIOA,Bit_RESET},
		{DOUT_DIR2, GPIOA,GPIO_Pin_3,RCC_AHB1Periph_GPIOA,Bit_RESET},
		{DOUT_ST3, GPIOA,GPIO_Pin_4 ,RCC_AHB1Periph_GPIOA,Bit_RESET},
		{DOUT_DIR3, GPIOA,GPIO_Pin_5 ,RCC_AHB1Periph_GPIOA,Bit_RESET},
		{DOUT_ST4, GPIOA,GPIO_Pin_6, RCC_AHB1Periph_GPIOA,Bit_RESET},
		{DOUT_DIR4, GPIOA,GPIO_Pin_7,RCC_AHB1Periph_GPIOA,Bit_RESET},
		{LED_GREEN ,GPIOD,GPIO_Pin_12,RCC_AHB1Periph_GPIOD,Bit_RESET},   // PD12=Gruene LED auf dem Discovery-Board
		{LED_ORANGE,GPIOD,GPIO_Pin_13,RCC_AHB1Periph_GPIOD,Bit_RESET},   // PD13=Orange LED auf dem Discovery-Board
		{LED_RED   ,GPIOD,GPIO_Pin_14,RCC_AHB1Periph_GPIOD,Bit_RESET},   // PD14=Rote LED auf dem Discovery-Board
		{LED_BLUE  ,GPIOD,GPIO_Pin_15,RCC_AHB1Periph_GPIOD,Bit_RESET},   // PD15=Blaue LED auf dem Discovery-Board
};



//--------------------------------------------------------------
// Init aller Digital-Out Pins
//--------------------------------------------------------------
void UB_DigOut_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	DOUT_NAME_t dig_pin;

	for(dig_pin=0;dig_pin<DOUT_ANZ;dig_pin++) {
		// Clock Enable
		RCC_AHB1PeriphClockCmd(DOUT_PIN[dig_pin].GPIO_CLK, ENABLE);

		// Config als Digital-Ausgang
		GPIO_InitStructure.GPIO_Pin = DOUT_PIN[dig_pin].GPIO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(DOUT_PIN[dig_pin].GPIO_PORT, &GPIO_InitStructure);

		// Default Wert einstellen
		if(DOUT_PIN[dig_pin].GPIO_INIT==Bit_RESET) {
			UB_DigOut_Lo(dig_pin);
		}
		else {
			UB_DigOut_Hi(dig_pin);
		}
	}
}


//--------------------------------------------------------------
// Digital-Out Pin auf Lo-Pegel setzen
//--------------------------------------------------------------
void UB_DigOut_Lo(DOUT_NAME_t dig_pin)
{
	DOUT_PIN[dig_pin].GPIO_PORT->BSRRH = DOUT_PIN[dig_pin].GPIO_PIN;
}

//--------------------------------------------------------------
// Digital-Out Pin auf Hi-Pegel setzen
//--------------------------------------------------------------
void UB_DigOut_Hi(DOUT_NAME_t dig_pin)
{
	DOUT_PIN[dig_pin].GPIO_PORT->BSRRL = DOUT_PIN[dig_pin].GPIO_PIN;
} 

//--------------------------------------------------------------
// Digital-Out Pin Pegel toggeln
//--------------------------------------------------------------
void UB_DigOut_Toggle(DOUT_NAME_t dig_pin)
{
	DOUT_PIN[dig_pin].GPIO_PORT->ODR ^= DOUT_PIN[dig_pin].GPIO_PIN;
}

//--------------------------------------------------------------
// Digital-Out Pin auf Lo oder Hi setzen
//--------------------------------------------------------------
void UB_DigOut_Pin(DOUT_NAME_t dig_pin,BitAction wert)
{
	if(wert==Bit_RESET) {
		UB_DigOut_Lo(dig_pin);
	}
	else {
		UB_DigOut_Hi(dig_pin);
	}
}
