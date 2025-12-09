/**
 * @file	main.c
 * @brief	EEE192
 *
 * @author	Christian Nikolai Rabaya <clrabaya@up.edu.ph>
 * @date	June 12 2024
 * This source code was created for EEE192 and is heavily inspired by
 * the source code created by Alberto de Villa <abdevilla@up.edu.ph> for
 * EEE 158 Module 6 and other source code templates created by the instructors
 * of EEE 158.
 */

/*
 * System configuration/build:
 * 	- Clock source == HSI (~16 MHz)
 * 		- No AHB & APB1/2 prescaling
 *	- Inputs:
 * 		- Active-LO NO-wired pushbutton @ PC13
 * 		- USART Input @ PA3 (USART2_RX)
 * 	- Outputs:
 * 		- Active-HI LED @ PA5
 *		- USART Output @ PA2 (USART2_TX)
 *
 * NOTE: This project uses the CMSIS standard for ARM-based microcontrollers;
 * 	 This allows register names to be used without regard to the exact
 * 	 addresses.
 */

#include "usart.h"
#include <stdint.h>	// C standard header; contains uint32_t, for example
#include <stdbool.h>// C99 bool
#include <stm32f4xx.h>	// Header for the specific device family

#include <stdio.h>	// Needed for snprintf()
#include <string.h>	// Needed for memcmp()
#include <stdlib.h> // Needed for getenv()

////////////////////////////////////////////////////////////////////////////

/*
 * IRQ data shared between the handlers and main()
 *
 * As asynchronous access is possible, all members are declared volatile.
 */
volatile struct {
	/*
	 * If set, the button has been pressed.
	 *
	 * This should be cleared in main().
	 */
	unsigned int pressed;

	/*
	 * Number of system ticks elapsed
	 */
	unsigned int nr_tick;

} irq_data;

/*
 * Handler for the interrupt
 *
 * This function name is special -- this name is used by the startup code (*.s)
 * to indicate the handler for this interrupt vector.
 */
void EXTI15_10_IRQHandler(void) {
	/*
	 * The hardware setup has PC13 being active-low. This must be taken
	 * into consideration to maintain logical consistency with the
	 * rest of the code.
	 */
	if (!(GPIOC->IDR & 0x2000))
		irq_data.pressed = 1;

	// Re-enable reception of interrupts on this line.
	EXTI->PR = (1 << 13);
}

// Handler for the system tick

////////////////////////////////////////////////////////////////////////////

// Function to initialize the system; called only once on device reset
void do_sys_config(void) {
	// See the top of this file for assumptions used in this initialization.

	////////////////////////////////////////////////////////////////////

	RCC->AHB1ENR |= (1 << 0);	// Enable GPIOA

	GPIOA->MODER &= ~(0b11 << 10);	// Set PA5 as input...
	GPIOA->MODER |= (0b10 << 10);	// ... then set it as alternate function.
	GPIOA->OTYPER &= ~(1 << 5);	// PA5 = push-pull output
	GPIOA->OSPEEDR &= ~(0b11 << 10);	// Fast mode (needed for PWM)
	GPIOA->OSPEEDR |= (0b10 << 10);

	/*
	 * For PA5 -- where the LED on the Nucleo board is -- only TIM2_CH1
	 * is PWM-capable, per the *device* datasheet. This corresponds to
	 * AF01.
	 */
	GPIOA->AFR[0] &= ~(0x00F00000);	// TIM2_CH1 on PA5 is AF01
	GPIOA->AFR[0] |= (0x00100000);

	////////////////////////////////////////////////////////////////////

	/*
	 * In this setup, TIM2 is used for brightness control (see reason
	 * above), while TIM1 is used to do input-capture on PA9 via
	 * Channel 2.
	 *
	 * The internal clock for TIM2 is the same as the APB1 clock;
	 * in turn, this clock is assumed the same as the system (AHB) clock
	 * without prescaling, which results to the same frequency as HSI
	 * (~16 MHz).
	 *
	 * TIM1, on the other hand, uses the APB2 clock; in turn, no prescaling
	 * is done here, which also yields the same value as HSI (~16 MHz).
	 */
	RCC->APB1ENR |= (1 << 0);	// Enable TIM2
	RCC->APB2ENR |= (1 << 0);	// Enable TIM1

	/*
	 * Classic PWM corresponds to the following:
	 * 	- Edge-aligned	(CMS  = CR1[6:5] = 0b00)
	 * 	- Upcounting	(DIR  = CR1[4:4] = 0)
	 *	- Repetitive	(OPM  = CR1[3:3] = 0)
	 * 	- PWM Mode #1	(CCxS[1:0] = 0b00, OCMx = 0b110)
	 * 		- These are in CCMRy; y=1 for CH1 & CH2, and y=2 for
	 * 		  CH3 & CH4.
	 */
	TIM2->CR1 &= ~(0b1111 << 0);
	TIM2->CR1 &= ~(1 << 0);		// Make sure the timer is off
	TIM2->CR1 |= (1 << 7);		// Preload ARR (required for PWM)
	TIM2->CCMR1 = 0x0068;		// Channel 1 (TIM2_CH1)

	/*
	 * Per the Nyquist sampling theorem (from EEE 147), to appear
	 * continuous the PWM frequency must be more than twice the sampling
	 * frequency. The human eye (the sampler) can usually go up to 24Hz;
	 * some, up to 40-60 Hz. To cover all bases, let's use 500 Hz.
	 *
	 * In PWM mode, the effective frequency changes to
	 *
	 * (f_clk) / (ARR*(PSC+1))
	 *
	 * since each period must be able to span all values on the interval
	 * [0, ARR). For obvious reasons, ARR must be at least equal to one.
	 */
	TIM2->ARR = 100;		// Integer percentages; interval = [0,100]
	TIM2->PSC = (320 - 1);	// (16MHz) / (ARR*(TIM2_PSC + 1)) = 500 Hz

	// Let main() set the duty cycle. We initialize at zero.
	TIM2->CCR1 = 0;

	/*
	 * The LED is active-HI; thus, its polarity bit must be cleared. Also,
	 * the OCEN bit must be enabled to actually output the PWM signal onto
	 * the port pin.
	 */
	TIM2->CCER = 0x0001;

	////////////////////////////////////////////////////////////////////

	// Pushbutton configuration
	RCC->AHB1ENR |= (1 << 2);	// Enable GPIOC

	GPIOC->MODER &= ~(0b11 << 26);	// Set PC13 as input...
	GPIOC->PUPDR &= ~(0b11 << 26);// ... without any pull-up/pull-down (provided externally).

	////////////////////////////////////////////////////////////////////

	/*
	 * Enable the system-configuration controller. If disabled, interrupt
	 * settings cannot be configured.
	 */
	RCC->APB2ENR |= (1 << 14);

	/*
	 * SYSCFG_EXTICR is a set of 4 registers, each controlling 4 external-
	 * interrupt lines. Within each register, 4 bits are used to select
	 * the source connected to each line:
	 * 	0b0000 = Port A
	 * 	0b0001 = Port B
	 * 	...
	 * 	0b0111 = Port H
	 *
	 * For the first EXTICR register:
	 *	Bits 0-3   correspond to Line 0;
	 * 	Bits 4-7   correspond to Line 1;
	 *	Bits 8-11  correspond to Line 2; and
	 * 	Bits 12-15 correspond to Line 3.
	 *
	 * The 2nd EXTICR is for Lines 4-7; and so on. Also, the line numbers
	 * map 1-to-1 to the corresponding bit in each port. Thus, for example,
	 * a setting of EXTICR2[11:8] == 0b0011 causes PD6 to be tied to
	 * Line 6.
	 *
	 * For this system, PC13 would end up on Line 13; thus, the
	 * corresponding setting is EXTICR4[7:4] == 0b0010. Before we set it,
	 * mask the corresponding interrupt line.
	 */
	EXTI->IMR &= ~(1 << 13);		// Mask the interrupt
	SYSCFG->EXTICR[3] &= (0b1111 << 4);	// Select PC13 for Line 13
	SYSCFG->EXTICR[3] |= (0b0010 << 4);

	/*
	 * Per the hardware configuration, pressing the button causes a
	 * falling-edge event to be triggered, and a rising-edge on release.
	 * Since we are only concerned with presses, just don't trigger on
	 * releases.
	 */
	EXTI->RTSR &= ~(1 << 13);
	EXTI->FTSR |= (1 << 13);

	/*
	 * Nothing more from the SCC, disable it to prevent accidental
	 * remapping of the interrupt lines.
	 */
	RCC->APB2ENR &= ~(1 << 14);

	////////////////////////////////////////////////////////////////////

	/*
	 *
	 * Per the STM32F411xC/E family datasheet, the handler for EXTI_15_10
	 * is at 40.
	 *
	 * Per the STM32F4 architecture datasheet, the NVIC IP registers
	 * each contain 4 interrupt indices; 0-3 for IPR[0], 4-7 for IPR[1],
	 * and so on. Each index has 8 bits denoting the priority in the top
	 * 4 bits; lower number = higher priority.
	 *
	 * Position 40 in the NVIC table would be at IPR[10][7:0]; or,
	 * alternatively, just IP[40].
	 */
	NVIC->IP[40] = (1 << 4);
	NVIC->IP[6] = (0b1111 << 4);	// SysTick; make it least-priority

	/*
	 * Per the STM32F4 architecture datasheet, the NVIC ISER/ICER registers
	 * each contain 32 interrupt indices; 0-31 for I{S/C}ER[0], 32-63 for
	 * I{S/C}ER[1], and so on -- one bit per line.
	 *
	 * ISER is written '1' to enable a particular interrupt, while ICER is
	 * written '1' to disable the same interrupt. Writing '0' has no
	 * effect.
	 *
	 * Position 25 in the NVIC table would be at I{S/C}ER[0][25:25]; while
	 * position 27 would be at I{S/C}ER[0][27:27].
	 */
	NVIC->ISER[0] = (1 << 6);	// Note: Writing '0' is a no-op
	NVIC->ISER[1] = (1 << 8);	// Note: Writing '0' is a no-op
	EXTI->IMR |= (1 << 13);		// Unmask the interrupt on Line 13
	TIM2->EGR |= (1 << 0);		// Trigger an update on TIM2
	TIM2->CR1 |= (1 << 0);		// Activate both timers
	irq_data.pressed = 0;

	/*
	 * Enable tick counting; the idea is to allow main() to perform
	 * periodic tasks.
	 */
	SysTick->LOAD = (20000 - 1);	// Target is 100 Hz with 2MHz clock
	SysTick->VAL = 0;
	SysTick->CTRL &= ~(1 << 2);	// Clock base = 16MHz / 8 = 2MHz
	SysTick->CTRL &= ~(1 << 16);
	SysTick->CTRL |= (0b11 << 0);	// Enable the tick

	// Do the initialization of USART last.
	usart1_init();
	usart2_init();
}

//////////////////////////////////////////////////
/////////////////// ESP8266 //////////////////////
//////////////////////////////////////////////////

// taken from EEE 158 Module 6 main.c
bool reply_USART1(const char *expected) {
	char rxb_data[128];
	unsigned int rxb_idx = 0;
	memset(rxb_data,0,sizeof(rxb_data));

	/////////////////////////////////////////////////////////////

	// Check for any data received via USART2.
	while(1) {

		struct usart1_rx_event usart1_evt;

		if (!usart1_rx_get_event(&usart1_evt))
			// Nothing to do here
			break;
		else if ((!usart1_evt.has_data) || (!usart1_evt.valid))
			break;

		/*
		 * [1] If an IDLE is detected, update the size.
		 *
		 * [2] If no data is present, we're done.
		 */
		if (usart1_evt.is_idle || (rxb_idx >= sizeof(rxb_data) - 1)) {
			break;
		}

		// Store the data
		rxb_data[rxb_idx++] = usart1_evt.c;
	}

	rxb_data[rxb_idx] = '\0';

	usart2_tx_send(rxb_data, sizeof(rxb_data));
	while (usart2_tx_is_busy());

	delay(1000);

	return (strstr(rxb_data, expected) != (NULL));
}

void send_USART1(const char *cmd) {
	usart1_tx_send(cmd, strlen(cmd));
	while (usart1_tx_is_busy());
	delay(1000);
}

void ESP_Init(char *WIFI_ID, char *PW) {
	char wifi[128];
	send_USART1("AT+RESTORE\r\n");

	delay(5000);
	send_USART1("AT\r\n");
	while (!reply_USART1("OK"));
	delay(1000);

	send_USART1("AT+CWMODE=1\r\n");
	while (!reply_USART1("OK"));
	delay(1000);

	sprintf(wifi, "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_ID, PW);
	send_USART1(wifi);
	while (!reply_USART1("OK"));
	delay(1000);
}

void ThingSpeak_Init(void) {
	delay(2000);
	send_USART1("AT+CIPMUX=0\r\n");
	while (!reply_USART1("OK"));
	delay(1000);
}

void ESP_Send(int FieldNum1, int data1, int FieldNum2, int data2, int FieldNum3, int data3) {
	IWDG_Refresh();

    const char *IP_ADD = "api.thingspeak.com";
    const char *variable_name = "API_KEY";
	char *API_KEY;
    API_KEY = getenv(variable_name);

	char TS_buffer[256];
	char http_buffer[256];
	char http_buffer1[128];

	// Establish TCP connection with ThingSpeak
	strcpy(TS_buffer, "AT+CIPSTART=\"TCP\",\"");
	strcat(TS_buffer, IP_ADD);
	strcat(TS_buffer, "\",80\r\n");
	send_USART1(TS_buffer);
	while (!reply_USART1("OK"));
	IWDG_Refresh();

	// Prepare HTTP request with data for three fields
	snprintf(http_buffer, sizeof(http_buffer),
	             "GET /update?api_key=%s&field%d=%d&field%d=%d&field%d=%d HTTP/1.1\r\n"
	             "Host: %s\r\n"
	             "Connection: close\r\n\r\n",
	             API_KEY, FieldNum1, data1, FieldNum2, data2, FieldNum3, data3, IP_ADD);

	IWDG_Refresh();
	// Send size of HTTP request
	snprintf(http_buffer1, sizeof(http_buffer1), "AT+CIPSEND=%d\r\n", (int) strlen(http_buffer));
	send_USART1(http_buffer1);

	IWDG_Refresh();
	// Send HTTP request
	send_USART1(http_buffer);
	delay(3000); // Add delay after sending HTTP request

	IWDG_Refresh();

	return;
}

///////////////////////////////////////////////////////////////////

// Initialize GPIO
void GPIO_Init(void) {
    // Enable GPIOA clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure PA0, PA4, and PA1 (PIR sensor outputs) as inputs
    GPIOA->MODER &= ~(1<<1); // PA0 as Output, MODER = 0b01
    GPIOA->MODER &= ~(1<<0);

    GPIOA->MODER &= ~(1<<3); // PA1 as Output, MODER = 0b01
    GPIOA->MODER &= ~(1<<2);

    GPIOA->MODER &= ~(1<<9); // PA4 as Output, MODER = 0b01
    GPIOA->MODER &= ~(1<<8);

    // Configure pull-up resistors for PA0, PA1, and PA4

    GPIOA->PUPDR &= ~(1<<1);
    GPIOA->PUPDR |= (1<<0);

    GPIOA->PUPDR &= ~(1<<3);
    GPIOA->PUPDR |= (1<<2);

    GPIOA->PUPDR &= ~(1<<9);
    GPIOA->PUPDR |= (1<<8);

    // Configure PA6 (LED) as output
    GPIOA->MODER &= ~(1<<13); // PA6 as Output, MODER = 0b01
    GPIOA->MODER |= (1<<12);

	GPIOA->ODR &= ~(1<<6); // PA6 initially LOW
}

// FOR WIFI DISCONNECT

void IWDG_Init() {
	IWDG->KR = 0x5555;

	IWDG->PR |= (1<<2);
	IWDG->PR |= (1<<1);
	IWDG->PR &= ~(1<<0);

	IWDG->RLR = 4095;
}

void IWDG_Refresh(void) {
	IWDG->KR = 0xAAAA;
}

////////////////////////////

int main(void) {
    GPIO_Init();
    IWDG_Init();
    do_sys_config();
    //ESP_Init("EEE192-429", "EEE192_Room429");
    ESP_Init("GlobeAtHome_c9920_2.4", "kingkongMASTER9413");
    ThingSpeak_Init();

    while (1) {
    	IWDG_Refresh();
    	/* Read the state of the PIR sensors */
    	int pir1 = (GPIOA->IDR & GPIO_IDR_ID0) != 0;
    	int pir2 = (GPIOA->IDR & GPIO_IDR_ID1) != 0;
    	int pir3 = (GPIOA->IDR & GPIO_IDR_ID4) != 0;

    	ESP_Send(1, pir1, 2, pir2, 3, pir3);

    	if (pir1 || pir2 || pir3) {
    		GPIOA->ODR |= (1<<6);
    		delay(1500);
    		GPIOA->ODR &= ~(1<<6);
    	}

    	delay(7500);
    }

}

void delay(int period) {
	int i;
	for (; period > 0; period--) {
		for (i = 0; i < 2657; i++)
			;
	}
}
