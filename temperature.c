#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "PLL.h"
#include "switch_counter_interrupt_TivaWare.h"
#include "driverlib/pin_map.h"
#include "driverlib//timer.h"



#define RED_MASK 		0x02	// red led
#define BLUE_MASK 	0x04	// blue led
#define GREEN_MASK 	0x08	// green led
#define MOTION_SENSOR 0x10	//motion sensor
int currMask;	//Mask to be displayed by motion sensor
int oldMask;
int clearLED = 0;
int updateTemp = 0;	//Enables the temperature updater in main
uint8_t currTemp;	//Current temperature to be displayed

#define DELAY 16000002	//Display length for LED
#define CLK_DIV 24
#define PERIOD 50000000

uint32_t ui32ADC0Value;	//The value read by the temperature sensor
volatile float ui32TempAvg;
volatile float  ui32TempValueC;
volatile float  ui32TempValueF;
char str[4] = {0, 0, 0, 0};

void uart_Init(void) {
	// start peripheral clocks
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);	

	// configure external pins
	GPIOPinConfigure(GPIO_PA0_U0RX);	
	GPIOPinConfigure(GPIO_PA1_U0TX);
	
	// mux pins to UART peripheral
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);	

	// configure UART system and packets
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); 
}

void PortFunctionInit(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);	//Enables GPIO E
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);	//Sets Pin E 3 for ADC functions
	
	volatile uint32_t ui32Loop; 
  
	// ENABLE THE PORT CLOCK
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOC;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;

	// Do a dummy read to insert a few cycles after enabling the peripheral.
	ui32Loop = SYSCTL_RCGC2_R;
	
	// unlock GPIO Port C and F
	// and allow changes
	GPIO_PORTC_LOCK_R = 0x4C4F434B;   
	GPIO_PORTC_CR_R |= 0x10;
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   
	GPIO_PORTF_CR_R |= 0x01;
	
	
	// ENABLE THE LED'S PINS AND MOTION SENSOR'S PIN FOR DIGITAL FUNCTIONS
	GPIO_PORTF_DEN_R |= ( RED_MASK | BLUE_MASK | GREEN_MASK);
	GPIO_PORTC_DEN_R |= (MOTION_SENSOR);

	// SET THE LED'S PINS AS OUTPUTS
	GPIO_PORTF_DIR_R |= RED_MASK | BLUE_MASK | GREEN_MASK;
	
	// SET MOTION SENSOR'S PIN AS AN INPUT
	GPIO_PORTC_DIR_R &= ~((unsigned int) (MOTION_SENSOR));
	
}

void configPortCInt() {
	// 1. Enable GPIOC in NVIC
	IntEnable(INT_GPIOC);  							
	
	// 2. Configure GPIOC interrupt priority as 4
	IntPrioritySet(INT_GPIOC, 0x04); 		
	
	// 3.1 Allow motion sensor's interrupt to go to the core
	GPIO_PORTC_IM_R |= MOTION_SENSOR;
	
	// 3.2 Configure the motion sensor's interrupt to trigger on edges
	GPIO_PORTC_IS_R &= ~MOTION_SENSOR;
	
	// 3.3 Configure motion sensor to use the GPIOIEV's setting
  GPIO_PORTC_IBE_R &= ~MOTION_SENSOR;

	// 3.4 Configure motion sensor to trigger on rising edge
  GPIO_PORTC_IEV_R |= MOTION_SENSOR;   

}

//ADC0 initialization
void ADC0_Init(void) {
	
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);	//activate the clock of ADC0
	SysCtlDelay(5);	//insert a few cycles after enabling the peripheral to allow the clock to be fully activated.

	ADCSequenceDisable(ADC0_BASE, 3); //disable ADC0 before the configuration is complete
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0); // will use ADC0, SS3, processor-trigger, priority 0
	
	//ADC0 SS3, sample from external temperature sensor, completion of this step will set RIS, only sample of the sequence
	ADCSequenceStepConfigure(ADC0_BASE,3,0,ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END); 

	IntPrioritySet(INT_ADC0SS3, 0x10);  	 	// configure ADC0 SS3 interrupt priority as 0
	IntEnable(INT_ADC0SS3);    							// enable interrupt 31 in NVIC (ADC0 SS1)
	ADCIntEnableEx(ADC0_BASE, ADC_INT_SS3); // arm interrupt of ADC0 SS1

	ADCSequenceEnable(ADC0_BASE, 3); //enable ADC0
}

void GPIOPortCHandler()
{
	GPIO_PORTF_DATA_R |= currMask;
	TimerEnable(TIMER1_BASE, TIMER_A);
	TimerLoadSet(TIMER1_BASE, TIMER_A, PERIOD - 1);
	//SysCtlDelay(DELAY);
	oldMask = currMask;
	clearLED = 1;
	//GPIO_PORTF_DATA_R &= ~currMask;
	GPIO_PORTC_ICR_R |= MOTION_SENSOR;
}

void Timer0A_Init(unsigned long period) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);			// start the timer clock
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);	// configure for periodic
  TimerLoadSet(TIMER0_BASE, TIMER_A, period-1);   	// set reload value
	IntPrioritySet(INT_TIMER0A, 0x00);
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);  // arm timeout interrupt
  TimerEnable(TIMER0_BASE, TIMER_A);      					// enable and start timer0A	
}

void Timer1A_Init(unsigned long period) {   	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);			// start the timer clock
  TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);	// configure for periodic
  TimerLoadSet(TIMER1_BASE, TIMER_A, period-1);   	// set reload value
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);  // arm timeout interrupt
  //TimerEnable(TIMER1_BASE, TIMER_A);      					// enable and start timer0A
}
		
//interrupt handler
void ADC0_Handler(void) {
	ADCIntClear(ADC0_BASE, 3); //clear interrupt flag of ADC0 SS1
	
	ADCSequenceDataGet(ADC0_BASE, 3, &ui32ADC0Value); //Load the captured data from FIFO
	ui32TempAvg = ui32ADC0Value * 3.3 / 4096;//(ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;
	ui32TempValueC = (ui32TempAvg - 0.5) * 100;//(1475 - ((2475 * ui32TempAvg)) / 4096)/10;
	ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;
	ADCProcessorTrigger(ADC0_BASE, 3); //Software trigger the next ADC sampling
}

void TIMER0AHandler(void) {
	updateTemp = 1;
	TIMER0_ICR_R |= 0x01;
}

int main(void) {
	Timer0A_Init(16000000);
	Timer1A_Init(PERIOD);
	//SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	// IntMasterDisable();
	ADC0_Init();
	PortFunctionInit();
	configPortCInt();
	uart_Init();
	IntMasterEnable();       		// globally enable interrupt
	ADCProcessorTrigger(ADC0_BASE, 3);
	
	while(1) {
		if (clearLED == 1)
		{
			if (TimerIntStatus(TIMER1_BASE, false) & TIMER_A) {
				TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
				TimerDisable(TIMER1_BASE, TIMER_A);
				
				GPIO_PORTF_DATA_R &= ~oldMask;	// toggle LED
				clearLED = 0;
			}
		}
		
		if (updateTemp == 1)
		{
			currTemp = ui32TempValueF;
			IntMasterDisable();
					
					// convert count to ASCII characters
			sprintf(str, "%03u", currTemp);
					
			
			
			UARTCharPut(UART0_BASE, str[0]); 
			UARTCharPut(UART0_BASE, str[1]); 
			UARTCharPut(UART0_BASE, str[2]);
			UARTCharPut(UART0_BASE, '\n'); 
			UARTCharPut(UART0_BASE, '\r');
			
			IntMasterEnable();
			if (ui32TempValueF > 74)
				currMask = BLUE_MASK;
			else if (ui32TempValueF < 72)
				currMask = RED_MASK;
			else
				currMask = GREEN_MASK;
			updateTemp = 0;
		}
	}
	
}
