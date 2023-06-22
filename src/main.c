#include <stm32f767xx.h>

#define RCC_GPIOB_EN 1<<1

#define MODER0_OUT 1 << 0
#define LED1_ON 1 << 0

#define MODER7_OUT 1<<14
#define LED2_ON 1 << 7

#define MODER14_OUT 1<<28
#define LED3_ON 1<<14

volatile uint32_t tick;
volatile uint32_t _tick;

void GPIO_Init(void);
void User_LED1_on(void);
void User_LED2_on(void);
void User_LED3_on(void);
void User_LED1_off(void);
void User_LED2_off(void);
void User_LED3_off(void);

/**
 * Systick configuration for delay implementation
 */
void DelayInit(void);
uint32_t getTick(void);
void delay(uint32_t seconds);

#define Green_LED() __asm("SVC #0")
#define Blue_LED() __asm("SVC #1")
#define Red_LED() __asm("SVC #2")
#define LED_Off() __asm("SVC #3")

int main(void){
	GPIO_Init();
	DelayInit();

	while(1){
		Green_LED();
		delay(1);
		LED_Off();
		Blue_LED();
		delay(1);
		LED_Off();
		Red_LED();
		delay(1);
		LED_Off();
	}

}

void GPIO_Init(void){
	RCC->AHB1ENR |= RCC_GPIOB_EN;
	GPIOB->MODER |= MODER0_OUT;
	GPIOB->MODER |= MODER7_OUT;
	GPIOB->MODER |= MODER14_OUT;
}

void DelayInit(void){
	// Update SystemCoreClock value
	SystemCoreClockUpdate();
	// Configure the SysTick timer to overflow every 1 s
	SysTick_Config(SystemCoreClock/100U);
	__enable_irq();
}

void SysTick_Handler(void){
	++tick;

}

uint32_t getTick(void){

	// Critical section
	__disable_irq();
	_tick = tick;
	__enable_irq();

	return _tick;
}

void delay(uint32_t seconds){
	seconds *= 100;
	uint32_t beginning = getTick();
	while(getTick() - beginning < seconds){

	}

}

void User_LED1_on(void){
	GPIOB->ODR |= LED1_ON;

}

void User_LED2_on(void){
	GPIOB->ODR |= LED2_ON;

}

void User_LED3_on(void){
	GPIOB->ODR |= LED3_ON;

}

void User_LED1_off(void){
	GPIOB->ODR = 0U;

}

void User_LED2_off(void){
	GPIOB->ODR = 0U;

}

void User_LED3_off(void){
	GPIOB->ODR = 0U;

}

void SVC_Handler(void)
{
  __asm(
    ".global SVC_Handler_Main\n"
    "TST lr, #4\n"
    "ITE EQ\n"
    "MRSEQ r0, MSP\n"
    "MRSNE r0, PSP\n"
    "B SVC_Handler_Main\n"
  ) ;
}

void SVC_Handler_Main( unsigned int *svc_args )
{
  unsigned int svc_number;

  /*
  * Stack contains:
  * r0, r1, r2, r3, r12, r14, the return address and xPSR
  * First argument (r0) is svc_args[0]
  */
  svc_number = ( ( char * )svc_args[ 6 ] )[ -2 ] ;
  switch( svc_number )
  {
    case 0:  
			GPIOB->ODR |= LED1_ON;
	break;
	case 1:
			GPIOB->ODR |= LED2_ON;
    break;
	case 2:
			GPIOB->ODR |= LED3_ON;
	break;
	case 3:
			GPIOB->ODR = 0U;
	break;
    default:    /* unknown SVC */
    break;
  }
}

