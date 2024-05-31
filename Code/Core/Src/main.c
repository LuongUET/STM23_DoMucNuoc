#include "main.h"
#include "config_clock.h"
#include "dwt_delay.h"
#include "stdio.h"

#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define SR04_TRIG_Pin GPIO_PIN_9
#define SR04_TRIG_GPIO_Port GPIOA
#define DC_Pin GPIO_PIN_4
#define DC_GPIO_Port GPIOA

// HC_SR04 variables
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

TIM_HandleTypeDef htim1;

static void GPIO_Init(void);
static void TIM1_Init(void);
static void gpio_init(void);
static void timer1_input_capture1_init(void);
static void HCSR04_Read (void)
{
	SR04_TRIG_GPIO_Port->BSRR = SR04_TRIG_Pin; // set chan TRIG High (PA9)
	delay_us(10);  // wait 10 us	
	SR04_TRIG_GPIO_Port->BSRR = (uint32_t)SR04_TRIG_Pin << 16u; // set chan TRIG Low (PA9)
	
	/* Enable the TIM Capture/Compare 1 interrupt */
	TIM1->DIER |= TIM_DIER_CC1IE;
	
	/* Enable the Input Capture channel */
  uint32_t tmp;
  tmp = TIM_CCER_CC1E << (TIM_CHANNEL_1 & 0x1FU); /* 0x1FU = 31 bits max shift */
  TIM1->CCER &= ~tmp;  /* Reset the CCxE Bit */

  TIM1->CCER |= (uint32_t)(TIM_CCx_ENABLE << (TIM_CHANNEL_1 & 0x1FU)); /* 0x1FU = 31 bits max shift */	
	TIM1->CR1 |= TIM_CR1_CEN; // Enable Timer1
}

void USART_Config(void) {
    // Enable clock cho USART2 và GPIOA
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

 // Config PA2 to TX (Output 50 MHz, Alternate function push-pull)
    GPIOA->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2); // Clear current mode and configuration bits for PA2
    GPIOA->CRL |= (GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1); // Set PA2 to Output mode, max speed 50 MHz, Alternate function output Push-pull

    // Config PA3 to RX (Input floating)
    GPIOA->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3); // Clear current mode and configuration bits for PA3
    GPIOA->CRL |= GPIO_CRL_CNF3_0; // Set PA3 to Input mode, Floating input


    // Config baud rate: 115200, SystemClock = 72Mhz, APB1/2 = 36Mhz
    // USARTDIV = fCK / (16 * baudrate)
    // 36 MHz / (16 * 115200) = 19.53125
    // Mantissa = 19, Fraction = 0.53125 * 16 = 8
    USART2->BRR = (234 << 4) | 6; // BRR = 0xEA6

    // Config USART (8-N-1, Enable TX và RX)
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void USART_SendChar(char c) {
    while (!(USART2->SR & USART_SR_TXE)); // Cho den khi TXE (Transmit Data Register Empty) duoc set
    USART2->DR = c;
}

void USART_SendString(const char *str) {
    while (*str) {
        USART_SendChar(*str++);
    }
}

uint8_t status;
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	gpio_init();
	USART_Config();
	timer1_input_capture1_init();
	delay_init();
	
  while (1)
  {
		uint32_t odr = LED_GPIO_Port->ODR;;
		HCSR04_Read();
		LED_GPIO_Port->BSRR = ((odr & LED_Pin) << 16) | (~odr & LED_Pin);

		delay_us(200000);
		
		if(Distance > 15){
			DC_GPIO_Port->BSRR = DC_Pin;
			status = 1;
		}
		else if(Distance < 5){
			DC_GPIO_Port->BSRR = (uint32_t)DC_Pin << 16u;
			status = 0;
		}
			char buffer[20];
			sprintf(buffer,"%d\n",(int)Distance);
			USART_SendString(buffer);
  }
}

static void timer1_input_capture1_init(void){
	// Enable clock cho TIM1 và GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_IOPAEN;

		// COnfig PA8 is Input capture (Alternate Function Input)
	GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8); // Xóa các bit c?u hình cu
	GPIOA->CRH |= GPIO_CRH_CNF8_1; // Input floating (CNF8[1:0] = 01)


	TIM1->PSC = 71; // Prescaler = 72-1, clock timer là 72 MHz / 72 = 1 MHz

	TIM1->ARR = 65535; // Max couter = 65535

	// Cau hình kênh 1 cua TIM1 là Input Capture
	TIM1->CCMR1 &= ~TIM_CCMR1_CC1S; // clear bit CC1S
	TIM1->CCMR1 |= TIM_CCMR1_CC1S_0; // (CC1 channel is configured as input, IC1 is mapped on TI1)

	// Config polarity và prescaler cho kênh 1
	TIM1->CCER &= ~TIM_CCER_CC1P; //  active rising edge (polarity)
	TIM1->CCMR1 &= ~TIM_CCMR1_IC1PSC; // khong chia tan (prescaler = 1)
	TIM1->CCMR1 &= ~TIM_CCMR1_IC1F; // No filter

	// Enable capture/compare cho kênh 1
	TIM1->CCER |= TIM_CCER_CC1E;

	// Enable counter TIM1
	TIM1->CR1 |= TIM_CR1_CEN;



	// Enable EXTI capture/compare cho kênh 1
	TIM1->DIER |= TIM_DIER_CC1IE;

	// Config NVIC cho EXTI TIM1
	NVIC_EnableIRQ(TIM1_CC_IRQn);
	NVIC_SetPriority(TIM1_CC_IRQn, 1);

}

static void gpio_init(void){
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // Enable IO port C Clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPDEN; // Enable IO port D Clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable IO port A Clock
	
	/* Config Led Pin PC13*/
	GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13); // clear bit
	GPIOC->CRH |= (GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0); //output 50 MHz
	GPIOC->CRH |= GPIO_CRH_CNF13_0; // push-pull (CNF13[1:0] = 00)

	/* Config Trig Pin PA9*/
	GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9); 
	GPIOA->CRH |= (GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0); 
	GPIOA->CRH |= GPIO_CRH_CNF9_0;

	/* Config LED3 Pin PA4 */
	GPIOA->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4); // Clear bits for PA4
	GPIOA->CRL |= (GPIO_CRL_MODE4_1 | GPIO_CRL_MODE4_0); // Output mode, max speed 50 MHz
	GPIOA->CRL |= GPIO_CRL_CNF4_0; // General purpose output push-pull
}


/* TIMER1 Capture Compare Isr */
void TIM1_CC_IRQHandler(void)
{
	
		TIM1->SR = ~TIM_DIER_CC1IE; // clear IT flag
	
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = TIM1->CCR1; // read first value
			
			Is_First_Captured = 1;  // set the first captured as true
			
			// Now change the polarity to falling edge
			TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); // reset
			TIM1->CCER |= TIM_CCER_CC1P; // set TIM_INPUTCHANNELPOLARITY_FALLING
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = TIM1->CCR1; // read second value

			TIM1->CNT = 0; // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2 - IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); // reset
			TIM1->CCER |= 0; // set TIM_INPUTCHANNELPOLARITY_RISING
						
			/* Disable the TIM Capture/Compare 1 interrupt */
			TIM1->DIER &= ~(TIM_DIER_CC1IE);
	
			/* Disable the Input Capture channel */
			uint32_t tmp;
			tmp = TIM_CCER_CC1E << (TIM_CHANNEL_1 & 0x1FU); /* 0x1FU = 31 bits max shift */
			TIM1->CCER &= ~tmp;  /* Reset the CCxE Bit */

			TIM1->CCER |= (uint32_t)(TIM_CCx_DISABLE << (TIM_CHANNEL_1 & 0x1FU)); /* 0x1FU = 31 bits max shift */	
			
			TIM1->CR1 &= ~(TIM_CR1_CEN); // Disable Timer1
		}
}
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

