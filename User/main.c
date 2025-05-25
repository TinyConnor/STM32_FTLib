#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_led.h"


int main(void)
{
	uint8_t ch;
	USART_Config();
	LED_GPIO_Config();
	// Usart_SendByte(DEBUG_USARTx, 100);
	printf("hello world\n");

	while (1) {
		ch = getchar();
		switch (ch) {
		case '1':
			LED_RED;
			break;
		case '2':
			LED_GREEN;
			break;
		case '3':
			LED_BLUE;
			break;
		case '4':
			LED_YELLOW;
			break;
		case '5':
			LED_PURPLE;
			break;
		case '6':
			LED_CYAN;
			break;
		case '7':
			LED_WHITE;
			break;
		default:
			LED_RGBOFF;
			break;
		}
	}
}
