#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_led.h"
#include "dma_mtm.h"

static void Delay(__IO uint32_t nCount);

#if 0

extern uint8_t SendBuff[SENDBUFF_SIZE];

int main(void)
{
	uint16_t i;
	USART_Config();
	USARTx_DMA_Config();
	LED_GPIO_Config();

	/* 填充传输数据数组 */
	for (i = 0;i < SENDBUFF_SIZE;i++) {
		SendBuff[i] = 'P';
	}

	///可以将DMA设置为循环模式

	/* USART1 向 DMA发出TX请求 */
	USART_DMACmd(DEBUG_USARTx, USART_DMAReq_Tx, ENABLE);

	while (1) {
		Delay(0xffff);
		LED1_TOGGLE;
	}

}
#else
extern const uint32_t aSRC_Const_Buffer[BUFFER_SIZE];
extern uint32_t aDST_Buffer[BUFFER_SIZE];

int main()
{
	/* 定义存放比较结果变量 */
	uint8_t TransferStatus;

	LED_GPIO_Config();
	LED_PURPLE;
	Delay(0xffff);
	DMA_Config();

	/* 等待DMA传输完成 */
	while (DMA_GetFlagStatus(DMA_FLAG_TC) == RESET);

	/* 比较传输后的数据 */
	TransferStatus = Buffercmp(aSRC_Const_Buffer, aDST_Buffer, BUFFER_SIZE);

	/* 相同亮红灯 不同亮蓝灯 */
	if (TransferStatus == 0) {
		LED_RED;
	} else {
		LED_BLUE;
	}


	while (1) {

	}
}

#endif

/**
 * @brief 延时函数
 */
static void Delay(__IO uint32_t nCount)
{
	for (; nCount != 0; nCount--);
}
