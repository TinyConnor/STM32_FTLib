#include "bsp_usart.h"

uint8_t SendBuff[SENDBUFF_SIZE];

/**
 * @brief 配置嵌套向量中断控制器
 */
static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 嵌套向量中断控制器组选择 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;    /**< 配置中断源 */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;/**< 抢占优先级 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;       /**< 子优先级 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          /**< 使能中断 */

    /* 初始化配置NVIC */
    NVIC_Init(&NVIC_InitStructure);
}


/**
 * @brief USART GPIO 工作参数配置
 */
void USART_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* 打开串口GPIO时钟 */
    DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
    /* 打开串口外设时钟 */
    DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

    /* TX引脚配置 */
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN; /**< TX引脚 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; /**< 复用推捥输出 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /**< 输出速率 */
    GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* RX引脚配置 */
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN; /**< RX引脚 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; /**< 浮空输入 */
    GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* 串口工作参数配置 */
    USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE; /**< 配置波特率 */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; /**< 配置帧数据字长 */
    USART_InitStructure.USART_StopBits = USART_StopBits_1; /**< 配置停止位 */
    USART_InitStructure.USART_Parity = USART_Parity_No; /**< 配置校验位 */
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;/**< 配置硬件流控制 */
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;/**< 配置工作模式，收发一起*/
    USART_Init(DEBUG_USARTx, &USART_InitStructure);

#if 0
    /* 串口中断优先级配置 */
    NVIC_Configuration();

    /* 使能串口接收中断 */
    USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);
#endif
    /* 使能串口 */
    USART_Cmd(DEBUG_USARTx, ENABLE);
}


/**
 * @brief 发送一个字符
 */
void Usart_SendByte(USART_TypeDef *pUSARTx, uint8_t ch)
{
    /* 发送一个字节数据 */
    USART_SendData(pUSARTx, ch);

    /* 等待发送数据寄存器为空 */
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

/**
 * @brief 发送字符串
 */
void Usart_SendString(USART_TypeDef *pUSARTx, char *str)
{
    uint32_t k = 0;
    do {
        Usart_SendByte(pUSARTx, *(str + k));
        k++;
    } while (*(str + k) != '\0');

    /* 等待发送完成 */
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET);
}

/**
 * @brief 发送16位数
 */
void Usart_SendHalfWord(USART_TypeDef *pUSARTx, uint16_t ch)
{
    uint8_t temp_h, temp_l;
    temp_h = (ch & 0xff00) >> 8;
    temp_l = ch & 0xff;

    /* 发送高位 */
    USART_SendData(pUSARTx, temp_h);
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);

    /* 发送低位 */
    USART_SendData(pUSARTx, temp_l);
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

/**
 * @brief 发送字符数组
 */
void Usart_SendArray(USART_TypeDef *pUSARTx, uint8_t *array, uint16_t num)
{
    uint8_t i;

    for (i = 0; i < num; i++) {
        /* 发送一个字节数据到USART */
        Usart_SendByte(pUSARTx, array[i]);

    }
    /* 等待发送完成 */
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET);
}

/**
 * @brief 重定向fputc函数
 */
int fputc(int ch, FILE *f)
{
    /* 发送一个字节到串口 */
    USART_SendData(DEBUG_USARTx, (uint8_t) ch);

    /* 等待发送数据寄存器为空 */
    while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);

    return (ch);
}

/**
 * @brief 重定向fgetc函数
 */
int fgetc(FILE *f)
{
    /* 等待串口输入数据 */
    while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);

    return (int) USART_ReceiveData(DEBUG_USARTx);
}

/**
 * @brief USARTx TX DMA配置,内存到外设(USART1->DR)
 */
void USARTx_DMA_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    /* 开启DMA时钟 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = USART_DR_ADDRESS;              /**< DMA源地址:串口数据寄存器地址 */
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32) SendBuff;                    /**< 内存地址 */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                        /**< 传输方向:从内存到外设 */
    DMA_InitStructure.DMA_BufferSize = SENDBUFF_SIZE;                         /**< 传输数据大小 */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;          /**< 外设地址不增 */
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                   /**< 内存地址自增 */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   /**< 外设数据单位 */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;           /**< 内存数据单位 */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                             /**< DMA模式:循环或一次 */
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                     /**< 优先级:中 */
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                              /**< 禁止内存到内存传输 */

    /* 配置DMA通道 */
    DMA_Init(USART_TX_DMA_CHANNEL, &DMA_InitStructure);
    /* 使能DMA */
    DMA_Cmd(USART_TX_DMA_CHANNEL, ENABLE);
}

