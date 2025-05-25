#include "bsp_usart.h"

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

    /* 串口中断优先级配置 */
    NVIC_Configuration();

    /* 使能串口接收中断 */
    USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);

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

