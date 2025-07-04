#include "dma_mtm.h"

/* 定义aSRC_Const_Buffer数组作为DMA传输数据源 const关键字将aSRC_Const_Buffer数组变量定义为常量类型 表示数据存储在内部的FLASH中 */
const uint32_t aSRC_Const_Buffer[BUFFER_SIZE] = {
                                    0x01020304,0x05060708,0x090A0B0C,0x0D0E0F10,
                                    0x11121314,0x15161718,0x191A1B1C,0x1D1E1F20,
                                    0x21222324,0x25262728,0x292A2B2C,0x2D2E2F30,
                                    0x31323334,0x35363738,0x393A3B3C,0x3D3E3F40,
                                    0x41424344,0x45464748,0x494A4B4C,0x4D4E4F50,
                                    0x51525354,0x55565758,0x595A5B5C,0x5D5E5F60,
                                    0x61626364,0x65666768,0x696A6B6C,0x6D6E6F70,
                                    0x71727374,0x75767778,0x797A7B7C,0x7D7E7F80 };
/* 定义DMA传输目标存储器  存储在内部的SRAM中 */
uint32_t aDST_Buffer[BUFFER_SIZE];


void DMA_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    /* 开启DMA时钟 */
    RCC_AHBPeriphClockCmd(DMA_CLOCK, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)aSRC_Const_Buffer;              /**< DMA源地址:串口数据寄存器地址 */
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) aDST_Buffer;                    /**< 内存地址 */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                        /**< 传输方向:从内存到外设 */
    DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;                         /**< 传输数据大小 */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;          /**< 外设地址不增 */
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                   /**< 内存地址自增 */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;   /**< 外设数据单位 */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;           /**< 内存数据单位 */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                             /**< DMA模式:循环或一次 */
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                     /**< 优先级:中 */
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;                              /**< 禁止内存到内存传输 */

    /* 配置DMA通道 */
    DMA_Init(DMA_CHANNEL, &DMA_InitStructure);
    /* 清楚DMA数据流传输完成标志位 */
    DMA_ClearFlag(DMA_FLAG_TC);
    /* 使能DMA */
    DMA_Cmd(DMA_CHANNEL, ENABLE);
}


/**
 * @brief 判断两个数据源是否相等
 * @param pBuffer 被比较数据指针
 * @param pBuffer1 比较的数据指针
 * @param BufferLength 比较的数据长度
 * @return 成功返回1 失败返回0
 */
uint8_t Buffercmp(const uint32_t *pBuffer, uint32_t *pBuffer1, uint16_t BufferLength)
{
    while (BufferLength--) {
        if (*pBuffer != *pBuffer1)
            return 0;
        pBuffer++;
        pBuffer1++;
    }
    return 1;
}

