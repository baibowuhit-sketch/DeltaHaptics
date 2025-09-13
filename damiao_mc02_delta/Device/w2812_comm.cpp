/*
 * w2812_comm.cpp
 *
 *  Created on: Mar 10, 2025
 *      Author: 19325
 */


#include "w2812_comm.h"
#include "main.h"

#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;

#define WS2812_LowLevel    0xC0     // 0
#define WS2812_HighLevel   0xF0     // 1

W2812Comm &W2812Comm::construction(void) {
    static W2812Comm obj;
    return obj;
}

W2812Comm::W2812Comm(){

}

void W2812Comm::ctrlColor(U8 r, U8 g, U8 b)
{
    U8 txbuf[24] = {0};
    U8 reset_data[100] = {0};
    for (U16 i = 0U; i < 8U; i++)
    {
        txbuf[7-i]  = (((g>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[15-i] = (((r>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[23-i] = (((b>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }

    HAL_SPI_Transmit(&WS2812_SPI_UNIT, reset_data, 0, 0xFFFF);
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);

    HAL_SPI_Transmit(&WS2812_SPI_UNIT, reset_data, 100, 0xFFFFFFFF);
}


