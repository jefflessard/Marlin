/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../platforms.h"

#if defined(HAL_STM32) && !defined(STM32H7xx)

#include "MarlinSPI.h"

void MarlinSPI::setClockDivider(uint8_t _div) {
  //TODO unused variables, maybe set _spi.handle.Init.BaudRatePrescaler instead
  _speed = spi_getClkFreq(&_spi);// / _div;
  _clockDivider = _div; 
}

void MarlinSPI::begin(void) {
  //TODO: only call spi_init if any parameter changed!!
  spi_init(&_spi, _speed, _dataMode, _bitOrder);

  // spi_init set 8bit always
  // TODO: copy the code from spi_init and handle data size, to avoid double init always!!
  if (_dataSize != SPI_DATASIZE_8BIT) {
    _spi.handle.Init.DataSize = _dataSize;
  }

   //Configure Rx&Tx DMAs
  _dmaRx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  _dmaRx.Init.PeriphInc = DMA_PINC_DISABLE;
  _dmaRx.Init.MemInc = DMA_MINC_ENABLE; //TODO minc ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
  _dmaRx.Init.PeriphDataAlignment = _spi.handle.Init.DataSize == SPI_DATASIZE_8BIT ? DMA_PDATAALIGN_BYTE : DMA_PDATAALIGN_HALFWORD;
  _dmaRx.Init.MemDataAlignment = _spi.handle.Init.DataSize == SPI_DATASIZE_8BIT ? DMA_PDATAALIGN_BYTE : DMA_PDATAALIGN_HALFWORD;
  _dmaRx.Init.Mode = DMA_NORMAL;
  _dmaRx.Init.Priority = DMA_PRIORITY_LOW;

  _dmaTx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  _dmaTx.Init.PeriphInc = DMA_PINC_DISABLE;
  _dmaTx.Init.MemInc = DMA_MINC_ENABLE; //TODO minc ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
  _dmaTx.Init.PeriphDataAlignment = _spi.handle.Init.DataSize == SPI_DATASIZE_8BIT ? DMA_PDATAALIGN_BYTE : DMA_PDATAALIGN_HALFWORD;
  _dmaTx.Init.MemDataAlignment = _spi.handle.Init.DataSize == SPI_DATASIZE_8BIT ? DMA_PDATAALIGN_BYTE : DMA_PDATAALIGN_HALFWORD;
  _dmaTx.Init.Mode = DMA_NORMAL;
  _dmaTx.Init.Priority = DMA_PRIORITY_LOW;

  #ifdef STM32F4xx
    _dmaRx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    _dmaTx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  #endif

  #ifdef SPI1_BASE
    if(_spi.handle.Instance==SPI1){
      SPI1_DMA_CLK_ENABLE();
      _dmaRx.Instance = SPI1_DMA_RX;
      _dmaTx.Instance = SPI1_DMA_TX;
      #ifdef STM32F4xx
        _dmaRx.Init.Channel = DMA_CHANNEL_3;
        _dmaTx.Init.Channel = DMA_CHANNEL_3;
      #endif
    }
  #endif
  #ifdef SPI2_BASE
    if(_spi.handle.Instance==SPI2){
      SPI2_DMA_CLK_ENABLE();
      _dmaRx.Instance = SPI2_DMA_RX;
      _dmaTx.Instance = SPI2_DMA_TX;
      #ifdef STM32F4xx
        _dmaRx.Init.Channel = DMA_CHANNEL_0;
        _dmaTx.Init.Channel = DMA_CHANNEL_0;
      #endif
    }
  #endif
  #ifdef SPI3_BASE
    if(_spi.handle.Instance==SPI3){
      SPI3_DMA_CLK_ENABLE();
      _dmaRx.Instance = SPI3_DMA_RX;
      _dmaTx.Instance = SPI3_DMA_TX;
      #ifdef STM32F4xx
        _dmaRx.Init.Channel = DMA_CHANNEL_0;
        _dmaTx.Init.Channel = DMA_CHANNEL_0;
      #endif
    }
  #endif
  __HAL_LINKDMA(&_spi.handle,hdmarx,_dmaRx);
  __HAL_LINKDMA(&_spi.handle,hdmatx,_dmaTx);


  HALOK(HAL_SPI_Init(&_spi.handle), "HAL_SPI_Init failed");
  __HAL_SPI_ENABLE(&_spi.handle);
}

void MarlinSPI::enableDMA(){
  HALOK(HAL_DMA_Init(&_dmaRx), "DMA RX HAL_DMA_Init failed");
  HALOK(HAL_DMA_Init(&_dmaTx), "DMA TX HAL_DMA_Init failed");
}

void MarlinSPI::disableDMA(){
  HAL_DMA_Abort(&_dmaRx); //don't use HALOK here
  HAL_DMA_DeInit(&_dmaRx);
  HAL_DMA_Abort(&_dmaTx); //don't use HALOK here
  HAL_DMA_DeInit(&_dmaTx);
}

byte MarlinSPI::transfer(uint8_t _data) {
  uint8_t rxData = 0xFF;
  HALOK(HAL_SPI_TransmitReceive(&_spi.handle, &_data, &rxData, 1, HAL_MAX_DELAY), "HAL_SPI_TransmitReceive failed");
  return rxData;
}

uint8_t MarlinSPI::dmaTransfer(const void *transmitBuf, void *receiveBuf, uint16_t length) {
  // const uint8_t ff = 0xFF;
  // // check for 2 lines transfer
  // if (transmitBuf == nullptr && _spi.handle.Init.Direction == SPI_DIRECTION_2LINES && _spi.handle.Init.Mode == SPI_MODE_MASTER) {
  //   transmitBuf = &ff;
  //   mincTransmit = false;
  // }

  if(!transmitBuf){
    transmitBuf = receiveBuf;
  }

  enableDMA();

  HALOK(HAL_DMA_Start(&_dmaRx, (uint32_t)&_spi.handle.Instance->DR, (uint32_t)receiveBuf, length), "RX HAL_DMA_Start failed");
  LL_SPI_EnableDMAReq_RX(_spi.handle.Instance); // Enable Rx DMA Request

  if (_spi.handle.Init.Direction == SPI_DIRECTION_2LINES){
    HALOK(HAL_DMA_Start(&_dmaTx, (uint32_t)transmitBuf, (uint32_t)&_spi.handle.Instance->DR, length), "TX HAL_DMA_Start failed");
    LL_SPI_EnableDMAReq_TX(_spi.handle.Instance); // Enable Tx DMA Request
  }
  
  //__HAL_SPI_ENABLE(&_spi.handle);
  HALOK(HAL_DMA_PollForTransfer(&_dmaTx, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY), "HAL_DMA_PollForTransfer failed");
  //HAL_DMA_PollForTransfer(&_dmaRx, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY);

  CLEAR_BIT(_spi.handle.Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN); // Disable Rx/Tx DMA Request  

  disableDMA();

  return 1;
}

uint8_t MarlinSPI::dmaSend(const void * transmitBuf, uint16_t length, bool minc) {
  enableDMA();

  HALOK(HAL_DMA_Start(&_dmaTx, (uint32_t)transmitBuf, (uint32_t)&_spi.handle.Instance->DR, length), "TX HAL_DMA_Start failed");
  LL_SPI_EnableDMAReq_TX(_spi.handle.Instance); // Enable Tx DMA Request

  //__HAL_SPI_ENABLE(&_spi.handle);
  HALOK(HAL_DMA_PollForTransfer(&_dmaRx, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY), "TX HAL_DMA_PollForTransfer failed");
  
  CLEAR_BIT(_spi.handle.Instance->CR2, SPI_CR2_TXDMAEN); // Disable Tx DMA Request
  if (_spi.handle.Init.Direction == SPI_DIRECTION_2LINES)
  {
    __HAL_SPI_CLEAR_OVRFLAG(&_spi.handle);
  }

  disableDMA();

  return 1; 
}

#endif // HAL_STM32 && !STM32H7xx
