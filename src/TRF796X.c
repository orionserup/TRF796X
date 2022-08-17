/**
 * \file TRF796X.c
 * \author Orion Serup (orionserup@gmail.com)
 * \brief 
 * \version 0.1
 * \date 2022-08-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "TRF796X.h"
#include <assert.h>
#include <string.h>

TRF796X* TRF796XInit(TRF796X* const dev, const TRF796XConfig* const config) {

    if(dev->config.usespi)
        TRF796XSetState(on);

    uint8_t status = TRF796XReadByte(dev, STAT_CTRL);
    if(dev->config.mode == DIRECT0 || dev->config.mode == DIRECT1)
        status |= 0x40;
    
    TRF796XWriteByte(dev, STAT_CTRL, status);

    uint8_t isostat = TRF796XReadByte(dev, ISO_CTRL);
    if(dev->config.mode != DIRECT0 || dev->config.mode != DIRECT1){

        isostat &= ~0x1f;
        isostat |= dev->config.mode;
    
    }
    else {

        isostat &= ~0x20;
        isostat |= (dev->config.mode == DIRECT1);

    }

}

void TRF796XEnable(const TRF796X* const dev, const bool state) {

    if (dev->config.en_gpio != -1)
        dev->config.write_gpio(dev->config.en_gpio, state);

}

void TRF796XSelect(const TRF796X* const dev, const bool state) {

    if(dev->config.cs_gpio != -1)
        dev->config.write_gpio(dev->config.cs_gpio, !state);

}

void TRF796XSetRFState(const TRF796X* const dev, const bool state) {

    uint8_t status = TRF796XReadByte(dev, STAT_CTRL);
    status |= state << 5;
    TRF796XWriteByte(dev, STAT_CTRL, status);

}

uint8_t TRF796XWriteCommand(const TRF796X* const dev, const TRF796XCommand command) {

    uint8_t cmd = command | 0x80;
    TRF796XSelect(dev, true);
    uint8_t result = (uint8_t)write(&command, 1);
    TRF796XSelect(dev, false);
    return result;

}

uint8_t TRF796XWrite(const TRF796X* const dev, const TRF796XAddress addr, const void* const data, const uint8_t size) {

    if(dev->config.write == NULL || !data || size == 0)
        return;

    TRF796XSelect(dev, true);

    static uint8_t buffer[257];
    buffer[0] = (size > 1)? addr | 0x20: addr;
    memcpy(data, buffer + 1, size);

    uint8_t result = (uint8_t)write(buffer, size + 1);

    TRF796XSelect(dev, false);

    return result;

}

uint8_t TRF796XWriteByte(const TRF796X* const dev, const TRF796XAddress addr, const uint8_t value) {

    return TRF796XWrite(dev, addr, &value, 1);

}

uint8_t TRF796XReadByte(const TRF796X* const dev, const TRF796XAddress addr) {

    static uint8_t buffer;
    
    if(TRF796XRead(dev, addr, &buffer, 1) != 1)
        return UINT8_MAX;

    return buffer;

}

uint8_t TRF796XRead(const TRF796X* const dev, const TRF796XAddress addr, void* const data, const uint8_t size) {

    if(!data || size == 0 || dev->config.write == NULL || dev->config.read == NULL)
        return;

    const uint8_t cmd = (size > 1)? addr | 0x60: addr;

    TRF796XSelect(dev, true);
    
    if(write(&cmd, 1) != 1)
        return 0;
    
    if(read(&data, size) != size)
        return 0;

    TRF796XSelect(dev, false);

    return size;

}

void TRFDeinit(TRF796X* const dev) {

    TRF796XSelect(dev, 0);
    TRF796XEnable(dev, 0);

}

// -------------------------- Transmission Functions ---------------------------- //

/**
 * \brief 
 * 
 */
static void TRF796XEmptyFIFO(const TRF796X* const dev);

const static uint8_t fifo_max_size = 12;

static uint16_t TRF796XTransmitBytes(TRF796X* const dev, const void* const data, const uint16_t bytes);

uint16_t TRF796XTransmit(TRF796X* const dev, const void* const data, const uint16_t size, const bool withcrc) {

    if(!dev || !data || size == 0)
        return 0;

    TRF796XWriteCommand(dev, withcrc? TX_W_CRC: TX_WO_CRC); // signal to start transmission with or without CRC

    // we are only doing complete bits 
    uint16_t txmetadata = size << 4; // only mark the complete bytes to send
    if(!TRF796XWrite(dev, TX_LEN_H, &txmetadata, 2)) // indicate that the transmission will be that many bytes 
        return 0;

    TRF796XEmptyFIFO(dev);

    return TRF796XTransmitBytes(dev, data, size);

}


uint16_t TRF796XTransmitBits(TRF796X* const dev, const void* const data, const uint16_t bits, const bool withcrc) {

    if(!dev || !data || bits == 0)
        return 0;

    TRF796XWriteCommand(dev, withcrc? TX_W_CRC: TX_WO_CRC); // signal to the processor that we to Transmit with ot without CRC

    uint16_t bytes = (bits & 0x7)? (bits & 0xF8) + 1: (bits & 0xF8); // the number of incomplete bytes, whole bytes plus another bytes for the last bits
    uint16_t txmetadata = (bits << 1); // we want the total number of bytes, counting the last incomplete one as one
    txmetadata |= (bits & 0x7)? 1: 0; // if there are extra bits that dont fit a byte mark that we are sending a broken byte
    
    if(!TRF796XWrite(dev, TX_LEN_H, &txmetadata, 2)) // We want to set the number of bytes and incomplete bits in the transaction
        return 0;
        
    return TRF796XTransmitBytes(dev, data, bytes) == bytes? bits: 0;

}

uint16_t TRF796XTransmitBytes(TRF796X* const dev, const void* const data, const uint16_t bytes) {

    TRF796XEmptyFIFO(dev);

    if(bytes < 12) {

        TRF796XWrite(dev, FIFO_IO, data, bytes);
        while(!dev->data.tx_done); // wait for the transmission to finish
        dev->data.tx_done = false; // we have completed the transaction, reset the flag
        return bytes;

    }

    TRF796XWrite(dev, FIFO_IO, data, 12);
    dev->data.fifo_empty = false;
    while(!dev->data.fifo_empty); // wait for the fifo to have only 3 bytes left

    uint8_t loops = (bytes - 12) / 9; // how many complete FIFO fills we can do given we have already sent 12 bytes

    for(int i = 0; i != loops; i++){
    
        TRF796XWrite(dev, FIFO_IO, data + 12 + 9 * i, 9); // we want to fill up the fifo with the data that should be written
        dev->data.fifo_empty = false;
        while(!dev->data.fifo_empty); // wait for the fifo to empty out

    }

    uint8_t rem = (bytes - 12) % 9;
    if(rem > 0) {

        TRF796XWrite(dev, FIFO_IO, data + 12 + 9 * loops, rem);
        dev->data.fifo_empty = false;
        while(!dev->data.fifo_empty);
    
    }

    while(!dev->data.tx_done); // wait for the fifo to empty and the transaction to finish
    dev->data.tx_done = false;

    return bytes;
    
}

static uint16_t TRF796XReceieveBytes(const TRF796X* const dev, void* const data, const uint16_t bytes) {

    while(!dev->data.rx_started);

    uint16_t index = 0;

};

uint16_t TRF796XReceive(TRF796X* const dev, void* const data, const uint16_t size, const bool withcrc) {

    if(!data || size == 0)
        return 0;

}

uint16_t TRF796XReceiveBits(TRF796X* const dev, void* const data, const uint16_t size, const bool withcrc) {



}

void TRF796XISR(TRF796X* const dev) {

    uint8_t irq_stat = TRF796XReadInterruptStatus(dev);

    if(irq_stat & IRQ_FIFO) {

        uint8_t fifo_stat = TRF796XReadByte(dev, FIFO_STAT);

        dev->data.fifo_size = (fifo_stat & 0xf) + 1;
        dev->data.fifo_full = fifo_stat & (1 << 6);
        dev->data.fifo_empty = fifo_stat & (1 << 5);
        dev->data.fifo_overflow = fifo_stat & (1 << 4);

    }

    dev->data.tx_done = irq_stat & IRQ_TX_END; 
    dev->data.rx_started = irq_stat & IRQ_RX_STRT;
    dev->data.no_response = irq_stat & IRQ_NO_RESP;

}

void TRF796XSelect(const TRF796X* const dev, const bool state) {

    if(dev->config.usespi && dev->config.cs_gpio != -1)
        dev->config.write_gpio(dev->config.cs_gpio, !state);

}
