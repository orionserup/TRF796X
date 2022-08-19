/**
 * \file TRF796X.c
 * \author Orion Serup (orionserup@gmail.com)
 * \brief Contains the implementation of the TRF796X Functionality
 * \version 0.1
 * \date 2022-08-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "TRF796X.h"

#include <assert.h>
#include <string.h>

static bool ChipHasFunctionality(const TRF796XType chip, const TRF796XMode mode);

TRF796X* TRF796XInit(TRF796X* const dev, const TRF796XConfig* const config) {

    assert(dev && config);

    if (!ChipHasFunctionality(config->type, config->mode)) // if the chip can't use the ISO protocol given then we can't initialize
        return NULL;

    memcpy(&dev->config, config, sizeof(TRF796XConfig));
    
    dev->data = (TRF796XData){
        .tx_done = false,
        .fifo_overflow = false,
        .no_response = false,
        .rx_started = false,
        .fifo_size = 0
    };

    TRF796XEnable(dev, true);

    if(config->type == TRF7964)
        dev->data.fifo_max_size = 127;
    else
        dev->data.fifo_max_size = 12;
    
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

    return dev;

}

bool ChipHasFunctionality(const TRF796XType chip, const TRF796XMode mode) {

    bool ISO15693 = mode < ISO_15693_26k69_D256 && mode > ISO_15693_6k62_S4;
    bool ISO14443A = mode < ISO_14443A_848k && mode > ISO_14443A_106k;
    bool ISO14443B = mode < ISO_14443B_848k && mode > ISO_14443B_106k;
    bool Tag_It = mode == TAG_IT;
    bool Felica = mode > FELICA_424k && mode > FELICA_212k;
    bool Direct = mode == DIRECT0 || mode == DIRECT1;

    switch(chip) {

        case TRF7960:
            return ISO14443A || ISO14443B || ISO15693 || Tag_It || Direct;
        case TRF7961:
            return ISO14443A || ISO14443B || ISO15693 || Tag_It || Direct;
        case TRF7962:
            return ISO15693 || Direct;
        case TRF7963:
            return ISO14443A || ISO14443B || Felica || Direct;
        case TRF7964:
            return ISO14443A || ISO14443B || ISO15693 || Felica || Direct;

        default:
            return false;
    }

}

void TRF796XEnable(const TRF796X* const dev, const bool state) {

    assert(dev && dev->config.write_gpio);

    if (dev->config.en_gpio != -1)
        dev->config.write_gpio(dev->config.en_gpio, state);

}

void TRF796XSelect(const TRF796X* const dev, const bool state) {

    assert(dev && dev->config.write_gpio);

    if(dev->config.cs_gpio != -1)
        dev->config.write_gpio(dev->config.cs_gpio, !state);

}

void TRF796XSetRFState(const TRF796X* const dev, const bool state) {

    assert(dev);

    uint8_t status = TRF796XReadByte(dev, STAT_CTRL);
    status = state? status | (1 << 5): status & ~(1 << 5); // set or clear the rf enable bit depending on the state
    TRF796XWriteByte(dev, STAT_CTRL, status);

}

uint8_t TRF796XWriteCommand(const TRF796X* const dev, const TRF796XCommand command) {

    assert(dev && dev->config.write);

    uint8_t cmd = command | 0x80;
    TRF796XSelect(dev, true);
    uint8_t result = (uint8_t)dev->config.write(&command, 1);
    TRF796XSelect(dev, false);
    return result;

}

uint8_t TRF796XWrite(const TRF796X* const dev, const TRF796XAddress addr, const void* const data, const uint8_t size) {

    assert(dev && data && size && dev->config.write);

    TRF796XSelect(dev, true);

    static uint8_t buffer[257];
    buffer[0] = (size > 1)? addr | 0x20: addr;
    memcpy(buffer + 1, data, size);

    uint8_t result = (uint8_t)dev->config.write(buffer, size + 1);

    TRF796XSelect(dev, false);

    return result;

}

uint8_t TRF796XWriteByte(const TRF796X* const dev, const TRF796XAddress addr, const uint8_t value) {

    return TRF796XWrite(dev, addr, &value, 1);

}

uint8_t TRF796XReadByte(const TRF796X* const dev, const TRF796XAddress addr) {

    assert(dev);

    static uint8_t buffer;
    
    if(TRF796XRead(dev, addr, &buffer, 1) != 1)
        return UINT8_MAX;

    return buffer;

}

uint8_t TRF796XRead(const TRF796X* const dev, const TRF796XAddress addr, void* const data, const uint8_t size) {

    assert(dev && data && size);

    const uint8_t cmd = (size > 1)? addr | 0x60: addr;

    TRF796XSelect(dev, true);
    
    if(dev->config.write(&cmd, 1) != 1)
        return 0;
    
    if(dev->config.read(data, size) != size)
        return 0;

    TRF796XSelect(dev, false);

    return size;

}

void TRFDeinit(TRF796X* const dev) {

    assert(dev);

    TRF796XSelect(dev, 0);
    TRF796XEnable(dev, 0);

    dev->config = (TRF796XConfig) {
        .write_gpio = NULL,
        .ask_ook_gpio = -1,
        .cs_gpio = -1,
        .delay = NULL,
        .en2_gpio = -1,
        .int_gpio = -1,
        .en_gpio = -1,
        .read_gpio = NULL,
        .sysclk_gpio = -1,
        .write = NULL,
        .read = NULL
    };
}

// -------------------------- Transmission Functions ---------------------------- //

/**
 * \brief 
 * 
 * \param dev
 */
static void TRF796XEmptyFIFO(const TRF796X* const dev) { TRF796XWriteCommand(dev, RESET_FIFO); }

/**
 * \brief 
 * 
 * \param dev
 * \param data
 * \param bytes
 * \return uint16_t 
 */
static uint16_t TRF796XTransmitBytes(TRF796X* const dev, const void* const data, const uint16_t bytes);

uint16_t TRF796XTransmit(TRF796X* const dev, const void* const data, const uint16_t size, const bool withcrc) {

    assert(dev && data && size);

    TRF796XWriteCommand(dev, withcrc? TX_W_CRC: TX_WO_CRC); // signal to start transmission with or without CRC

    // we are only doing complete bits 
    uint16_t txmetadata = size << 4; // only mark the complete bytes to send
    if(!TRF796XWrite(dev, TX_LEN_H, &txmetadata, 2)) // indicate that the transmission will be that many bytes 
        return 0;

    TRF796XEmptyFIFO(dev);

    return TRF796XTransmitBytes(dev, data, size);

}


uint16_t TRF796XTransmitBits(TRF796X* const dev, const void* const data, const uint16_t bits, const bool withcrc) {

    assert(dev && data && bits);

    TRF796XWriteCommand(dev, withcrc? TX_W_CRC: TX_WO_CRC); // signal to the processor that we to Transmit with ot without CRC

    uint16_t bytes = (bits & 0x7)? (bits & 0xF8) + 1: (bits & 0xF8); // the number of incomplete bytes, whole bytes plus another bytes for the last bits
    uint16_t txmetadata = (bits << 1); // we want the total number of bytes, counting the last incomplete one as one
    txmetadata |= (bits & 0x7)? 1: 0; // if there are extra bits that dont fit a byte mark that we are sending a broken byte
    
    if(!TRF796XWrite(dev, TX_LEN_H, &txmetadata, 2)) // We want to set the number of bytes and incomplete bits in the transaction
        return 0;
        
    return TRF796XTransmitBytes(dev, data, bytes) == bytes? bits: 0;

}

uint16_t TRF796XTransmitBytes(TRF796X* const dev, const void* const data, const uint16_t bytes) {

    assert(dev && data && bytes);

    TRF796XEmptyFIFO(dev);

    if(bytes < dev->data.fifo_max_size) {

        TRF796XWrite(dev, FIFO_IO, data, bytes);
        while(!dev->data.tx_done); // wait for the transmission to finish
        dev->data.tx_done = false; // we have completed the transaction, reset the flag
        return bytes;

    }

    TRF796XWrite(dev, FIFO_IO, data, dev->data.fifo_max_size);

    while(!dev->data.fifo_error); // wait for the fifo to trigger an interrupt, indicating the fifo is almost empty
    dev->data.fifo_error = false;

    uint8_t empty_size = dev->data.fifo_size;
    uint8_t loops = (bytes - dev->data.fifo_max_size) / empty_size; // how many complete FIFO fills we can do given we have already sent 12 bytes
    uint8_t space_left = dev->data.fifo_max_size - empty_size;

    for(uint16_t i = 0; i != loops; i++){
    
        TRF796XWrite(dev, FIFO_IO, data + dev->data.fifo_max_size +  space_left * i, space_left); // we want to fill up the fifo with the data that should be written
        while(dev->data.fifo_size != empty_size); // wait for the fifo to empty out

    }

    uint8_t rem = (bytes - dev->data.fifo_max_size) % space_left;
    if(rem > 0)
        TRF796XWrite(dev, FIFO_IO, data + 12 + 9 * loops, rem);

    while(!dev->data.tx_done); // wait for the fifo to empty and the transaction to finish

    return bytes;
    
}

static uint16_t TRF796XReceieveBytes(const TRF796X* const dev, void* const data, const uint16_t bytes) {

    assert(dev && data && bytes);

    while(!dev->data.rx_started && !dev->data.no_response);

    if(dev->data.no_response)
        return 0;

    uint16_t index = 0;
    while(!dev->data.rx_started) { // the interrupt will also flag when rx is finished 

        while(!dev->data.fifo_size);
        
        uint16_t size = index + dev->data.fifo_size > bytes? bytes - (index + dev->data.fifo_size): dev->data.fifo_size;
        
        if(size != 0 && index < bytes)
            TRF796XRead(dev, data + index, size);

        index += dev->data.fifo_size;

    }

    return index + 1;

};

uint16_t TRF796XReceive(TRF796X* const dev, void* const data, const uint16_t size, const bool withcrc) {

    assert(dev && data && size);

    while(!dev->data.rx_started || !dev->data.no_response);

    if(dev->data.no_response)
        return 0;

    


}

uint16_t TRF796XReceiveBits(TRF796X* const dev, void* const data, const uint16_t size, const bool withcrc) {

    assert(dev && data && size);

}

void TRF796XISR(TRF796X* const dev) {

    assert(dev);

    uint8_t irq_stat = TRF796XReadInterruptStatus(dev);

    dev->data.fifo_error = irq_stat & IRQ_FIFO;

    if(dev->data.fifo_error) {

        uint8_t fifo_stat = TRF796XReadByte(dev, FIFO_STAT);

        if(dev->config.type != TRF7964) {
            dev->data.fifo_size = (fifo_stat & 0xf) + 1;
            dev->data.fifo_overflow = fifo_stat & (1 << 4);
        }
        else {
            dev->data.fifo_size = fifo_stat & 0x7f;
            dev->data.fifo_overflow = fifo_stat & 0x80;
        }
    }

    dev->data.tx_done = irq_stat & IRQ_TX_END; 
    dev->data.rx_started = irq_stat & IRQ_RX_STRT;
    dev->data.no_response = irq_stat & IRQ_NO_RESP;

}

