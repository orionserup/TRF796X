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

static TRF796XConfig cfg = {

    .ask_ook_gpio = -1,
    .int_gpio = -1,
    .mod_gpio = -1,
    
    .read_gpio = NULL,
    .write_gpio = NULL,

    .read = NULL,
    .write = NULL,
    
    .usespi = true

};

static uint8_t irq_flags = 0x0;

static uint32_t (*write)(const void* const data, const uint32_t size);
static uint32_t (*read)(void* const data, const uint32_t size);

void TRF796XInit(const TRF796XConfig* const config) {

    cfg.ask_ook_gpio = config->ask_ook_gpio;
    cfg.int_gpio = config->int_gpio;
    cfg.mod_gpio = config->mod_gpio;

    cfg.read_gpio = config->read_gpio;
    cfg.write_gpio = config->write_gpio;

    cfg.read = config->read;
    cfg.write = config->write;

    cfg.usespi = config->usespi;

    read = cfg.read;
    write = cfg.write;

    if(cfg.usespi)
        TRF796XSetState(on);

    uint8_t status = TRF796XReadByte(STAT_CTRL);
    if(cfg.mode == DIRECT0 || cfg.mode == DIRECT1)
        status |= 0x40;
    
    TRF796XWriteByte(STAT_CTRL, status);

    uint8_t isostat = TRF796XReadByte(ISO_CTRL);
    if(cfg.mode != DIRECT0 || cfg.mode != DIRECT1){

        isostat &= ~0x1f;
        isostat |= cfg.mode;
    
    }
    else {

        isostat &= ~0x20;
        isostat |= (cfg.mode == DIRECT1);

    }

}

void TRF796XEnable(const bool state) {

    if (cfg.en_gpio != -1)
        cfg.write_gpio(cfg.en_gpio, state);

}

void TRF796XSelect(const bool state) {

    if(cfg.cs_gpio != -1)
        cfg.write_gpio(cfg.cs_gpio, !state);

}

void TRF796XSetRFState(const bool state) {

    uint8_t status = TRF796XReadByte(STAT_CTRL);
    status |= state << 5;
    TRF796XWriteByte(STAT_CTRL, status);

}

uint8_t TRF796XWriteCommand(const TRF796XCommand command) {

    uint8_t cmd = command | 0x80;
    TRF796XSelect(true);
    uint8_t result = (uint8_t)write(&command, 1);
    TRF796XSelect(false);
    return result;

}

uint8_t TRF796XWrite(const TRF796XAddress addr, const void* const data, const uint8_t size) {

    if(write == NULL || !data || size == 0)
        return;

    TRF796XSelect(true);

    static uint8_t buffer[257];
    buffer[0] = (size > 1)? addr | 0x20: addr;
    memcpy(data, buffer + 1, size);

    uint8_t result = (uint8_t)write(buffer, size + 1);

    TRF796XSelect(false);

    return result;

}

uint8_t TRF796XWriteByte(const TRF796XAddress addr, const uint8_t value) {

    return TRF796XWrite(addr, &value, 1);

}

uint8_t TRF796XReadByte(const TRF796XAddress addr) {

    static uint8_t buffer;
    
    if(TRF796XRead(addr, &buffer, 1) != 1)
        return UINT8_MAX;

    return buffer;

}

uint8_t TRF796XRead(const TRF796XAddress addr, void* const data, const uint8_t size) {

    if(!data || size == 0 || write == NULL || read == NULL)
        return;

    const uint8_t cmd = (size > 1)? addr | 0x60: addr;

    TRF796XSelect(true);
    
    if(write(&cmd, 1) != 1)
        return 0;
    
    if(read(&data, size) != size)
        return 0;

    TRF796XSelect(false);

    return size;

}

void TRFDeinit() {

    write = NULL;
    read = NULL;

    TRF796XSelect(0);
    TRF796XEnable(0);

    cfg.ask_ook_gpio = -1;
    cfg.int_gpio = -1;
    cfg.mod_gpio = -1;
    cfg.en_gpio = -1;
    cfg.en2_gpio = -1;
    cfg.cs_gpio = -1;

    cfg.read_gpio = NULL;
    cfg.write_gpio = NULL;

    cfg.read = NULL;
    cfg.write = NULL;
    
    cfg.usespi = false;

}

// -------------------------- Transmission Functions ---------------------------- //

/**
 * \brief 
 * 
 */
static void TRF796XEmptyFIFO();

const static uint8_t fifo_max_size = 12;
static volatile bool rx_started = false;
static volatile bool no_response = false;
static volatile bool tx_done = false;
static volatile bool fifo_full = false;
static volatile bool fifo_empty = true;
static volatile bool fifo_overflow = false;
static volatile uint8_t fifo_size = 0;

static uint16_t TRF796XTransmitBytes(const void* const data, const uint16_t bytes);

uint16_t TRF796XTransmit(const void* const data, const uint16_t size, const bool withcrc) {

    if(!data || size == 0)
        return;

    TRF796XWriteCommand(withcrc? TX_W_CRC: TX_WO_CRC); // signal to start transmission with or without CRC

    // we are only doing complete bits 
    uint16_t txmetadata = size << 4; // only mark the complete bytes to send
    if(!TRF796XWrite(TX_LEN_H, &txmetadata, 2)) // indicate that the transmission will be that many bytes 
        return 0;

    TRF796XEmptyFIFO();

    return TRF796XTransmitBytes(data, size);

}


uint16_t TRF796XTransmitBits(const void* const data, const uint16_t bits, const bool withcrc) {

    if(!data || bits == 0)
        return;

    TRF796XWriteCommand(withcrc? TX_W_CRC: TX_WO_CRC); // signal to the processor that we to Transmit with ot without CRC

    uint16_t bytes = (bits & 0x7)? (bits & 0xF8) + 1: (bits & 0xF8); // the number of incomplete bytes, whole bytes plus another bytes for the last bits
    uint16_t txmetadata = (bits << 1); // we want the total number of bytes, counting the last incomplete one as one
    txmetadata |= (bits & 0x7)? 1: 0; // if there are extra bits that dont fit a byte mark that we are sending a broken byte
    
    if(!TRF796XWrite(TX_LEN_H, &txmetadata, 2)) // We want to set the number of bytes and incomplete bits in the transaction
        return 0;
        
    return TRF796XTransmitBytes(data, bytes) == bytes? bits: 0;

}

uint16_t TRF796XTransmitBytes(const void* const data, const uint16_t bytes) {

    TRF796XEmptyFIFO();

    if(bytes < 12) {

        TRF796XWrite(FIFO_IO, data, bytes);
        while(!tx_done); // wait for the transmission to finish
        tx_done = false; // we have completed the transaction, reset the flag
        return bytes;

    }

    TRF796XWrite(FIFO_IO, data, 12);
    fifo_empty = false;
    while(!fifo_empty); // wait for the fifo to have only 3 bytes left

    uint8_t loops = (bytes - 12) / 9; // how many complete FIFO fills we can do given we have already sent 12 bytes

    for(uint16_t i = 0; i != loops; i++){
    
        TRF796XWrite(FIFO_IO, data + 12 + 9 * i, 9); // we want to fill up the fifo with the data that should be written
        fifo_empty = false;
        while(!fifo_empty); // wait for the fifo to empty out

    }

    uint8_t rem = (bytes - 12) % 9;
    if(rem > 0) {

        TRF796XWrite(FIFO_IO, data + 12 + 9 * loops, rem);
        fifo_empty = false;
        while(!fifo_empty);
    
    }

    while(!tx_done); // wait for the fifo to empty and the transaction to finish
    tx_done = false;

    return bytes;
    
}

static uint16_t TRF796XReceieveBytes(void* const data, const uint16_t bytes) {

    while(!rx_started);

    uint16_t index = 0;

};

uint16_t TRF796XReceive(void* const data, const uint16_t size, const bool withcrc) {

    if(!data || size == 0)
        return;


}

uint16_t TRF796XReceiveBits(void* const data, const uint16_t size, const bool withcrc) {



}

void TRF796XISR() {

    uint8_t irq_stat = TRF796XReadInterruptStatus();

    if(irq_stat & IRQ_FIFO) {

        uint8_t fifo_stat = TRF796XReadByte(FIFO_STAT);

        fifo_size = (fifo_stat & 0xf) + 1;
        fifo_full = fifo_stat & (1 << 6);
        fifo_empty = fifo_stat & (1 << 5);
        fifo_overflow = fifo_stat & (1 << 4);

    }

    tx_done = irq_stat & IRQ_TX_END; 
    rx_started = irq_stat & IRQ_RX_STRT;
    no_response = irq_stat & IRQ_NO_RESP;

}

void TRF796XSelect(const bool state) {

    if(cfg.usespi && cfg.cs_gpio != -1)
        cfg.write_gpio(cfg.cs_gpio, !state);

}
