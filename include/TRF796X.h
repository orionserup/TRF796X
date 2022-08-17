/**
 * \file TRF796X.h
 * \author Orion Serup (orionserup@gmail.com)
 * \brief 
 * \version 0.1
 * \date 2022-08-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define on true
#define off false

/// All of the Commands that can be run on the TRF796X
typedef enum TRF796XCOMMAND {

    IDLE = 0x1,             ///< Puts the Device into Idle Mode
    SW_INIT = 0x3,          ///< The Same as a Power On Reset
    RESET_FIFO = 0xF,       ///< Clears the FIFO
    TX_WO_CRC = 0x10,       ///< Transmit without CRC
    TX_W_CRC = 0x11,        ///< Transmit with CRC
    BLOCK_RX = 0x16,        ///< Block the Reception of Data
    EN_RX = 0x17,           ///< Enable The Reception of Data
    TEST_INT_RF = 0x18,     ///< Test the Internal RF
    TEST_EXT_RF = 0x19,     ///< Test the External RF
    RX_GAIN_ADJ = 0x1A      ///< Adjust the Gain for the Reception

} TRF796XCommand;

typedef enum TRF796XINT {

    // Readable In the Status Register But Not Settable as a Trigger

    IRQ_TX_END = 0x80,      ///< End of Transmission
    IRQ_RX_STRT = 0x40,     ///< Start of Reception

    // Readable from the Status Register and Settable as a Trigger

    IRQ_FIFO = 0x20,        ///< Issue with the FIFO, see the FIFO Status Register \ref FIFO_STAT
    IRQ_CRC_ERR = 0x10,     ///< CRC Error
    IRQ_PAR_ERR = 0x08,     ///< Parity Error
    IRQ_FRAM_ERR = 0x04,    ///< Framing or EOF Error
    IRQ_COL_ERR = 0x02,     ///< Collision Error
    IRQ_NO_RESP = 0x01      ///< No-Response Time Out Error

} TRF796XInterrupt;

typedef enum TRF796XMODE {

    FELICA_212k = 0x1A,     ///< FeLiCa Spec Running at 212kbps
    FELICA_424k = 0x1B,     ///< FeLiCa Spec Running at 424kbps

    ISO_14443A_106k = 0x08, ///< ISO14443A Spec Running at 106kbps
    ISO_14443A_212k = 0x09, ///< ISO14443A Spec Running at 212kbps
    ISO_14443A_424k = 0x0A, ///< ISO14443A Spec Running at 424kbps
    ISO_14443A_848k = 0x0B, ///< ISO14443A Spec Running at 848kbps 

    ISO_14443B_106k = 0x0C, ///< ISO14443B Spec Running at 106kbps
    ISO_14443B_212k = 0x0D, ///< ISO14443B Spec Running at 212kbps
    ISO_14443B_424k = 0x0E, ///< ISO14443B Spec Running at 424kbps
    ISO_14443B_848k = 0x0F, ///< ISO14443B Spec Running at 848kbps

    DIRECT0 = 0,            ///< Read Directly From the Analog Front End
    DIRECT1 = 1             ///< Read Decoded Data without Application Framing

} TRF796XMode;

typedef struct TRF790XTYPE {

    TRF7960 = 0x0,
    TRF7963 = 0x1

} TRF796XType;

/// An enumerated type for all of the internal addresses
typedef enum TRF796XADDRESS {

    // Main Control Registers //

    STAT_CTRL = 0x0,        ///< Chip Status and Chip Control Register
    ISO_CTRL = 0x1,         ///< Control for the ISO Protocol Register
    
    // Protocol Subsetting Registers //
    
    ISO14443B_TX_OPT = 0x2, ///< Options for the ISO 14443B Transmission Register
    ISO14443A_HBR_OPT = 0x3,///< High Bit Rate Options for ISO 14443A Register
    TX_PL_CTRL = 0x6,       ///< Tranmission Pulse Length Control Register      
    RX_NR_WAIT = 0x7,       ///< Reception No Response Wait Register
    RX_WAIT_TIME = 0x8,     ///< Reception Wait Time Register   
    MOD_CLK_CTRL = 0x9,     ///< Modulation and Clock Control Register
    RX_SPEC_SETTINGS = 0xA, ///< Reception Special Settings Register
    REG_IO_CTRL = 0xB,      ///< Regulator and IO Control Register
    
    // Status Registers //
    
    IRQ_STAT = 0xC,         ///< Interrupt Status Register
    COL_POS_INT_MASK = 0xD, ///< Collision Position and Interrupt Mask Register
    COL_POS = 0xE,          ///< Collision Position Register
    RSSI_LVL_OSC_STAT = 0xF,///< RSSI Level and Oscillation Status Register

    // FIFO Registers //

    TEST1 = 0x1A,           ///< First Test Register
    TEST2 = 0x1B,           ///< Second Test Register
    FIFO_STAT = 0x1C,       ///< FIFO Status Regitster
    TX_LEN_L = 0x1D,        ///< Transmission Length Low Byte Register
    TX_LEN_H = 0x1E,        ///< Transmission Length High Byte Register
    FIFO_IO = 0x1F          ///< FIFO I/O Register, Base register, spans for 12 Bytes

} TRF796XAddress;

/// All of the Things needed to work with the TRF796X
typedef struct TRF796XCONFIG {

    void (*write_gpio)(const int gpio, const bool state);                       ///< A Function Pointer to Write the GPIO
    void (*read_gpio)(const int gpio);                                          ///< A Function Pointer to read a GPIO State

    int mod_gpio;       ///< The GPIO for the bidirection modulation pin
    int ask_ook_gpio;   ///< The GPIO for the ASK/OOK modulation mode selection pin or the received analog signal
    int int_gpio;       ///< The GPIO for the IRQ pin
    int sysclk_gpio;    ///< The GPIO for the SYS_CLK pin 
    int en_gpio;        ///< The GPIO for the Enable Pin
    int en2_gpio;       ///< The GPIO for the second Enable Pin
    int cs_gpio;        ///< The GPIO for the Chip Select for the SPI Peripheral

    uint32_t (write)(const void* const data, const uint32_t size);         ///< Function to Write to the Device, either SPI or Parallel Interface depending on the usespi flag
    uint32_t (read)(void* const data, const uint32_t size);                ///< Function to Read from the Device, either SPI or Parallel Interface depending on the usespi flag

    void (*delay)(const uint16_t useconds);                                ///< Function to Delay the MCU Until 

    bool usespi;        ///< Whether or not to use SPI to communicate 

    TRF796XMode mode;       ///< The Mode of Operation Between The ISO Standards and being a remote
    TRF796XType devicetype; ///< The Specific Submodel of the device

} TRF796XConfig;

/// Runtime data for the chip, all of the flags and other information
typedef struct TRF796XDATA {

    volatile bool rx_started;   ///< If the Reception has started
    volatile bool no_response;  ///< If there was no response detected within the time frame specified
    volatile bool tx_done;      ///< If the transmission is done
    volatile bool fifo_full;    ///< If the FIFO buffer is full (>9 bytes)
    volatile bool fifo_empty;   ///< If the FIFO buffer is empty (<3 bytes)
    volatile bool fifo_overflow;///< If the FIFO is overfilled
    volatile uint8_t fifo_size; ///< How many elements are in the FIFO buffer currently

} TRF796XData;

/// A Device With the Device Specific Data, aka runtime flags and information
typedef struct TRF796X {

    TRF796XConfig config;   ///< The configuration data for the physical interface
    TRF796XData data;       ///< The runtime data and flags for transmission and stuff

} TRF796X;

// -------------------- Init and Deinit Functions ------------------- //

/**
 * \brief Configures and sets up the TRF796X 
 * 
 * \param[in] config: The Configuration to set up the TRF796X
 * \param[in] dev: Device to Initialize
 * 
 * \returns TRF796X*: The Inited device address, NULL if there is an issue with the init
 */
TRF796X* TRF796XInit(TRF796X* const dev, const TRF796XConfig* const config);

/**
 * \brief Deinitializes all of the buffers and such of the peripheral
 * 
 * \param[in] dev: Device to Deinitialize
 * 
 */
void TRFDeinit(TRF796X* const dev);

// ------------------------ Interrupt Functions ------------------------ //

/**
 * \brief Enables the Interrupt That is given, enums are IRQ_*
 * 
 * \param[in] dev: Device to enable the interrupt for 
 * \param[in] irq: Interrupt Bit Mask to Set 
 */
void TRF796XEnableInterrupt(const TRF796X* const dev, const TRF796XInterrupt irq);

/**
 * \brief Reads the Interrupt Status Register and Returns the value
 * 
 * \param[in] dev: Device to read the interrupt status of
 * 
 * \return uint8_t: The Value of the Interrupt Status Register
 */
uint8_t TRF796XReadInterruptStatus(const TRF796X* const dev);

/**
 * \brief Enables Multiple Interrupts at Once 
 * 
 * \param[in] dev: Device to Enable the Interrupts for
 * \param[in] irqmask: Mask of the Interrupts to set, see \ref TRF796XInterrupt, or'd interrupt bit masks
 */
void TRF796XEnableInterrupts(const TRF796X* const dev, const uint8_t irqmask);

/**
 * \brief The Interrupt Service Routine for The Device, Manages all of the Flags and Cleanup
 * 
 * \param[in] dev: Device to run the interrupt for
 * 
 */
void TRF796XISR(const TRF796X* const dev);

// ------------------- Basic State Setting Functions ------------------- //

/**
 * \brief Enables or Disables the Device via the EN Pin
 * 
 * \param[in] dev: Device to Enable or Disable
 * \param[in] state: Whether the device is Enabled
 */
void TRF796XEnable(const TRF796X* const dev, const bool state);

/**
 * \brief Does a software reset of the TRF96X
 * 
 * \param[in] dev: Device to Reset
 * 
 */
void TRF796XReset(const TRF796X* const dev);

/**
 * \brief Turns on or off the RF Transceiver
 * 
 * \param[in] dev: Device to Enable or Disable the RF Transceiver for
 * \param[in] state: The State of the RF Transceiver we want to write
 */
void TRF796XSetRFState(const TRF796X* const dev, const bool state);

// -------------------- Basic Chip Reading and Writing Functions ----------------- //

/**
 * \brief Writes a command To the Device see \ref TRF796XCommand
 * 
 * \param[in] dev: Device to Write a Command To
 * \param[in] command: Command to write to the device
 * 
 * \returns uint8_t: How many bytes were written, 0 for error
 */
uint8_t TRF796XWriteCommand(const TRF796X* const dev, const TRF796XCommand command);

/**
 * \brief Writes to the device starting at an address and continuing
 * 
 * \param[in] dev: Device to write to
 * \param[in] addr: Base Address to Start Writing from
 * \param[in] data: The data to write to the address and beyond
 * \param[in] size: How many bytes to write
 * 
 * \returns uint8_t: How many bytes were written, zero for error
 */
uint8_t TRF796XWrite(const TRF796X* const dev, const TRF796XAddress addr, const void* const data, const uint8_t size);

/**
 * \brief Write a single byte to the device at a given address
 * 
 * \param[in] dev: Device to write a byte to
 * \param[in] addr: The Address to Write to see \ref TRF796XAddress
 * \param[in] value: The value to write at that address
 * 
 * \returns uint8_t: The Number of Bytes Written
 */
uint8_t TRF796XWriteByte(const TRF796X* const dev, const TRF796XAddress addr, const uint8_t value);

/**
 * \brief Uses the CS Pin to Select the Device for Writing and Reading Only Applies if using spi
 * 
 * \param[in] dev: The Device to Select
 * \param[in] state: Whether or Not to Select the Device
 * 
 */
void TRF796XSelect(const TRF796X* const dev, const bool state);

/**
 * \brief Read from the device at a given address
 * 
 * \param[in] dev: Device to Read from
 * \param[in] addr: Address to read from
 * \param[out] data: Buffer to write to
 * \param[in] size: Number of Bytes to Read
 * 
 * \returns uint8_t: The Number of Read Bytes
 */
uint8_t TRF796XRead(const TRF796X* const dev, const TRF796XAddress addr, void* const data, const uint8_t size);

// ---------------------- Basic Transmission and Reception Functions -------------------- //

/**
 * \brief Transmit a buffer using the RF Communication protocol
 * 
 * \param[in] dev: Device to transmit From
 * \param[in] data: Buffer to send
 * \param[in] size: How Many Bytes to Send from the Buffer
 * 
 * \returns uint16_t: The Number of bytes that were transmitted, 0 if error
 */
uint16_t TRF796XTransmit(TRF796X* const dev, const void* const data, const uint16_t size, const bool withcrc);

/**
 * \brief Transmits A Number of Bits using RF
 * 
 * \param[in] dev: Device to transmit from
 * \param[in] data: Buffer to Transmit from
 * \param[in] bits: The Number of Bits to Send
 * \param[in] withcrc: If we want to transmit with CRC
 * 
 * \returns uint16_t: The Number of Bits that were successfully transmitted 
 */
uint16_t TRF796XTransmitBits(TRF796X* const dev, const void* const data, const uint16_t bits, const bool withcrc);

/**
 * \brief Receive Bytes from the Antenna, first waiting for anything to come in
 * 
 * \param[in] dev: Device to receive to
 * \param[out] data: The Buffer to Write to
 * \param[in] size: How many bytes to receive, hopefully
 * \param[in] withcrc: If we want to verify with CRC
 * 
 * \returns uint16_t: The number of bytes that were actually received, 0 for error
 */
uint16_t TRF796XReceive(TRF796X* const dev, void* const data, const uint16_t size, const bool withcrc);

/**
 * \brief Receive a number of Bits from the System Transmitted by a Tag or another reader
 * 
 * \param[in] dev: The Device to Receive to
 * \param[out] data: The Buffer to Receive To
 * \param[in] bits: How many bits we want to receive
 * \param[in] withcrc: If we want to verify reception with CRC
 * 
 * \returns uint16_t: The Number of Bits that were actually received before a stop condition or timeout
 */
uint16_t TRF796XReceieveBits(TRF796X* const dev, void* const data, const uint16_t bits, const bool withcrc);

