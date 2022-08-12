# Texas Instruments TRF796X NFC/RFID Reader IC Driver Library #

## Authors: [Orion Serup](orionserup@gmail.com) ##

### This library is a hardware-agnostic usage layer to allow full usage of the TRF796X to read and write to NFC Tags as well as other devices easily and efficiently ###

#### Usage ####

To use this library we need a minimal set of information and functions to configure the driver layer:

You need either SPI or Parallel Read and Write Functions in the form:

        uint16_t read(void* const buffer, const uint16_t size); // function should return number of bytes read, zero if error
        uint16_t write(const void* const buffer, const uint16_t size); // returns the number of bytes written, zero for error

These functions should simply write and read to either the spi or parallel data bus, these peripherals and all other dependencies need to be initialized beforehand

You also need a delay function in the form:

        void delay(const uint16_t useconds); // delays for useconds microseconds

In addition you need a function to manipulate the GPIO, in the form of:

        bool gpio_read(const int gpio); // reads the gpio corresponding with the int parameter
        void gpio_write(const int gpio, const bool value); // sets the state of the given gpio

Like with the SPI/Parallel Interface the gpio needs to be configured beforehand and ready to go

With these functions, the gpio pins of the different signals need to be declared, we need the pins for en, en2, ask_ook, mod, sys_clk, cs, and irq

Once all of these values and functions are ready to use we can fill out a configuration struct with their values. The struct is as follows:

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

            void (*delay)(const uint16_t useconds);                                     ///< Function to Delay the MCU Until 

            bool usespi;        ///< Whether or not to use SPI to communicate 

            TRF796XMode mode;   ///< The Mode of Operation Between The ISO Standards and being a remote

        } TRF796XConfig;

If we are using SPI to communicate, set the usespi flag, false for parallel interface. Supply the reading and writing functions to the read and write member function pointers. Supply all of the GPIO that are used, -1 if unused or not connected.
The mode member has to do with the communication method and communication speed:

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

Select a mode based on what your needs are and what spec the other device will be using.

Once the initialization struct has been filled with the necessary fields and is ready to go you can run `TRF796XInit(&config);`

After that you want to setup an interrupt on the irq pin, have the Interrupt Service Routine for that pin call `TRF796XISR();`

After that everything should be ready to go, see the API Guide for the rest of the information. 