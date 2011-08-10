#include "ArduinoMap.h"
#include <avr/pgmspace.h>

namespace ArduinoMap 
{

#define AP_PA 0x20
#define AP_PB 0x23
#define AP_PC 0x26
#define AP_PD 0x29
#define AP_PE 0x2C
#define AP_PF 0x2F
#define AP_PG 0x32
#define AP_PH 0x100
#define AP_PI 0xFFFF
#define AP_PJ 0x103
#define AP_PK 0x106
#define AP_PL 0x109
#define AP_PN 0xFFFF


#if ((defined __AVR_ATmega2560__) || (defined __AVR_ATmega1280__))
const uint16_t __portmap[70] PROGMEM = 
{
  AP_PE  , // PE 0 ** 0 ** USART0_RX
  AP_PE  , // PE 1 ** 1 ** USART0_TX
  AP_PE  , // PE 4 ** 2 ** PWM2
  AP_PE  , // PE 5 ** 3 ** PWM3
  AP_PG  , // PG 5 ** 4 ** PWM4
  AP_PE  , // PE 3 ** 5 ** PWM5
  AP_PH  , // PH 3 ** 6 ** PWM6
  AP_PH  , // PH 4 ** 7 ** PWM7
  AP_PH  , // PH 5 ** 8 ** PWM8
  AP_PH  , // PH 6 ** 9 ** PWM9
  AP_PB  , // PB 4 ** 10 ** PWM10
  AP_PB  , // PB 5 ** 11 ** PWM11
  AP_PB  , // PB 6 ** 12 ** PWM12
  AP_PB  , // PB 7 ** 13 ** PWM13
  AP_PJ  , // PJ 1 ** 14 ** USART3_TX
  AP_PJ  , // PJ 0 ** 15 ** USART3_RX
  AP_PH  , // PH 1 ** 16 ** USART2_TX
  AP_PH  , // PH 0 ** 17 ** USART2_RX
  AP_PD  , // PD 3 ** 18 ** USART1_TX
  AP_PD  , // PD 2 ** 19 ** USART1_RX
  AP_PD  , // PD 1 ** 20 ** I2C_SDA
  AP_PD  , // PD 0 ** 21 ** I2C_SCL
  AP_PA  , // PA 0 ** 22 ** D22
  AP_PA  , // PA 1 ** 23 ** D23
  AP_PA  , // PA 2 ** 24 ** D24
  AP_PA  , // PA 3 ** 25 ** D25
  AP_PA  , // PA 4 ** 26 ** D26
  AP_PA  , // PA 5 ** 27 ** D27
  AP_PA  , // PA 6 ** 28 ** D28
  AP_PA  , // PA 7 ** 29 ** D29
  AP_PC  , // PC 7 ** 30 ** D30
  AP_PC  , // PC 6 ** 31 ** D31
  AP_PC  , // PC 5 ** 32 ** D32
  AP_PC  , // PC 4 ** 33 ** D33
  AP_PC  , // PC 3 ** 34 ** D34
  AP_PC  , // PC 2 ** 35 ** D35
  AP_PC  , // PC 1 ** 36 ** D36
  AP_PC  , // PC 0 ** 37 ** D37
  AP_PD  , // PD 7 ** 38 ** D38
  AP_PG  , // PG 2 ** 39 ** D39
  AP_PG  , // PG 1 ** 40 ** D40
  AP_PG  , // PG 0 ** 41 ** D41
  AP_PL  , // PL 7 ** 42 ** D42
  AP_PL  , // PL 6 ** 43 ** D43
  AP_PL  , // PL 5 ** 44 ** D44
  AP_PL  , // PL 4 ** 45 ** D45
  AP_PL  , // PL 3 ** 46 ** D46
  AP_PL  , // PL 2 ** 47 ** D47
  AP_PL  , // PL 1 ** 48 ** D48
  AP_PL  , // PL 0 ** 49 ** D49
  AP_PB  , // PB 3 ** 50 ** SPI_MISO
  AP_PB  , // PB 2 ** 51 ** SPI_MOSI
  AP_PB  , // PB 1 ** 52 ** SPI_SCK
  AP_PB  , // PB 0 ** 53 ** SPI_SS
  AP_PF  , // PF 0 ** 54 ** A0
  AP_PF  , // PF 1 ** 55 ** A1
  AP_PF  , // PF 2 ** 56 ** A2
  AP_PF  , // PF 3 ** 57 ** A3
  AP_PF  , // PF 4 ** 58 ** A4
  AP_PF  , // PF 5 ** 59 ** A5
  AP_PF  , // PF 6 ** 60 ** A6
  AP_PF  , // PF 7 ** 61 ** A7
  AP_PK  , // PK 0 ** 62 ** A8
  AP_PK  , // PK 1 ** 63 ** A9
  AP_PK  , // PK 2 ** 64 ** A10
  AP_PK  , // PK 3 ** 65 ** A11
  AP_PK  , // PK 4 ** 66 ** A12
  AP_PK  , // PK 5 ** 67 ** A13
  AP_PK  , // PK 6 ** 68 ** A14
  AP_PK  // PK 7 ** 69 ** A15
};

const uint8_t __pinmap[70] PROGMEM = 
{
  0, // ** 0 ** USART0_RX
  1, // ** 1 ** USART0_TX
  4, // ** 2 ** PWM2
  5, // ** 3 ** PWM3
  5, // ** 4 ** PWM4
  3, // ** 5 ** PWM5
  3, // ** 6 ** PWM6
  4, // ** 7 ** PWM7
  5, // ** 8 ** PWM8
  6, // ** 9 ** PWM9
  4, // ** 10 ** PWM10
  5, // ** 11 ** PWM11
  6, // ** 12 ** PWM12
  7, // ** 13 ** PWM13
  1, // ** 14 ** USART3_TX
  0, // ** 15 ** USART3_RX
  1, // ** 16 ** USART2_TX
  0, // ** 17 ** USART2_RX
  3, // ** 18 ** USART1_TX
  2, // ** 19 ** USART1_RX
  1, // ** 20 ** I2C_SDA
  0, // ** 21 ** I2C_SCL
  0, // ** 22 ** D22
  1, // ** 23 ** D23
  2, // ** 24 ** D24
  3, // ** 25 ** D25
  4, // ** 26 ** D26
  5, // ** 27 ** D27
  6, // ** 28 ** D28
  7, // ** 29 ** D29
  7, // ** 30 ** D30
  6, // ** 31 ** D31
  5, // ** 32 ** D32
  4, // ** 33 ** D33
  3, // ** 34 ** D34
  2, // ** 35 ** D35
  1, // ** 36 ** D36
  0, // ** 37 ** D37
  7, // ** 38 ** D38
  2, // ** 39 ** D39
  1, // ** 40 ** D40
  0, // ** 41 ** D41
  7, // ** 42 ** D42
  6, // ** 43 ** D43
  5, // ** 44 ** D44
  4, // ** 45 ** D45
  3, // ** 46 ** D46
  2, // ** 47 ** D47
  1, // ** 48 ** D48
  0, // ** 49 ** D49
  3, // ** 50 ** SPI_MISO
  2, // ** 51 ** SPI_MOSI
  1, // ** 52 ** SPI_SCK
  0, // ** 53 ** SPI_SS
  0, // ** 54 ** A0
  1, // ** 55 ** A1
  2, // ** 56 ** A2
  3, // ** 57 ** A3
  4, // ** 58 ** A4
  5, // ** 59 ** A5
  6, // ** 60 ** A6
  7, // ** 61 ** A7
  0, // ** 62 ** A8
  1, // ** 63 ** A9
  2, // ** 64 ** A10
  3, // ** 65 ** A11
  4, // ** 66 ** A12
  5, // ** 67 ** A13
  6, // ** 68 ** A14
  7 // ** 69 ** A15
};
  
#else
#error NO PINS DEFINED FOR PROCESSOR
#endif


  Port getPort(int pinnum)
  {
    Port port(pgm_read_word(&(__portmap[pinnum])));
    return port;
  }

  uint8_t getPinnum(int pinnum)
  {
    uint8_t p = pgm_read_byte(&(__pinmap[pinnum]));
    return p;
  }

  Pin getArduinoPin(int pinnum)
  {
    Pin p = Pin(getPort(pinnum), 
                getPinnum(pinnum)
               );
    return p;
  }


}; // namespace






