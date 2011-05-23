#ifndef	_SERIAL_H
#define	_SERIAL_H

#include	<stdint.h>
#include	<avr/io.h>
#include	<avr/pgmspace.h>

// initialise serial subsystem
void serial_init(unsigned long BAUD);

// return number of characters in the receive buffer, and number of spaces in the send buffer
uint8_t serial_rxchars(void);
// uint8_t serial_txchars(void);

// read one character
uint8_t serial_popchar(void);
// send one character
void serial_writechar(uint8_t data);

// read/write many characters
// uint8_t serial_recvblock(uint8_t *block, int blocksize);
void serial_writeblock(void *data, int datalen);
void serial_writenum(uint16_t data, uint16_t radix);
void serial_writestr(const char *data);
void serial_labelnum(const char* label, uint16_t num);
#endif	/* _SERIAL_H */
