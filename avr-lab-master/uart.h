#ifndef UART_H
#define UART_H

#include <avr/io.h>

#define BAUDRATE 38400
#define UBRR (F_CPU/16/BAUDRATE-1)

extern void uart_init(void);

#endif