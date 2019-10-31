#ifndef DHT_H
#define DHT_H

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>

#define DHT_DDR DDRB
#define DHT_PORT PORTB
#define DHT_PIN PINB
#define DHT_PIN_BIT PB0 // Pin 8

typedef enum {
    SUCCESS = 0,
    ERROR_AWAITING_HIGH = 1,
    ERROR_AWAITING_LOW = 2,
    ERROR_AWAITING_LOOP = 3,
    ERROR_CHECKSUM = 4
} dht_status;

typedef struct {
    uint8_t temperature;
    uint8_t humidity;
} dht_measurement;

extern void dht_init();

extern dht_status dht_read(dht_measurement* measurement);

void dht_outputMode();

void dht_inputMode();

void dht_high();

void dht_low();

bool dht_read_bit();

bool dht_read_byte(uint8_t* value);

bool dht_awaitHigh(uint8_t timeout_us);

bool dht_awaitLow(uint8_t timeout_us);

#endif