#include "dht.h"

/**
 *  Based on information in: http://akizukidenshi.com/download/ds/aosong/DHT11.pdf
 *  Port manipulation information: https://www.arduino.cc/en/Reference/PortManipulation
 */

/**
 * Initializes the DHT, i.e. wait for a second for it ti settle.
 */
void dht_init() {
    /*
        Step 1
        
        After power on DHT11 (DHT11 on after power to wait 1S across the unstable state during this period
        can not send any instruction), the test environment temperature and humidity data, and record the data,
        while DHT11 the DATA data lines pulled by pull-up resistor has been to maintainhigh; the DHT11 the
        DATA pin is in input state, the moment of detection of external signals.
    */
    _delay_ms(1000);
}

/**
 * Read the current measurements from the sensor.
 * 
 * measurement - A pointer to the target dht_measurement.
 * 
 * returns - The dht_status after the sensor operation.
 */
dht_status dht_read(dht_measurement* measurement) {
    /*
        Step 2

        (a)

        Microprocessor I / O set to output at the same time output low, and low hold time can not be less
        than 18ms,
        
        (b)
        
        then the microprocessor I / O is set to input state, due to the pull-up resistor, a
        microprocessor/ O DHT11 the dATA data lines also will be high, waiting DHT11 to answer signal, send
        the signal as shown:
     */
    // Step 2a
    dht_outputMode();
    dht_low();
    _delay_ms(18);

    // Step 2b
    dht_high();
    dht_inputMode();

    if(!dht_awaitLow(40)) {
        return ERROR_AWAITING_LOW;
    }

    /*
        Step 3

        DATA pin is detected to an external signal of DHT11 low, waiting for external signal low end the
        delay DHT11 DATA pin in the output state, the output low of 80 microseconds as the response signal,
        followed by the output of 80 micro-seconds of high notification peripheral is ready to receive data, the
        microprocessor I / O at this time in the input state is detected the I / O low (DHT11 response signal), wait
        80 microseconds highdata receiving and sending signals as shown:
    */

    if(!dht_awaitHigh(80)) {
        return ERROR_AWAITING_HIGH;
    }

    if(!dht_awaitLow(80)) {
        return ERROR_AWAITING_LOW;
    }

    /*
        Step 4

        Output by DHT11 the DATA pin 40, the microprocessor receives 40 data bits of data "0" format: the
        low level of 50 microseconds and 26-28 microseconds according to the changes in the I / O levellevel,
        bit data "1" format: the high level of low plus, 50 microseconds to 70 microseconds. Bit data "0", "1" signal
        format as shown:
    */

   uint8_t humidity_msb;
   uint8_t humidity_lsb;
   uint8_t temperature_msb;
   uint8_t temperature_lsb;
   uint8_t parity;

   if(!dht_read_byte(&humidity_msb)) {
       return ERROR_AWAITING_LOOP;
   }

   if(!dht_read_byte(&humidity_lsb)) {
       return ERROR_AWAITING_LOOP;
   }

   if(!dht_read_byte(&temperature_msb)) {
       return ERROR_AWAITING_LOOP;
   }

   if(!dht_read_byte(&temperature_lsb)) {
       return ERROR_AWAITING_LOOP;
   }

   if(!dht_read_byte(&parity)) {
       return ERROR_AWAITING_LOOP;
   }

   uint8_t parity_actual = humidity_msb + humidity_lsb + temperature_msb + temperature_lsb;

    /*
        End signal:

        Continue to output the low 50 microseconds after DHT11 the DATA pin output 40 data, and
        changed the input state, along with pull-up resistor goes high. But DHT11 internal re-test environmental
        temperature and humidity data, and record the data, waiting for the arrival of the external signal.
    */


   // Reset sensor
    _delay_us(50);
    dht_outputMode();
    dht_low();

    // Check if parity was correct
    if(parity_actual != parity) {
        measurement->temperature = -1;
        measurement->humidity = -1;

        return ERROR_CHECKSUM;
    }

    measurement->temperature = temperature_msb;
    measurement->humidity = humidity_msb;

    return SUCCESS;
}

/**
 * Switches the dht pin to output mode by manipulating the Data Direction Register (DDR).
 */ 
void dht_outputMode() {
    DHT_DDR |= (1 << DHT_PIN_BIT);
}

/**
 * Switches the dht pin to input mode by manipulating the Data Direction Register (DDR).
 */ 
void dht_inputMode() {
    DHT_DDR &= ~(1 << DHT_PIN_BIT);
}

/**
 * Writes a high value to Port Data Register.
 */ 
void dht_high() {
    DHT_PORT |= (1 << DHT_PIN_BIT);
}

/**
 * Writes a high value to Port Data Register.
 */ 
void dht_low() {
    DHT_PORT &= ~(1 << DHT_PIN_BIT);
}

/**
 * Reads the value from the Input Pins Register.
 * 
 * returns - True if high, false if low.
 */
bool dht_read_bit() {
    return (DHT_PIN & (1 << DHT_PIN_BIT)) != 0;
}

/**
 * Reads a byte from the DHT11 sensor into the supplied int pointer variable.
 * 
 * value - The pointer to the int variable.
 * 
 * Documented in step 4: http://akizukidenshi.com/download/ds/aosong/DHT11.pdf
 * 
 * returns - True if successfully sampled, otherwise false.
 */
bool dht_read_byte(uint8_t* value) {
    uint8_t temp = 0;

    for(int i = 8; --i >= 0; ) {
        if(!dht_awaitHigh(60)) {
            return false;
        }

        if(!dht_awaitLow(40)) {
            // Check if high (40 us more)
            if(!dht_awaitLow(40)) {
               return false;
            }

            temp |= (1 << i);
        }
    }

    *value = temp;

    return true;
}

/**
 * Awaits the requested state for the Port Input Register or until a timeout occurs.
 * 
 * state - The desired target state, i.e. true = high, false = low.
 * timeout_us - The timeout in microseconds.
 * 
 * returns - True if successfully recieved desired state, otherwise false, i.e. timeout. 
 */
bool dht_await(bool state, uint8_t timeout_us) {
    for(int i = timeout_us >> 1; --i >= 0; ) {
        if(dht_read_bit() == state) {
            return true;
        }

        _delay_us(2);
    }

    return false;
}

/**
 * Awaits the high state for the Port Input Register or until a timeout occurs.
 * 
 * timeout_us - The timeout in microseconds.
 * 
 * returns - True if successfully recieved desired state, otherwise false, i.e. timeout. 
 */
bool dht_awaitHigh(uint8_t timeout_us) {
    return dht_await(true, timeout_us);
}

/**
 * Awaits the low state for the Port Input Register or until a timeout occurs.
 * 
 * timeout_us - The timeout in microseconds.
 * 
 * returns - True if successfully recieved desired state, otherwise false, i.e. timeout. 
 */
bool dht_awaitLow(uint8_t timeout_us) {
    return dht_await(false, timeout_us);
}