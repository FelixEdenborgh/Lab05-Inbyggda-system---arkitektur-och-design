#include "uart.h"
#include "dht.h"
#include <time.h>

int main(void) {
    // Ensure printf is available
    uart_init();
    // Wait for sensor to settle
    dht_init();

    // Structure holding measurements
    dht_measurement measurement;

    while(true) {
        // Read values into struct and check dht_status
        dht_status status = dht_read(&measurement);

        // Only print if SUCCESS. Sometimes the sensor needs to re-read
        if(status == SUCCESS) {
            printf("Temp: %d, Humidity: %d\n", measurement.temperature, measurement.humidity);
        }

        // Wait for 1 second
        _delay_ms(1000);
    }

    return 0;
}