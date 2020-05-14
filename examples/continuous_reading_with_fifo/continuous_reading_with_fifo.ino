
#include "Wire.h"
#include "adxl372.h"

#define BUFFER_SIZE   (160*6)

#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define SERIAL Serial
    #define SYS_VOL   3.3
#else
    #define SERIAL Serial
    #define SYS_VOL   5
#endif

ADXL372 acc;
xyz_t xyz;

uint8_t buffer[BUFFER_SIZE] = {0,};

void setup() {
    SERIAL.begin(115200);

    acc.begin();

    SERIAL.println(acc.id(), HEX);
    acc.timing_ctrl(RATE_400);
    acc.measurement_ctrl(BW_200, true);
    acc.fifo_ctrl(STREAMED, FIFO_XYZ);
    acc.power_ctrl(MEASUREMENT_MODE);

    SERIAL.println("unit 100 mg");
}

void loop() {
    uint16_t samples = acc.samples_in_fifo();
    // To ensure that data is not overwritten and stored out of order,
    // at least one sample set must be left in the FIFO after every read
    if (samples > 12) {
        samples = (samples > BUFFER_SIZE) ? BUFFER_SIZE : (samples / 6 - 1) * 6;

        acc.fifo_read(buffer, samples);
        for (uint16_t i = 0; i < samples; i += 6) {
            // convert raw data
            xyz_t* xyz = acc.format(buffer + i);
            SERIAL.print(samples);
            SERIAL.print('\t');
            SERIAL.print(xyz->x);
            SERIAL.print('\t');
            SERIAL.print(xyz->y);
            SERIAL.print('\t');
            SERIAL.println(xyz->z);
        }

    }

    delay(40);
}
