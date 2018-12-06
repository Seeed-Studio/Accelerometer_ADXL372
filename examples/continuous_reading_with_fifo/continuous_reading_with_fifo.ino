
#include "Wire.h"
#include "adxl372.h"

#define BUFFER_SIZE   (160*6)

ADXL372 acc;
xyz_t xyz;

uint8_t buffer[BUFFER_SIZE] = {0,};

void setup() {
  Serial.begin(115200);

  acc.begin();
  
  Serial.println(acc.id(), HEX);
  acc.timing_ctrl(RATE_400);
  acc.measurement_ctrl(BW_200, true);
  acc.fifo_ctrl(STREAMED, FIFO_XYZ);
  acc.power_ctrl(MEASUREMENT_MODE);

  Serial.println("unit 100 mg");
}

void loop() {
  uint16_t samples = acc.samples_in_fifo();
  // To ensure that data is not overwritten and stored out of order, 
  // at least one sample set must be left in the FIFO after every read        
  if (samples > 12) {
    samples = (samples > BUFFER_SIZE)? BUFFER_SIZE : (samples / 6 - 1) * 6;

    acc.fifo_read(buffer, samples);
    for (uint16_t i=0; i<samples; i+=6) {
      // convert raw data
      xyz_t *xyz = acc.format(buffer + i);
      Serial.print(samples);
      Serial.print('\t');
      Serial.print(xyz->x);
      Serial.print('\t');
      Serial.print(xyz->y);
      Serial.print('\t');
      Serial.println(xyz->z);
    }

  }

  delay(40);
}
