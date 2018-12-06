
#include "Wire.h"
#include "adxl372.h"

ADXL372 acc;
xyz_t xyz;

void setup() {
  Serial.begin(115200);

  acc.begin();
  
  Serial.println(acc.id(), HEX);
  acc.timing_ctrl(RATE_400);
  acc.measurement_ctrl(BW_200, true);
  acc.power_ctrl(MEASUREMENT_MODE);
}

void loop() {
  if (acc.status() & DATA_READY) {
    acc.read(&xyz);
    Serial.print(xyz.x);
    Serial.print('\t');
    Serial.print(xyz.y);
    Serial.print('\t');
    Serial.println(xyz.z);
  }

  delay(10);
}
