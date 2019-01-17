
#include "Wire.h"
#include "adxl372.h"

#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SERIALUSB
  #define SYS_VOL   3.3
#else
  #define SERIAL Serial
  #define SYS_VOL   5
#endif


float cali_data[3];

#define CALI_BUF_LEN           15
#define CALI_INTERVAL_TIME     250

float cali_buf[3][CALI_BUF_LEN];


ADXL372 acc;
xyz_t xyz;

float deal_cali_buf(float *buf)
{
	float cali_val = 0;
	
	for(int i = 0;i < CALI_BUF_LEN;i++)
	{
		cali_val += buf[i];
	}
	cali_val = cali_val/CALI_BUF_LEN;
	return (float)cali_val;
}


void calibration(void)
{
	SERIAL.println("Please Place the module horizontally!");
	delay(1000);
	SERIAL.println("Start calibration........");
	
  
	for(int i=0;i<CALI_BUF_LEN;i++)
	{
		while (!(acc.status() & DATA_READY)); 
		acc.read(&xyz);
		cali_buf[0][i] = xyz.x;
		cali_buf[1][i] = xyz.y;
		cali_buf[2][i] = xyz.z;
		delay(CALI_INTERVAL_TIME);
		SERIAL.print('.');
	}
	SERIAL.println('.');
	for(int i=0;i<3;i++)
	{
		cali_data[i] =  deal_cali_buf(cali_buf[i]);
		if(2 == i){
      
			cali_data[i] -= 10;
		}
		SERIAL.println(cali_data[i]);
	}
	SERIAL.println("Calibration OK!!");
}


void setup() {
  SERIAL.begin(115200);

  acc.begin();
  
  SERIAL.println(acc.id(), HEX);
  acc.timing_ctrl(RATE_400);
  acc.measurement_ctrl(BW_200, true);
  acc.power_ctrl(MEASUREMENT_MODE);
  acc.setActiveTime(10);

  calibration();
}

void loop() {
  if (acc.status() & DATA_READY) {
    acc.read(&xyz);
    SERIAL.print("X position acc = ");
    SERIAL.print((xyz.x - cali_data[0]) / 10.0);
    SERIAL.println(" g ");
    SERIAL.print("Y position acc = ");
    SERIAL.print((xyz.y - cali_data[1]) / 10.0);
    SERIAL.println(" g ");
    SERIAL.print("Z position acc = ");
    SERIAL.print((xyz.z - cali_data[2]) / 10.0);
    SERIAL.println(" mg ");
  }
  SERIAL.println("   ");
  SERIAL.println("   ");
  delay(1000);
}
