#include <Wire.h>
#include <vl53l7ch.h>
 
#define DEV_I2C Wire
#define LPN_PIN 6

void print_result(VL53LMZ_ResultsData *Result);
 
// Components.
VL53L7CH sensor_vl53l7ch_top(&DEV_I2C, LPN_PIN);

uint8_t res = VL53LMZ_RESOLUTION_8X8;
char report[256];
uint8_t status;

void setup()
{
  Serial.begin(460800);
  delay(500);
  Serial.println("\nI2C Scanner");
  delay(500);

  DEV_I2C.setSCL(1);
  DEV_I2C.setSDA(0);
  DEV_I2C.begin();
  
  // Configure VL53L7CH component:
  sensor_vl53l7ch_top.begin();
  status = sensor_vl53l7ch_top.init();
  sensor_vl53l7ch_top.set_resolution(VL53LMZ_RESOLUTION_8X8);
 
  // Start Measurements
  status = sensor_vl53l7ch_top.start_ranging();
  sensor_vl53l7ch_top.set_ranging_frequency_hz(16);
}

void loop()
{
  VL53LMZ_ResultsData Results;
  uint8_t NewDataReady = 0;
 
  do {
    status = sensor_vl53l7ch_top.check_data_ready(&NewDataReady);
  } while (!NewDataReady);
 
  if ((!status) && (NewDataReady != 0)) {
    status = sensor_vl53l7ch_top.get_ranging_data(&Results);
    print_result(&Results, "0");
  }
  delay(1);
}
 
void print_result(VL53LMZ_ResultsData *Result, const char *sensor_id)
{
  Serial.print("Sensor: ");Serial.print(sensor_id);Serial.print(" ");
  const uint8_t zones_per_line = 8; // 8x8 Resolution
  for (uint8_t row = 0; row < zones_per_line; row++) {
    for (uint8_t col = 0; col < zones_per_line; col++) {
      uint8_t zone_index = row * zones_per_line + col;
      
      if (Result->nb_target_detected[zone_index] > 0) {
        Serial.print(Result->distance_mm[zone_index * VL53LMZ_NB_TARGET_PER_ZONE]);
      } else {
        Serial.print("9999");
        // The value 9999 is used when the sensor doesnt get a value or recieves an error.
        // My following programs then uses this value to proceed
      }
      Serial.print(" ");
    }
  }
  Serial.println("");
}
 
