//This script measures values from the HX711 through an ESP32 as the strain gauge installation is tested on the tensile test machiene.

#include "HX711.h"

#define DOUT_PIN 4
#define SCK_PIN 14

HX711 scale;

void setup() {
  Serial.begin(115200);
  delay(1000);

  scale.begin(DOUT_PIN, SCK_PIN);
  scale.set_gain(128);

}

void loop() {
  if (scale.is_ready()) {
    long value = scale.read(); //used to evaluate output with constant load

    //long value = scale.read_average(10);  //used while measuring applied load

    Serial.println(value);
  } 

}