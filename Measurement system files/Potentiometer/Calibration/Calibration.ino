//This plot is used for calibration of the potentiometer with sampling frequency of 250Hz. An average is used for logging to the serial monitor giving an output of approximately 
//1 value per second

const int potPin = 32;

const float fs = 250.0;  // Sampling frequency [Hz]
const unsigned long samplePeriod_us = 1000000.0 / fs;

unsigned long lastSample = 0;

long sumRaw = 0;
int sampleCount = 0;

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);        // 12-bit ADC: 0–4095
  analogSetAttenuation(ADC_11db);  // Input range approximately 0–3.3 V

  delay(1000);
}

void loop() {
  if (micros() - lastSample >= samplePeriod_us) {
    lastSample += samplePeriod_us;

    int raw = analogRead(potPin);

    sumRaw += raw;
    sampleCount++;

    // Print one averaged value per second
    if (sampleCount >= fs) {
      float averageRaw = sumRaw / (float)sampleCount;

      Serial.println(averageRaw, 3);

      sumRaw = 0;
      sampleCount = 0;
    }
  }
}
