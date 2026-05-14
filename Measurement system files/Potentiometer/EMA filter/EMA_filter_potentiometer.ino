// This test reads the potentiometer at a sampling frequency fs.
// An EMA low-pass filter with cutoff frequency fc is applied.
// Both the raw and filtered ADC values are logged for later analysis.

const int potPin = 32; // ADC pin
const float fs = 250.0; // Sampling frequency
const float fc = 8.0; // cutoff frequency (Hz)

//filter parameters
const float h = 1.0 / fs; // Sampling time
const float Tf = 1.0 / (2.0 * PI * fc); // Time constant
const float a = h / (Tf + h); // EMA coefficient

unsigned long lastSample = 0;
unsigned long samplePeriod_us = 1000000.0 / fs;   // Period in microseconds


float y_prev = 0; // y(t_k-1) (previous filter output)
float y = 0; // y(t_k) (current filter output)

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // 12bit ADC (0–4095)
  analogSetAttenuation(ADC_11db); //input range 0-3.3V

  int raw0 = analogRead(potPin);
  y_prev = raw0; //starting filter on measured value
}

void loop() {

  if (micros() - lastSample >= samplePeriod_us) {

    lastSample += samplePeriod_us;

    // u(t_k) (raw input)
    int raw = analogRead(potPin);


    // EMA filter
    // y(t_k) = (1-a)*y(t_k-1) + a*u(t_k)
    y = (1.0 - a) * y_prev + a * raw;

    // Store current output for next iteration
    y_prev = y;

    float t = millis() / 1000.0;

    // Print raw and filtered
     Serial.print(t, 3);
    Serial.print(",");
    Serial.print(raw);
    Serial.print(",");
    Serial.println(y, 3);

    
  }
}

