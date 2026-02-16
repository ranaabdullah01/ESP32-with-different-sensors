#define PIN_RED   15
#define PIN_GREEN 2
#define PIN_BLUE  4

#define LEDC_FREQ     5000   // 5 kHz
#define LEDC_RESOLUTION 8     // 0–255

#define NUM_COLORS 10

// Array of 10 colors (R, G, B) 0–255
const uint8_t colors[NUM_COLORS][3] = {
  {255,   0,   0}, // Red
  {  0, 255,   0}, // Green
  {  0,   0, 255}, // Blue
  {255, 255,   0}, // Yellow
  {  0, 255, 255}, // Cyan
  {255,   0, 255}, // Magenta
  {255, 255, 255}, // White
  {255, 165,   0}, // Orange
  {128,   0, 128}, // Purple
  { 64, 224, 208}  // Turquoise
};

void setup() {
  ledcAttach(PIN_RED, LEDC_FREQ, LEDC_RESOLUTION);
  ledcAttach(PIN_GREEN, LEDC_FREQ, LEDC_RESOLUTION);
  ledcAttach(PIN_BLUE, LEDC_FREQ, LEDC_RESOLUTION);
}

void loop() {
  for (int i = 0; i < NUM_COLORS; i++) {
    ledcWrite(PIN_RED, colors[i][0]);
    ledcWrite(PIN_GREEN, colors[i][1]);
    ledcWrite(PIN_BLUE, colors[i][2]);

    delay(2000);
  }
}