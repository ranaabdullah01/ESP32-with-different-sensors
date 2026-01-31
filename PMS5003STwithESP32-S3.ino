#include <Arduino.h>
#include <HardwareSerial.h>

// PMS5003T UART
#define PMS_RX_PIN 18 //change RX pin according to your esp32
#define PMS_TX_PIN 17 //change RX pin according to your esp32
#define PMS_SERIAL Serial1

// Global variables to store sensor data
uint16_t currentPM1 = 0;      // Added PM1.0
uint16_t currentPM25 = 0;
uint16_t currentPM10 = 0;
float currentTemp = 0.0;
float currentHum = 0.0;
int currentAQI = 0;

// AQI calculation function
int calculateAQI(float pm25);
void parseFrame(uint8_t* frame);
void printAQIInfo(int aqi);
bool verifyChecksum(uint8_t* frame, uint16_t length);
void debugFrameLayout(uint8_t* frame, int total_len);

void setup() {
  Serial.begin(115200);
  
  // Initialize PMS5003T serial
  PMS_SERIAL.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  
  delay(3000);
  
  Serial.println("PMS5003T - AQI Monitor with PM1.0 Support");
  Serial.println("==========================================");
  Serial.println("Based on official datasheet v2.6");
  Serial.println("Waiting for sensor data...\n");
}

void loop() {
  static uint8_t buffer[128];
  static int index = 0;
  
  // Read PMS sensor data
  while (PMS_SERIAL.available()) {
    buffer[index] = PMS_SERIAL.read();
    
    if (index >= 31) {
      for (int start = 0; start <= index - 31; start++) {
        if (buffer[start] == 0x42 && buffer[start+1] == 0x4D) {
          uint16_t frame_len = (buffer[start+2] << 8) | buffer[start+3];
          int total_len = frame_len + 4;
          
          if (index - start + 1 >= total_len) {
            parseFrame(&buffer[start]);
            int remaining = index - start - total_len + 1;
            for (int i = 0; i <= remaining; i++) {
              buffer[i] = buffer[start + total_len + i];
            }
            index = remaining;
            break;
          }
        }
      }
    }
    
    index = (index + 1) % 128;
  }
  
  delay(10);
}

bool verifyChecksum(uint8_t* frame, uint16_t length) {
  uint16_t checksum = 0;
  for (int i = 0; i < length - 2; i++) {
    checksum += frame[i];
  }
  uint16_t received_checksum = (frame[length-2] << 8) | frame[length-1];
  return checksum == received_checksum;
}

void debugFrameLayout(uint8_t* frame, int total_len) {
  Serial.println("\nDetailed Frame Layout (Based on Datasheet):");
  Serial.println("Byte Pos  Hex     Dec  Description");
  Serial.println("--------  ---     ---  -----------");
  
  for (int i = 0; i < total_len; i += 2) {
    if (i + 1 < total_len) {
      uint16_t value = (frame[i] << 8) | frame[i+1];
      
      switch(i) {
        case 0:  Serial.printf("0-1:     0x%02X%02X  %5d  Start chars (BM)\n", frame[0], frame[1], value); break;
        case 2:  Serial.printf("2-3:     0x%02X%02X  %5d  Frame length\n", frame[2], frame[3], value); break;
        case 4:  Serial.printf("4-5:     0x%02X%02X  %5d  PM1.0 CF=1 (Data1)\n", frame[4], frame[5], value); break;
        case 6:  Serial.printf("6-7:     0x%02X%02X  %5d  PM2.5 CF=1 (Data2)\n", frame[6], frame[7], value); break;
        case 8:  Serial.printf("8-9:     0x%02X%02X  %5d  PM10 CF=1 (Data3)\n", frame[8], frame[9], value); break;
        case 10: Serial.printf("10-11:   0x%02X%02X  %5d  PM1.0 atm (Data4)\n", frame[10], frame[11], value); break;
        case 12: Serial.printf("12-13:   0x%02X%02X  %5d  PM2.5 atm (Data5)\n", frame[12], frame[13], value); break;
        case 14: Serial.printf("14-15:   0x%02X%02X  %5d  PM10 atm (Data6)\n", frame[14], frame[15], value); break;
        case 16: Serial.printf("16-17:   0x%02X%02X  %5d  >0.3um count (Data7)\n", frame[16], frame[17], value); break;
        case 18: Serial.printf("18-19:   0x%02X%02X  %5d  >0.5um count (Data8)\n", frame[18], frame[19], value); break;
        case 20: Serial.printf("20-21:   0x%02X%02X  %5d  >1.0um count (Data9)\n", frame[20], frame[21], value); break;
        case 22: Serial.printf("22-23:   0x%02X%02X  %5d  >2.5um count (Data10)\n", frame[22], frame[23], value); break;
        case 24: Serial.printf("24-25:   0x%02X%02X  %5d  >5.0um count (Data11)\n", frame[24], frame[25], value); break;
        case 26: Serial.printf("26-27:   0x%02X%02X  %5d  >10um count (Data12)\n", frame[26], frame[27], value); break;
        case 28: Serial.printf("28-29:   0x%02X%02X  %5d  Temperature (Data13) RAW\n", frame[28], frame[29], value); break;
        case 30: Serial.printf("30-31:   0x%02X%02X  %5d  Humidity (Data14) RAW\n", frame[30], frame[31], value); break;
        case 32: Serial.printf("32-33:   0x%02X%02X  %5d  Firmware version (Data15)\n", frame[32], frame[33], value); break;
        case 34: Serial.printf("34-35:   0x%02X%02X  %5d  Error code (Data16)\n", frame[34], frame[35], value); break;
        default: 
          if (i == total_len - 2) {
            Serial.printf("%02d-%02d:   0x%02X%02X  %5d  Checksum\n", i, i+1, frame[i], frame[i+1], value);
          } else {
            Serial.printf("%02d-%02d:   0x%02X%02X  %5d  Reserved/Unknown\n", i, i+1, frame[i], frame[i+1], value);
          }
          break;
      }
    }
  }
}

int calculateAQI(float pm25) {
  struct Breakpoint {
    float lowPM25;
    float highPM25;
    int lowAQI;
    int highAQI;
  };
  
  static const Breakpoint breakpoints[] = {
    {0.0, 12.0, 0, 50},
    {12.1, 35.4, 51, 100},
    {35.5, 55.4, 101, 150},
    {55.5, 150.4, 151, 200},
    {150.5, 250.4, 201, 300},
    {250.5, 350.4, 301, 400},
    {350.5, 500.4, 401, 500}
  };
  
  int numBreakpoints = sizeof(breakpoints) / sizeof(breakpoints[0]);
  
  if (pm25 < 0) pm25 = 0;
  if (pm25 > 500.4) pm25 = 500.4;
  
  for (int i = 0; i < numBreakpoints; i++) {
    if (pm25 >= breakpoints[i].lowPM25 && pm25 <= breakpoints[i].highPM25) {
      float aqi = ((float)(breakpoints[i].highAQI - breakpoints[i].lowAQI) / 
                  (breakpoints[i].highPM25 - breakpoints[i].lowPM25)) * 
                  (pm25 - breakpoints[i].lowPM25) + breakpoints[i].lowAQI;
      return (int)(aqi + 0.5);
    }
  }
  
  return 0;
}

void printAQIInfo(int aqi) {
  Serial.printf("AQI: %d - ", aqi);
  
  if (aqi <= 50) {
    Serial.println("Good");
  } else if (aqi <= 100) {
    Serial.println("Moderate");
  } else if (aqi <= 150) {
    Serial.println("Unhealthy for Sensitive Groups");
  } else if (aqi <= 200) {
    Serial.println("Unhealthy");
  } else if (aqi <= 300) {
    Serial.println("Very Unhealthy");
  } else {
    Serial.println("Hazardous - Health emergency!");
  }
}

void parseFrame(uint8_t* frame) {
  uint16_t frame_len = (frame[2] << 8) | frame[3];
  int total_len = frame_len + 4;
  
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 2000) {
    return;
  }
  lastPrint = millis();
  
  Serial.printf("\n=== PMS5003T Frame: %d bytes (0x%04X) ===\n", total_len, total_len);
  
  // Verify checksum
  if (!verifyChecksum(frame, total_len)) {
    Serial.println("⚠️ CHECKSUM ERROR!");
    debugFrameLayout(frame, total_len);
    return;
  }
  
  // Extract PM values
  uint16_t pm10_cf1 = (frame[4] << 8) | frame[5];      // PM1.0 CF=1
  uint16_t pm25_cf1 = (frame[6] << 8) | frame[7];      // PM2.5 CF=1
  uint16_t pm100_cf1 = (frame[8] << 8) | frame[9];     // PM10 CF=1
  uint16_t pm10_atm = (frame[10] << 8) | frame[11];    // PM1.0 atm (environmental)
  uint16_t pm25_atm = (frame[12] << 8) | frame[13];    // PM2.5 atm (environmental)
  uint16_t pm100_atm = (frame[14] << 8) | frame[15];   // PM10 atm (environmental)
  
  // Store PM values
  currentPM1 = pm10_atm;      // PM1.0 is stored in the "PM10 atm" field of early bytes
  currentPM25 = pm25_atm;
  currentPM10 = pm100_atm;
  
  // Particle counts
  uint16_t particles_03um = (frame[16] << 8) | frame[17];
  uint16_t particles_05um = (frame[18] << 8) | frame[19];
  uint16_t particles_10um = (frame[20] << 8) | frame[21];
  uint16_t particles_25um = (frame[22] << 8) | frame[23];
  uint16_t particles_50um = (frame[24] << 8) | frame[25];
  uint16_t particles_100um = (frame[26] << 8) | frame[27];
  
  // Extract Temperature and Humidity
  int16_t temp_raw = (frame[28] << 8) | frame[29];
  uint16_t hum_raw = (frame[30] << 8) | frame[31];
  float temperature = temp_raw / 10.0;
  float humidity = hum_raw / 10.0;
  
  // Check if values are plausible for indoor environment
  bool plausible = (temperature >= 15.0 && temperature <= 35.0 && 
                   humidity >= 20.0 && humidity <= 80.0);
  
  if (!plausible) {
    Serial.println("\n⚠️  WARNING: Implausible T/H readings for indoor environment!");
    
    // Try alternative: bytes 30-33
    if (total_len >= 34) {
      int16_t temp_alt = (frame[30] << 8) | frame[31];
      uint16_t hum_alt = (frame[32] << 8) | frame[33];
      float temp_alt_val = temp_alt / 10.0;
      float hum_alt_val = hum_alt / 10.0;
      
      if (temp_alt_val >= 15.0 && temp_alt_val <= 35.0 && 
          hum_alt_val >= 20.0 && hum_alt_val <= 80.0) {
        temperature = temp_alt_val;
        humidity = hum_alt_val;
        Serial.printf("   Using alt bytes 30-33: %.1f°C, %.1f%%\n", temperature, humidity);
      }
    }
  }
  
  // Store temperature and humidity
  currentTemp = temperature;
  currentHum = humidity;
  
  // Calculate AQI
  currentAQI = calculateAQI((float)currentPM25);
  
  // Display in Serial Monitor
  Serial.println("\n=== SENSOR READINGS ===");
  Serial.printf("PM1.0:       %d μg/m³\n", currentPM1);
  Serial.printf("PM2.5:       %d μg/m³\n", currentPM25);
  Serial.printf("PM10:        %d μg/m³\n", currentPM10);
  Serial.printf("Temperature: %.1f°C\n", currentTemp);
  Serial.printf("Humidity:    %.1f%%\n", currentHum);
  
  // Display particle counts (optional)
  Serial.println("\n=== PARTICLE COUNTS ===");
  Serial.printf(">0.3μm: %d particles\n", particles_03um);
  Serial.printf(">0.5μm: %d particles\n", particles_05um);
  Serial.printf(">1.0μm: %d particles\n", particles_10um);
  Serial.printf(">2.5μm: %d particles\n", particles_25um);
  Serial.printf(">5.0μm: %d particles\n", particles_50um);
  Serial.printf(">10μm:  %d particles\n", particles_100um);
  
  Serial.println("\n--- AIR QUALITY INDEX ---");
  Serial.printf("AQI: %d - ", currentAQI);
  printAQIInfo(currentAQI);
  Serial.println("========================================\n");
}