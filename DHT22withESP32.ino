#include <DHT.h>

// DHT22 connections
#define DHTPIN 15        // GPIO4 connected to DHT22 data pin
#define DHTTYPE DHT22   // DHT22 sensor type

// Create DHT object
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  Serial.println("DHT22 Temperature & Humidity Sensor Test");
  
  // Initialize DHT sensor
  dht.begin();
  
  delay(2000);
}

void loop() {
  // Wait a few seconds between measurements
  delay(2000);
  
  // Read temperature as Celsius
  float temperature = dht.readTemperature();
  
  // Read temperature as Fahrenheit
  float temperatureF = dht.readTemperature(true);
  
  // Read humidity
  float humidity = dht.readHumidity();
  
  // Check if any reads failed
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  
  // Calculate heat index (apparent temperature)
  float heatIndexC = dht.computeHeatIndex(temperature, humidity, false);
  float heatIndexF = dht.computeHeatIndex(temperatureF, humidity);
  
  // Print results
  Serial.println("-----------------------------------");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C / ");
  Serial.print(temperatureF);
  Serial.println(" °F");
  
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  
  Serial.print("Heat Index: ");
  Serial.print(heatIndexC);
  Serial.print(" °C / ");
  Serial.print(heatIndexF);
  Serial.println(" °F");
}