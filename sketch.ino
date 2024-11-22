#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>
#include <ArduinoJson.h>


#define RXD2 16
#define TXD2 17
#define BETA 3950
#define analogPin 34

// Variables de datos
int steps = 0;
int heartRate = 75;
const int threshold = 1.5; // Umbral para detectar pasos
float batteryLevel = 100.0;

Adafruit_MPU6050 mpu;

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* serverURL = "https://trailsync-h4cpdje8dza6esed.brazilsouth-01.azurewebsites.net/api/v1/boots/touristId=1/serviceId=1/bootId=1"; // Cambia esto por la URL de tu backend

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // GPS
  Wire.begin();  // I2C

  // Inicializar MPU6050
  if (!mpu.begin()) {
    Serial.println("No se detecta el MPU6050!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // Conectar a Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop() {
  // Leer temperatura
  float temperature = readTemperature();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" ℃");

  // Conteo de pasos
  steps += countSteps();
  Serial.print("Steps: ");
  Serial.println(steps);

  // Simulación de frecuencia cardíaca
  heartRate = random(60, 100);  // Genera un valor aleatorio entre 60 y 100 BPM
  Serial.print("Heart Rate: ");
  Serial.print(heartRate);
  Serial.println(" BPM");

  // Enviar datos al backend
  sendDataToBackend(temperature, steps, heartRate);

  delay(1000);  // Espera antes de la siguiente lectura
}

// Función para leer temperatura
float readTemperature() {
  int analogValue = analogRead(analogPin);
  return 1 / (log(1 / (4095. / analogValue - 1)) / BETA + 1.0 / 298.15) - 273.15;
}

// Función para contar pasos usando el acelerómetro
int countSteps() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (abs(a.acceleration.z) > threshold) {
    delay(500); // Evita contar un paso varias veces
    return 1;
  }
  return 0;
}

void batteryPorcentage(){
  batteryLevel -= 0.1;
  if (batteryLevel < 0) batteryLevel = 0;
  Serial.print("Battery Level: ");
  Serial.print(batteryLevel);
  Serial.println("%");
}

String getBootData() {
  HTTPClient http;
  http.begin(serverURL);

  int httpResponseCode = http.GET();
  String response = "";

  if (httpResponseCode == 200) {
    response = http.getString();
    Serial.println("Datos obtenidos del servidor:");
    Serial.println(response);
  } else {
    Serial.print("Error al obtener datos. Código: ");
    Serial.println(httpResponseCode);
  }

  http.end();
  return response;
}

// Función para enviar datos al backend
void sendDataToBackend(float temperature, int steps, int heartRate) {
  batteryPorcentage();
  if (WiFi.status() == WL_CONNECTED) {
    String currentData = getBootData();
    if (currentData != "") {
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, currentData);

      // Actualizar solo los campos necesarios
      doc["temperature"] = temperature;
      doc["steps"] = steps;
      doc["heartRate"] = heartRate;
      doc["batery"] = batteryLevel;

      String jsonData;
      serializeJson(doc, jsonData);

      Serial.println("JSON enviado:");
      Serial.println(jsonData);

      // Enviar datos al servidor
      int httpResponseCode = sendPUTRequest(serverURL, jsonData.c_str());

      if (httpResponseCode == 200) {
        Serial.println("Actualizado con éxito en el servidor.");
      } else {
        Serial.print("Error en la solicitud. Código de respuesta HTTP: ");
        Serial.println(httpResponseCode);
      }
    }
  } else {
    Serial.println("Not connected to WiFi");
  }
}


int sendPUTRequest(const char* serverURL, const char* jsonData) {
  HTTPClient http;
  http.begin(serverURL);
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.PUT(jsonData);

  if (httpResponseCode == -1) {
    Serial.print("Error: ");
    Serial.println(http.errorToString(httpResponseCode).c_str());
  }

  http.end();
  return httpResponseCode;
}