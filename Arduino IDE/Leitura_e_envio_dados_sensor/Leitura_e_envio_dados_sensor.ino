#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Arduino_JSON.h>
#include "SPIFFS.h"
#include <Adafruit_SSD1306.h>

// Network credentials
const char* ssid = "ssssss";
const char* password = "cenouras";

// Create an SSD1306 object
Adafruit_SSD1306 display;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;
unsigned long previousTime = 0.0;
float time_delta = 0.0;
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 200;

// Create a sensor object
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;
float accX, accY, accZ;
float temperature;

// Gyroscope sensor deviation
float gyroXerror = 0.14;
float gyroYerror = 0.09;
float gyroZerror = 0.09;

// Offset values for gyroscopes
float gyroXoffset = 0.0;
float gyroYoffset = 0.0;
float gyroZoffset = 0.0;

float vX_now = 0.0;
float vX_ant = 0.0;
float vY_now = 0.0;
float vY_ant = 0.0;
float vZ_now = 0.0;
float vZ_ant = 0.0;

float X_now = 0.0;
float X_ant = 0.0;
float Y_now = 0.0;
float Y_ant = 0.0;
float Z_now = 0.0;
float Z_ant = 0.0;

// Variables for new referential
float accX_world = 0.0;
float accY_world = 0.0;
float accZ_world = 0.0;

// Constants for sensor calibration
float ACCEL_X_OFFSET = 0.0;
float ACCEL_Y_OFFSET = 0.0;
float ACCEL_Z_OFFSET = 0.0;

float gyroX_temp;
float gyroY_temp;
float gyroZ_temp;

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

// Number of samples for gyro offset calculation
const int numOffsetSamples = 3000;
int offsetSampleCount = 0;

// Initialize MPU6050
void initMPU(){
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}

void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.println(WiFi.localIP());

}


String getGyroReadings(){
  mpu.getEvent(&a, &g, &temp);

  gyroX_temp = g.gyro.x - gyroXoffset;
  if(abs(gyroX_temp) > gyroXerror)  {
    gyroX += gyroX_temp*3;
  }
  
  gyroY_temp = g.gyro.y - gyroYoffset;
  if(abs(gyroY_temp) > gyroYerror) {
    gyroY += gyroY_temp*3;
  }

  gyroZ_temp = g.gyro.z - gyroZoffset;
  
  if(abs(gyroZ_temp) > gyroZerror) {
    gyroZ += gyroZ_temp*3;
  }

  readings["gyroX"] = String(gyroX);
  readings["gyroY"] = String(gyroY);
  readings["gyroZ"] = String(gyroZ);

  String jsonString = JSON.stringify(readings);
  return jsonString;
}

String getAccReadings() {
  mpu.getEvent(&a, &g, &temp);
  float accel_x, accel_y, accel_z;
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z; 
  readings["accX"] = String(accX);
  readings["accY"] = String(accY);
  readings["accZ"] = String(accZ);
  String accString = JSON.stringify (readings);
  return accString;
}

String getTemperature(){
  mpu.getEvent(&a, &g, &temp);
  temperature = temp.temperature;
  return String(temperature);
}

void transformToWorldCoordinates(float gyroX, float gyroY, float gyroZ, float accX, float accY, float accZ, float& accX_world, float& accY_world, float& accZ_world, float& roll, float& pitch, float& yaw) {
  // Convert roll, pitch, and yaw angles from degrees to radians
  roll = radians(gyroX);
  pitch = radians(gyroY);
  yaw = radians(gyroZ);
  
  // Convert accelerometer readings to world coordinates
  accX_world = (accX * cos(pitch) * cos(yaw) + accY * (sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw)) + accZ * (cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw)));
  accY_world = (accX * cos(pitch) * sin(yaw) + accY * (sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(yaw)) + accZ * (cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw)));
  accZ_world = -accX * sin(pitch) + accY * sin(roll) * cos(pitch) + accZ * cos(roll) * cos(pitch);

    // Verificar os limites de accX_world
  if (accX_world < 2 && accX_world > -1.5) {
    accX_world = 0.0;
    vX_now = 0.0;
    //X_now = X_ant;
  }

    // Verificar os limites de accY_world
  if (accY_world < 1.5 && accY_world > -1.0) {
    accY_world = 0.0;
    vY_now = 0.0;
    //Y_now = Y_ant;
  }

    // Verificar os limites de accZ_world
  if (accZ_world < 0.2 && accZ_world > -9.4) {
    accZ_world = 0.0;
    vZ_now = 0.0;
    //Z_now = Z_ant;
  }
  
    // Calculate velocity using gyroscope data
  vX_now = vX_ant + accX_world * (time_delta/1000);  // Equação da velocidade
  vX_ant = vX_now;

  vY_now = vY_ant + accY_world * (time_delta/1000); // Equação da velocidade
  vY_ant = vY_now;

  vZ_now = vZ_ant + accZ_world * (time_delta/1000);  // Equação da velocidade
  vZ_ant = vZ_now;

    // Calculate position using velocity
  X_now = X_ant + vX_ant * (time_delta/1000) + (accX_world * pow(time_delta/1000, 2)) / 2.0;  // Equação da posição
  X_ant = X_now;

  Y_now = Y_ant + vY_ant * (time_delta/1000) + (accY_world * pow(time_delta/1000, 2)) / 2.0;  // Equação da posição
  Y_ant = Y_now;
    
  Z_now = Z_ant + vZ_ant * (time_delta/1000) + (accZ_world * pow(time_delta/1000, 2)) / 2.0;  // Equação da posição
  Z_ant = Z_now;
}

const int INTERRUPT_PIN = 5;

void setup() {
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), serialPrint,RISING);
  Serial.begin(115200);
  initWiFi();
  initSPIFFS();
  initMPU();

  // Calculate gyro offsets
  float gyroXsum = 0.0;
  float gyroYsum = 0.0;
  float gyroZsum = 0.0;

  // Read gyro values for offset calculation
  for (int i = 0; i < numOffsetSamples; i++) {
    mpu.getEvent(&a, &g, &temp);
    gyroXsum += g.gyro.x;
    gyroYsum += g.gyro.y;
    gyroZsum += g.gyro.z;
    offsetSampleCount++;
    
    if (offsetSampleCount >= numOffsetSamples) {
      break;
    }
  }
     
  // Calculate gyro offsets
  gyroXoffset = gyroXsum / offsetSampleCount;
  gyroYoffset = gyroYsum / offsetSampleCount;
  gyroZoffset = gyroZsum / offsetSampleCount;

  // Print the calculated offsets
  Serial.print("Gyro offsets: X=");
  Serial.print(gyroXoffset);
  Serial.print("  Y=");
  Serial.print(gyroYoffset);
  Serial.print("  Z=");
  Serial.println(gyroZoffset);

  Wire.begin();
  Serial.println("MPU6050 OLED printing");

  if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }
  Serial.println("Found a MPU-6050 sensor");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  // Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX=0;
    gyroY=0;
    gyroZ=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetX", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetY", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroY=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetZ", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroZ=0;
    request->send(200, "text/plain", "OK");
  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();

  // Initialize the SSD1306 display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.display();

  pinMode(17, INPUT);
}

void loop() {
  unsigned long currentTime = millis();
  time_delta = (currentTime - previousTime);
  previousTime = currentTime;

  float gyroXsum = 0.0;
  float gyroYsum = 0.0;
  float gyroZsum = 0.0;
  
  if (offsetSampleCount < numOffsetSamples) {
    // Offset calculation is still in progress
    mpu.getEvent(&a, &g, &temp);
    gyroXsum += g.gyro.x;
    gyroYsum += g.gyro.y;
    gyroZsum += g.gyro.z;
    offsetSampleCount++;
  } else {
    if ((millis() - lastTime) > gyroDelay) {
      // Update gyro readings
      String gyroReadings = getGyroReadings();
      events.send(gyroReadings.c_str(), "gyro", millis());
      lastTime = millis();
    }

    if ((millis() - lastTimeAcc) > accelerometerDelay) {
      // Update accelerometer readings
      String accReadings = getAccReadings();
      events.send(accReadings.c_str(), "accelerometer", millis());
      lastTimeAcc = millis();
    }

    if ((millis() - lastTimeTemperature) > temperatureDelay) {
      // Update temperature readings
      String tempReadings = getTemperature();
      events.send(tempReadings.c_str(), "temperature", millis());
      lastTimeTemperature = millis();
    }
    currentTime = millis();

  }

  // Calculate world coordinates
  transformToWorldCoordinates(gyroX, gyroY, gyroZ, accX, accY, accZ, accX_world, accY_world, accZ_world, roll, pitch, yaw);

  accX_world -= 0;
  accY_world -= 0;
  accZ_world -= 9.7;
  
  // Atualizar o tempo da última atualização
  currentTime = millis();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("X:");
  display.print(accX_world);
  display.print(" Gx:");
  display.println(gyroX);
  display.print("Y:");
  display.print(accY_world);
  display.print(" Gy:");
  display.println(gyroY);
  display.print("Z:");
  display.print(accZ_world);
  display.print(" Gz:");
  display.println(gyroZ);
  
  // Quando o botão é premido aparece "Pistola Ativada"
  if (digitalRead(17) == HIGH) {
    display.println("Pistola ativada");
  }
  display.display();

  // Print the readings to the serial monitor
  Serial.print(accX_world);
  Serial.print(", ");
  Serial.print(accY_world);
  Serial.print(", ");
  Serial.print(accZ_world);
  Serial.print(", ");
  Serial.print(gyroX);
  Serial.print(", ");
  Serial.print(gyroY);
  Serial.print(", ");
  Serial.println(gyroZ);
}

void serialPrint()
{
  String gyroReadings = getGyroReadings();
  Serial.println(gyroReadings);
}
