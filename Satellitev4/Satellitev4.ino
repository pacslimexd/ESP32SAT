#include <WiFi.h> //Wifi TCP
#include <Wire.h> //I2C
#include <Adafruit_INA219.h> //https://github.com/adafruit/Adafruit_INA219
#include <SparkFunLSM6DS3.h> //https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library
#include <Adafruit_AHTX0.h>  //https://github.com/adafruit/Adafruit_AHTX0
#include <ESP32Servo.h>      //https://madhephaestus.github.io/ESP32Servo/annotated.html

WiFiClient client;
const char* ssid = "STARMAN";
const char* password = "waitinginthesky";
const char* GNDstationIP = "192.168.4.1";
const uint16_t GNDstationPort = 80;

bool wifiConnected = false;
bool apConnected = false;
bool serverConnected = false;
const unsigned long Timeout = 10000; 
unsigned long lastConnectionCheck = 0;
const unsigned long connectionInterval = 30000; // 30 segundos

Adafruit_AHTX0 aht20;
LSM6DS3 lsm6ds3(I2C_MODE, 0x6A);
Adafruit_INA219 ina219;

const int LEDPWMresolution = 8;  
const int LEDPWMfreq = 10000;
const int LEDStep = 32;
const int R = 40;
const int G = 41;
const int B = 42;
int redIntensity = 0;
int greenIntensity = 0;
int blueIntensity = 0;

Servo myServo;            
const int servoPin = 4;   
const int servoStep = 10;
int servoAngle = 90;

//******CÓDIGO PRINCIPAL******

void setup() {
  Serial.begin(115200);
  while (!Serial);
  displayHeader();
  Wire.begin(8,9); //SDA:8, SCL:9

  ledcSetClockSource(LEDC_AUTO_CLK);
  ledcAttach(R, LEDPWMfreq, LEDPWMresolution);
  ledcAttach(G, LEDPWMfreq, LEDPWMresolution);
  ledcAttach(B, LEDPWMfreq, LEDPWMresolution);

  myServo.setPeriodHertz(50);            // Configura frecuencia PWM a 50Hz
  myServo.attach(servoPin, 500, 2500);   // Pin, min y max ancho de pulso (μs)
  myServo.write(servoAngle);             // Ángulo inicial 

  bool aht20_on = aht20.begin();
  bool sm6ds3_on = (lsm6ds3.begin() == 0);
  bool ina219_on = ina219.begin();
  if (ina219_on) ina219.setCalibration_32V_2A();

  if (ina219_on && sm6ds3_on && aht20_on) {
    Serial.println("Sensors initialized!");
  } else {
    Serial.println("Sensors failed to initialize.");
  }
}

void loop() {
  unsigned long now = millis();
  bool timeToCheck = false;

  // Si estamos conectados, checar cada 30s
  if (wifiConnected) {
    if (now - lastConnectionCheck >= connectionInterval) {
      timeToCheck = true;
    }
  } else {
    timeToCheck = true;
  }

  // Checar conexión
  if (timeToCheck) {
    lastConnectionCheck = now;
    apConnected = (WiFi.status() == WL_CONNECTED);
    serverConnected = client.connected();
    wifiConnected = apConnected && serverConnected;
    if (!wifiConnected) {
      Serial.println("Rechecking connection...");
      client.stop(); // Limpia cualquier intento previo
      connectToWiFi();
    }
  }

  // Comunicación
  if (wifiConnected) {
    receiveMessages();
    transMessages();
  }
}

//******FUNCIONES******

void displayHeader() {
  Serial.println();
  Serial.println("***********************************");
  Serial.println("*        Satellite Segment        *");
  Serial.println("*              v.4.0              *");
  Serial.println("*         Name: Satellite         *");
  Serial.println("***********************************");
  Serial.println(); 
}

void connectToWiFi() {
  // Verifica conexión a AP (solo si está desconectado)
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("Connecting to GND Station WiFi AP: %s...\n", ssid);
    WiFi.begin(ssid, password);

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < Timeout) {
      delay(500);
      Serial.print(".");
    }

    apConnected = (WiFi.status() == WL_CONNECTED);
    Serial.println(apConnected ? "\nConnected to WiFi." : "\nFailed to connect to WiFi.");
  } else {
    apConnected = true;
  }

  // Intentar reconectar al servidor solo si no está conectado
  if (apConnected && !client.connected()) {
    Serial.printf("Connecting to GND Station Server IP: %s:%u...\n", GNDstationIP, GNDstationPort);
    unsigned long startTime = millis();
    while (!client.connect(GNDstationIP, GNDstationPort) && millis() - startTime < Timeout) {
      delay(500);
      Serial.print(".");
    }
    serverConnected = client.connected();
    Serial.println(serverConnected ? "\nConnected to GND Station Server!" : "\nFailed to connect to server.");
  }
}

void receiveMessages() {
  while (client.available()) {
    String R_message = client.readStringUntil('\n');
    R_message.trim();

    if (R_message.length() > 0) {

      if (!R_message.startsWith("GND:ACK:")) {
        client.print("SAT:ACK: ");
        client.println(R_message);
      }

      if (R_message.startsWith("GND:ACK:")) {
        Serial.println(R_message);
        return;
      }

      if (R_message.startsWith("/cmd")) {
        if (R_message.startsWith("/cmd R ") || R_message.startsWith("/cmd G ") || R_message.startsWith("/cmd B ")) {
          String color = R_message.substring(5, 6);
          String arg = R_message.substring(7); arg.trim();
          handleRGBCommand(color, arg);

        } else if (R_message.startsWith("/cmd servo ")) {
          String arg = R_message.substring(11); arg.trim();
          handleServoCommand(arg);

        } else {
          Serial.printf("GND:MSG: %s\n", R_message);
        }

      } else if (R_message.startsWith("/tel")) {

        if (R_message == "/tel temperature") {
          sensors_event_t humidity, temp;
          aht20.getEvent(&humidity, &temp);
          client.printf("Temperature at SAT is %.2f C\n", temp.temperature);
          Serial.printf("Temperature was asked from GND. Lecture: %.2f C\n", temp.temperature);

        } else if (R_message == "/tel humidity") {
          sensors_event_t humidity, temp;
          aht20.getEvent(&humidity, &temp);
          client.printf("Humidity at SAT is %.2f %%\n", humidity.relative_humidity);
          Serial.printf("Humidity was asked from GND. Lecture: %.2f %%\n", humidity.relative_humidity);

        } else if (R_message == "/tel gyro") {
          float gx = lsm6ds3.readFloatGyroX();
          float gy = lsm6ds3.readFloatGyroY();
          float gz = lsm6ds3.readFloatGyroZ();
          client.printf("Gyroscope of SAT [X: %.2f | Y: %.2f | Z: %.2f] deg/s\n", gx, gy, gz);
          Serial.printf("Gyroscope requested from GND. Gx: %.2f | Gy: %.2f | Gz: %.2f deg/s\n", gx, gy, gz);

        } else if (R_message == "/tel acceleration") {
          float ax = lsm6ds3.readFloatAccelX();
          float ay = lsm6ds3.readFloatAccelY();
          float az = lsm6ds3.readFloatAccelZ();
          client.printf("Acceleration of SAT is [X: %.2f | Y: %.2f | Z: %.2f] m/ss\n", ax, ay, az);
          Serial.printf("Acceleration was asked from GND. Ax: %.2f | Ay: %.2f | Az: %.2f m/ss\n", ax, ay, az);

        } else if (R_message == "/tel current") {
          float current = ina219.getCurrent_mA();
          client.printf("Current of SAT is: %.2f mA\n", current);
          Serial.printf("Current was asked from GND. Lecture: %.2f mA\n", current);

        } else if (R_message == "/tel voltage") {
          float voltage = ina219.getBusVoltage_V();
          client.printf("Voltage of SAT is: %.2f V\n", voltage);
          Serial.printf("Voltage was asked from GND. Lecture: %.2f V\n", voltage);

        } else if (R_message == "/tel power") {
          float power = ina219.getPower_mW();
          client.printf("Power of SAT is: %.3f W\n", power / 1000.0);
          Serial.printf("Power was asked from GND. Lecture: %.3f W\n", power / 1000.0);

        } else if (R_message == "/tel rssi" || R_message.startsWith("/rssi tel ")) {
          handleRSSICommand(R_message, false);

        } else {
          Serial.printf("GND:MSG: %s\n", R_message);
        }

      } else {
        Serial.printf("GND:MSG: %s\n", R_message);
      }
    }
  }
}

void transMessages() {
  if (Serial.available()) {
    String T_message = Serial.readStringUntil('\n');
    T_message.trim();

    if (T_message.length() > 0) {

      if (T_message.startsWith("/cmd")) {
        if (T_message.startsWith("/cmd R ") || T_message.startsWith("/cmd G ") || T_message.startsWith("/cmd B ")) {
          String color = T_message.substring(5, 6);
          String arg = T_message.substring(7); arg.trim();
          handleRGBCommand(color, arg);

        } else if (T_message.startsWith("/cmd servo ")) {
          String arg = T_message.substring(11); arg.trim();
          handleServoCommand(arg);

        } else {
          client.println(T_message);
          Serial.printf("SAT:LOC: %s\n", T_message);
        }

      } else if (T_message.startsWith("/tel")) {
        if (T_message == "/tel temperature") {
          sensors_event_t humidity, temp;
          aht20.getEvent(&humidity, &temp);
          Serial.println("SAT:CMD: /tel temperature");
          Serial.printf("Temperature is %.2f C\n", temp.temperature);

        } else if (T_message == "/tel humidity") {
          sensors_event_t humidity, temp;
          aht20.getEvent(&humidity, &temp);
          Serial.println("SAT:CMD: /tel humidity");
          Serial.printf("Humidity is %.2f %%\n", humidity.relative_humidity);

        } else if (T_message == "/tel gyro") {
          float gx = lsm6ds3.readFloatGyroX();
          float gy = lsm6ds3.readFloatGyroY();
          float gz = lsm6ds3.readFloatGyroZ();
          Serial.println("SAT:CMD: /tel gyro");
          Serial.printf("Gyroscope [X: %.2f | Y: %.2f | Z: %.2f] deg/s\n", gx, gy, gz);

        } else if (T_message == "/tel acceleration") {
          float ax = lsm6ds3.readFloatAccelX();
          float ay = lsm6ds3.readFloatAccelY();
          float az = lsm6ds3.readFloatAccelZ();
          Serial.println("SAT:CMD: /tel acceleration");
          Serial.printf("Acceleration - [X: %.2f | Y: %.2f | Z: %.2f] m/ss\n", ax, ay, az);

        } else if (T_message == "/tel current") {
          float current = ina219.getCurrent_mA();
          Serial.println("SAT:CMD: /tel current");
          Serial.printf("Current: %.2f mA\n", current);

        } else if (T_message == "/tel voltage") {
          float voltage = ina219.getBusVoltage_V();
          Serial.println("SAT:CMD: /tel voltage");
          Serial.printf("Voltage: %.2f V\n", voltage);

        } else if (T_message == "/tel power") {
          float power = ina219.getPower_mW();
          Serial.println("SAT:CMD: /tel power");
          Serial.printf("Power: %.4f W\n", power / 1000.0);

        } else if (T_message == "/tel rssi" || T_message.startsWith("/rssi tel ")) {
          handleRSSICommand(T_message, true);

        } else {
          client.println(T_message);
          Serial.printf("SAT:LOC: %s\n", T_message);
        }

      } else {
        client.println(T_message);
        Serial.printf("SAT:LOC: %s\n", T_message);
      }
    }
  }
}

void handleRSSICommand(String cmd, bool fromSAT) {
  if (fromSAT) {
    Serial.println("RSSI can not be asked from SAT.");
    return;
  }

  if (cmd == "/tel rssi") {
    int rssi = WiFi.RSSI();
    client.printf("RSSI data is %4d dBm\n", rssi);
    Serial.printf("RSSI was asked from GND. Lecture: %4d dBm\n", rssi);

  } else if (cmd.startsWith("/rssi tel ")) {
    String arg = cmd.substring(11);
    arg.trim();
    bool isValid = arg.length() > 0 && arg.toInt() > 0;

    if (isValid) {
      int durationSec = arg.toInt();
      if (durationSec >= 1 && durationSec <= 10) {
        int numSamples = durationSec * 2;
        int rssiVector[numSamples];
        int timeVector[numSamples];
        unsigned long start = millis();

        client.printf("Beginning RSSI sampling for %d seconds (every 500 ms)...\n", durationSec);
        Serial.printf("GND requested RSSI telemetry for %d seconds.\n", durationSec);

        for (int i = 0; i < numSamples; i++) {
          rssiVector[i] = WiFi.RSSI();
          timeVector[i] = millis() - start;
          delay(500);
        }

        client.print("RSSI: [");
        for (int i = 0; i < numSamples; i++) {
          client.print(rssiVector[i]);
          if (i < numSamples - 1) client.print(", ");
        }
        client.println("] dBm");

        client.print("Time: [");
        for (int i = 0; i < numSamples; i++) {
          client.print(timeVector[i]);
          if (i < numSamples - 1) client.print(", ");
        }
        client.println("] ms");

        Serial.println("RSSI telemetry sent to GND.");
      } else {
        client.println("Duration must be between 1 and 10 seconds.");
      }
    } else {
      client.println("Invalid numeric format in /rssi tel command.");
    }
  }
}

void handleRGBCommand(String color, String valueStr) {
  int* intensity;
  int pin;
  const char* label;

  if (color == "R") {
    intensity = &redIntensity; pin = R; label = "Red";
  } else if (color == "G") {
    intensity = &greenIntensity; pin = G; label = "Green";
  } else if (color == "B") {
    intensity = &blueIntensity; pin = B; label = "Blue";
  } else {
    client.println("Invalid color channel.");
    return;
  }

  if (valueStr == "up") {
    *intensity = min(255, *intensity + LEDStep);
  } else if (valueStr == "down") {
    *intensity = max(0, *intensity - LEDStep);
  } else {
    bool isValid = true;
    for (int i = 0; i < valueStr.length(); i++) {
      if (!isDigit(valueStr[i])) {
        isValid = false;
        break;
      }
    }

    if (isValid) {
      int val = valueStr.toInt();
      if (val >= 0 && val <= 255) {
        *intensity = val;
      } else {
        client.println("Invalid intensity range. Must be 0–255.");
        return;
      }
    } else {
      client.println("Invalid intensity format. Must be numeric.");
      return;
    }
  }

  ledcWrite(pin, *intensity);
  client.printf("%s LED set to %d.\n", label, *intensity);
  Serial.printf("%s LED set to %d from GND.\n", label, *intensity);
}

void handleServoCommand(String valueStr) {
  if (valueStr == "up") {
    servoAngle = min(180, servoAngle + servoStep);
  } else if (valueStr == "down") {
    servoAngle = max(0, servoAngle - servoStep);
  } else {
    bool isValid = true;
    for (int i = 0; i < valueStr.length(); i++) {
      if (!isDigit(valueStr[i])) {
        isValid = false;
        break;
      }
    }

    if (isValid) {
      int angle = valueStr.toInt();
      if (angle >= 0 && angle <= 180) {
        servoAngle = angle;
      } else {
        client.println("Invalid angle. Must be 0–180.");
        return;
      }
    } else {
      client.println("Invalid format. Servo angle must be numeric.");
      return;
    }
  }

  myServo.write(servoAngle);
  client.printf("Servo angle set to %d deg.\n", servoAngle);
  Serial.printf("Servo angle set to %d deg from GND.\n", servoAngle);
}