#include <WiFi.h>
WiFiClient client; //crea un objeto tipo WifiClient para establecer conexión TCP.
WiFiServer server(80); //crea un objeto tipo WifiServer con el puerto 80.

const char* ssid = "STARMAN";
const char* password = "waitinginthesky";
const unsigned long statusPrintInterval = 20000; // 20 segundos
unsigned long lastStatusPrint = 0;
bool commandMode = false;

//******CÓDIGO PRINCIPAL******

void setup() {
  Serial.begin(115200);
  while (!Serial);

  displayHeader();

  // Inicializa el Access Point
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("Satellite AP server IP: ");
  Serial.println(myIP);

  // Inicia el servidor TCP
  server.begin();
  Serial.println("Satellite Server started. Waiting for Ground Station...");
}

void loop() {
  handleClientConnection();

  if (client.connected()) {
    receiveMessages();
    sendMessages();
  } else {
    //Si no hay conexión del satelite con un cliente estará buscándola.
    if ( (millis()-lastStatusPrint) > statusPrintInterval ) {
      Serial.println("Waiting for Ground Station to connect...");
      lastStatusPrint = millis();
    }
  }
}

//******FUNCIONES******

void displayHeader() {
  Serial.println();
  Serial.println("***********************************");
  Serial.println("*         Satellite Segment       *");
  Serial.println("*              v.1.0              *");
  Serial.println("*       Name: Satellite           *");
  Serial.println("***********************************");
  Serial.println(); 
}

void handleClientConnection() {
  //si no hay ningún cliente asignado, o si el cliente que teníamos ya se desconectó
  if (!client || !client.connected()) {
    //server: es un objeto WiFiServer, previamente inicializado con un puerto.
    //server.available() revisa si algún cliente ha intentado conectarse al servidor. 
    //Si sí, devuelve un objeto WiFiClient representando esa conexión.
    WiFiClient newClient = server.available();
    if (newClient) {
      client = newClient;
      Serial.println("Ground Station connected!");
    }
  }
}

void receiveMessages() {
  while (client.available()) {
    String R_message = client.readStringUntil('\n');
    R_message.trim();

    if (R_message.length() > 0) {

      if (R_message.startsWith("GND:ACK:")){
        Serial.println(R_message);
      }
      else{
      Serial.print("GND:MSG: ");
      Serial.println(R_message);
      }
      //ACK
      if (!R_message.startsWith("GND:ACK:") && !R_message.startsWith("SAT:ACK:")) {
      // Solo respondemos con ACK si NO es un ACK
      client.print("SAT:ACK:");
      client.println(R_message);

      
      }

      

      // Procesar comandos
      if ( (R_message == "CMD: START") && !commandMode) {
        commandMode = true;
        Serial.println("[!] Command Mode Activated");
      } else if ( (R_message == "CMD: STOP") && commandMode) {
        commandMode = false;
        Serial.println("[!] Command Mode Deactivated");
      } else if (commandMode && R_message == "TEL: RSSI") {
        int rssi = WiFi.RSSI();
        String reply = "RSSI: " + String(rssi) + " dBm";
        client.println(reply);
        Serial.println(reply);
      }
    }
  }
}

void sendMessages() {
  if (Serial.available()) {
    String T_message = Serial.readStringUntil('\n');
    T_message.trim();

    if (T_message.length() > 0) {
      if (T_message == "CMD: START") {
        commandMode = true;
        Serial.println("[!] Command Mode Activated");
      } else if (T_message == "CMD: STOP") {
        commandMode = false;
        Serial.println("[!] Command Mode Deactivated");
      }

      client.println(T_message); // Enviar al otro dispositivo
      Serial.print(commandMode ? "SAT:CMD: " : "SAT:LOC: ");
      Serial.println(T_message);
    }
  }
}
