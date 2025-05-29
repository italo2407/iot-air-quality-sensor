#include "DHT.h"
#include <MHZ19.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>

// Pines de sensores
#define DHTPIN 4
#define MQ135_PIN 32
#define LED_PIN 5         // LED fijo (indicador)
#define RELAY_PIN 19        // Pin del relé para el ventilador

// Umbrales
// #define TEMP_UMBRAL 28    // Temperatura en °C
// #define MQ135_UMBRAL 300    // Valor crudo MQ135

// definimos macro para indicar función y línea de código en los mensajes
#define DEBUG_STRING "["+String(__FUNCTION__)+"():"+String(__LINE__)+"]   "

// Tipo de sensor DHT
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

WiFiClient wClient;
PubSubClient mqtt_client(wClient);

// WiFi credentials
const String ssid = "DIGIFIBRA-Akuf";
const String password = "fTKuDzCfTTFU";

// MQTT credentials and server
const String mqtt_server = "35.195.41.101";
//const String mqtt_user = "IOT9";
//const String mqtt_pass = "uFEVF1rL";

String PLACA_ID = "ESP32-IOT-0001";
String DEVICE_ID = "SENSOR-0001";

// MQTT Pub topics
String topic_PUB_conexion = "sensor/conexion/status";
String topic_PUB_datos = "sensor/data";
String topic_PUB_actuator_status = "sensor/actuator/status";
// MQTT Sub topics
String topic_SUB_fan = "sensor/"+ DEVICE_ID + "/actuator/fan/cmd";
String topic_SUB_light = "sensor/"+ DEVICE_ID + "/actuator/light/cmd";
String topic_SUB_config = "sensor/" + DEVICE_ID + "/config";

// Función para interpretar calidad del aire (MQ135)
String interpretarCalidad(int valor) {
  if (valor <= 200) return "Excelente";
  else if (valor <= 533) return "Aceptable";
  else if (valor <= 800) return "Mala";
  else return "Muy mala";
}

void conectaWifi() {
  Serial.println(DEBUG_STRING+"Connecting to " + ssid);
 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();
  Serial.println(DEBUG_STRING+"WiFi connected, IP address: " + WiFi.localIP().toString());
}

void conectaMQTT() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Attempt to connect and set the Last Will Message (LWM) in the topic sensor/conexion/status
    // The LWM will be sent if the device disconnects unexpectedly
    String mensaje = getConexionStatus("OFF");
    
    if (mqtt_client.connect(DEVICE_ID.c_str(), NULL, NULL, topic_PUB_conexion.c_str(), 0, true, mensaje.c_str())){
      Serial.println(DEVICE_ID + " conectado a MQTT Broker!");

      // Publish a connection message
      publishConexion();

      // Topic subscriptions
      mqtt_client.subscribe(topic_SUB_fan.c_str());
      mqtt_client.subscribe(topic_SUB_light.c_str());
      mqtt_client.subscribe(topic_SUB_config.c_str());
    } else {
      // Wait 5 seconds before retrying
      Serial.print("ERROR: ");
      Serial.println(mqtt_client.state());
      Serial.println("Retrying in 5s...");
      delay(5000);
    }
  }
}

String getConexionStatus(String status) {
  JsonDocument doc;
  String mensaje;
  doc["device_id"] = DEVICE_ID;
  doc["placa_id"] = PLACA_ID;
  doc["status"] = status;

  // Objeto anidado para WiFi
  JsonObject network_info = doc.createNestedObject("network_info");
  network_info["ip"] = WiFi.localIP().toString();
  network_info["mac"] = WiFi.macAddress();
  network_info["rssi"] = WiFi.RSSI();
  network_info["ssid"] = WiFi.SSID();
  serializeJson(doc, mensaje);

  return mensaje;
}

void publishConexion() {
  String mensaje = getConexionStatus("ON");

  Serial.println();
  Serial.println(DEBUG_STRING+"Topic   : "+ topic_PUB_conexion);
  Serial.println(DEBUG_STRING+"Payload : "+ mensaje);
  mqtt_client.publish(topic_PUB_conexion.c_str(), mensaje.c_str());
}

void procesaMensaje(char* topic, byte* payload, unsigned int length) {
  // Save the message received
  String mensaje=""; // mejor String que el buffer de bytes
  for(int i=0; i<length; i++) mensaje+= (char)payload[i];
  Serial.println();
  Serial.println(DEBUG_STRING+"Mensaje recibido ["+ String(topic) +"] \"" + mensaje + "\" "+String(length)+" bytes" );
 
  JsonDocument jsonDocument;
  DeserializationError error = deserializeJson(jsonDocument, mensaje);
  
  if (error) {
    Serial.print(("JSON deserialization failed: "));
    Serial.println(error.f_str());
    return;
  }
  
  String dataStr = jsonDocument["data"];
  JsonDocument dataDoc;
  error = deserializeJson(dataDoc, dataStr);
  if (error) {
    Serial.print("JSON deserialization (data) failed: ");
    Serial.println(error.f_str());
    return;
  }


  if(String(topic) == topic_SUB_fan) {
    String status = dataDoc["command"];
    Serial.println(status);
    bool encendidoHigh = (status=="ON");

    if(encendidoHigh) {
      Serial.println(DEBUG_STRING+"Encendiendo ventilador");
      digitalWrite(RELAY_PIN, LOW);  // Enciende el ventilador
    } else {
      Serial.println(DEBUG_STRING+"Apagando ventilador");
      digitalWrite(RELAY_PIN, HIGH); // Apaga el ventilador
    }

    // Publish the confirmation that the ACTUATOR status is changed
    publishActuatorStatus(encendidoHigh, "fan"); 
  }

  if(String(topic) == topic_SUB_light) {
    String status = dataDoc["command"];
    Serial.println(status);
    bool encendidoHigh = (status=="ON");

    if(encendidoHigh) {
      Serial.println(DEBUG_STRING+"Encendiendo Led");
      digitalWrite(LED_PIN, HIGH); // Enciende el led
    } else {
      Serial.println(DEBUG_STRING+"Apagando Led");
      digitalWrite(LED_PIN, LOW); // Apaga el led
    }

    // Publish the confirmation that the ACTUATOR status is changed
    publishActuatorStatus(encendidoHigh, "light"); 
  }

  if(String(topic) == topic_SUB_config) {
    Serial.println(DEBUG_STRING+"Configurando parámetros del sensor...");
    unsigned long data_sending_interval = dataDoc["data_sending_interval"];
    unsigned long check_threshold_interval = dataDoc["check_threshold_interval"];
    int temp_threshold_max = dataDoc["temp_threshold_max"];
    int ppm_threshold_max = dataDoc["ppm_threshold_max"];
    
    TEMP_UMBRAL = temp_threshold_max;
    MQ135_UMBRAL = ppm_threshold_max;
    intervaloChequeo = check_threshold_interval; // Convertir a milisegundos
    intervaloLoop = data_sending_interval; // Convertir a milisegundos
  }
}

void publishActuatorStatus(bool status, String type) {  
  JsonDocument doc;
  String mensaje;
  
  time_t now;
  time(&now);

  doc["device_id"] = DEVICE_ID;
  doc["actuator_type"] = type;
  doc["status"] = status;
  doc["timestamp"] = now;
  serializeJson(doc, mensaje);

  Serial.println();
  Serial.println(DEBUG_STRING+"Topic   : "+ topic_PUB_actuator_status);
  Serial.println(DEBUG_STRING+"Payload : "+ mensaje);
  mqtt_client.publish(topic_PUB_actuator_status.c_str(), mensaje.c_str());
  
}

void setup() {
  Serial.begin(115200);  // Monitor serie
  Serial.println("Empieza setup...");
  
  configTime(0, 0, "pool.ntp.org");

  dht.begin();
  pinMode(MQ135_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  // Asegurar que están apagados al iniciar
  digitalWrite(LED_PIN, LOW);
  digitalWrite(RELAY_PIN, HIGH);

  conectaWifi();

  mqtt_client.setServer(mqtt_server.c_str(), 1883);
  mqtt_client.setBufferSize(512);
  mqtt_client.setCallback(procesaMensaje);
  
  conectaMQTT();

  Serial.println("Identificador placa: " + PLACA_ID);
  Serial.println("Topic publicacion: " + topic_PUB_conexion);
  Serial.println("Topic publicacion: " + topic_PUB_datos );
  Serial.println("Topic publicacion: " + topic_PUB_actuator_status );
  Serial.println("Topic subscripcion : " + topic_SUB_fan);
  Serial.println("Topic subscripcion : " + topic_SUB_light);
  
  Serial.println("Sistema de monitoreo ambiental iniciado");
}

int TEMP_UMBRAL = 28;    // Temperatura en °C
int MQ135_UMBRAL = 300;  // Valor crudo MQ135

unsigned long ultimoChequeo = 0;
unsigned long intervaloChequeo = 120000; // 2 minutos en milisegundos
unsigned long intervaloLoop = 2000; // Intervalo de loop en milisegundos

void loop() {
  mqtt_client.loop();

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Error al leer del sensor DHT11");
    return;
  }

  int gasRaw = analogRead(MQ135_PIN);
  int ppm = map(gasRaw, 0, 3000, 0, 1000);
  float gasPercent = (gasRaw / 4095.0) * 100.0;
  String calidadTexto = interpretarCalidad(ppm);

  // Mostrar datos en el monitor serie
  Serial.print("Humedad: ");
  Serial.print(h);
  Serial.print(" %\tTemperatura: ");
  Serial.print(t);
  Serial.print(" °C\t");

  Serial.print("MQ135: ");
  Serial.print(gasRaw);
  Serial.print(" | ");
  Serial.print("ppm: ");
  Serial.print(ppm);
  Serial.print(" | ");
  Serial.print(gasPercent, 1);
  Serial.print(" % | Estado: ");
  Serial.print(calidadTexto);
  Serial.print("\t");

  // Build the json message for sending the data
  JsonDocument docDatos;
  time_t now;
  time(&now);

  docDatos["device_id"] = DEVICE_ID;
  docDatos["temperature"] = t;
  docDatos["humidity"] = h;
  docDatos["ppm"] = ppm;
  docDatos["timestamp"] = now;
  
  String mensajeDatos;
  serializeJson(docDatos, mensajeDatos);

  // Publish data and connection status
  Serial.println();
  Serial.println(DEBUG_STRING+"Topic   : "+ topic_PUB_datos);
  Serial.println(DEBUG_STRING+"Payload : "+ mensajeDatos);
  mqtt_client.publish(topic_PUB_datos.c_str(), mensajeDatos.c_str());

  // Activar relé y LED si se supera algún umbral
  // Validar umbrales cada 2 minutos
  if (millis() - ultimoChequeo >= intervaloChequeo) {
    ultimoChequeo = millis();
    Serial.println(DEBUG_STRING+"Chequeando condiciones ambientales...");
    if (t > TEMP_UMBRAL || gasRaw > MQ135_UMBRAL) {
      digitalWrite(RELAY_PIN, LOW);   // Enciende ventilador
      digitalWrite(LED_PIN, HIGH);     // Enciende LED
      Serial.println("Ventilador activado");
      publishActuatorStatus(true, "fan"); // Publica estado del ventilador
      publishActuatorStatus(true, "light"); // Publica estado del LED
    } else {
      digitalWrite(RELAY_PIN, HIGH);    // Apaga ventilador
      digitalWrite(LED_PIN, LOW);      // Apaga LED
      Serial.println("Condiciones normales");
      publishActuatorStatus(false, "fan"); // Publica estado del ventilador
      publishActuatorStatus(false, "light"); // Publica estado del LED
    } 
  }

  delay(intervaloLoop);
}