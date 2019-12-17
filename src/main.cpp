#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

Adafruit_BME280 bme; // I2C

unsigned long delayTime;
 
const char* ssid = "YourWiFi";
const char* password =  "YourPW";

DynamicJsonDocument doc(512);

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "IPADRESS_FRom_MqTt";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

float temperature = 0;
float humidity = 0;
float pressure = 0;
int counter = 0;

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */

void connectToNetwork() {

  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Establishing connection to WiFi..");
    if(counter >= 20) {
      ESP.restart();
    }
    counter++;
  }
  Serial.println("Connected to network");
}

void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}

void callback(char* topic, byte* message, unsigned int length) {
  //Serial.print("Message arrived on topic: ");
  //Serial.print(topic);
  //Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  /*if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  } */
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("sleeping-room");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void setup() {
  Serial.begin(115200);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  delayTime = 10000;

  printValues();

  connectToNetwork();
 
  Serial.println(WiFi.macAddress());
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}


void loop() { 
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
    
    // Temperature in Celsius
    temperature = bme.readTemperature();   
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;

    doc["temperature"] =  temperature;
    doc["humidity"] =  humidity;
    doc["pressure"] = pressure;


    char buffer[512];
    serializeJson(doc, buffer);
 
    client.publish("sleeping-room", buffer);

    delay(1000);
    Serial.flush(); 

    WiFi.disconnect(true); // disconnects STA Mode
    /* while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Establishing connection to WiFi..");
    } */

    delay(1000);
    WiFi.softAPdisconnect(true); // disconnects AP Mode 
    delay(1000); 
    Serial.println("Going to sleep now");

    esp_deep_sleep_start();
  }
}
