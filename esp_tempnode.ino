/*
 * RevSpace SensorNode
 *
 * 1wire Temp sensor on an ESP8266 connected to GPIO 5 (don't forget pullup 4.7k!)
 *
 *
 */
#include <EEPROM.h>
#include <OneWire.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

#include <PubSubClient.h>

// WiFi settings
const char ssid[] = "revspace-pub-2.4ghz";  //  your network SSID (name)
const char pass[] = "";       // your network password
const char* mqtt_server = "mosquitto.space.revspace.nl";

void onMqttMessage(char* topic, byte* payload, unsigned int length);

WiFiClient espClient;
PubSubClient client(mqtt_server, 1883, onMqttMessage, espClient);

OneWire  ds(5);  // on pin 10 (a 4.7K resistor is necessary)

long lastMsg = 0;
char msg[50];
int value = 0;
long lastReconnectAttempt = 0;
float celsius, fahrenheit;
String adres1wire;

void setup() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();

  // We start by connecting to a WiFi network
  Serial.print("Connecting to ");

  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(50);
    Serial.print(".");
  }
  Serial.println("");

  Serial.print("WiFi connected to: ");
  Serial.println(ssid);

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();

}

void loop() {

  // put your main code here, to run repeatedly:
  if (client.connected()) {
    // Client connected
    client.loop();
  } else {
    Serial.println(".");
    long verstreken = millis();
    if (verstreken - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = verstreken;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }

  while(temperatuur());
  delay(10000);
}

boolean reconnect() {
  if (client.connect("sensor01")) {
    Serial.println("Reconnected to MQTT");
    // Once connected, publish an announcement...
    client.publish("revspace/sensorgrid", "hello world!");
    client.loop();
  }
  return client.connected();
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {

}


void mqtt_publish (String topic, String message) {
  Serial.println();
  Serial.print("Publishing ");
  Serial.print(message);
  Serial.print(" to ");
  Serial.println(topic);

  char t[100], m[100];
  topic.toCharArray(t, sizeof t);
  message.toCharArray(m, sizeof m);
  
  client.publish(t, m, /*retain*/ 1);
}

boolean temperatuur() {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  adres1wire = "";
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();

    return false;

  }

  Serial.print("ROM =");
  for ( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }
  for ( i = 0; i < 8; i++) {
    if (addr[i] < 16) {
      adres1wire += "0";
    }
    adres1wire += String(addr[i], HEX);
  }
  Serial.println();

  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return false;
  }
  Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return false;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.println("°C");

// Byte 2 is zone
// Byte 3 is id
  String zone    = String(data[2], DEC);
  String id      = String(data[3], DEC);
  String topic   = "revspace/sensors/temperature/" + zone + "/" + id;
  String message = String(celsius, 2) + " °C";
  mqtt_publish(topic, message);
   
  return true;
}

