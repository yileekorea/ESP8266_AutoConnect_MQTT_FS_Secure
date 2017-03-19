#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <PubSubClient.h>
#include <OneWire.h>

#pragma GCC diagnostic ignored "-Wwrite-strings"
 
#define DBG_SERIAL      Serial
 
OneWire  ds(2);  // on pin 2 (a 4.7K resistor is necessary)

//for LED status
#include <Ticker.h>
Ticker ticker;
#define ESP_LED 2
/////////////////////
// Pin Definitions //
/////////////////////
const int ANALOG_PIN = A0; // The only analog pin on the Thing
const int DIGITAL_PIN = 12; // Digital pin to be read


//define your default values here, if there are different values in config.json, they are overwritten.
//length should be max size + 1 
char mqtt_server[40]= "blynk-cloud.com";
char mqtt_port[6] = "8442";
char mqtt_user_id[16] = "yipine";
char mqtt_user_pwd[16] = "yipine";
char blynk_token[33] = "8fa7f712af4648f9b7f4add8e3e2b015";
//default custom static IP
char static_ip[16]; // = "192.168.30.200";
char static_gw[16]; // = "192.168.30.1";
char static_sn[16]; // = "255.255.255.0";

//flag for saving data
bool shouldSaveConfig = false;
/*
char * MQTT_SERVER = "iot2ym.iptime.org";
int MQTT_PORT = 8883;
char * MQTT_USER_ID = "yipine";
char * MQTT_USER_PWD = "yipine";
String clientName;
*/

char * MQTT_SERVER = "iot2better.iptime.org";
int MQTT_PORT = 8883;
char * MQTT_USER_ID = "yiorange";
char * MQTT_USER_PWD = "yiorange";
String clientName;

boolean flagWifi = false;
boolean flagMqtt = false;

unsigned long mqttTry = 0;
unsigned long tempTry = 0;

WiFiClientSecure wifiClient;
//WiFiClient wifiClient;
PubSubClient * mqttClient;
WiFiServer server(80);

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}
const char* fingerprint = "93:E6:74:63:96:C4:B2:B0:B2:BA:F3:7D:12:6D:51:C4:76:E5:D7:0E";

void verifyFingerprint();

/*
 * subscribe
 */
void mqttCallback(char* topic, byte* payload, unsigned int length)
{
    char buffer[80];
    int len = length >= 79 ? 79 : length;
    memcpy(buffer, payload, len);
    buffer[length] = 0;
   
    DBG_SERIAL.print(">> Topic: ");
    DBG_SERIAL.print(topic);
    DBG_SERIAL.print(">> Payload: ");
    DBG_SERIAL.println(buffer);
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient->connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID

    clientName += "esp8266-";
    uint8_t mac[6];
    WiFi.macAddress(mac);
    clientName += macToStr(mac);
    clientName += "-";
    clientName += String(micros() & 0xff, 16);
    
    // Attempt to connect
    //if (client.connect(clientId.c_str())) {
    if (mqttClient->connect(clientName.c_str(), MQTT_USER_ID, MQTT_USER_PWD)) {
        Serial.println("connected");
        // Once connected, publish an announcement...
        mqttClient->publish("TEMP", "hello world");
        // ... and resubscribe
        mqttClient->subscribe("TEMP");
    } else {
        Serial.print("failed, rc=");
        Serial.print(mqttClient->state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
    }
  }
}

String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}

void tick()
{
  //toggle state
  //int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  int state = digitalRead(ESP_LED);  // get the current state of GPIO1 pin
  //digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
  digitalWrite(ESP_LED, !state);     // set pin to the opposite state
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();

  flagWifi = false;
  flagMqtt = false;

  //set led pin as output
  //pinMode(BUILTIN_LED, OUTPUT);
  pinMode(ESP_LED, OUTPUT);
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);

  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");


  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          //strcpy(mqtt_user_id, json["mqtt_user_id"]);
          strcpy(blynk_token, json["blynk_token"]);

          if(json["ip"]) {
            Serial.println("setting custom ip from config");
            //static_ip = json["ip"];
            //static_gw = json["gateway"];
            //static_sn = json["subnet"];
            strcpy(static_ip, json["ip"]);
            strcpy(static_gw, json["gateway"]);
            strcpy(static_sn, json["subnet"]);
            //strcat(static_ip, json["ip"]);
            Serial.println(static_ip);
/*            Serial.println("converting ip");
            IPAddress ip = ipFromCharArray(static_ip);
            Serial.println(ip);*/
          } else {
            Serial.println("no custom ip in config");
          }
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
  Serial.println(static_ip);
  Serial.println(blynk_token);
  Serial.println(mqtt_server);

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);
  //WiFiManagerParameter custom_mqtt_user_id("user_id", "mqtt user id", mqtt_user_id, 16);
  WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 34);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset settings - for testing
  //Serial.println("reset settings----------");
  //wifiManager.resetSettings();

  //set config save notify callback
  Serial.println("saveConfigCallback----------");
  wifiManager.setSaveConfigCallback(saveConfigCallback);

/*
  if (static_ip) {
      //set static ip
      Serial.println("set Static IP address");
      IPAddress _ip,_gw,_sn;
      _ip.fromString(static_ip);
      _gw.fromString(static_gw);
      _sn.fromString(static_sn);
    
      wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);
  }
*/

  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  //wifiManager.addParameter(&custom_mqtt_user_id);
  wifiManager.addParameter(&custom_blynk_token);

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //--------------------------------------------------------------------------------------------------
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  //wifiManager.setAPCallback(configModeCallback);
  //--------------------------------------------------------------------------------------------------
 
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  Serial.println("autoConnect----------");
  //if (!wifiManager.autoConnect()) {
  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey");
  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  //strcpy(mqtt_user_id, custom_mqtt_user_id.getValue());
  strcpy(blynk_token, custom_blynk_token.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    //json["mqtt_user_id"] = mqtt_user_id;
    json["blynk_token"] = blynk_token;

    json["ip"] = WiFi.localIP().toString();
    json["gateway"] = WiFi.gatewayIP().toString();
    json["subnet"] = WiFi.subnetMask().toString();

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.prettyPrintTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  ticker.detach();
  //keep LED on
  //digitalWrite(BUILTIN_LED, LOW);
  digitalWrite(ESP_LED, LOW);
  flagWifi = true;

  // Generate client name based on MAC address and last 8 bits of microsecond counter
  // String clientName;
  clientName += "esp8266-";
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName += macToStr(mac);
  clientName += "-";
  clientName += String(micros() & 0xff, 16);
  Serial.print("Connecting to ");
  Serial.print(MQTT_SERVER);
  Serial.print(" as ");
  Serial.println(clientName);
  delay(3000); 

  server.begin();

  // check the fingerprint of io.adafruit.com's SSL cert
  verifyFingerprint();

}

void loop() {
  // put your main code here, to run repeatedly:
  if ( flagWifi == true )
  {
    if ( mqttClient == NULL )
        mqttClient = new PubSubClient(MQTT_SERVER, MQTT_PORT, mqttCallback, wifiClient);
       
    if ( flagMqtt == false )
    {
        if ( mqttTry == 0 || millis() - mqttTry >= 20000UL ) // 20sec
        {
            //if ( mqttClient->connect(CLIENT_ID) )
            if ( mqttClient->connect(clientName.c_str(), MQTT_USER_ID, MQTT_USER_PWD) )
            {
                flagMqtt = true;
                DBG_SERIAL.println("Connected to MQTT broker");
               
                // Publish
                if (mqttClient->publish("/TEST", "Hi~~~")) {
                    DBG_SERIAL.println("Publish ok");
                }
                else {
                    DBG_SERIAL.println("Publish failed");
                }
               
                if (mqttClient->subscribe("/DEVICE")) {
                    DBG_SERIAL.println("Subscribe ok");
                }
                else {
                    DBG_SERIAL.println("Subscribe failed");
                }
            }
            else {
                DBG_SERIAL.println("MQTT connect failed");
            }
        }
    }
    else {
      if (!mqttClient->connected()) {
          reconnect();
      }
      mqttClient->loop();
    }
  }

  //if ( flagMqtt == true && (tempTry == 0 || ((millis() - tempTry) > 2000UL)) )  // 2sec
  if ( flagMqtt == true && (tempTry == 0 || ((millis() - tempTry) > 1000UL)) )  // 1sec
  {
    String pl = readFromOneWire();
    Serial.print(pl);
    Serial.println();

    if ( mqttClient->publish("TEMP", (char *)pl.c_str()) )
        DBG_SERIAL.println("Publish Temp~ OK------------------------------>");
    else
        DBG_SERIAL.println("Publish failed.................................");
   
    tempTry = millis();
  }

  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }    

  // Read the first line of the request
  String req = client.readStringUntil('\r');
  Serial.println(req);
  client.flush();

  // Match the request
  int val = -1; // We'll use 'val' to keep track of both the
                // request type (read/set) and value if set.
  if (req.indexOf("/led/0") != -1)
    val = 0; // Will write LED low
  else if (req.indexOf("/led/1") != -1)
    val = 1; // Will write LED high
  else if (req.indexOf("/read") != -1)
  {
    val = -2; // Will print pin reads
  }
  // Otherwise request will be invalid. We'll say as much in HTML

  // Set GPIO5 according to the request
  if (val >= 0)
    digitalWrite(ESP_LED, val);

  client.flush();

  // Prepare the response. Start with the common header:
  String s = "HTTP/1.1 200 OK\r\n";
  s += "Content-Type: text/html\r\n\r\n";
  s += "<!DOCTYPE HTML>\r\n<html>\r\n";
  // If we're setting the LED, print out a message saying we did
  if (val >= 0)
  {
    s += "LED is now ";
    s += (val)?"on":"off";
  }
  else if (val == -2)
  { // If we're reading pins, print out those values:
    s += "Analog Pin = ";
    s += String(analogRead(ANALOG_PIN));
    s += "<br>"; // Go to the next line.
    //s += "Digital Pin 12 = ";
    //s += String(digitalRead(DIGITAL_PIN));
    s += "<br>"; // Go to the next line.
    s += "<br>"; // Go to the next line.
    s += "WILL RESET ESP system soon!!!!";

    s += "</html>\n";
  
    // Send the response to the client
    client.print(s);
    delay(1);
    Serial.println("Client disonnected");
    Serial.println("WILL reset ESP system soon!!!!");

    delay(9000);
    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;
    //reset settings - for testing
    Serial.println("reset settings----------");
    wifiManager.resetSettings();
    delay(9000);
    ESP.reset();
  }
  else
  {
    s += "Invalid Request.<br> Try /led/1, /led/0, or /read.";
  }
  s += "</html>\n";

  // Send the response to the client
  client.print(s);
  delay(1);
  Serial.println("Client disonnected");

  // The client will actually be disconnected 
  // when the function returns and 'client' object is detroyed
}


/*
 * Temperature measurement
 */
String readFromOneWire()
{
    String payload = "{\"temp\":";

    //==========================================================
    byte numSensor = 0;
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8];
    //byte id[10];
    float celsius[10];
    float fahrenheit[10];
    
    //if ( !ds.search(addr)) {
    //Serial.println("No more addresses.");
    //Serial.println();
    //ds.reset_search();
    //delay(250);
    //return payload;
    //}
    
    while (ds.search(addr)) {
    //    measure ();
    Serial.print("ROM =");
    for ( i = 0; i < 8; i++) {
        Serial.write(' ');
        Serial.print(addr[i], HEX);
    }
    
    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return payload;
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
          return payload;
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
    
    //id[numSensor] = numSensor+1;
    celsius[numSensor] = (float)raw / 16.0;
    fahrenheit[numSensor] = celsius[numSensor] * 1.8 + 32.0;
    //Serial.print("  ID = ");
    //Serial.print(id[numSensor]);
    Serial.print("  Temperature = ");
    Serial.print(celsius[numSensor]);
    Serial.print(" Celsius, ");
    Serial.print(fahrenheit[numSensor]);
//    Serial.println(" Fahrenheit");
    Serial.println(" Celsius");
    numSensor += 1;
  }
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    
    //==========================================================
    for ( i = 0; i < numSensor ; i++) {
        float f = celsius[i]; //first ~ numSensor one-wire temperature celsius
        
        // celsius based first sensor
        if ( isnan(f) )
            payload += "0";
        else
            payload += f;   // *C
            
        if(i == (numSensor-1))
            payload += "}";
        else
            payload += ",\"temp\":";
    }       
/*
    float f = celsius[0]; //first one-wire temperature celsius
    float t = celsius[1]; //2nd one-wire temperature celsius
   
    // celsius based first sensor
    if ( isnan(t) )
        payload += "0";
    else
        payload += t;   // *C
 
    payload += ",\"humidity\":";
   
    // celsius based second sensor
    if ( isnan(f) )
        payload += "0";
    else
        payload += f;   // %
       
    payload += "}";
*/
    return payload;
}

void verifyFingerprint() {

  //const char* host = MQTT_SERVER;

  Serial.print("Connecting to ");
  Serial.println(MQTT_SERVER);

  if (! wifiClient.connect(MQTT_SERVER, MQTT_PORT)) {
    Serial.println("Connection failed. Halting execution.");
    while(1);
  }

  if (wifiClient.verify(fingerprint, MQTT_SERVER)) {
    Serial.println("Connection secure.");
  } else {
    Serial.println("Connection insecure! Halting execution.");
    while(1);
  }

}
