/*
Javier Cruceño
  Related:
    Complehttps://RandomNerdTutorials.com/esp8266-nodemcu-mqtt-publish-ds18b20-arduino/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>

#include "FS.h" // SPIFFS is declared
//#include "LittleFS.h" // LittleFS is declared
#include <Ticker.h>
#include <AsyncMqttClient.h>

#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <ArduinoJson.h>  
#include <ESPAsyncWiFiManager.h>  

// TODO: Modificar segun dispositivo agregando los topics que sean necesarios
char  mqtt_server[50]; 
char  mqtt_port[5];
char  control_topic[20];

char  log_topic[21] ;
//TODO: Aadd sensor topocs here
char  temperature_topic[16];

// ------------------------
unsigned int interval = 60;


/* OneWire DS18B20 Configuration */
// NOTE: Especiico del dispositivo, otros dispositivos pueden no necesitar esta libreria
  // GPIO where the DS18B20 is connected to
  const int oneWireBus = 14;          
  // Setup a oneWire instance to communicate with any OneWire devices
  OneWire oneWire(oneWireBus);
  // Pass our oneWire reference to Dallas Temperature sensor 
  DallasTemperature sensors(&oneWire);
  // Temperature value
  float temp;

/*ESP AsyncWebSever config*/
  AsyncWebServer server(80);
  DNSServer dns;

/*  ESP Async Wifi Manager Config */
  AsyncWiFiManager wifiManager(&server,&dns);
  //flag for saving data
  bool shouldSaveConfig = false;

  //callback notifying us of the need to save config
  void saveConfigCallback () {
    Serial.println("Should save config");
    shouldSaveConfig = true;
  }

  void configModeCallback (AsyncWiFiManager *myWiFiManager) {
    Serial.println("Entered config mode");
    Serial.println(WiFi.softAPIP());
    //if you used auto generated SSID, print it
    Serial.println(myWiFiManager->getConfigPortalSSID());
  }
/* FileSystem config  */
  void loadConfigFromFS(){
    // TODO: Modificar segun dispositivo agregando los topics que sean necesarios

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


          const size_t capacity = JSON_OBJECT_SIZE(5) + 196 ;
          DynamicJsonDocument doc(capacity);
          DeserializationError error = deserializeJson(doc, configFile);
            // Test if parsing succeeds.
          if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println("failed to load json config");
            Serial.println(error.f_str());
          }
          else{
            serializeJsonPretty(doc, Serial);
            Serial.println();
            Serial.println("----------------------------------");
            strcpy(mqtt_server, doc["mqtt_server"]);
            Serial.println(mqtt_server);
            strcpy(mqtt_port, doc["mqtt_port"]);
            Serial.println(mqtt_port);
            strcpy(control_topic, doc["device_topic"]);
            Serial.println(control_topic);
            // Aadd sensor topics here
            strcpy(temperature_topic, doc["temperature_topic"]);
            Serial.println(temperature_topic);


            //--------------------------
            interval = doc["delay"];

            Serial.println(interval);

            Serial.println("----------------------------------");
          } 

        }
        configFile.close();
      }
    }
    else {
      Serial.println("failed to mount FS");
    }
  }

  void saveConfigToFS(){
    // TODO: Modificar segun dispositivo agregando los topics que sean necesarios

    if (SPIFFS.begin()){
      Serial.println("saving config");
      const size_t capacity =  JSON_OBJECT_SIZE(5) + 196 ;
      DynamicJsonDocument doc(capacity);

      doc["mqtt_server"] = mqtt_server;
      doc["mqtt_port"] = mqtt_port;
      doc["device_topic"] = control_topic;
      //TODO: Aadd sensor topics here
      doc["temperature_topic"] = temperature_topic;

      //------------------------
      doc["delay"] = interval;

      File configFile = SPIFFS.open("/config.json", "w");
      if (!configFile) {
        Serial.println("failed to open config file for writing");
      }
      serializeJsonPretty(doc, Serial);
      serializeJsonPretty(doc, configFile);
      configFile.close();
    }
  }


AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;
/*  MQTT Payload config */
  #define MSG_BUFFER_SIZE	(10)
  #define LOG_BUFFER_SIZE (250)
  char msg[MSG_BUFFER_SIZE];
  char log_msg[LOG_BUFFER_SIZE];
  unsigned long lastMsg = 0;
  unsigned long live_since =0;
 /*
 TODO: General function to submit system logs
void publish_log(char *payload){
    snprintf(log_msg, LOG_BUFFER_SIZE, "{\"log\": \"%s\"}", payload);
    mqttClient.publish(log_topic, 0, false, log_msg, LOG_BUFFER_SIZE);
    strcpy(log_msg, "");
}
*/
void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  // TODO: WIfi AP name and password from config file
  if (!wifiManager.autoConnect("ESP866-DS18B20", "12345678", 3)) {
    Serial.println("Fallo al conectar al Wifi y timeout alcanzado");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
  }
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  Serial.println(mqtt_server);
  Serial.println(mqtt_port);
  Serial.println(control_topic);
  //TODO: Aadd sensor topics here
  Serial.println(temperature_topic);
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  delay(1000);
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}


void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  mqttClient.subscribe(control_topic, 0);

 
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  
  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("Payload:");
  Serial.println(payload);

  const size_t capacity = JSON_OBJECT_SIZE(5) + 196 ;
  DynamicJsonDocument doc(capacity);
  DeserializationError error = deserializeJson(doc, payload);

  if (error){  
     snprintf(log_msg, LOG_BUFFER_SIZE, "{\"log\": \" %s - BAD JSON  \"}", payload);
     mqttClient.publish(log_topic, 0, false, log_msg, LOG_BUFFER_SIZE);

  }
  else{
    // Compando para settear el delay solo si es mayor a 5 segundos
    int delay_value = doc["delay"];
    if (delay_value && delay_value > 5) {
      Serial.println("seteando nuevo delay");
      interval = delay_value;
      saveConfigToFS();
    }
    else if(delay_value){
      // Si es menor a 5 segundos no se actualiza y se manda un log de valo invalido
      snprintf(log_msg, LOG_BUFFER_SIZE, "{\"log\": \" %s - INVALID VALUE \"}", payload);
      mqttClient.publish(log_topic, 0, false, log_msg, LOG_BUFFER_SIZE);


    }
    else{
      snprintf(log_msg, LOG_BUFFER_SIZE, "{\"log\": \" %s - INVALID CMD  \"}", payload);
      mqttClient.publish(log_topic, 0, false, log_msg, LOG_BUFFER_SIZE);

    }
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}



/* Setup */
void setup() {
  sensors.begin();
  Serial.begin(115200);

  loadConfigFromFS();
  //delay(1000);

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length

  AsyncWiFiManagerParameter config_mqtt_server("server", "MQTT server", mqtt_server, 40);
  AsyncWiFiManagerParameter config_mqtt_port("port", "MQTT port", mqtt_port, 5);
  AsyncWiFiManagerParameter config_control_topic("control_topic", "Topic de Control", control_topic, 20);
  AsyncWiFiManagerParameter config_temperature_topic("temperature_topic", "Topic Temperatura", temperature_topic, 16);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around

  //Serial.println(mqtt_server);
  //set config save notify callback

  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setAPCallback(configModeCallback);
    //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  wifiManager.setDebugOutput(false);
  //add all your parameters here
  wifiManager.addParameter(&config_mqtt_server);
  wifiManager.addParameter(&config_mqtt_port);
  wifiManager.addParameter(&config_control_topic);

  // TODO: Add sensor topics here
  wifiManager.addParameter(&config_temperature_topic);


  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds

  wifiManager.setTimeout(120);
 // Serial.println(mqtt_server);
    Serial.println("Connecting to Wi-Fi...");

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  //Serial.println(mqtt_server);
  mqttClient.setServer( mqtt_server, atoi(mqtt_port));

  connectToWifi();

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //save the custom parameters to FS
  if (shouldSaveConfig) {
      //read updated parameters
    strcpy(mqtt_server, config_mqtt_server.getValue());
            Serial.println(mqtt_server);
    strcpy(mqtt_port, config_mqtt_port.getValue());
            Serial.println(mqtt_port);
    strcpy(control_topic, config_control_topic.getValue());
            Serial.println(control_topic);

  // TODO: Add sensor topics here
    strcpy(temperature_topic, config_temperature_topic.getValue());
            Serial.println(temperature_topic);

    
    saveConfigToFS();    //end save
    shouldSaveConfig=false;
    ESP.restart();
  }

  strcpy(log_topic, control_topic);
  strcat(log_topic, "/log");

  Serial.print("\nlocal ip: ");
  Serial.println(WiFi.localIP());
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    //TODO: index page for control and info
    request->send(200, "text/plain", "Hi! I am ESP8266.");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();

  //publish_log("Equipo iniciado con exito !");

}

void loop() {

    AsyncElegantOTA.loop();
    unsigned long now = millis();
    if (now - lastMsg > interval * 1000) {

      lastMsg = now;
    
      /* Toma de lecturas y envio de datos*/
      sensors.requestTemperatures(); 
      // Temperature in Celsius degrees
      temp = sensors.getTempCByIndex(0);

      snprintf (msg, 7, "%.2f", temp);
      Serial.print("Publish message: ");
      Serial.println(msg);
      mqttClient.publish(temperature_topic, 0, false, msg, 7);

      //client.publish(topic, msg);

    }
    if (now - live_since > interval * 1000 * 10) {
      ESP.restart();
    }
  
}