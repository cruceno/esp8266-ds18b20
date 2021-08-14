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

// Ip settings 
char use_static[2] = "0";
char static_ip[16] = "";
char static_gw[16] = "";
char static_sn[16] = "";
char static_dns1[16] = "";
char static_dns2[16] = "";



// ------------------------
unsigned int interval = 60;
unsigned long live_since =0, check_wifi = 60;
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
   // Serial.println("Should save config");
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

          const size_t capacity = JSON_OBJECT_SIZE(10) + 260;
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

            //--------------------------
            interval = doc["delay"];
            Serial.println(interval);
            Serial.println(temperature_topic);
            strcpy(static_ip, doc["static_ip"]);
            Serial.println(static_ip);
            strcpy(static_gw, doc["static_gw"]);
            Serial.println(static_gw);
            strcpy(static_sn, doc["static_sn"]);
            Serial.println(static_sn);
            strcpy(static_dns1, doc["static_dns1"]);
            Serial.println(static_dns1);
            strcpy(static_dns2, doc["static_dns2"]);
            Serial.println(static_dns2);

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
      //Serial.println("saving config");
      const size_t capacity = JSON_OBJECT_SIZE(10) + 260;
      DynamicJsonDocument doc(capacity);

      doc["mqtt_server"] = mqtt_server;
      doc["mqtt_port"] = mqtt_port;
      doc["device_topic"] = control_topic;
      //TODO: Aadd sensor topics here
      doc["temperature_topic"] = temperature_topic;

      //------------------------
      doc["delay"] = interval;
      doc["static_ip"] = static_ip;
      doc["static_gw"] = static_gw;
      doc["static_sn"] = static_sn;
      doc["static_dns1"] = static_dns1;
      doc["static_dns2"] = static_dns2;

      File configFile = SPIFFS.open("/config.json", "w");
      if (!configFile) {
        Serial.println("failed to open config file for writing");
      }
      serializeJsonPretty(doc, Serial);
      serializeJsonPretty(doc, configFile);
      configFile.close();
      Serial.println("\n--------------------------------");
      Serial.println("|Update configuration complete.|");
      Serial.println("--------------------------------");
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

 /*
 TODO: General function to submit system logs
void publish_log(char *payload){
    snprintf(log_msg, LOG_BUFFER_SIZE, "{\"log\": \"%s\"}", payload);
    mqttClient.publish(log_topic, 0, false, log_msg, LOG_BUFFER_SIZE);
    strcpy(log_msg, "");
}
*/
void connectToWifi() {
  Serial.println("Connecting to Wi-Fi function ... ");
  // TODO: WIfi AP name and password from config file
  if (!wifiManager.autoConnect("ESP8266-DS18B20", "123456789")) {
    Serial.println("Fallo al conectar al Wifi y timeout alcanzado");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
  }
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
 /* Serial.println(mqtt_server);
  Serial.println(mqtt_port);
  Serial.println(control_topic);
  //TODO: Aadd sensor topics here
  Serial.println(temperature_topic);*/
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

void read_and_publish_sensors(){
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

/* Setup */
void setup() {
  // Iniciando debug serie
  Serial.begin(115200);
  delay(2000);

  // Iniciando termocupla
  sensors.begin();
  
  
  // Cargando archivo de configuracion
  loadConfigFromFS();
  delay(1000);

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length

  AsyncWiFiManagerParameter config_mqtt_server("server", "MQTT server", mqtt_server, 40);
  AsyncWiFiManagerParameter config_mqtt_port("port", "MQTT port", mqtt_port, 5);
  AsyncWiFiManagerParameter config_control_topic("control_topic", "Topic de Control", control_topic, 20);
  //TODO: Add sensor topics here
  AsyncWiFiManagerParameter config_temperature_topic("temperature_topic", "Topic Temperatura", temperature_topic, 16);
  AsyncWiFiManagerParameter config_use_static("use_static", "Usar Ip estatica", use_static, 2);

  AsyncWiFiManagerParameter config_static_ip("ip", "IP del dispositivo", static_ip, 16);
  AsyncWiFiManagerParameter config_static_gw("gw", "Puesta de enlace", static_gw, 16);
  AsyncWiFiManagerParameter config_static_sn("sn", "Mascara de red", static_sn, 16);
  AsyncWiFiManagerParameter config_static_dns1("dns1", "Servidor DNS 1", static_dns1, 16);
  AsyncWiFiManagerParameter config_static_dns2("dns2", "servidor DNS 2", static_dns2, 16);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setDebugOutput(false);
  //add all your parameters here
  wifiManager.addParameter(&config_mqtt_server);
  wifiManager.addParameter(&config_mqtt_port);
  wifiManager.addParameter(&config_control_topic);
  // TODO: Add sensor topics here
  wifiManager.addParameter(&config_temperature_topic);

  wifiManager.addParameter(&config_static_ip);
  wifiManager.addParameter(&config_static_gw);
  wifiManager.addParameter(&config_static_sn);
  wifiManager.addParameter(&config_static_dns1);
  wifiManager.addParameter(&config_static_dns2);

  //set static ip or dhcp
   if (!*static_ip) {

      Serial.println("Usar DHCP");
    }

    else {
       //set static ip
      //the commented bit only works for ESP8266 core 2.1.0 or newer
      IPAddress _ip,_gw,_sn, _dns1, _dns2;
      _ip.fromString(static_ip);
      _gw.fromString(static_gw);
      _sn.fromString(static_sn);
      _dns1.fromString(static_dns1);
      _dns2.fromString(static_dns2);
      wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn, _dns1, _dns2);
      Serial.println("Usar IP Estatica");
      
    }
  //

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds

  wifiManager.setTimeout(120);

  Serial.println("Set Up MQTT Service");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer( mqtt_server, atoi(mqtt_port));
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);

  //Serial.println("Connecting to Wi-Fi...");
  // TODO: WIfi AP name and password from config file
  /*
  if (!wifiManager.autoConnect("ESP866-DS18B20", "123456789", 3)) {
    Serial.println("Fallo al conectar al Wifi y timeout alcanzado");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
  }*/

  connectToWifi();
    
  
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  //if you get here you have connected to the WiFi
  //Serial.println("connected...yeey :)");

  //save the custom parameters to FS
  if (shouldSaveConfig) {    
    Serial.println("\n---------------------");
    Serial.println("| Saving new config |\n---------------------\n");

    //read updated parameters
    strcpy(mqtt_server, config_mqtt_server.getValue());
    strcpy(mqtt_port, config_mqtt_port.getValue());
    strcpy(control_topic, config_control_topic.getValue());


    // TODO: Add sensor topics here
    strcpy(temperature_topic, config_temperature_topic.getValue());


    //------------------------------------------


    strcpy(static_ip, config_static_ip.getValue());
    strcpy(static_gw, config_static_gw.getValue());
    strcpy(static_sn, config_static_sn.getValue());
    strcpy(static_dns1, config_static_dns1.getValue());
    strcpy(static_dns2, config_static_dns2.getValue());
    
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
    request->send(200, "text/plain", "Yo soy la termocupla 1.");
  });
  server.on("/factoryReset", HTTP_GET, [](AsyncWebServerRequest *request) {
    //TODO: index page for control and info
    wifiManager.resetSettings();
    ESP.restart();
    
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
      read_and_publish_sensors();
    }
    if (now - live_since > interval * 1000 * 10) {
      ESP.restart();
    }
  
}