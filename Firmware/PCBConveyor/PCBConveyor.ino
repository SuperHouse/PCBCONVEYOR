/*
  PCB Conveyor Belt Firmware

  Written by Jonathan Oxer <jon@oxer.com.au>

  Controlled using GCODE. Understands these codes:

  "G28" does a Y-axis homing sequence. Arguments are ignore.
  "G0 Y<distance>" moves the Y axis to the defined mm width.
  "M03 S<speed>" sets the direction as clockwise (left to right) (default).
  "M04 S<speed>" sets the direction as counterclockwise (right to left).
  "M05 S<speed>" stops the conveyor.

  The <speed> argument is in mm/minute. Range is 600 - 2200mm/min.

  No Y-axis movement is allowed until the home sequence has been completed.

  Arduino IDE ESP32 board profile:
  Go into Preferences -> Additional Board Manager URLs
  Add:
   https://dl.espressif.com/dl/package_esp32_index.json

  Then go to Tools -> Board -> Boards Manager..., update, and add "esp32 by Espressif Systems"

  Arduino IDE settings under "Tools":
    - Board:            ESP32 Dev Module
    - Upload speed:     115200bps
    - CPU Speed:        240MHz
    - Flash Frequency:  40MHz
    - Flash Mode:       DOUT
    - Flash Size:       4MB
    - Partition Scheme: Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)
    - Core Debug Level: None
    - PSRAM:            Disabled

  To do:
    - Select pins for:
      - H-bridge motor control
      - Ready-in/out for left and right
      - PCB sensors
*/
#define VERSION "0.2"


/*--------------------------- Configuration ---------------------------------*/
// Configuration should be done in the included file:
#include "config.h"

/*--------------------------- Libraries -------------------------------------*/
#include <Stepper.h>                  // To drive the Y axis
#include <WiFi.h>                     // ESP32 WiFi driver
#include <PubSubClient.h>             // For MQTT
#include <ESPmDNS.h>                  // For OTA
#include <WiFiUdp.h>                  // For OTA
#include <ArduinoOTA.h>               // For OTA
#include <Arduino_GFX_Library.h>      // SPI LCD

/*--------------------------- Global Variables ------------------------------*/
#define  STOP   0
#define  LEFT   3
#define  RIGHT  4

uint8_t  g_homed              = false;
uint32_t step_count           = 0;
float    g_current_y_position = 0.0;

uint8_t  g_y_direction        = STOP; //
uint16_t g_y_speed            = 0;    // mm/min

#define  MAX_SERIAL_INPUT             50

// Wifi
#define  WIFI_CONNECT_INTERVAL       500   // Wait 500ms intervals for wifi connection
#define  WIFI_CONNECT_MAX_ATTEMPTS    10   // Number of attempts/intervals to wait

// MQTT
char g_mqtt_message_buffer[150];      // General purpose buffer for MQTT messages
char g_mqtt_command_topic[50];        // MQTT topic for receiving commands
char g_mqtt_tele_topic[50];           // MQTT topic for telemetry

// LCD
uint16_t g_lcd_width       = 0;
uint16_t g_lcd_height      = 0;
uint8_t  g_backlight_level = BACKLIGHT_LEVEL_HIGH;

// General
char g_device_id[23];                 // Unique ID from ESP chip ID

/*--------------------------- Function Signatures ---------------------------*/


/*--------------------------- Instantiate Global Objects --------------------*/
Stepper yAxisStepper(steps_per_revolution, PIN_IN1, PIN_IN3, PIN_IN2, PIN_IN4);

// MQTT
#if ENABLE_WIFI
WiFiClient esp_client;
PubSubClient client(esp_client);
#endif

Arduino_DataBus *bus = new Arduino_ESP32SPI(21 /* DC */, 15 /* CS */, 14 /* SCK */, 13 /* MOSI */, -1 /* MISO */);
Arduino_GFX *gfx = new Arduino_ST7796(bus, 22 /* RST */, 3 /* rotation */);

/*--------------------------- Program ---------------------------------------*/
/* Resources */
#include "motors.h"
#include "gcode.h"
#include "mqtt_comms.h"
#include "serial_comms.h"

/*
  Setup
*/
void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(LIMIT_SENSOR_Y_PIN,  INPUT );
  pinMode(DRIVER_Y_A_PIN,      OUTPUT);
  pinMode(DRIVER_Y_B_PIN,      OUTPUT);

  ledcSetup(2, MOTOR_PWM_FREQUENCY, 8);  // Channel, frequency, resolution
  pinMode(TFT_BL_PIN,          OUTPUT);
  ledcAttachPin(TFT_BL_PIN,         2);  // Pin, channel
  ledcWrite(2, g_backlight_level);

  ledcSetup(0, MOTOR_PWM_FREQUENCY, 8);  // Channel, frequency, resolution
  ledcSetup(1, MOTOR_PWM_FREQUENCY, 8);  // Channel, frequency, resolution
  ledcAttachPin(DRIVER_Y_A_PIN,     0);  // Pin, channel
  ledcAttachPin(DRIVER_Y_B_PIN,     1);  // Pin, channel

  g_input_buffer.reserve(MAX_SERIAL_INPUT);
  yAxisStepper.setSpeed(13);  // RPM. Experiment with setting this higher.

  // Initialize LCD
  gfx->begin();
  g_lcd_width  = gfx->width();
  g_lcd_height = gfx->height();

  // Display banner
  gfx->setTextSize(3);
  gfx->fillScreen(BLACK);
  gfx->setTextColor(BLUE);
  gfx->setCursor(150, 8);
  gfx->println("SuperHouse");
  gfx->setTextColor(RED);
  gfx->print("    PCB Conveyor v");
  gfx->println(VERSION);
  gfx->println("");

  WiFi.mode(WIFI_STA);

  // We need a unique device ID for our MQTT client connection
  uint64_t chip_id = ESP.getEfuseMac(); // The chip ID is essentially its MAC address (length 6 bytes)
  uint16_t chip = (uint16_t)(chip_id >> 32);
  snprintf(g_device_id, 23, "%04x%08x", chip, (uint32_t)chip_id); // Use all the bytes
  Serial.print("Device ID: ");
  Serial.println(g_device_id);

  // Set up MQTT topics
  sprintf(g_mqtt_command_topic, "cmnd/%s/COMMAND",  g_device_id);  // For receiving commands
  sprintf(g_mqtt_tele_topic,    "tele/%s/TELE",     g_device_id);  // For telemetry

  // Report the MQTT topics to the serial console
  Serial.println("MQTT topics:");
  Serial.println(g_mqtt_command_topic);     // For receiving commands
  Serial.println(g_mqtt_tele_topic);        // For telemetry

  // Report the MQTT topics to the LCD
  gfx->setTextSize(2);
  //gfx->setCursor(0, 40);
  gfx->setTextColor(WHITE);
  gfx->print(" Commands:   ");
  gfx->setTextColor(GREEN);
  gfx->println(g_mqtt_command_topic);
  gfx->setTextColor(WHITE);
  gfx->print(" Telemetry:  ");
  gfx->setTextColor(GREEN);
  gfx->println(g_mqtt_tele_topic);

  // Connect to WiFi
  if (initWifi()) {
    Serial.println("Wifi [CONNECTED]");
    Serial.println(WiFi.localIP());
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("Connection Failed! Rebooting...");
      delay(1000);
      ESP.restart();
    }
    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);

    // Hostname defaults to esp3232-[MAC]
    // ArduinoOTA.setHostname("myesp32");

    // No authentication by default
    // ArduinoOTA.setPassword("admin");

    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
    ArduinoOTA.onStart([]()
    {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    gfx->setTextColor(WHITE);
    gfx->print(" IP Address: ");
    gfx->setTextColor(GREEN);
    gfx->println(WiFi.localIP());
  } else {
    Serial.println("Wifi [FAILED]");
    gfx->setTextColor(WHITE);
    gfx->print(" IP Address: ");
    gfx->setTextColor(RED);
    gfx->println("FAILED");
  }

  /* Set up the MQTT client */
  client.setServer(mqtt_broker, 1883);
  client.setCallback(callback);
}

/*
   Main loop
*/
void loop()
{
#if ENABLE_WIFI
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!client.connected()) {
      reconnectMqtt();
    }
  }
  client.loop();  // Process any outstanding MQTT messages
  ArduinoOTA.handle();
#endif
  listenToSerialStream();
  setConveyorMotorSpeed();
}

/**
  Connect to Wifi. Returns false if it can't connect.
*/
bool initWifi() {
  // Clean up any old auto-connections
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
  }
  WiFi.setAutoConnect(false);

  // RETURN: No SSID, so no wifi!
  if (sizeof(ssid) == 1) {
    return false;
  }

  // Connect to wifi
  WiFi.begin(ssid, password);

  // Wait for connection set amount of intervals
  int num_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && num_attempts <= WIFI_CONNECT_MAX_ATTEMPTS)
  {
    delay(WIFI_CONNECT_INTERVAL);
    num_attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    return false;
  } else {
    return true;
  }
}
