/*
  PCB Conveyor Belt Firmware

  Written by Jonathan Oxer <jon@oxer.com.au>

  Controlled using GCODE. Understands these codes:

  "G28" does a Y-axis homing sequence. Arguments are ignored.
  "G0 Y<distance>" move the Y axis to the defined mm width.

  "M03 S<speed>"           Set the direction as clockwise (left to right) (default).
  "M04 S<speed>"           Set the direction as counterclockwise (right to left).
  "M05 S<speed>"           Stop the conveyor.

  "M10"                    Clamp a PCB                                      **DEFINED BUT NOT USED**
  "M11"                    Unclamp a PCB                                    **DEFINED BUT NOT USED**
  "M17 S<position>"        Request status of a sensor at <position>         **NOT YET IMPLEMENTED**

  "M50 S<speed>"           Load to middle immediately                       **NOT YET IMPLEMENTED**
  "M51 S<speed>"           Load to middle when ready-in/out                 **NOT YET IMPLEMENTED**
  "M52 S<speed>"           Load to end immediately                          **NOT YET IMPLEMENTED**
  "M53 S<speed>"           Load to end when ready-in/out                    **NOT YET IMPLEMENTED**
  "M54 S<speed>"           Move first board on the conveyor to the end      **NOT YET IMPLEMENTED**
  "M55 S<speed>"           Unload immediately
  "M56 S<speed>"           Unload when ready-in/out
  "M57 S<speed> P<dwell>"  Unload at a timed interval
  "M58 S<speed>"           Load and unload when ready-in/out                **NOT YET IMPLEMENTED**
  "M59 S<speed> P<dwell>"  Load when ready-in/out, unload at timed interval **NOT YET IMPLEMENTED**

  The <speed> argument is in mm/minute. Range is 600 - 2200mm/min.

  The <dwell> argument is in seconds.

  NOTE: There is no way to set the speed of the conveyor without giving
  left / right / stop as well. Perhaps add a config value for speed. Allan
  suggested a ramp up/down on speed change.
  ALSO: There is no way to set the direction without sending a move command.

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

  1. Make all the peripherals work
    To do:
    - Calibrate speed control of Y axis motor
    - Ready-in detection
    - Ready-out control
    Done:
    - PCB sensors
    - Homing sensor

  2. Direct control of operations
    To do:
    - Prevent Y-axis movement until the home sequence has been completed.
    Done:
    - Home
    - Set width
    - Move left / right / stop

  3. Objective-driven / mode operations
    - Unload PCB triggered by Ready-In from next machine (for feeding boards to PnP)
    - Load PCB to exit position
    - Load PCB to mid position
    Done:
    - Unload PCB triggered by M-Code command.
    - Timed unloading of PCBs (for feeding boards to reflow)

  BUGS:
    - MCU reboots periodically, for no reason I can see.
    - After rebooting, it won't respond to serial comms.

  TO DO:
    - PANIC stop!
    - Power to stepper through NC pins of limit switch at front position to prevent close collisions
    - Select pins for:
      - Read "s88 automation standard"
*/
#define VERSION "2.0"


/*--------------------------- Configuration ---------------------------------*/
// Configuration should be done in the included file:
#include "config.h"
#include "shhh_secret.h"

/*--------------------------- Libraries -------------------------------------*/
#include <Stepper.h>             // To drive the Y axis
#include <WiFi.h>                     // ESP32 WiFi driver
#include <PubSubClient.h>             // For MQTT
#include <ESPmDNS.h>                  // For OTA
#include <WiFiUdp.h>                  // For OTA
#include <ArduinoOTA.h>               // For OTA
#if ENABLE_LCD
#include <Arduino_GFX_Library.h>      // SPI LCD
#endif
#include <CAN.h>                      // By Sandeep Mistry
#include "Adafruit_VL53L0X.h"         // For ToF board sensors
#include <Adafruit_MCP23X17.h>        // For ToF sensors and in/out connections

/*--------------------------- Global Variables ------------------------------*/
#define  STOP   0
#define  LEFT   3
#define  RIGHT  4

#define  UNTRIPPED  0
#define  TRIPPED    1

#define  STATE_BEGIN       0
#define  STATE_IDLE        1
#define  STATE_ERROR       2
#define  STATE_STOPPED    10
#define  STATE_CONSTANT   11

// M55 S<speed>: Unload one board and then stop. This assumes a board is already sitting
// on the conveyor. Ignores ready-in/out.
#define  STATE_UNLOAD_NOW_BEGIN         550
#define  STATE_UNLOAD_NOW_MOVING        551
#define  STATE_UNLOAD_NOW_REACHED_END   552
#define  STATE_UNLOAD_NOW_CLEARED_END   553
#define  STATE_UNLOAD_NOW_RUNON         554   // Continue running briefly after PCB cleared sensor
// What state do we transition to after 23? 1? 10?

// M56 S<speed>: Unload when ready-in/out
#define  STATE_UNLOAD_RIRO_BEGIN        560
#define  STATE_UNLOAD_RIRO_MOVING       561
#define  STATE_UNLOAD_RIRO_REACHED_END  562
#define  STATE_UNLOAD_RIRO_CLEARED_END  563
#define  STATE_UNLOAD_RIRO_RUNON        564

// M57 S<speed> P<interval>: Unload boards at timed intervals. This assumes boards are
// already sitting on the conveyor. Ignores ready-in/out.
#define  STATE_UNLOAD_TIMED_BEGIN       570
#define  STATE_UNLOAD_TIMED_MOVING      571
#define  STATE_UNLOAD_TIMED_REACHED_END 572
#define  STATE_UNLOAD_TIMED_CLEARED_END 573
#define  STATE_UNLOAD_TIMED_PAUSE       574

#define  OUT_OF_RANGE           4     // TOF sensors return 4 when out of range

uint8_t  g_homed              = false;
uint16_t g_state              = STATE_BEGIN;
uint32_t g_last_state_change  = 0;    // timestamp of last state change
uint32_t step_count           = 0;
float    g_current_y_position = 0.0;

uint8_t  g_x_direction        = STOP; //
uint16_t g_x_requested_speed  = 0;    // mm/min
uint16_t g_x_actual_speed     = 0;    // mm/min
int16_t  g_requested_pause    = 0;    // Seconds. -1 indicates not set or invalid

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

// PCB sensors
VL53L0X_RangingMeasurementData_t g_pcb_sensor_l_reading;
VL53L0X_RangingMeasurementData_t g_pcb_sensor_m_reading;
VL53L0X_RangingMeasurementData_t g_pcb_sensor_r_reading;
bool     g_entrance_sensor = UNTRIPPED;
bool     g_middle_sensor   = UNTRIPPED;
bool     g_exit_sensor     = UNTRIPPED;
uint8_t  g_exit_sensor_count = 0;

// Ready-in / Ready-out handshaking
bool     g_ready_in_left   = false;
bool     g_ready_in_right  = false;


// General
char g_device_id[23];                 // Unique ID from ESP chip ID

// State machine variables
uint32_t g_runon_began      = 0;      // ms since runon began

/*--------------------------- Function Signatures ---------------------------*/
void initialise_pcb_sensors();
void debug_sensor_values();
void perform_state_transition(uint16_t g_state);

/*--------------------------- Macros ----------------------------------------*/

#if STATE_DEBUGGING
#define STATE_DEBUG_PRINT(x) Serial.print(x)
#define STATE_DEBUG_PRINTLN(x) Serial.println(x)
#else
#define STATE_DEBUG_PRINT(x)
#define STATE_DEBUG_PRINTLN(x)
#endif

/*--------------------------- Instantiate Global Objects --------------------*/
Stepper yAxisStepper(steps_per_revolution, PIN_Y_IN1, PIN_Y_IN3, PIN_Y_IN2, PIN_Y_IN4);
Adafruit_MCP23X17 mcp23017;
Adafruit_VL53L0X pcb_sensor_l = Adafruit_VL53L0X();
Adafruit_VL53L0X pcb_sensor_m = Adafruit_VL53L0X();
Adafruit_VL53L0X pcb_sensor_r = Adafruit_VL53L0X();

// MQTT
#if ENABLE_WIFI
WiFiClient esp_client;
PubSubClient client(esp_client);
#endif

#if ENABLE_LCD
Arduino_DataBus *bus = new Arduino_ESP32SPI(21 /* DC */, 15 /* CS */, 14 /* SCK */, 13 /* MOSI */, -1 /* MISO */);
Arduino_GFX *gfx = new Arduino_ST7796(bus, 22 /* RST */, 3 /* rotation */);
#endif

/*--------------------------- Program ---------------------------------------*/
/* Resources */
#include "motors.h"
#include "gcode.h"
#include "mqtt_comms.h"
#include "serial_comms.h"
#include "can_comms.h"
#include "pcb_sensors.h"
#include "riro.h"

/*
  Setup
*/
void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(LIMIT_SENSOR_Y_PIN,  INPUT );

  // #define CONV_MAX_SPEED  2200 // mm/minute
  // M03 S2200
  // 0 - 1023     500 - 800   300
  pinMode(PIN_X_IN1,           OUTPUT);
  pinMode(PIN_X_IN2,           OUTPUT);
  ledcSetup(0, MOTOR_PWM_FREQUENCY, 10); // Channel, frequency, resolution
  ledcSetup(1, MOTOR_PWM_FREQUENCY, 10); // Channel, frequency, resolution
  ledcAttachPin(PIN_X_IN1,          0);  // Pin, channel
  ledcAttachPin(PIN_X_IN2,          1);  // Pin, channel

  g_input_buffer.reserve(MAX_SERIAL_INPUT);
  yAxisStepper.setSpeed(13);  // RPM. Experiment with setting this higher.


#if ENABLE_LCD
  pinMode(TFT_BL_PIN,          OUTPUT);
  ledcSetup(2, MOTOR_PWM_FREQUENCY, 8);  // Channel, frequency, resolution
  ledcAttachPin(TFT_BL_PIN,         2);  // Pin, channel
  ledcWrite(2, g_backlight_level);

  // Initialize LCD
  gfx->begin();
  gfx->setRotation(1);
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
#endif

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

#if ENABLE_LCD
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
#endif

  // Set up the I/O expander for limit sensors and in/out connections
  Wire.begin(SDA_PIN, SCL_PIN);
  if (mcp23017.begin_I2C(MCP23017_ADDR)) {
    Serial.println("MCP23017 initialised ok");
    mcp23017.pinMode(PCB_SENSOR_L_XSHUT, OUTPUT);
    mcp23017.pinMode(PCB_SENSOR_M_XSHUT, OUTPUT);
    mcp23017.pinMode(PCB_SENSOR_R_XSHUT, OUTPUT);
    mcp23017.digitalWrite(PCB_SENSOR_L_XSHUT, LOW);
    mcp23017.digitalWrite(PCB_SENSOR_M_XSHUT, LOW);
    mcp23017.digitalWrite(PCB_SENSOR_R_XSHUT, LOW);
  } else {
    Serial.println("MCP23017 failed to initialise");
  }

  // Set up PCB sensors
  initialise_pcb_sensors();

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
#if ENABLE_LCD
    gfx->setTextColor(WHITE);
    gfx->print(" IP Address: ");
    gfx->setTextColor(GREEN);
    gfx->println(WiFi.localIP());
#endif
  } else {
    Serial.println("Wifi [FAILED]");
#if ENABLE_LCD
    gfx->setTextColor(WHITE);
    gfx->print(" IP Address: ");
    gfx->setTextColor(RED);
    gfx->println("FAILED");
#endif
  }

  /* Set up the MQTT client */
  client.setServer(mqtt_broker, 1883);
  client.setCallback(callback);

  /* Set up the CAN interface */
  if (ESP.getChipRevision() >= 2)
  {
    if (!CAN.begin(CAN_BUS_SPEED * 2))
    {
      Serial.println("CAN failed!");
    } else {
      Serial.print("CAN 2 initialised at ");
      Serial.println(CAN_BUS_SPEED);
    }
  } else {
    if (!CAN.begin(250E3))
    {
      Serial.println("CAN failed!");
    } else {
      Serial.print("CAN initialised at ");
      Serial.println(250E3);
    }
  }
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
  //readCANMessages();
  setConveyorMotorSpeed();
  read_pcb_sensors();
  //debug_sensor_values();
  process_state_machine();
  check_ready_in();
}

void process_state_machine()
{
  switch (g_state)
  {
    case STATE_BEGIN:  // 0
      g_x_direction = STOP;
      break;

    case STATE_IDLE:  // 1
      g_x_direction = STOP;
      break;

    case STATE_ERROR:
      Serial.println("ERROR STATE");
      g_x_direction = STOP;
      break;

    /* UNLOAD_NOW block */
    case STATE_UNLOAD_NOW_BEGIN:  //
      g_x_direction = RIGHT;
      setConveyorMotorSpeed();
      perform_state_transition(STATE_UNLOAD_NOW_MOVING);
      break;

    case STATE_UNLOAD_NOW_MOVING:  //
      // Check exit sensor
      if (TRIPPED == g_exit_sensor)
      {
        perform_state_transition(STATE_UNLOAD_NOW_REACHED_END);
      }
      if (millis() > g_last_state_change + (UNLOAD_TIMEOUT * 1000))
      {
        g_x_direction = STOP;
        perform_state_transition(STATE_IDLE);
      }
      break;

    case STATE_UNLOAD_NOW_REACHED_END:  //
      // Check exit sensor
      if (UNTRIPPED == g_exit_sensor)
      {
        g_exit_sensor_count++;
        if (g_exit_sensor_count > SENSOR_DEBOUNCE_COUNT)
        {
          g_exit_sensor_count = 0;
          perform_state_transition(STATE_UNLOAD_NOW_CLEARED_END);
        }
      } else {
        g_exit_sensor_count = 0;
      }
      break;

    case STATE_UNLOAD_NOW_CLEARED_END:  // 23
      // Begin the runon timer
      g_runon_began = millis();
      perform_state_transition(STATE_UNLOAD_NOW_RUNON);
      break;

    case STATE_UNLOAD_NOW_RUNON:  //
      // Check the runon timer
      if (millis() > g_runon_began + RUNON_TIME)
      {
        g_x_direction = STOP;
        perform_state_transition(STATE_IDLE);
      }
      break;


    /* UNLOAD_RIRO block */
    case STATE_UNLOAD_RIRO_BEGIN:  //
      //g_x_direction = STOP;
      if (g_ready_in_right)
      {
        perform_state_transition(STATE_UNLOAD_RIRO_MOVING);
      }
      break;

    case STATE_UNLOAD_RIRO_MOVING:  //
      g_x_direction = RIGHT;
      setConveyorMotorSpeed();
      // Check exit sensor
      if (TRIPPED == g_exit_sensor)
      {
        perform_state_transition(STATE_UNLOAD_RIRO_REACHED_END);
      }
      if (millis() > g_last_state_change + (UNLOAD_TIMEOUT * 1000))
      {
        g_x_direction = STOP;
        perform_state_transition(STATE_IDLE);
      }
      break;

    case STATE_UNLOAD_RIRO_REACHED_END:  //
      // Check exit sensor
      if (UNTRIPPED == g_exit_sensor)
      {
        g_exit_sensor_count++;
        if (g_exit_sensor_count > SENSOR_DEBOUNCE_COUNT)
        {
          g_exit_sensor_count = 0;
          perform_state_transition(STATE_UNLOAD_RIRO_CLEARED_END);
        }
      } else {
        g_exit_sensor_count = 0;
      }
      break;

    case STATE_UNLOAD_RIRO_CLEARED_END:  // 23
      // Begin the runon timer
      g_runon_began = millis();
      perform_state_transition(STATE_UNLOAD_RIRO_RUNON);
      break;

    case STATE_UNLOAD_RIRO_RUNON:  //
      // Check the runon timer
      if (millis() > g_runon_began + RUNON_TIME)
      {
        g_x_direction = STOP;
        perform_state_transition(STATE_UNLOAD_RIRO_BEGIN);
      }
      break;


    /* UNLOAD_TIMED block */
    case STATE_UNLOAD_TIMED_BEGIN:  //
      g_x_direction = RIGHT;
      setConveyorMotorSpeed();
      perform_state_transition(STATE_UNLOAD_TIMED_MOVING);
      break;

    case STATE_UNLOAD_TIMED_MOVING:  //
      // Check exit sensor
      if (TRIPPED == g_exit_sensor)
      {
        perform_state_transition(STATE_UNLOAD_TIMED_REACHED_END);
      }
      if (millis() > g_last_state_change + (UNLOAD_TIMEOUT * 1000))
      {
        g_x_direction = STOP;
        perform_state_transition(STATE_IDLE);
      }
      break;

    case STATE_UNLOAD_TIMED_REACHED_END:  //
      // Check exit sensor
      if (UNTRIPPED == g_exit_sensor)
      {
        g_exit_sensor_count++;
        if (g_exit_sensor_count > SENSOR_DEBOUNCE_COUNT)
        {
          g_exit_sensor_count = 0;
          perform_state_transition(STATE_UNLOAD_TIMED_CLEARED_END);
        }
      } else {
        g_exit_sensor_count = 0;
      }
      break;

    // Board has been unloaded, waiting for the next board to arrive at the exit
    case STATE_UNLOAD_TIMED_CLEARED_END:  //
      // Check exit sensor
      if (TRIPPED == g_exit_sensor)
      {
        g_exit_sensor_count++;
        if (g_exit_sensor_count > SENSOR_DEBOUNCE_COUNT)
        {
          g_exit_sensor_count = 0;
          perform_state_transition(STATE_UNLOAD_TIMED_PAUSE);
          Serial.println("Leaving");
        }
      } else {
        g_exit_sensor_count = 0;
      }
      if (millis() > g_last_state_change + (UNLOAD_TIMEOUT * 1000))
      {
        g_x_direction = STOP;
        perform_state_transition(STATE_IDLE);
      }
      break;

    case STATE_UNLOAD_TIMED_PAUSE:  //
      g_x_direction = STOP;
      // Check the pause timer
      if (millis() > g_last_state_change + (g_requested_pause * 1000))
      {
        Serial.println("Leaving 2");
        perform_state_transition(STATE_UNLOAD_TIMED_BEGIN);
      }
      break;

    /* Catchall */
    default:
      perform_state_transition(STATE_ERROR);
      break;
  }
}


void perform_state_transition(uint16_t new_state)
{
  STATE_DEBUG_PRINT  ("sm: [");
  STATE_DEBUG_PRINT  (g_state);
  STATE_DEBUG_PRINT  (" -> ");
  STATE_DEBUG_PRINT  (new_state);
  STATE_DEBUG_PRINT("] @ ");
  STATE_DEBUG_PRINT(millis());
  STATE_DEBUG_PRINT(", delta ");
  STATE_DEBUG_PRINTLN(millis() - g_last_state_change);

  g_state = new_state;
  g_last_state_change = millis();
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
