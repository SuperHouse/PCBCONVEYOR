/* ----------------- General config -------------------------------- */
#define  ENABLE_WIFI               true
#define  ENABLE_MQTT               true

#define  LIMIT_BACKOFF              100  // Steps, make it mm later
#define  HOME_SWITCH_OFFSET         296  // mm from 0 width to the homed position
#define  MINIMUM_CONVEYOR_POSITION   45  // Never go smaller than this
#define  MAXIMUM_CONVEYOR_POSITION  300  // Never go bigger than this
#define  MINIMUM_SPEED              600  // Never go slower than this
#define  MAXIMUM_SPEED             2200  // Never go faster than this

#define  BACKLIGHT_LEVEL_HIGH        70
#define  BACKLIGHT_LEVEL_LOW         20

/* WiFi */
const char* ssid                   = "";     // Your WiFi SSID
const char* password               = "";     // Your WiFi password

/* MQTT */
const char* mqtt_broker            = "192.168.1.111"; // IP address of your MQTT broker
const char* status_topic           = "events";        // MQTT topic to report startup

/* Serial */
#define  SERIAL_BAUD_RATE        115200  // Speed for USB serial console

/* ----------------- Hardware-specific config ---------------------- */
/* Y axis stepper motor */
#define  PIN_IN1                  27 //12 //19
#define  PIN_IN2                  26 //18
#define  PIN_IN3                   5
#define  PIN_IN4                  25 //17
#define  MOTOR_PWM_FREQUENCY    5000  // Base frequency for PWM

const int steps_per_revolution = 2048;  // change this to fit the number of steps per revolution
const float steps_per_mm       = 39.47;

/* LCD */
#define  TFT_BL_PIN               23

/* Y axis limit sensor */
#define  LIMIT_SENSOR_Y_PIN       35 //16  // 

/* Conveyor motor driver */
#define  DRIVER_Y_A_PIN           99  // 25
#define  DRIVER_Y_B_PIN           99  // 26

/* PCB sensors */
#define  PCB_SENSOR_LEFT_PIN      99
#define  PCB_SENSOR_MIDDLE_PIN    99
#define  PCB_SENSOR_RIGHT_PIN     99

/* Ready-in / ready-out connections */
#define  READY_IN_LEFT_PIN        99
#define  READY_OUT_LEFT_PIN       99
#define  READY_IN_RIGHT_PIN       99
#define  READY_OUT_RIGHT_PIN      99
