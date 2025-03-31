/* ----------------- General config -------------------------------- */
#define  ENABLE_WIFI               true
#define  ENABLE_MQTT               true
#define  ENABLE_LCD               false

#define  LIMIT_BACKOFF              100  // Steps, make it mm later
#define  HOME_SWITCH_OFFSET         297  // mm from 0 width to the homed position
#define  MINIMUM_CONVEYOR_POSITION   45  // Never go smaller than this
#define  MAXIMUM_CONVEYOR_POSITION  300  // Never go bigger than this
#define  MINIMUM_SPEED              600  // Never go slower than this
#define  MAXIMUM_SPEED             2200  // Never go faster than this

#define  RUNON_TIME                   0  // ms. Runtime after unload sensor cleared.
#define  LOAD_TIMEOUT                30  // Seconds. Stop if nothing appears within this time.
#define  UNLOAD_TIMEOUT              30  // Seconds. Stop if nothing gets to the exit within this time.
// Note: UNLOAD_TIMEOUT may be the wrong way of thinking about this, because the speed
// of the conveyor can vary. It probably needs to know how long it is, and then know
// how fast it's running so it can work out how long it needs to run.

#define  BACKLIGHT_LEVEL_HIGH        70
#define  BACKLIGHT_LEVEL_LOW         20

#define  SERIAL_DEBUGGING         false
#define  CAN_DEBUGGING            false
#define  STATE_DEBUGGING           true

/* WiFi */
//const char* ssid                   = "YOUR SSID";     // Your WiFi SSID
//const char* password               = "YOUR PSK";     // Your WiFi password

/* MQTT */
//const char* mqtt_broker            = "192.168.1.185"; // IP address of your MQTT broker
//const char* mqtt_username          = "YOUR USER";
//const char* mqtt_password          = "YOUR PASS";
//const char* status_topic           = "events";        // MQTT topic to report startup

/* Serial */
#define  SERIAL_BAUD_RATE        115200  // Speed for USB serial console

/* ----------------- Hardware-specific config ---------------------- */
/* X axis drive motors */
#define  MOTOR_PWM_FREQUENCY   5000  // Base frequency for PWM
#define  PIN_X_IN1               32
#define  PIN_X_IN2               33
#define  MOTOR_PWM_AT_MIN       200
#define  MOTOR_PWM_AT_MAX      1023

/* Y axis stepper motor */
#define  PIN_Y_IN1               27 //12 //19
#define  PIN_Y_IN2               26 //18
#define  PIN_Y_IN3               12
#define  PIN_Y_IN4               25 //17
const int steps_per_revolution = 2048;  // change this to fit the number of steps per revolution
//const int steps_per_revolution = 1024;  // change this to fit the number of steps per revolution
const float steps_per_mm       = 39.47;

/* Y axis limit sensor */
#define  LIMIT_SENSOR_Y_PIN       35 //16  // 

/* LCD */
#define  TFT_BL_PIN               23

/* PCB sensors */
#define  PCB_TRIGGER_HEIGHT       45    // Anything detected lower than this means a PCB is present at the sensor
#define  SENSOR_DEBOUNCE_COUNT    10    // Consecutive untriggered reads for board to be considered absent
#define  PCB_SENSOR_L_ADDR      0x30
#define  PCB_SENSOR_M_ADDR      0x31
#define  PCB_SENSOR_R_ADDR      0x32
#define  GPA0 0                         // Port A of IO expander
#define  GPA1 1                         // Port A of IO expander
#define  GPA2 2                         // Port A of IO expander
#define  PCB_SENSOR_L_XSHUT     GPA0
#define  PCB_SENSOR_M_XSHUT     GPA1
#define  PCB_SENSOR_R_XSHUT     GPA2

/* CAN bus */
#define  CAN_BUS_SPEED         250E3
//#define  CAN_RX_PIN                4
//#define  CAN_TX_PIN                5

/* I2C */
#define  SDA_PIN                  18
#define  SCL_PIN                  19
#define  MCP23017_ADDR          0x20

/* Ready-in / ready-out connections */
#define  READY_IN_LEFT_PIN        99
#define  READY_OUT_LEFT_PIN       99
#define  READY_IN_RIGHT_PIN       99
#define  READY_OUT_RIGHT_PIN      99
