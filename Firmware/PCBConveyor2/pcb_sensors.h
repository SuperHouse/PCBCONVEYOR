#ifndef H_PCB_SENSORS
#define H_PCB_SENSORS

void initialise_pcb_sensors()
{
  // At this point all the XSHUT pins should be pulled low from setup,
  // but for completeness we'll make sure:
  mcp23017.digitalWrite(PCB_SENSOR_L_XSHUT, LOW);
  mcp23017.digitalWrite(PCB_SENSOR_M_XSHUT, LOW);
  mcp23017.digitalWrite(PCB_SENSOR_R_XSHUT, LOW);
  delay(10);

  // Bring them all up:
  mcp23017.digitalWrite(PCB_SENSOR_L_XSHUT, HIGH);
  mcp23017.digitalWrite(PCB_SENSOR_M_XSHUT, HIGH);
  mcp23017.digitalWrite(PCB_SENSOR_R_XSHUT, HIGH);
  delay(10);

  // Disable all but L sensor:
  mcp23017.digitalWrite(PCB_SENSOR_M_XSHUT, LOW);
  mcp23017.digitalWrite(PCB_SENSOR_R_XSHUT, LOW);
  delay(10);

  // Initialise the L sensor:
  if (!pcb_sensor_l.begin(PCB_SENSOR_L_ADDR))
  {
    Serial.println("Failed to initialise left PCB sensor.");
  } else {
    Serial.println("Initialised left PCB sensor.");
  }

  // Bring up the M sensor:
  mcp23017.digitalWrite(PCB_SENSOR_M_XSHUT, HIGH);
  delay(10);
  if (!pcb_sensor_m.begin(PCB_SENSOR_M_ADDR))
  {
    Serial.println("Failed to initialise middle PCB sensor.");
  } else {
    Serial.println("Initialised middle PCB sensor.");
  }
  //
  // Bring up the R sensor:
  mcp23017.digitalWrite(PCB_SENSOR_R_XSHUT, HIGH);
  delay(10);
  if (!pcb_sensor_r.begin(PCB_SENSOR_R_ADDR))
  {
    Serial.println("Failed to initialise right PCB sensor.");
  } else {
    Serial.println("Initialised right PCB sensor.");
  }
}

void read_pcb_sensors()
{
  // ​​TODO: Add hysteresis logic to OUT OF RANGE tests near line 72,
  // and remove it from the state machine

  // NOTE: We need to account for direction of travel, and map l/r
  // to entrance / exit positions
  //  uint8_t entrance_sensor = 1;
  //  uint8_t middle_sensor = 2;
  //  uint8_t exit_sensor   = 3;

  //  uint16_t l_sensor_reading = g_pcb_sensor_l_reading.RangeMilliMeter;
  //  uint16_t m_sensor_reading = g_pcb_sensor_m_reading.RangeMilliMeter;

  pcb_sensor_l.rangingTest(&g_pcb_sensor_l_reading, false);
  uint16_t l_sensor_reading = g_pcb_sensor_l_reading.RangeMilliMeter;
  pcb_sensor_m.rangingTest(&g_pcb_sensor_m_reading, false);
  uint16_t m_sensor_reading = g_pcb_sensor_m_reading.RangeMilliMeter;
  pcb_sensor_r.rangingTest(&g_pcb_sensor_r_reading, false);
  uint16_t r_sensor_reading = g_pcb_sensor_r_reading.RangeMilliMeter;

  if (OUT_OF_RANGE != l_sensor_reading && l_sensor_reading < PCB_TRIGGER_HEIGHT)
  {
    g_entrance_sensor = TRIPPED;
  } else {
    g_entrance_sensor = UNTRIPPED;
  }

  if (OUT_OF_RANGE != m_sensor_reading && m_sensor_reading < PCB_TRIGGER_HEIGHT)
  {
    g_middle_sensor = TRIPPED;
  } else {
    g_middle_sensor = UNTRIPPED;
  }

  if (OUT_OF_RANGE != r_sensor_reading && r_sensor_reading < PCB_TRIGGER_HEIGHT)
  {
    g_exit_sensor = TRIPPED;
  } else {
    g_exit_sensor = UNTRIPPED;
  }
}

void debug_sensor_values()
{
  pcb_sensor_l.rangingTest(&g_pcb_sensor_l_reading, false);
  if (g_pcb_sensor_l_reading.RangeStatus != 4)
  {
    Serial.print("1: ");
    Serial.println(g_pcb_sensor_l_reading.RangeMilliMeter);
  } else {
    //Serial.println("Ranging error");
  }
  pcb_sensor_m.rangingTest(&g_pcb_sensor_m_reading, false);
  if (g_pcb_sensor_m_reading.RangeStatus != 4)
  {
    Serial.print("               2: ");
    Serial.println(g_pcb_sensor_m_reading.RangeMilliMeter);
  } else {
    //Serial.println("Ranging error");
  }
  pcb_sensor_r.rangingTest(&g_pcb_sensor_r_reading, false);
  if (g_pcb_sensor_r_reading.RangeStatus != 4)
  {
    Serial.print("                                3: ");
    Serial.println(g_pcb_sensor_r_reading.RangeMilliMeter);
  } else {
    //Serial.println("Ranging error");
  }
  delay(50);
}

#endif H_PCB_SENSORS
