#ifndef H_GCODE
#define H_GCODE

String g_input_buffer = "";

#define GCODE_HOME              28
#define GCODE_MOVE               0
#define MCODE_SPINDLE_RIGHT     03
#define MCODE_SPINDLE_LEFT      04
#define MCODE_SPINDLE_STOP      05


/*
  Look for character /code/ in the inputBuffer and read the float that immediately follows it.
  @return the value found.  If nothing is found, /default_value/ is returned.
  @input code the character to look for.
  @input defaultVal the return value if /code/ is not found.
*/
float parseGCodeParameter(char code, float default_value) {
  int code_position = g_input_buffer.indexOf(code);
  if (code_position != -1)  // The code has been found in the buffer.
  {
    // Find the end of the number (separated by " " (space))
    int delimiter_position = g_input_buffer.indexOf(" ", code_position + 1);
    float parsed_value = g_input_buffer.substring(code_position + 1, delimiter_position).toFloat();
    return parsed_value;
  } else {
    return default_value;
  }
}


/*

*/
void processGCodeMessage()
{
  uint8_t valid_command_found = false;
  int8_t  command_code = -1;

  // Strip comments
  g_input_buffer.remove(g_input_buffer.indexOf(";"));
  g_input_buffer.trim();

  /*-- Check for G-code messages --*/
  // Extract the command, default -1 if not found
  command_code = parseGCodeParameter('G', -1);

  switch (command_code)
  {
    case GCODE_HOME:
      {
        valid_command_found = true;
        Serial.println("Homing start");
#if ENABLE_MQTT
        client.publish(g_mqtt_tele_topic, "Homing start");
#endif
        homeYAxis();
        g_homed = true;
        Serial.println("Homing complete");
#if ENABLE_MQTT
        client.publish(g_mqtt_tele_topic, "Homing complete");
#endif
        break;
      }

    case GCODE_MOVE:
      {
        valid_command_found = true;
        Serial.println("GCODE move!");
        // Don't move unless we've already been homed.
        if (false == g_homed)
        {
          Serial.println("Home the device first using command 'G28'");
          client.publish(g_mqtt_tele_topic, "Home the device first using command 'G28'");
          break;
        }

        // Extract the requested position from the GCODE message.
        float requested_y_position = parseGCodeParameter('Y', 0);
        if (requested_y_position > MAXIMUM_CONVEYOR_POSITION)
        {
          Serial.print("Can't move to greater than ");
          Serial.print(MAXIMUM_CONVEYOR_POSITION);
          Serial.println("mm");
#if ENABLE_MQTT
          sprintf(g_mqtt_message_buffer, "Can't move to greater than %i mm", MAXIMUM_CONVEYOR_POSITION);
          client.publish(g_mqtt_tele_topic, g_mqtt_message_buffer);
#endif
          break;
        }

        if (requested_y_position < MINIMUM_CONVEYOR_POSITION)
        {
          Serial.print("Can't move to smaller than ");
          Serial.print(MINIMUM_CONVEYOR_POSITION);
          Serial.println("mm");
#if ENABLE_MQTT
          sprintf(g_mqtt_message_buffer, "Can't move to less than %i mm", MINIMUM_CONVEYOR_POSITION);
          client.publish(g_mqtt_tele_topic, g_mqtt_message_buffer);
#endif
          break;
        }

        // The requested position is within spec, so continue.
        Serial.print("Current position: ");
        Serial.println(g_current_y_position);
        Serial.print("Desired position: ");
        Serial.println(requested_y_position);

        float y_position_delta = requested_y_position - g_current_y_position;
        Serial.print("Position delta: ");
        Serial.println(y_position_delta);

        // This is where to convert desired mm to steps
        int32_t movement_steps = (int)(y_position_delta * steps_per_mm);

        Serial.print("Steps to move: ");
        Serial.println(movement_steps);

#if ENABLE_MQTT
        sprintf(g_mqtt_message_buffer, "Current: %.2f, Requested: %.2f, Delta: %.2f, Steps: %i",
                g_current_y_position, requested_y_position, y_position_delta, movement_steps);
        client.publish(g_mqtt_tele_topic, g_mqtt_message_buffer);
#endif

        yAxisStepper.step(movement_steps);
        g_current_y_position = requested_y_position;
        break;
      }
  }

  /*-- Check for M-code messages --*/
  // Extract the command, default -1 if not found
  command_code = parseGCodeParameter('M', -1);

  switch (command_code)
  {
    case MCODE_SPINDLE_STOP:
      {
        valid_command_found = true;
        g_y_direction = STOP;
        // Extract the requested speed from the GCODE message.
        uint16_t requested_speed = parseGCodeParameter('S', -1);
        setRequestedSpeed(requested_speed);
        Serial.println("Conveyor stop");
#if ENABLE_MQTT
        client.publish(g_mqtt_tele_topic, "Conveyor stop");
#endif
        break;
      }

    case MCODE_SPINDLE_RIGHT:
      {
        valid_command_found = true;
        g_y_direction       = RIGHT;
        // Extract the requested speed from the GCODE message.
        uint16_t requested_speed = parseGCodeParameter('S', -1);
        setRequestedSpeed(requested_speed);
        Serial.println("Conveyor right");
#if ENABLE_MQTT
        client.publish(g_mqtt_tele_topic, "Conveyor right");
#endif
        break;
      }

    case MCODE_SPINDLE_LEFT:
      {
        valid_command_found = true;
        g_y_direction       = LEFT;
        // Extract the requested speed from the GCODE message.
        uint16_t requested_speed = parseGCodeParameter('S', -1);
        setRequestedSpeed(requested_speed);
        Serial.println("Conveyor left");
#if ENABLE_MQTT
        client.publish(g_mqtt_tele_topic, "Conveyor left");
#endif
        break;
      }
  }

  if (!valid_command_found)
  {
    Serial.println("Unknown or empty command ignored");
#if ENABLE_MQTT
    client.publish(g_mqtt_tele_topic, "Unknown or empty command ignored");
#endif
  }
}

#endif H_GCODE
