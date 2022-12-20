#ifndef H_MOTORS
#define H_MOTORS

/*
   Move the Y axis until the limit switch is tripped, then back off
*/
void homeYAxis()
{
  // Move towards the back until the limit is tripped
  step_count = 0;
  while (digitalRead(LIMIT_SENSOR_Y_PIN) == LOW)
  {
    yAxisStepper.step(1);
    step_count++;
  }
  delay(50); // Just to reduce the shock of changing direction

  // Move off the limit switch
  yAxisStepper.step(-LIMIT_BACKOFF);

  g_current_y_position = HOME_SWITCH_OFFSET;
}

void setRequestedSpeed(uint16_t requested_speed)
{
  if ((requested_speed >= MINIMUM_SPEED && requested_speed <= MAXIMUM_SPEED)
      || 0 == requested_speed)
  {
    // Set global speed
    g_y_speed = requested_speed;
  }
}

/*

*/
void setConveyorMotorSpeed()
{
  if (STOP == g_y_direction)
  {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }

  // Calculate motor pwm value here from requested speed.
  // Perhaps use a map function?
  uint8_t motor_speed = 0;
  //if(g_y_speed

  if (LEFT == g_y_direction)
  {
    ledcWrite(0, 0);
    ledcWrite(1, motor_speed);
  }

  if (RIGHT == g_y_direction)
  {
    ledcWrite(0, motor_speed);
    ledcWrite(1, 0);
  }
}

#endif H_MOTORS
