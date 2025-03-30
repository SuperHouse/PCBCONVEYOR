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
    g_x_requested_speed = requested_speed;
    Serial.print("Setting to ");
    Serial.println(requested_speed);
    Serial.print("Setting to ");
    Serial.println(g_x_requested_speed);
  } else {
    g_x_requested_speed = 0;
  }
}

/*

*/
void setConveyorMotorSpeed()
{
  uint16_t motor_pwm = 0;
  motor_pwm = map(g_x_requested_speed, MINIMUM_SPEED, MAXIMUM_SPEED, MOTOR_PWM_AT_MIN, MOTOR_PWM_AT_MAX);
  motor_pwm = constrain(motor_pwm, MOTOR_PWM_AT_MIN, MOTOR_PWM_AT_MAX);

  if (STOP == g_x_direction)
  {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }

  if (LEFT == g_x_direction)
  {
    ledcWrite(0, 0);
    ledcWrite(1, motor_pwm);
    //    Serial.println(g_x_requested_speed);
  }

  if (RIGHT == g_x_direction)
  {
    ledcWrite(0, motor_pwm);
    ledcWrite(1, 0);
    //    Serial.print("Setting right speed to: ");
    //    Serial.println(g_x_requested_speed);
  }
}

#endif H_MOTORS
