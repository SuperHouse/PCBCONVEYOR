#ifndef H_MQTT_COMMS
#define H_MQTT_COMMS

/**
  Reconnect to MQTT broker, and publish a notification to the status topic
*/

void reconnectMqtt()
{
#if ENABLE_WIFI
  char mqtt_client_id[23];
  sprintf(mqtt_client_id, "%s", g_device_id);
  Serial.print("MQTT client id: ");
  Serial.println(mqtt_client_id);

  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection to ");
    Serial.print(mqtt_broker);
    Serial.print(" as ");
    Serial.print(mqtt_client_id);
    Serial.print("... ");
    // Attempt to connect
    if (client.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement
      sprintf(g_mqtt_message_buffer, "Device %s starting up, version %s", mqtt_client_id, VERSION);
      client.publish(g_mqtt_tele_topic, g_mqtt_message_buffer);
      // Resubscribe
      client.subscribe(g_mqtt_command_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
#endif
}


/**
  This callback is invoked when an MQTT message is received.
*/
void callback(char* topic, byte* message, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
  }
  Serial.println();

  g_input_buffer = (char *)message;
  processGCodeMessage();
}

#endif H_MQTT_COMMS
