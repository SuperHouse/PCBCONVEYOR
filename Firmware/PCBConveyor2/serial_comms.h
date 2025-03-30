#ifndef H_SERIAL_COMMS
#define H_SERIAL_COMMS

/*

*/
void listenToSerialStream()
{
  while (Serial.available())
  {
    // Get the received byte, convert to char for adding to buffer
    char receivedChar = (char)Serial.read();

#if SERIAL_DEBUGGING
    Serial.print(receivedChar);
#endif

    g_input_buffer += receivedChar;

    // If the received character is a newline, processGCodeMessage
    if (receivedChar == '\n')
    {
      processGCodeMessage();
      g_input_buffer = "";
    }
  }
}

#endif H_SERIAL_COMMS
