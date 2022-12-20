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

    // print back for debugging
    //#ifdef DEBUG
    Serial.print(receivedChar);
    //#endif

    // Add character to buffer
    g_input_buffer += receivedChar;

    // If the received character is a newline, processGCodeMessage
    if (receivedChar == '\n')
    {
      processGCodeMessage();
      //clear buffer
      g_input_buffer = "";
    }
  }
}

#endif H_SERIAL_COMMS
