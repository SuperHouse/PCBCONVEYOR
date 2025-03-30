#ifndef H_CAN_COMMS
#define H_CAN_COMMS

/**
   Process incoming CAN frames
*/
void readCANMessages()
{
  uint8_t packet_size = CAN.parsePacket();
  if (packet_size)
  {
    uint16_t j1939_source_address = CAN.packetId() & 0xFF; // Not being used yet!

    char can_payload[8];
    g_input_buffer = "";
    while (CAN.available())
    {
      g_input_buffer += (char)CAN.read();
    }

#if CAN_DEBUGGING
    Serial.println();
    Serial.print("Source: 0x");
    Serial.println(j1939_source_address, HEX);

    Serial.print("B:>");
    Serial.print(g_input_buffer);
    Serial.println("<");
#endif
    // Sanity check the message?
    // Temporary filter to remove T-Rex MCM status messages
    if (j1939_source_address != 0x80)
    {
      processGCodeMessage();
    }
  }
}

#endif H_CAN_COMMS
