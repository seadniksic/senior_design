#include "UartComms.h"


static UartComms_t uartComms = {0};
static elapsedMillis commTimer;

void UartComms_Init()
{
    HWSERIAL.begin(115200);
    uartComms.time_since_last_serialize = commTimer;
    uartComms.commsNeedReset = false;
}

// As of right now this is blocking, not ideal
#pragma message("should just put this into a struct and pass the struct.")

void UartComms_Run(UartReadBuffer &read_buffer, UartWriteBuffer &write_buffer, \
    Joystick_Input &js_in, GUI_Data &gui_data, elapsedMillis &rcv_clock)
{
    // protobuf
    // now the first byte will be a special sync byte indicating start of message. 
    // the second byte will be the number of messages

    uint8_t num_bytes = 0;
    uint8_t data;
    bool available_packet = false;
    
    if(HWSERIAL.available() > 0)
    {
      data = HWSERIAL.read();
      if(data == SYNC_BYTE)
      {
        // found start of message
        // read number of bytes to read
        rcv_clock = 0; //reset elaspedmillis
        while(HWSERIAL.available() < 0)
        {
          //stall till the next byte comes in
          if(rcv_clock > TIMEOUT_NUMBYTES)
          {
            Serial.println("Message timeout while waiting for num_bytes!");
            return;
          }
        }

        // read the number of bytes
        num_bytes = HWSERIAL.read();

        rcv_clock = 0; //reset elaspedmillis
        while(1)
        {
          // check for timeout.
          if(rcv_clock > TIMEOUT_DATA)
          {
            Serial.println("Message timeout while waiting for data!");
            return;
          }

          // read the actual message.
          // you can use readbytes() to read multiple bytes!!
          if(num_bytes != 0 && HWSERIAL.available() > 0)
          {
            data = HWSERIAL.read();
            read_buffer.push(data);
            if(--num_bytes == 0)
            {
              available_packet = true;
              break;
            } 
          }
        }
      }
      else
      {
        Serial.println("sync byte not found yet.");
      }
    }

    if(available_packet)
    {
      auto deserialize_status = js_in.deserialize(read_buffer);
      if(::EmbeddedProto::Error::NO_ERRORS == deserialize_status)
      {
        uint32_t btn_status = (uint32_t)js_in.get_button();
        uartComms.time_since_last_serialize = 0;


        // Serial.println(btn_status);
        // returns enum class Buttons derived from uint32_t type.
        // so should be able to bit shift.
        // theres 14 fields in buttons
        // going to 16 to just print all 2 bytes
        #ifdef PRINT_MSG

        for(uint8_t i = 0; i < 16; i++)
        {
          if(i % 4 == 0)
          {
            Serial.print(" ");
          }
          if((btn_status >> i) & 0x1)
          {
            // Serial.printf("%u: 1, ", i);
            Serial.print("1");
          }
          else
          {
            // Serial.printf("%u: 0, ", i);
            Serial.print("0");
          }
        }

        int16_t LJOY_X = (int16_t)js_in.get_LJOY_X();
        int16_t LJOY_Y = (int16_t)js_in.get_LJOY_Y();
        int16_t RJOY_X = (int16_t)js_in.get_RJOY_X();
        int16_t RJOY_Y = (int16_t)js_in.get_RJOY_Y();
        uint8_t TR = (uint8_t)js_in.get_TR();
        uint8_t TL = (uint8_t)js_in.get_TL();

        // Serial.println();
        Serial.printf(", LJOY_X: %6.0d, LJOY_Y: %6.0d, ", LJOY_X, LJOY_Y);
        Serial.printf("RJOY_X: %6.0d, RJOY_Y: %6.0d, ", RJOY_X, RJOY_Y);
        Serial.printf("TR: %3u, TL: %3u", TR, TL);
        Serial.println();

        #endif

        // only transmit if we have recieved a message, for now
        // #warning "we dont need to send data back at this rate, so split up this function"
        // auto serialization_status = gui_data.serialize(write_buffer);
        // if(::EmbeddedProto::Error::NO_ERRORS == serialization_status)
        // {
        //   // Serial write docs
        //   // https://cdn.arduino.cc/reference/en/language/functions/communication/serial/write/
        //   // Serial.println("Writing data");
        //   HWSERIAL.write(SYNC_BYTE_WRITE);


        //   // first transmit the number of bytes in the message.
        //   const uint8_t n_bytes = write_buffer.get_size();
        //   HWSERIAL.write(n_bytes);

        //   // Now transmit the actual data.
        //   HWSERIAL.write(write_buffer.get_data(), write_buffer.get_size());
        // }

      }
      else
      {
        Serial.println("Failed to serialize message.");
        HWSERIAL.flush();
      }

      available_packet = false;
    }
}

void UartComms_PopulateReply(GUI_Data &gui_data, const float &cpu_temp)
{
    gui_data.clear();
    gui_data.set_cpu_temp((int32_t)cpu_temp);
}

void UartComms_ClearBuffers(UartReadBuffer &read_buffer, UartWriteBuffer &write_buffer)
{
    read_buffer.clear();
    write_buffer.clear();
}

elapsedMillis* UartComms_GetTimeSinceLastRead()
{
  return &uartComms.time_since_last_serialize;
}