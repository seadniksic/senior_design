#include "UartComms.h"


static UartReadBuffer read_buffer;
static UartWriteBuffer write_buffer;
static Joystick_Input js_in;
static GUI_Data gui_data;
static IMU_Data imu_data;
static elapsedMillis rcv_clock;
static elapsedMillis commTimer;

static UartComms_t uartComms = {0};

void UartComms_Init()
{
    HWSERIAL.begin(115200);

    uartComms.read_buffer= &read_buffer;
    uartComms.write_buffer= &write_buffer;
    uartComms.js_in = &js_in;
    uartComms.gui_data = &gui_data;
    uartComms.rcv_clock = rcv_clock;
    uartComms.time_since_last_serialize = commTimer;
    uartComms.commsNeedReset = false;
}

// As of right now this is blocking, not ideal
#pragma message("should just put this into a struct and pass the struct.")

void UartComms_RcvControls()
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
        uartComms.rcv_clock = 0; //reset elaspedmillis
        while(HWSERIAL.available() < 0)
        {
          //stall till the next byte comes in
          if(uartComms.rcv_clock > TIMEOUT_NUMBYTES)
          {
            Serial.println("Message timeout while waiting for num_bytes!");
            return;
          }
        }

        // read the number of bytes
        num_bytes = HWSERIAL.read();

        uartComms.rcv_clock = 0; //reset elaspedmillis
        while(1)
        {
          // check for timeout.
          if(uartComms.rcv_clock > TIMEOUT_DATA)
          {
            Serial.println("Message timeout while waiting for data!");
            return;
          }

          // read the actual message.
          // you can use readbytes() to read multiple bytes!!
          if(num_bytes != 0 && HWSERIAL.available() > 0)
          {
            data = HWSERIAL.read();
            uartComms.read_buffer->push(data);
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
      auto deserialize_status = js_in.deserialize(*uartComms.read_buffer);
      if(::EmbeddedProto::Error::NO_ERRORS == deserialize_status)
      {
        uint32_t btn_status = (uint32_t)uartComms.js_in->get_button();
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

    }
}

void UartComms_PopulateGUIReply(const float &cpu_temp)
{
    uartComms.gui_data->clear();
    uartComms.gui_data->set_cpu_temp((int32_t)cpu_temp);
}

void UartComms_ClearBuffers()
{
    uartComms.read_buffer->clear();
    uartComms.write_buffer->clear();
}

void UartComms_ClearJoystick()
{
  uartComms.js_in->clear();
}

Joystick_Input* UartComms_GetJoystick()
{
  return uartComms.js_in;
}

elapsedMillis* UartComms_GetTimeSinceLastRead()
{
  return &uartComms.time_since_last_serialize;
}