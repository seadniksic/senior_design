#include <Arduino.h>
#include <MessageInterface.h>
#include <Fields.h>
#include <ReadBufferSection.h>
#include <locomotion.h>
#include <elapsedMillis.h>
#include <joystick.h>
#include <bno055.h>
#include <UartReadBuffer.h>
#include <UartWriteBuffer.h>
#include "uart_messages.h"

#define HWSERIAL Serial2 // pins 7 and 8

void main_prog()
{

  // init the LED
  pinMode(13, OUTPUT);

  // Setup serial
  Serial.begin(SERIAL_BAUD);

  // delay to allow for serial monitor setup and what not
  delay(2000);

  // setup
  // joystick_init();
  // locomotion_init();
  // bno055::init();
  HWSERIAL.begin(115200);

  // variables
  elapsedMillis LED_clock;
  uint8_t LED_state = HIGH;

  elapsedMillis print_clock;

  //protobuf shit
  UartReadBuffer read_buffer;
  UartWriteBuffer write_buffer;
  Joystick_Input js_in;
  Reply outgoing_reply;
  elapsedMillis rcv_clock;
  
  bool print_once = false;

  while(1)
  {
    if(LED_clock > 750 )
    {
      LED_clock -= 750;
      LED_state = !LED_state;
      // digitalWrite(JOY_LED_PIN, LED_state);
      digitalWrite(13, LED_state);
    }

    // if (print_clock > 200)
    // {
    //   joystick_print();
    //   print_clock -= 200;
    // }

    // delay(100);
    // bno055::get_euler_ypr();
    // bno055::print_calibration();

    // protobuf
    // now the first byte will be a special sync byte indicating start of message. 
    #define SYNC_BYTE 0x64
    #define TIMEOUT 30 //ms
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
        while(HWSERIAL.available() < 0)
        {
          //stall till the next byte comes in
        }

        // read the number of bytes
        num_bytes = HWSERIAL.read();

        rcv_clock = 0; //reset elaspedmillis
        while(1)
        {
          // check for timeout.
          if(rcv_clock > TIMEOUT)
          {
            Serial.println("Message timeout!");
            break;
          }

          // read the actual message.
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
        // Serial.println(btn_status);
        // returns enum class Buttons derived from uint32_t type.
        // so should be able to bit shift.
        // theres 14 fields in buttons
        for(uint8_t i = 0; i < 14; i++)
        {
          if((btn_status >> i) & 0x1)
          {
            Serial.printf("%u: 1, ", i);
          }
          else
          {
            Serial.printf("%u: 0, ", i);
          }
        }
        // Serial.println();

        int16_t LJOY_X = (int16_t)js_in.get_LJOY_X();
        int16_t LJOY_Y = (int16_t)js_in.get_LJOY_Y();
        int16_t RJOY_X = (int16_t)js_in.get_RJOY_X();
        int16_t RJOY_Y = (int16_t)js_in.get_RJOY_Y();
        uint8_t TR = (uint8_t)js_in.get_TR();
        uint8_t TL = (uint8_t)js_in.get_TL();

        Serial.printf("LJOY_X: %6.0d, LJOY_Y: %6.0d, RJOY_X: %6.0d, RJOY_Y: %6.0d, TR: %u, TL: %u", LJOY_X, LJOY_Y, RJOY_X, RJOY_Y, TR, TL);
        Serial.println();

      }
      else
      {
        Serial.println("Failed to serialize message.");
      }

      available_packet = false;

    }









    #if 0
    uint8_t data;

    // read all of the data
    while(HWSERIAL.available() > 0)
    {
      // Serial.println("reading in a byte of data");
      print_once = false;
      data = HWSERIAL.read();
      read_buffer.push(data);
    }

    // this is just doing recieve for now
    auto deserialize_status = received_command.deserialize(read_buffer);
    
    if(::EmbeddedProto::Error::NO_ERRORS == deserialize_status)
    {
      uint32_t rcv_value = received_command.get_value(); // get the value
      
      if( print_once == false)
        Serial.println(rcv_value); // send the value over serial monitor

      print_once = true;
    }
    #endif

    read_buffer.clear();
    write_buffer.clear();

    delay(10);

    // joystick_run();

  }
  
}


void setup() {
  // do not use this function, write setup code in main_prog
  return;
}

void loop() {
  // implement all code in main
  
  main_prog(); // main_prog should never return
}


