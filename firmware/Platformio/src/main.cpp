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
  Command received_command;
  Reply outgoing_reply;
  
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


