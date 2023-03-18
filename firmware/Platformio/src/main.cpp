#include <Arduino.h>
#include <Locomotion.h>
#include <elapsedMillis.h>
#include <Joystick.h>
#include <bno055.h>
#include <UartComms.h>


void main_prog()
{

  // init the LED
  pinMode(13, OUTPUT);

  // Setup serial
  Serial.begin(SERIAL_BAUD);

  // delay to allow for serial monitor setup and what not
  delay(2000);

  // setup
  Joystick_Init();
  Locomotion_Init();
  // bno055::init();
  UartComms_Init();

  // variables
  elapsedMillis LED_clock;
  uint8_t LED_state = HIGH;

  elapsedMillis print_clock;
  elapsedMillis joy_update_clock;

  //protobuf shit
  UartReadBuffer read_buffer;
  UartWriteBuffer write_buffer;
  Joystick_Input js_in;
  Reply outgoing_reply;
  elapsedMillis rcv_clock;

  while(1)
  {
    if(LED_clock > 750 )
    {
      LED_clock -= 750;
      LED_state = !LED_state;
      digitalWrite(JOY_LED_PIN, LED_state);
      digitalWrite(13, LED_state);
    }

    if (print_clock > 100)
    {
      Joystick_Print();
      print_clock -= 100;
    }


    UartComms_Run(read_buffer, write_buffer, js_in, outgoing_reply, rcv_clock);
    



  
    if(joy_update_clock > 100)
    {
      Joystick_Store_State(js_in);
      joy_update_clock -= 100;
      Joystick_Run();
    }

    // bno055::get_euler_ypr();
    // bno055::print_calibration();

    UartComms_ClearBuffers(read_buffer, write_buffer);


    delay(10);

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


