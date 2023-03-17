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
  // joystick_init();
  locomotion_init();
  // bno055::init();
  UartComms_Init();

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

    
    UartComms_Run(read_buffer, write_buffer, js_in, outgoing_reply, rcv_clock);


    uint32_t btn_status = (uint32_t)js_in.get_button();

    if((btn_status >> 6) & 0x1)
    {
      drive_forward();
    }
    else if((btn_status >> 7) & 0x1)
    {
      drive_backward();
    }
    else if((btn_status >> 8) & 0x1)
    {
      drive_left();
    }
    else if(btn_status >> 9 & 0x1)
    {
      drive_right();
    }
    else{
      all_axis_off();
    }

    UartComms_ClearBuffers(read_buffer, write_buffer);
  

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


