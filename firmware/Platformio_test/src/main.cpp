#include <Arduino.h>

#include <locomotion.h>
#include <elapsedMillis.h>
#include <joystick.h>

void main_prog()
{

  //setup
  joystick_init();
  locomotion_init();

  // variables
  elapsedMillis LED_clock;
  uint8_t LED_state = HIGH;

  while(1)
  {
    if( LED_clock > 750 )
    {
      LED_clock -= 750;
      LED_state = !LED_state;
      digitalWrite(JOY_LED_PIN, LED_state);

    }
    
    if(dpad_down_pressed())
    {
      drive_backward();
    }
    else if(dpad_left_pressed())
    {
      drive_left();
    }
    else if(dpad_right_pressed())
    {
      drive_right(); 
    }
    else if(dpad_up_pressed())
    {
      drive_forward();
    }
    else
    {
      all_axis_off();
    }
  }
  
}


void setup() {
  // do not use this function, write setup code in main_prog
  return;
}

void loop() {
  // implement all code in main
  // main should never return
  main_prog();
}


