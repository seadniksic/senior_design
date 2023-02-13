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

  elapsedMillis print_clock;
  Serial.begin(115200);

  while(1)
  {
    if(LED_clock > 750 )
    {
      LED_clock -= 750;
      LED_state = !LED_state;
      digitalWrite(JOY_LED_PIN, LED_state);
    }

    // if (print_clock > 200)
    // {
    //   joystick_print();
    //   print_clock -= 200;
    // }

    delay(100);

    joystick_run();

    


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


