#include <Arduino.h>

#include <locomotion.h>
#include <elapsedMillis.h>
#include <joystick.h>
#include <bno055.h>

void main_prog()
{

  // init the LED
  pinMode(13, OUTPUT);

  // Setup serial.
  Serial.begin(SERIAL_BAUD);

  // delay to allow for serial monitor setup and what not
  delay(5000);

  // setup
  joystick::init();
  locomotion_init();
  // bno055::init();

  // variables
  elapsedMillis LED_clock;
  uint8_t LED_state = HIGH;

  elapsedMillis print_clock;
  
  elapsedMillis joy_update_clock;

  while(1)
  {
    if(LED_clock > 750 )
    {
      LED_clock -= 750;
      LED_state = !LED_state;
      digitalWrite(JOY_LED_PIN, LED_state);
      digitalWrite(13, LED_state);
    }

    if (print_clock > 200)
    {
      joystick::print();
      print_clock -= 200;
    }

    // delay(100);
    // bno055::getTemp();

    if (joy_update_clock > 100)
    {
      joystick::store_joy_state();
      joy_update_clock -= 100;
    }

    joystick::run();

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


