#include <Arduino.h>
#include <Locomotion.h>
#include <elapsedMillis.h>
#include <Joystick.h>
#include <bno055.h>
#include <UartComms.h>
#include <InternalTemperature.h>
#include <Lightbar.h>
#include "pinout.h"
#include "config.h"


void main_prog()
{

  // init the onboard LED
  pinMode(ONBOARD_LED_PIN, OUTPUT);

  // Setup serial
  Serial.begin(SERIAL_BAUD);

  // delay to allow for serial monitor connection
  delay(2000);

  // setup
  Joystick_Init();
  Locomotion_Init();
  // bno055::init();
  UartComms_Init();
  LightBar_Init();
  InternalTemperature.begin(TEMPERATURE_NO_ADC_SETTING_CHANGES);

  // variables
  uint8_t lightbar_pwm;

  elapsedMillis LED_clock;
  uint8_t LED_state = HIGH;

  elapsedMillis print_clock;
  elapsedMillis joy_update_clock;
  elapsedMillis cpu_temp_clock;
  elapsedMillis comms_clock;

  //protobuf
  UartReadBuffer read_buffer;
  UartWriteBuffer write_buffer;
  Joystick_Input js_in;
  GUI_Data gui_data;
  elapsedMillis rcv_clock;

  // pre while loop code
  LightBar_State(true);

  /* serovs */
  pinMode(PAN_SERVO_PIN, OUTPUT);
  pinMode(TILT_SERVO_PIN, OUTPUT);
  uint8_t servo_pan = 0;
  uint8_t servo_tilt = 0;

  // analogWrite(PAN_SERVO_PIN, 255);
  // analogWrite(TILT_SERVO_PIN, 255);

  for(uint8_t i = 0; i <= 255/5; i++)
  {
    analogWrite(PAN_SERVO_PIN, servo_pan);
    delay(100);
    servo_pan+=5;
  }

  // while(1)
  // {

  //   analogWrite(PAN_SERVO_PIN, 0);
  //   delay(1000);
  //   analogWrite(PAN_SERVO_PIN, 90);
  //   delay(1000);
  //   analogWrite(PAN_SERVO_PIN, 180);
  //   delay(1000);
  // }


  while(1)
  {
    if(LED_clock > 750 )
    {
      LED_clock -= 750;
      LED_state = !LED_state;
      digitalWrite(ONBOARD_LED_PIN, LED_state);
    }

    // test sending data back to the jetson
    if(cpu_temp_clock > 1000)
    {
      cpu_temp_clock -= 1000;
      float temp = InternalTemperature.readTemperatureF();
      Serial.print("CPU TEMP: ");
      Serial.println(temp);
      UartComms_PopulateReply(gui_data, temp);
    }

    if(comms_clock > 1)
    {
      comms_clock -= 1;
      // Run serial comms to get in the data from the Jetson
      UartComms_Run(read_buffer, write_buffer, js_in, gui_data, rcv_clock);

      if((*UartComms_GetTimeSinceLastRead()) > SERIAL_COMMS_RECEIVE_TIMEOUT)
      {
        // Serial.println("Resetting comms, lost communication!");
        js_in.clear();
      }

      UartComms_ClearBuffers(read_buffer, write_buffer);
    }

    if (print_clock > 100)
    {
      Joystick_Print();
      print_clock -= 100;
    }
  
    // Run joystick at 20hz 
    if(joy_update_clock > 50)
    {
      joy_update_clock -= 50;
      Joystick_Store_State(js_in);
      Joystick_Run();
    }

    // bno055::get_euler_ypr();
    // bno055::print_calibration();


    

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


