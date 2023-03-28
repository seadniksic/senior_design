#include <Arduino.h>
#include <Locomotion.h>
#include <elapsedMillis.h>
#include <Joystick.h>
#include <bno055.h>
#include <UartComms.h>
#include <InternalTemperature.h>
#include <LightBar.h>
#include <CameraGimbal.h>
#include "pinout.h"
#include "config.h"

#warning "it would be cool if could do like y to enter imu calib mode and then use led patterns to give the status on that"

void main_prog()
{

  // init the onboard LED
  pinMode(ONBOARD_LED_PIN, OUTPUT);

  // Setup serial
  Serial.begin(SERIAL_BAUD);

  // delay to allow for serial monitor connection
  delay(2000);

  // setup
  CameraGimbal_Init();
  Joystick_Init();
  Locomotion_Init();
  // bno055::init();
  UartComms_Init();
  LightBar_Init();

  InternalTemperature.begin(TEMPERATURE_NO_ADC_SETTING_CHANGES);

  // Clocks
  elapsedMillis LED_clock;
  uint8_t LED_state = HIGH;

  elapsedMillis print_clock;
  elapsedMillis joy_update_clock;
  elapsedMillis cpu_temp_clock;
  elapsedMillis comms_clock;
  elapsedMillis imu_data;
  elapsedMillis servo_clock;

  // pre while loop code
  // LightBar_State(true);


  while(1)
  {
    if(LED_clock > 750 )
    {
      LED_clock -= 750;
      LED_state = !LED_state;
      digitalWrite(ONBOARD_LED_PIN, LED_state);
    }

    if(servo_clock > 300)
    {
      servo_clock -= 300;
      Serial.printf("P: %u, T:%u, S:%u\n", CameraGimbal_Get_Pan(), CameraGimbal_Get_Tilt(), CameraGimbal_Get_Speed());

    }

    // test sending data back to the jetson
    if(cpu_temp_clock > 1000)
    {
      cpu_temp_clock -= 1000;
      float temp = InternalTemperature.readTemperatureF();
      // Serial.print("CPU TEMP: ");
      // Serial.println(temp);
      UartComms_PopulateGUIReply(temp);
    }

    if(comms_clock > 1)
    {
      comms_clock -= 1;
      // Run serial comms to get in the data from the Jetson
      UartComms_RcvControls();

      if((*UartComms_GetTimeSinceLastRead()) > SERIAL_COMMS_RECEIVE_TIMEOUT)
      {
        Serial.println("Resetting comms, lost communication!");
        UartComms_ClearJoystick();
      }

      UartComms_ClearBuffers();
    }

    if (print_clock > 100)
    {
      // Joystick_Print();
      print_clock -= 100;
    }
  
    // Run joystick at 20hz 
    if(joy_update_clock > 50)
    {
      joy_update_clock -= 50;
      Joystick_Store_State(UartComms_GetJoystick());
      Joystick_Run();
    }

    if (imu_data > 50)
    {
      imu_data -= 50;
      // bno055::get_euler_ypr();
      // bno055::get_lia_xyz();

      // bno055::print_calibration();
    }

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


