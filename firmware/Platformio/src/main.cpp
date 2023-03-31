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
  bno055::init(false);
  UartComms_Init();
  LightBar_Init();

  InternalTemperature.begin(TEMPERATURE_NO_ADC_SETTING_CHANGES);

  // Clocks
  elapsedMillis LED_clock;
  uint8_t LED_state = HIGH;

  elapsedMillis print_clock;
  elapsedMillis joy_update_clock;
  elapsedMillis joy_comms_clock;
  elapsedMillis slam_data;
  elapsedMillis servo_clock;
  elapsedMillis gui_data_clock;

  // pre while loop code
  // LightBar_State(true);

  SLAM_Data * sd_local = UartComms_GetSLAMData();
  Joystick_Input * joy_local = UartComms_GetJoystick();

  uint16_t count = 0;

  bool reset_comms_status = false;



  while(1)
  {

    // loop_time = 0;


    if(LED_clock > 750 )
    {
      LED_clock -= 750;
      LED_state = !LED_state;
      digitalWrite(ONBOARD_LED_PIN, LED_state);
    }

    if(servo_clock > 300)
    {
      servo_clock -= 300;
      // Serial.printf("P: %u, T:%u, S:%u\n", CameraGimbal_Get_Pan(), CameraGimbal_Get_Tilt(), CameraGimbal_Get_Speed());
    }

    if(joy_comms_clock > 1)
    {
      joy_comms_clock -= 1;
      // Run serial comms to get in the data from the Jetson
      UartComms_RcvControls();

      if((*UartComms_GetTimeSinceLastRead()) > SERIAL_COMMS_RECEIVE_TIMEOUT)
      {
        reset_comms_status = true;
        UartComms_ClearJoystick();
      }
      else
      {
        reset_comms_status = false;
      }


      UartComms_ClearReadBuffer();
    }

    if (print_clock > 100)
    {
      // Joystick_Print();
      if(reset_comms_status)
      {
        Serial.println("Resetting comms, lost communication!");
      }
      print_clock -= 100;
    }
  
    // Run joystick at 20hz 
    if(joy_update_clock > 50)
    {
      joy_update_clock -= 50;
      Joystick_Store_State(joy_local);
      Joystick_Run();
    }

    if (slam_data > 10)
    {
      slam_data -= 10;
      // Serial.println("running slam loop");

      //store all the data
      bno055::get_euler_ypr(sd_local);
      bno055::get_lia_xyz(sd_local);
      CameraGimbal_StoreAngles(sd_local);

      // end
      UartComms_SendSLAMData();
      UartComms_ClearWriteBuffer();
      // bno055::print_calibration();
    }

    if(gui_data_clock > 200)
    {
      gui_data_clock -= 200;
      // this stuff can just be in uartcomms? the temp reading?
      // new joystick angles to be sent!
      float temp = InternalTemperature.readTemperatureF();
      // Serial.print("CPU TEMP: ");
      // Serial.println(temp);
      // UartComms_PopulateGUIReply(temp);
      // UartComms_PopulateGUIReply((float)++count);
      UartComms_PopulateGUIReply(temp);
      UartComms_SendGUIData();
      UartComms_ClearWriteBuffer();

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


