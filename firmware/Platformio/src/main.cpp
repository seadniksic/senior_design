#include <Arduino.h>
#include "config.h"
#include "pinout.h"
#include <Locomotion.h>
#include <elapsedMillis.h>
#include <Joystick.h>
#include <bno055.h>
#include <UartComms.h>
#include <InternalTemperature.h>
#include <LightBar.h>
#include <CameraGimbal.h>
#include "main.h"
#include <I2C_Def.h>
#include <HTS221.h>


void main_prog()
{
  // Initialize the onboard LED
  pinMode(ONBOARD_LED_PIN, OUTPUT);

  // Setup Serial Monitor
  Serial.begin(SERIAL_BAUD);

  // Delay to allow for serial monitor connection and monitoring
  delay(2000);

  // Init I2C for sensors
  // I2C_Def_Init();



  // Initialize Modules
  CameraGimbal_Init();
  Joystick_Init();
  Locomotion_Init();
  // bno055::init(I2C_Def_GetBnoDev(), false);
  HTS221_Init();
  UartComms_Init();
  LightBar_Init();
  CPUTemp_Init();

  // Task Scheduling
  elapsedMillis heartbeat;
  elapsedMillis print_clock;
  elapsedMillis joy_update_clock;
  elapsedMillis joy_comms_clock;
  elapsedMillis slam_data;
  elapsedMillis servo_clock;
  elapsedMillis gui_data_clock;

  // Variables
  SLAM_Data * slam_local = UartComms_GetSLAMData();
  Joystick_Input * joy_local = UartComms_GetJoystick();
  GUI_Data * gui_local = UartComms_GetGUIData();

  bool reset_comms_status = false;
  uint8_t LED_state = HIGH;


  // Locomotion_Run();  // Main while loop

  while(1)
  {

    if(heartbeat > TS_HEARTBEAT )
    {
      heartbeat -= TS_HEARTBEAT;

      LED_state = !LED_state;
      digitalWrite(ONBOARD_LED_PIN, LED_state);
    }

# if 0

    if(joy_comms_clock > TS_JOY_COMMS)
    {
      joy_comms_clock -= TS_JOY_COMMS;

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

    if (print_clock > TS_PRINT_1)
    {
      print_clock -= TS_PRINT_1;

      Joystick_Print();

      if(reset_comms_status)
      {
        Serial.println("Resetting comms, lost communication!");
      }
    }
  
    if(joy_update_clock > TS_JOY_UPDATE)
    {
      joy_update_clock -= TS_JOY_UPDATE;

      Joystick_Store_State(joy_local);
      Joystick_Run();
    }

    if (slam_data > TS_SLAM_COMMS)
    {
      slam_data -= TS_SLAM_COMMS;
      // Serial.println("running slam loop");

      //Store all the data
      bno055::get_euler_ypr(slam_local);
      bno055::get_lia_xyz(slam_local);
      CameraGimbal_StoreAngles(slam_local);

      // Send the data 
      UartComms_SendSLAMData();

      // Clear the write buffers
      UartComms_ClearWriteBuffer();
    }

    #endif

    if(gui_data_clock > TS_GUI_DATA)
    {
      gui_data_clock -= TS_GUI_DATA;

      // Populate the Data
      // UartComms_PopulateGUITempCPU(InternalTemperature.readTemperatureF());
      // CameraGimbal_StoreAngles(gui_local);
      // CameraGimbal_StoreHomeAngles(gui_local);
      // bno055::store_calib_status(gui_local);
      // Joystick_Store_Control_State(gui_local);
      HTS221_ReadData(gui_local);
      
      // Send the data
      // UartComms_SendGUIData();

      // Clear the buffer
      // UartComms_ClearWriteBuffer();
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


