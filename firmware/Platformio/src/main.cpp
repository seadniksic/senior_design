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

////////////////////////////
///////  GLOBAL VARS  //////
////////////////////////////

The_Watcher_t g_watcher = {0};

///////////////////////////
///   HELPER FUNCTIONS  ///
///////////////////////////

void Watcher_StoreData(GUI_Data * gd)
{
  uint32_t dev_status = ((uint32_t)g_watcher.bno_init) | \
                        (((uint32_t)g_watcher.hts_init) << 1);

  gd->set_dev_status((GUI_Data::Device_Status)dev_status);
  gd->set_i2c_msg_failed(g_watcher.i2c_msg_failed);
  gd->set_i2c_msg_passed(g_watcher.i2c_msg_passed);
  gd->set_uart_msg_failed(g_watcher.uart_msg_failed);
  gd->set_uart_msg_passed(g_watcher.uart_msg_passed);
  gd->set_lost_sync_byte(g_watcher.lost_sync_byte);
  gd->set_reseting_comms(g_watcher.resetting_comms);
  gd->set_uptime(g_watcher.uptime);
}



////////////////////////////
/////// MAIN PROGRAM  //////
////////////////////////////

void main_prog()
{
  // Initialize the onboard LED
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, HIGH);

  // Setup Serial Monitor
  Serial.begin(SERIAL_BAUD);

  // Delay to allow for serial monitor connection and monitoring
  delay(2000);

  // Init I2C for sensors
  // Master1 is defined in imx_rt1060_i2c_driver.cpp, pins 16 and 17 on teensy 4.1 
  I2CMaster & g_master = Master1; // declaring reference variable
  I2CDevice hts = I2CDevice(g_master, HTS221_I2CADDR, _LITTLE_ENDIAN);
  I2CDevice bno = I2CDevice(g_master, BNO_I2C_ADDRESS, _LITTLE_ENDIAN);
  I2C_Def_Init(g_master);

  // Initialize Modules
  CameraGimbal_Init();
  Joystick_Init();
  Locomotion_Init();
  g_watcher.bno_init = bno055::init(&bno, false);
  g_watcher.hts_init = HTS221_Init(&hts);
  UartComms_Init();
  LightBar_Init();
  CPUTemp_Init();

  // Task Scheduling
  elapsedMillis heartbeat;
  elapsedMillis print_clock;
  elapsedMillis joy_update_clock;
  elapsedMillis joy_comms_clock;
  elapsedMillis slam_data;
  elapsedMillis servo_update_clock;
  elapsedMillis gui_data_clock;

  // Variables
  elapsedMillis uptime_clock;
  SLAM_Data * slam_local = UartComms_GetSLAMData();
  Joystick_Input * joy_local = UartComms_GetJoystick();
  GUI_Data * gui_local = UartComms_GetGUIData();

  bool reset_comms_status = false;
  uint8_t LED_state = HIGH;


  while(1)
  {

    if(!g_watcher.bno_init || !g_watcher.hts_init)
    {
      if(heartbeat > TS_HEARTBEAT_DESTRESS)
      {
        heartbeat -= TS_HEARTBEAT_DESTRESS;

        LED_state = !LED_state;
        digitalWrite(ONBOARD_LED_PIN, LED_state);
      }
    }
    else
    {
      if(heartbeat > TS_HEARTBEAT)
      {
        heartbeat -= TS_HEARTBEAT;

        LED_state = !LED_state;
        digitalWrite(ONBOARD_LED_PIN, LED_state);
      }
    }

    if(joy_comms_clock > TS_JOY_COMMS)
    {
      joy_comms_clock -= TS_JOY_COMMS;

      UartComms_RcvControls();

      // Reset joystick only once after it looses comms
      if(((*UartComms_GetTimeSinceLastRead()) > SERIAL_COMMS_RECEIVE_TIMEOUT) && reset_comms_status == false)
      {
        reset_comms_status = true;
        g_watcher.resetting_comms++;
        UartComms_ClearJoystick();
      }

      // Clear flag if comms restablished
      if(((*UartComms_GetTimeSinceLastRead()) <= SERIAL_COMMS_RECEIVE_TIMEOUT) && reset_comms_status == true)
      {
        reset_comms_status = false;
      }

      UartComms_ClearReadBuffer();
    }

    if (print_clock > TS_PRINT_1)
    {
      print_clock -= TS_PRINT_1;

      // Joystick_Print();

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

    if(gui_data_clock > TS_GUI_DATA)
    {
      gui_data_clock -= TS_GUI_DATA;

      // Populate the Data
      UartComms_PopulateGUITempCPU(InternalTemperature.readTemperatureF());
      CameraGimbal_StoreAngles(gui_local);
      CameraGimbal_StoreHomeAngles(gui_local);
      bno055::store_calib_status(gui_local);
      Joystick_Store_Control_State(gui_local);
      HTS221_ReadData(gui_local);
      g_watcher.uptime = ((uint32_t)uptime_clock) / 1000;
      Watcher_StoreData(gui_local);
      
      // Send the data
      UartComms_SendGUIData();

      // Clear the buffer
      UartComms_ClearWriteBuffer();
    }

    if(servo_update_clock > TS_SERVO_UPDATE)
    {
      servo_update_clock -= TS_SERVO_UPDATE;
      CameraGimbal_Run();
    }

  }  
}

///////////////////////////
/// SILLY ARDUINO STUFF ///
///////////////////////////

void setup() {
  // do not use this function, write setup code in main_prog
  return;
}

void loop() {
  // implement all code in main 
  
  main_prog(); // main_prog should never return
}


