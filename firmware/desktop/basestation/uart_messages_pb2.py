# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: uart_messages.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import builder as _builder
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x13uart_messages.proto\"\xd4\x03\n\x0eJoystick_Input\x12,\n\x06\x62utton\x18\x01 \x01(\x0e\x32\x17.Joystick_Input.ButtonsH\x00\x88\x01\x01\x12\x13\n\x06LJOY_X\x18\x02 \x01(\x11H\x01\x88\x01\x01\x12\x13\n\x06LJOY_Y\x18\x03 \x01(\x11H\x02\x88\x01\x01\x12\x13\n\x06RJOY_X\x18\x04 \x01(\x11H\x03\x88\x01\x01\x12\x13\n\x06RJOY_Y\x18\x05 \x01(\x11H\x04\x88\x01\x01\x12\x0f\n\x02TR\x18\x06 \x01(\x11H\x05\x88\x01\x01\x12\x0f\n\x02TL\x18\x07 \x01(\x11H\x06\x88\x01\x01\"\xd8\x01\n\x07\x42uttons\x12\x08\n\x04NONE\x10\x00\x12\t\n\x05\x42TN_A\x10\x01\x12\t\n\x05\x42TN_B\x10\x02\x12\t\n\x05\x42TN_X\x10\x04\x12\t\n\x05\x42TN_Y\x10\x08\x12\r\n\tBTN_START\x10\x10\x12\x0e\n\nBTN_SELECT\x10 \x12\x0b\n\x07\x44PAD_UP\x10@\x12\x0e\n\tDPAD_DOWN\x10\x80\x01\x12\x0e\n\tDPAD_LEFT\x10\x80\x02\x12\x0f\n\nDPAD_RIGHT\x10\x80\x04\x12\x0f\n\nBTN_THUMBR\x10\x80\x08\x12\x0f\n\nBTN_THUMBL\x10\x80\x10\x12\x0b\n\x06\x42TN_TR\x10\x80 \x12\x0b\n\x06\x42TN_TL\x10\x80@B\t\n\x07_buttonB\t\n\x07_LJOY_XB\t\n\x07_LJOY_YB\t\n\x07_RJOY_XB\t\n\x07_RJOY_YB\x05\n\x03_TRB\x05\n\x03_TL\"\xf5\x01\n\tSLAM_Data\x12\x12\n\x05lia_x\x18\x01 \x01(\x02H\x00\x88\x01\x01\x12\x12\n\x05lia_y\x18\x02 \x01(\x02H\x01\x88\x01\x01\x12\x12\n\x05lia_z\x18\x03 \x01(\x02H\x02\x88\x01\x01\x12\x12\n\x05\x65ul_y\x18\x04 \x01(\x02H\x03\x88\x01\x01\x12\x12\n\x05\x65ul_p\x18\x05 \x01(\x02H\x04\x88\x01\x01\x12\x12\n\x05\x65ul_r\x18\x06 \x01(\x02H\x05\x88\x01\x01\x12\x10\n\x03pan\x18\x07 \x01(\x11H\x06\x88\x01\x01\x12\x11\n\x04tilt\x18\x08 \x01(\x11H\x07\x88\x01\x01\x42\x08\n\x06_lia_xB\x08\n\x06_lia_yB\x08\n\x06_lia_zB\x08\n\x06_eul_yB\x08\n\x06_eul_pB\x08\n\x06_eul_rB\x06\n\x04_panB\x07\n\x05_tilt\"\xd7\x07\n\x08GUI_Data\x12\x15\n\x08\x63pu_temp\x18\x01 \x01(\x11H\x00\x88\x01\x01\x12\x19\n\x0c\x63\x61lib_status\x18\x02 \x01(\x11H\x01\x88\x01\x01\x12\x15\n\x08hts_temp\x18\x03 \x01(\x11H\x02\x88\x01\x01\x12\x19\n\x0chts_humidity\x18\x04 \x01(\x11H\x03\x88\x01\x01\x12\x16\n\tbatt_temp\x18\x05 \x01(\x11H\x04\x88\x01\x01\x12\x1b\n\x0e\x63urr_servo_pan\x18\x06 \x01(\x11H\x05\x88\x01\x01\x12\x1c\n\x0f\x63urr_servo_tilt\x18\x07 \x01(\x11H\x06\x88\x01\x01\x12\x1b\n\x0ehome_servo_pan\x18\x08 \x01(\x11H\x07\x88\x01\x01\x12\x1c\n\x0fhome_servo_tilt\x18\t \x01(\x11H\x08\x88\x01\x01\x12\x35\n\x0bloco_status\x18\n \x01(\x0e\x32\x1b.GUI_Data.Locomotion_StatusH\t\x88\x01\x01\x12\x30\n\ndev_status\x18\x0b \x01(\x0e\x32\x17.GUI_Data.Device_StatusH\n\x88\x01\x01\x12\x1b\n\x0ei2c_msg_failed\x18\x0c \x01(\x11H\x0b\x88\x01\x01\x12\x1b\n\x0ei2c_msg_passed\x18\r \x01(\x11H\x0c\x88\x01\x01\x12\x1c\n\x0fuart_msg_failed\x18\x0e \x01(\x11H\r\x88\x01\x01\x12\x1c\n\x0fuart_msg_passed\x18\x0f \x01(\x11H\x0e\x88\x01\x01\x12\x1b\n\x0elost_sync_byte\x18\x10 \x01(\x11H\x0f\x88\x01\x01\x12\x1b\n\x0ereseting_comms\x18\x11 \x01(\x11H\x10\x88\x01\x01\"L\n\x11Locomotion_Status\x12\x08\n\x04NONE\x10\x00\x12\x14\n\x10\x43ONTROLS_INHIBIT\x10\x01\x12\x17\n\x13NORMAL_DRIVING_MODE\x10\x02\"G\n\rDevice_Status\x12\n\n\x06NONE_1\x10\x00\x12\x14\n\x10\x42NO_INIT_SUCCESS\x10\x01\x12\x14\n\x10HTS_INIT_SUCCESS\x10\x02\x42\x0b\n\t_cpu_tempB\x0f\n\r_calib_statusB\x0b\n\t_hts_tempB\x0f\n\r_hts_humidityB\x0c\n\n_batt_tempB\x11\n\x0f_curr_servo_panB\x12\n\x10_curr_servo_tiltB\x11\n\x0f_home_servo_panB\x12\n\x10_home_servo_tiltB\x0e\n\x0c_loco_statusB\r\n\x0b_dev_statusB\x11\n\x0f_i2c_msg_failedB\x11\n\x0f_i2c_msg_passedB\x12\n\x10_uart_msg_failedB\x12\n\x10_uart_msg_passedB\x11\n\x0f_lost_sync_byteB\x11\n\x0f_reseting_commsb\x06proto3')

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'uart_messages_pb2', globals())
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _JOYSTICK_INPUT._serialized_start=24
  _JOYSTICK_INPUT._serialized_end=492
  _JOYSTICK_INPUT_BUTTONS._serialized_start=207
  _JOYSTICK_INPUT_BUTTONS._serialized_end=423
  _SLAM_DATA._serialized_start=495
  _SLAM_DATA._serialized_end=740
  _GUI_DATA._serialized_start=743
  _GUI_DATA._serialized_end=1726
  _GUI_DATA_LOCOMOTION_STATUS._serialized_start=1278
  _GUI_DATA_LOCOMOTION_STATUS._serialized_end=1354
  _GUI_DATA_DEVICE_STATUS._serialized_start=1356
  _GUI_DATA_DEVICE_STATUS._serialized_end=1427
# @@protoc_insertion_point(module_scope)
