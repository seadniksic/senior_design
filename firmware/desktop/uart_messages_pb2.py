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




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x13uart_messages.proto\"\xb7\x02\n\x0eJoystick_Input\x12\'\n\x06\x62utton\x18\x01 \x01(\x0e\x32\x17.Joystick_Input.Buttons\x12\x0c\n\x04LJOY\x18\x02 \x01(\x05\x12\x0c\n\x04RJOY\x18\x03 \x01(\x05\x12\n\n\x02TR\x18\x04 \x01(\x05\x12\n\n\x02TL\x18\x05 \x01(\x05\"\xc7\x01\n\x07\x42uttons\x12\t\n\x05\x42TN_A\x10\x00\x12\t\n\x05\x42TN_B\x10\x01\x12\t\n\x05\x42TN_X\x10\x02\x12\t\n\x05\x42TN_Y\x10\x03\x12\r\n\tBTN_START\x10\x04\x12\x0e\n\nBTN_SELECT\x10\x05\x12\x0b\n\x07\x44PAD_UP\x10\x06\x12\r\n\tDPAD_DOWN\x10\x07\x12\r\n\tDPAD_LEFT\x10\x08\x12\x0e\n\nDPAD_RIGHT\x10\t\x12\x0e\n\nBTN_THUMBR\x10\n\x12\x0e\n\nBTN_THUMBL\x10\x0b\x12\n\n\x06\x42TN_TR\x10\x0c\x12\n\n\x06\x42TN_TL\x10\r\"\x1a\n\x05Reply\x12\x11\n\tled_state\x18\x01 \x01(\x08\x62\x06proto3')

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'uart_messages_pb2', globals())
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _JOYSTICK_INPUT._serialized_start=24
  _JOYSTICK_INPUT._serialized_end=335
  _JOYSTICK_INPUT_BUTTONS._serialized_start=136
  _JOYSTICK_INPUT_BUTTONS._serialized_end=335
  _REPLY._serialized_start=337
  _REPLY._serialized_end=363
# @@protoc_insertion_point(module_scope)
