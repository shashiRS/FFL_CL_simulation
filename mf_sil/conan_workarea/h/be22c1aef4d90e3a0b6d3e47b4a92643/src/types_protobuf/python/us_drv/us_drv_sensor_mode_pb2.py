# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_drv/us_drv_sensor_mode.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_drv/us_drv_sensor_mode.proto',
  package='pb.us_drv.us_drv_sensor_mode',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1fus_drv/us_drv_sensor_mode.proto\x12\x1cpb.us_drv.us_drv_sensor_mode*\x7f\n\x0fUsDrvSensorMode\x12\x16\n\x12US_SENSOR_MODE_OFF\x10\x00\x12\x1a\n\x16US_SENSOR_MODE_STANDBY\x10\x01\x12\x19\n\x15US_SENSOR_MODE_NORMAL\x10\x02\x12\x1d\n\x19US_SENSOR_MODE_DIAGNOSTIC\x10\x03'
)

_USDRVSENSORMODE = _descriptor.EnumDescriptor(
  name='UsDrvSensorMode',
  full_name='pb.us_drv.us_drv_sensor_mode.UsDrvSensorMode',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='US_SENSOR_MODE_OFF', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_SENSOR_MODE_STANDBY', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_SENSOR_MODE_NORMAL', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_SENSOR_MODE_DIAGNOSTIC', index=3, number=3,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=65,
  serialized_end=192,
)
_sym_db.RegisterEnumDescriptor(_USDRVSENSORMODE)

UsDrvSensorMode = enum_type_wrapper.EnumTypeWrapper(_USDRVSENSORMODE)
US_SENSOR_MODE_OFF = 0
US_SENSOR_MODE_STANDBY = 1
US_SENSOR_MODE_NORMAL = 2
US_SENSOR_MODE_DIAGNOSTIC = 3


DESCRIPTOR.enum_types_by_name['UsDrvSensorMode'] = _USDRVSENSORMODE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)