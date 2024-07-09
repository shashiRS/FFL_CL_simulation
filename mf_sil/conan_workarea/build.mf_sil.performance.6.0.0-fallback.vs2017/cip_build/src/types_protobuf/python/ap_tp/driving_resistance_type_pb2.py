# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_tp/driving_resistance_type.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_tp/driving_resistance_type.proto',
  package='pb.ap_tp.driving_resistance_type',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n#ap_tp/driving_resistance_type.proto\x12 pb.ap_tp.driving_resistance_type*\x9f\x01\n\x15\x44rivingResistanceType\x12\x08\n\x04NONE\x10\x00\x12\x0f\n\x0b\x46\x41LLING_LOW\x10\x01\x12\x12\n\x0e\x46\x41LLING_MEDIUM\x10\x02\x12\x10\n\x0c\x46\x41LLING_HIGH\x10\x03\x12\x0e\n\nRISING_LOW\x10\x04\x12\x11\n\rRISING_MEDIUM\x10\x05\x12\x0f\n\x0bRISING_HIGH\x10\x06\x12\x11\n\rWHEEL_STOPPER\x10\x07'
)

_DRIVINGRESISTANCETYPE = _descriptor.EnumDescriptor(
  name='DrivingResistanceType',
  full_name='pb.ap_tp.driving_resistance_type.DrivingResistanceType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NONE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FALLING_LOW', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FALLING_MEDIUM', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FALLING_HIGH', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RISING_LOW', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RISING_MEDIUM', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RISING_HIGH', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='WHEEL_STOPPER', index=7, number=7,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=74,
  serialized_end=233,
)
_sym_db.RegisterEnumDescriptor(_DRIVINGRESISTANCETYPE)

DrivingResistanceType = enum_type_wrapper.EnumTypeWrapper(_DRIVINGRESISTANCETYPE)
NONE = 0
FALLING_LOW = 1
FALLING_MEDIUM = 2
FALLING_HIGH = 3
RISING_LOW = 4
RISING_MEDIUM = 5
RISING_HIGH = 6
WHEEL_STOPPER = 7


DESCRIPTOR.enum_types_by_name['DrivingResistanceType'] = _DRIVINGRESISTANCETYPE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
