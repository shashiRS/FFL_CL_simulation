# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_common/driving_direction.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_common/driving_direction.proto',
  package='pb.ap_common.driving_direction',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n!ap_common/driving_direction.proto\x12\x1epb.ap_common.driving_direction*f\n\x10\x44rivingDirection\x12\x15\n\x11\x44IRECTION_UNKNOWN\x10\x00\x12\x0e\n\nSTANDSTILL\x10\x01\x12\x14\n\x10\x44RIVING_FORWARDS\x10\x02\x12\x15\n\x11\x44RIVING_BACKWARDS\x10\x03'
)

_DRIVINGDIRECTION = _descriptor.EnumDescriptor(
  name='DrivingDirection',
  full_name='pb.ap_common.driving_direction.DrivingDirection',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='DIRECTION_UNKNOWN', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STANDSTILL', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DRIVING_FORWARDS', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DRIVING_BACKWARDS', index=3, number=3,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=69,
  serialized_end=171,
)
_sym_db.RegisterEnumDescriptor(_DRIVINGDIRECTION)

DrivingDirection = enum_type_wrapper.EnumTypeWrapper(_DRIVINGDIRECTION)
DIRECTION_UNKNOWN = 0
STANDSTILL = 1
DRIVING_FORWARDS = 2
DRIVING_BACKWARDS = 3


DESCRIPTOR.enum_types_by_name['DrivingDirection'] = _DRIVINGDIRECTION
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
