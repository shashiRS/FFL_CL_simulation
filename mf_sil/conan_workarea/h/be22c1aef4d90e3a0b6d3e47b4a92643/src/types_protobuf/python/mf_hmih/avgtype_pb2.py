# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/avgtype.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/avgtype.proto',
  package='pb.mf_hmih.avgtype',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x15mf_hmih/avgtype.proto\x12\x12pb.mf_hmih.avgtype*\xe4\x03\n\x07\x41VGType\x12\x0f\n\x0bNO_AVG_TYPE\x10\x00\x12\x19\n\x15PARK_IN_PARALLEL_LEFT\x10\x01\x12\x1a\n\x16PARK_IN_PARALLEL_RIGHT\x10\x02\x12\"\n\x1ePARK_IN_PERPENDICULAR_LEFT_FWD\x10\x03\x12\"\n\x1ePARK_IN_PERPENDICULAR_LEFT_BWD\x10\x04\x12#\n\x1fPARK_IN_PERPENDICULAR_RIGHT_FWD\x10\x05\x12#\n\x1fPARK_IN_PERPENDICULAR_RIGHT_BWD\x10\x06\x12\x1a\n\x16PARK_OUT_PARALLEL_LEFT\x10\x07\x12\x1b\n\x17PARK_OUT_PARALLEL_RIGHT\x10\x08\x12#\n\x1fPARK_OUT_PERPENDICULAR_LEFT_FWD\x10\t\x12#\n\x1fPARK_OUT_PERPENDICULAR_LEFT_BWD\x10\n\x12$\n PARK_OUT_PERPENDICULAR_RIGHT_FWD\x10\x0b\x12$\n PARK_OUT_PERPENDICULAR_RIGHT_BWD\x10\x0c\x12\x0f\n\x0bREM_MAN_FWD\x10\r\x12\x0f\n\x0bREM_MAN_BWD\x10\x0e\x12\x0e\n\nGP_GENREAL\x10\x0f'
)

_AVGTYPE = _descriptor.EnumDescriptor(
  name='AVGType',
  full_name='pb.mf_hmih.avgtype.AVGType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NO_AVG_TYPE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_IN_PARALLEL_LEFT', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_IN_PARALLEL_RIGHT', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_IN_PERPENDICULAR_LEFT_FWD', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_IN_PERPENDICULAR_LEFT_BWD', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_IN_PERPENDICULAR_RIGHT_FWD', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_IN_PERPENDICULAR_RIGHT_BWD', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_OUT_PARALLEL_LEFT', index=7, number=7,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_OUT_PARALLEL_RIGHT', index=8, number=8,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_OUT_PERPENDICULAR_LEFT_FWD', index=9, number=9,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_OUT_PERPENDICULAR_LEFT_BWD', index=10, number=10,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_OUT_PERPENDICULAR_RIGHT_FWD', index=11, number=11,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_OUT_PERPENDICULAR_RIGHT_BWD', index=12, number=12,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REM_MAN_FWD', index=13, number=13,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REM_MAN_BWD', index=14, number=14,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GP_GENREAL', index=15, number=15,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=46,
  serialized_end=530,
)
_sym_db.RegisterEnumDescriptor(_AVGTYPE)

AVGType = enum_type_wrapper.EnumTypeWrapper(_AVGTYPE)
NO_AVG_TYPE = 0
PARK_IN_PARALLEL_LEFT = 1
PARK_IN_PARALLEL_RIGHT = 2
PARK_IN_PERPENDICULAR_LEFT_FWD = 3
PARK_IN_PERPENDICULAR_LEFT_BWD = 4
PARK_IN_PERPENDICULAR_RIGHT_FWD = 5
PARK_IN_PERPENDICULAR_RIGHT_BWD = 6
PARK_OUT_PARALLEL_LEFT = 7
PARK_OUT_PARALLEL_RIGHT = 8
PARK_OUT_PERPENDICULAR_LEFT_FWD = 9
PARK_OUT_PERPENDICULAR_LEFT_BWD = 10
PARK_OUT_PERPENDICULAR_RIGHT_FWD = 11
PARK_OUT_PERPENDICULAR_RIGHT_BWD = 12
REM_MAN_FWD = 13
REM_MAN_BWD = 14
GP_GENREAL = 15


DESCRIPTOR.enum_types_by_name['AVGType'] = _AVGTYPE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
