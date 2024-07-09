# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_manager/active_maneuvering_function.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_manager/active_maneuvering_function.proto',
  package='pb.mf_manager.active_maneuvering_function',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n,mf_manager/active_maneuvering_function.proto\x12)pb.mf_manager.active_maneuvering_function*\x8a\x02\n\x19\x41\x63tiveManeuveringFunction\x12\x11\n\rFUNCTION_NONE\x10\x00\x12\x1e\n\x1a\x46UNCTION_COLLISION_WARNING\x10\x01\x12\x11\n\rFUNCTION_LSCA\x10\x02\x12\x1e\n\x1a\x46UNCTION_AUTOMATED_PARKING\x10\x03\x12!\n\x1d\x46UNCTION_TRAILER_HITCH_ASSIST\x10\x04\x12#\n\x1f\x46UNCTION_TRAILER_REVERSE_ASSIST\x10\x05\x12\x1c\n\x18\x46UNCTION_TRAINED_PARKING\x10\x06\x12!\n\x1dMAX_NUM_MANEUVERING_FUNCTIONS\x10\x07'
)

_ACTIVEMANEUVERINGFUNCTION = _descriptor.EnumDescriptor(
  name='ActiveManeuveringFunction',
  full_name='pb.mf_manager.active_maneuvering_function.ActiveManeuveringFunction',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='FUNCTION_NONE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FUNCTION_COLLISION_WARNING', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FUNCTION_LSCA', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FUNCTION_AUTOMATED_PARKING', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FUNCTION_TRAILER_HITCH_ASSIST', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FUNCTION_TRAILER_REVERSE_ASSIST', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FUNCTION_TRAINED_PARKING', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MAX_NUM_MANEUVERING_FUNCTIONS', index=7, number=7,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=92,
  serialized_end=358,
)
_sym_db.RegisterEnumDescriptor(_ACTIVEMANEUVERINGFUNCTION)

ActiveManeuveringFunction = enum_type_wrapper.EnumTypeWrapper(_ACTIVEMANEUVERINGFUNCTION)
FUNCTION_NONE = 0
FUNCTION_COLLISION_WARNING = 1
FUNCTION_LSCA = 2
FUNCTION_AUTOMATED_PARKING = 3
FUNCTION_TRAILER_HITCH_ASSIST = 4
FUNCTION_TRAILER_REVERSE_ASSIST = 5
FUNCTION_TRAINED_PARKING = 6
MAX_NUM_MANEUVERING_FUNCTIONS = 7


DESCRIPTOR.enum_types_by_name['ActiveManeuveringFunction'] = _ACTIVEMANEUVERINGFUNCTION
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)