# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_trjctl/moco_te_fun_mode.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_trjctl/moco_te_fun_mode.proto',
  package='pb.ap_trjctl.moco_te_fun_mode',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n ap_trjctl/moco_te_fun_mode.proto\x12\x1dpb.ap_trjctl.moco_te_fun_mode*\xbb\x02\n\x0fMOCO_te_FunMode\x12\x19\n\x15MOCO_FUN_MODE_INVALID\x10\x00\x12\x18\n\x14MOCO_FUN_MODE_CRUISE\x10\x01\x12\x1c\n\x18MOCO_FUN_MODE_CRUISE_LAT\x10\x02\x12\x1d\n\x19MOCO_FUN_MODE_CRUISE_LONG\x10\x03\x12\x1b\n\x17MOCO_FUN_MODE_SPEED_LIM\x10\x04\x12!\n\x1dMOCO_FUN_MODE_SAFETY_LONG_AEB\x10\x05\x12!\n\x1dMOCO_FUN_MODE_SAFETY_LONG_VDS\x10\x06\x12\x1c\n\x18MOCO_FUN_MODE_SAFETY_LAT\x10\x07\x12\x1a\n\x16MOCO_FUN_MODE_MANEUVER\x10\x08\x12\x19\n\x15MOCO_FUN_MODE_OFFROAD\x10\t'
)

_MOCO_TE_FUNMODE = _descriptor.EnumDescriptor(
  name='MOCO_te_FunMode',
  full_name='pb.ap_trjctl.moco_te_fun_mode.MOCO_te_FunMode',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='MOCO_FUN_MODE_INVALID', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MOCO_FUN_MODE_CRUISE', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MOCO_FUN_MODE_CRUISE_LAT', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MOCO_FUN_MODE_CRUISE_LONG', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MOCO_FUN_MODE_SPEED_LIM', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MOCO_FUN_MODE_SAFETY_LONG_AEB', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MOCO_FUN_MODE_SAFETY_LONG_VDS', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MOCO_FUN_MODE_SAFETY_LAT', index=7, number=7,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MOCO_FUN_MODE_MANEUVER', index=8, number=8,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MOCO_FUN_MODE_OFFROAD', index=9, number=9,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=68,
  serialized_end=383,
)
_sym_db.RegisterEnumDescriptor(_MOCO_TE_FUNMODE)

MOCO_te_FunMode = enum_type_wrapper.EnumTypeWrapper(_MOCO_TE_FUNMODE)
MOCO_FUN_MODE_INVALID = 0
MOCO_FUN_MODE_CRUISE = 1
MOCO_FUN_MODE_CRUISE_LAT = 2
MOCO_FUN_MODE_CRUISE_LONG = 3
MOCO_FUN_MODE_SPEED_LIM = 4
MOCO_FUN_MODE_SAFETY_LONG_AEB = 5
MOCO_FUN_MODE_SAFETY_LONG_VDS = 6
MOCO_FUN_MODE_SAFETY_LAT = 7
MOCO_FUN_MODE_MANEUVER = 8
MOCO_FUN_MODE_OFFROAD = 9


DESCRIPTOR.enum_types_by_name['MOCO_te_FunMode'] = _MOCO_TE_FUNMODE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)