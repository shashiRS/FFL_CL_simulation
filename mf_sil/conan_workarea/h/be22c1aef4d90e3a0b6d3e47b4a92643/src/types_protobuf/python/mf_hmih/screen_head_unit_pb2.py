# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/screen_head_unit.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/screen_head_unit.proto',
  package='pb.mf_hmih.screen_head_unit',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1emf_hmih/screen_head_unit.proto\x12\x1bpb.mf_hmih.screen_head_unit*\xbe\x04\n\x0eScreenHeadUnit\x12\x15\n\x11SYSTEM_NOT_ACTIVE\x10\x00\x12\x13\n\x0fSCANNING_ACTIVE\x10\x01\x12\x1b\n\x17PARKING_SPACE_SELECTION\x10\x02\x12\x15\n\x11REMOTE_APP_ACTIVE\x10\x03\x12\x13\n\x0fMANEUVER_ACTIVE\x10\x04\x12\x15\n\x11MANEUVER_FINISHED\x10\x05\x12\x18\n\x14MANEUVER_INTERRUPTED\x10\x06\x12\x18\n\x14UNDO_MANEUVER_ACTIVE\x10\x07\x12\x14\n\x10START_REMOTE_APP\x10\x08\x12\x11\n\rPARK_OUT_INIT\x10\t\x12\x11\n\rPARK_OUT_SIDE\x10\n\x12\x0c\n\x08MENU_REM\x10\x0b\x12\x1c\n\x18MANEUVER_ACTIVE_LONG_MAN\x10\x0c\x12\n\n\x06REM_SV\x10\r\x12\x0b\n\x07REM_MAN\x10\x0e\x12\x12\n\x0eREM_MAN_ACTIVE\x10\x0f\x12\x16\n\x12KEY_CONTROL_ACTIVE\x10\x10\x12\x0e\n\nPDC_ACTIVE\x10\x11\x12\x0c\n\x08GP_START\x10\x12\x12\x16\n\x12GP_MANEUVER_ACTIVE\x10\x13\x12\x0e\n\nDIAG_ERROR\x10\x14\x12\x16\n\x12MP_SPACE_SELECTION\x10\x15\x12\x16\n\x12MP_USER_ADJUSMENTS\x10\x16\x12\x19\n\x15MP_WAITING_FOR_FINISH\x10\x17\x12\x12\n\x0eREVERSE_ASSIST\x10\x18\x12\x1a\n\x16PRE_CONDITIONS_NOT_MET\x10\x19'
)

_SCREENHEADUNIT = _descriptor.EnumDescriptor(
  name='ScreenHeadUnit',
  full_name='pb.mf_hmih.screen_head_unit.ScreenHeadUnit',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='SYSTEM_NOT_ACTIVE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SCANNING_ACTIVE', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARKING_SPACE_SELECTION', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REMOTE_APP_ACTIVE', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MANEUVER_ACTIVE', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MANEUVER_FINISHED', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MANEUVER_INTERRUPTED', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UNDO_MANEUVER_ACTIVE', index=7, number=7,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='START_REMOTE_APP', index=8, number=8,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_OUT_INIT', index=9, number=9,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_OUT_SIDE', index=10, number=10,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MENU_REM', index=11, number=11,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MANEUVER_ACTIVE_LONG_MAN', index=12, number=12,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REM_SV', index=13, number=13,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REM_MAN', index=14, number=14,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REM_MAN_ACTIVE', index=15, number=15,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='KEY_CONTROL_ACTIVE', index=16, number=16,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PDC_ACTIVE', index=17, number=17,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GP_START', index=18, number=18,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GP_MANEUVER_ACTIVE', index=19, number=19,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DIAG_ERROR', index=20, number=20,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MP_SPACE_SELECTION', index=21, number=21,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MP_USER_ADJUSMENTS', index=22, number=22,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MP_WAITING_FOR_FINISH', index=23, number=23,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REVERSE_ASSIST', index=24, number=24,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PRE_CONDITIONS_NOT_MET', index=25, number=25,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=64,
  serialized_end=638,
)
_sym_db.RegisterEnumDescriptor(_SCREENHEADUNIT)

ScreenHeadUnit = enum_type_wrapper.EnumTypeWrapper(_SCREENHEADUNIT)
SYSTEM_NOT_ACTIVE = 0
SCANNING_ACTIVE = 1
PARKING_SPACE_SELECTION = 2
REMOTE_APP_ACTIVE = 3
MANEUVER_ACTIVE = 4
MANEUVER_FINISHED = 5
MANEUVER_INTERRUPTED = 6
UNDO_MANEUVER_ACTIVE = 7
START_REMOTE_APP = 8
PARK_OUT_INIT = 9
PARK_OUT_SIDE = 10
MENU_REM = 11
MANEUVER_ACTIVE_LONG_MAN = 12
REM_SV = 13
REM_MAN = 14
REM_MAN_ACTIVE = 15
KEY_CONTROL_ACTIVE = 16
PDC_ACTIVE = 17
GP_START = 18
GP_MANEUVER_ACTIVE = 19
DIAG_ERROR = 20
MP_SPACE_SELECTION = 21
MP_USER_ADJUSMENTS = 22
MP_WAITING_FOR_FINISH = 23
REVERSE_ASSIST = 24
PRE_CONDITIONS_NOT_MET = 25


DESCRIPTOR.enum_types_by_name['ScreenHeadUnit'] = _SCREENHEADUNIT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
