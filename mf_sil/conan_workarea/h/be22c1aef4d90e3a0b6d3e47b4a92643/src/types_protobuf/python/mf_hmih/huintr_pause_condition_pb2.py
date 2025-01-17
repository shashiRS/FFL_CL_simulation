# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/huintr_pause_condition.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/huintr_pause_condition.proto',
  package='pb.mf_hmih.huintr_pause_condition',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n$mf_hmih/huintr_pause_condition.proto\x12!pb.mf_hmih.huintr_pause_condition*\x95\x02\n\x14HUIntrPauseCondition\x12\n\n\x06NO_MSG\x10\x00\x12\x1a\n\x16\x44RIVER_DOOR_NOT_CLOSED\x10\x01\x12\x1d\n\x19\x43O_DRIVER_DOOR_NOT_CLOSED\x10\x02\x12\x15\n\x11\x44YN_STAT_OBJ_TRAJ\x10\x03\x12\x12\n\x0eSEAT_BELT_OPEN\x10\x04\x12\x0e\n\nORVM_CLOSE\x10\x05\x12\x12\n\x0e\x42RAKE_PRESS_TH\x10\x06\x12\x15\n\x11\x41PA_HOLD_RELEASED\x10\x07\x12\x1c\n\x18REAR_PSG_DOOR_NOT_CLOSED\x10\x08\x12\x18\n\x14TRUNK_LID_NOT_CLOSED\x10\t\x12\x18\n\x14TAIL_GATE_NOT_CLOSED\x10\n'
)

_HUINTRPAUSECONDITION = _descriptor.EnumDescriptor(
  name='HUIntrPauseCondition',
  full_name='pb.mf_hmih.huintr_pause_condition.HUIntrPauseCondition',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NO_MSG', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DRIVER_DOOR_NOT_CLOSED', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CO_DRIVER_DOOR_NOT_CLOSED', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DYN_STAT_OBJ_TRAJ', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SEAT_BELT_OPEN', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ORVM_CLOSE', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BRAKE_PRESS_TH', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='APA_HOLD_RELEASED', index=7, number=7,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REAR_PSG_DOOR_NOT_CLOSED', index=8, number=8,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TRUNK_LID_NOT_CLOSED', index=9, number=9,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TAIL_GATE_NOT_CLOSED', index=10, number=10,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=76,
  serialized_end=353,
)
_sym_db.RegisterEnumDescriptor(_HUINTRPAUSECONDITION)

HUIntrPauseCondition = enum_type_wrapper.EnumTypeWrapper(_HUINTRPAUSECONDITION)
NO_MSG = 0
DRIVER_DOOR_NOT_CLOSED = 1
CO_DRIVER_DOOR_NOT_CLOSED = 2
DYN_STAT_OBJ_TRAJ = 3
SEAT_BELT_OPEN = 4
ORVM_CLOSE = 5
BRAKE_PRESS_TH = 6
APA_HOLD_RELEASED = 7
REAR_PSG_DOOR_NOT_CLOSED = 8
TRUNK_LID_NOT_CLOSED = 9
TAIL_GATE_NOT_CLOSED = 10


DESCRIPTOR.enum_types_by_name['HUIntrPauseCondition'] = _HUINTRPAUSECONDITION
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
