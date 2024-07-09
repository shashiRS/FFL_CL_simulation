# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_tp/pose_type.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_tp/pose_type.proto',
  package='pb.ap_tp.pose_type',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x15\x61p_tp/pose_type.proto\x12\x12pb.ap_tp.pose_type*\xfa\x03\n\x08PoseType\x12\x16\n\x12T_PARALLEL_PARKING\x10\x00\x12\x16\n\x12T_PERP_PARKING_FWD\x10\x01\x12\x16\n\x12T_PERP_PARKING_BWD\x10\x02\x12\x1d\n\x19T_ANGLED_PARKING_STANDARD\x10\x03\x12\x1c\n\x18T_ANGLED_PARKING_REVERSE\x10\x04\x12\x11\n\rT_REM_MAN_FWD\x10\x05\x12\x11\n\rT_REM_MAN_BWD\x10\x06\x12\x1a\n\x16T_PERP_PARKING_OUT_FWD\x10\x07\x12\x1a\n\x16T_PERP_PARKING_OUT_BWD\x10\x08\x12\x15\n\x11T_PAR_PARKING_OUT\x10\t\x12!\n\x1dT_ANGLED_PARKING_STANDARD_OUT\x10\n\x12 \n\x1cT_ANGLED_PARKING_REVERSE_OUT\x10\x0b\x12\n\n\x06T_UNDO\x10\x0c\x12\x0c\n\x08T_GP_FWD\x10\r\x12\x0c\n\x08T_GP_BWD\x10\x0e\x12\x10\n\x0cT_GP_OUT_FWD\x10\x0f\x12\x10\n\x0cT_GP_OUT_BWD\x10\x10\x12\x11\n\rT_GP_FWD_AXIS\x10\x11\x12\x11\n\rT_GP_BWD_AXIS\x10\x12\x12\x15\n\x11T_GP_OUT_FWD_AXIS\x10\x13\x12\x15\n\x11T_GP_OUT_BWD_AXIS\x10\x14\x12\x0f\n\x0bT_UNDEFINED\x10\x15'
)

_POSETYPE = _descriptor.EnumDescriptor(
  name='PoseType',
  full_name='pb.ap_tp.pose_type.PoseType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='T_PARALLEL_PARKING', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_PERP_PARKING_FWD', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_PERP_PARKING_BWD', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_ANGLED_PARKING_STANDARD', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_ANGLED_PARKING_REVERSE', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_REM_MAN_FWD', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_REM_MAN_BWD', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_PERP_PARKING_OUT_FWD', index=7, number=7,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_PERP_PARKING_OUT_BWD', index=8, number=8,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_PAR_PARKING_OUT', index=9, number=9,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_ANGLED_PARKING_STANDARD_OUT', index=10, number=10,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_ANGLED_PARKING_REVERSE_OUT', index=11, number=11,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_UNDO', index=12, number=12,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_GP_FWD', index=13, number=13,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_GP_BWD', index=14, number=14,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_GP_OUT_FWD', index=15, number=15,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_GP_OUT_BWD', index=16, number=16,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_GP_FWD_AXIS', index=17, number=17,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_GP_BWD_AXIS', index=18, number=18,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_GP_OUT_FWD_AXIS', index=19, number=19,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_GP_OUT_BWD_AXIS', index=20, number=20,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='T_UNDEFINED', index=21, number=21,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=46,
  serialized_end=552,
)
_sym_db.RegisterEnumDescriptor(_POSETYPE)

PoseType = enum_type_wrapper.EnumTypeWrapper(_POSETYPE)
T_PARALLEL_PARKING = 0
T_PERP_PARKING_FWD = 1
T_PERP_PARKING_BWD = 2
T_ANGLED_PARKING_STANDARD = 3
T_ANGLED_PARKING_REVERSE = 4
T_REM_MAN_FWD = 5
T_REM_MAN_BWD = 6
T_PERP_PARKING_OUT_FWD = 7
T_PERP_PARKING_OUT_BWD = 8
T_PAR_PARKING_OUT = 9
T_ANGLED_PARKING_STANDARD_OUT = 10
T_ANGLED_PARKING_REVERSE_OUT = 11
T_UNDO = 12
T_GP_FWD = 13
T_GP_BWD = 14
T_GP_OUT_FWD = 15
T_GP_OUT_BWD = 16
T_GP_FWD_AXIS = 17
T_GP_BWD_AXIS = 18
T_GP_OUT_FWD_AXIS = 19
T_GP_OUT_BWD_AXIS = 20
T_UNDEFINED = 21


DESCRIPTOR.enum_types_by_name['PoseType'] = _POSETYPE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)