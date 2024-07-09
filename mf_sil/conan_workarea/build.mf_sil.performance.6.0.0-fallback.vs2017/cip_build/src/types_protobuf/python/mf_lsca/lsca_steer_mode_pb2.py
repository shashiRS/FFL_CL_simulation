# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_lsca/lsca_steer_mode.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_lsca/lsca_steer_mode.proto',
  package='pb.mf_lsca.lsca_steer_mode',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1dmf_lsca/lsca_steer_mode.proto\x12\x1apb.mf_lsca.lsca_steer_mode*\x99\x01\n\x0fLSCA_STEER_MODE\x12\x13\n\x0fSTEER_MODE_NONE\x10\x00\x12\x1a\n\x16STEER_MODE_ANGLE_FRONT\x10\x01\x12\x19\n\x15STEER_MODE_ANGLE_REAR\x10\x02\x12#\n\x1fSTEER_MODE_ANGLE_FRONT_AND_REAR\x10\x03\x12\x15\n\x11STEER_MODE_TORQUE\x10\x04'
)

_LSCA_STEER_MODE = _descriptor.EnumDescriptor(
  name='LSCA_STEER_MODE',
  full_name='pb.mf_lsca.lsca_steer_mode.LSCA_STEER_MODE',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='STEER_MODE_NONE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STEER_MODE_ANGLE_FRONT', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STEER_MODE_ANGLE_REAR', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STEER_MODE_ANGLE_FRONT_AND_REAR', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STEER_MODE_TORQUE', index=4, number=4,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=62,
  serialized_end=215,
)
_sym_db.RegisterEnumDescriptor(_LSCA_STEER_MODE)

LSCA_STEER_MODE = enum_type_wrapper.EnumTypeWrapper(_LSCA_STEER_MODE)
STEER_MODE_NONE = 0
STEER_MODE_ANGLE_FRONT = 1
STEER_MODE_ANGLE_REAR = 2
STEER_MODE_ANGLE_FRONT_AND_REAR = 3
STEER_MODE_TORQUE = 4


DESCRIPTOR.enum_types_by_name['LSCA_STEER_MODE'] = _LSCA_STEER_MODE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)