# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_trjctl/drv_res_type.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_trjctl/drv_res_type.proto',
  package='pb.ap_trjctl.drv_res_type',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1c\x61p_trjctl/drv_res_type.proto\x12\x19pb.ap_trjctl.drv_res_type*\xe5\x01\n\nDrvResType\x12\x0f\n\x0b\x44RVRES_NONE\x10\x00\x12\x16\n\x12\x44RVRES_FALLING_LOW\x10\x01\x12\x19\n\x15\x44RVRES_FALLING_MEDIUM\x10\x02\x12\x17\n\x13\x44RVRES_FALLING_HIGH\x10\x03\x12\x15\n\x11\x44RVRES_RISING_LOW\x10\x04\x12\x18\n\x14\x44RVRES_RISING_MEDIUM\x10\x05\x12\x16\n\x12\x44RVRES_RISING_HIGH\x10\x06\x12\x18\n\x14\x44RVRES_WHEEL_STOPPER\x10\x07\x12\x17\n\x13MAX_NUM_DRVRES_TYPE\x10\x08'
)

_DRVRESTYPE = _descriptor.EnumDescriptor(
  name='DrvResType',
  full_name='pb.ap_trjctl.drv_res_type.DrvResType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='DRVRES_NONE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DRVRES_FALLING_LOW', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DRVRES_FALLING_MEDIUM', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DRVRES_FALLING_HIGH', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DRVRES_RISING_LOW', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DRVRES_RISING_MEDIUM', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DRVRES_RISING_HIGH', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DRVRES_WHEEL_STOPPER', index=7, number=7,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MAX_NUM_DRVRES_TYPE', index=8, number=8,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=60,
  serialized_end=289,
)
_sym_db.RegisterEnumDescriptor(_DRVRESTYPE)

DrvResType = enum_type_wrapper.EnumTypeWrapper(_DRVRESTYPE)
DRVRES_NONE = 0
DRVRES_FALLING_LOW = 1
DRVRES_FALLING_MEDIUM = 2
DRVRES_FALLING_HIGH = 3
DRVRES_RISING_LOW = 4
DRVRES_RISING_MEDIUM = 5
DRVRES_RISING_HIGH = 6
DRVRES_WHEEL_STOPPER = 7
MAX_NUM_DRVRES_TYPE = 8


DESCRIPTOR.enum_types_by_name['DrvResType'] = _DRVRESTYPE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
