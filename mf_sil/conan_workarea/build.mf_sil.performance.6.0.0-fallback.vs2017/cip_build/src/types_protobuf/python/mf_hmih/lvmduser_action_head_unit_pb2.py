# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/lvmduser_action_head_unit.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/lvmduser_action_head_unit.proto',
  package='pb.mf_hmih.lvmduser_action_head_unit',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\'mf_hmih/lvmduser_action_head_unit.proto\x12$pb.mf_hmih.lvmduser_action_head_unit*j\n\x16LVMDUserActionHeadUnit\x12\x17\n\x13LVMD_NO_USER_ACTION\x10\x00\x12\x1d\n\x19LVMD_TAP_ON_ACT_DEACT_BTN\x10\x01\x12\x18\n\x14LVMD_TAP_ON_MUTE_BTN\x10\x02'
)

_LVMDUSERACTIONHEADUNIT = _descriptor.EnumDescriptor(
  name='LVMDUserActionHeadUnit',
  full_name='pb.mf_hmih.lvmduser_action_head_unit.LVMDUserActionHeadUnit',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='LVMD_NO_USER_ACTION', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LVMD_TAP_ON_ACT_DEACT_BTN', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LVMD_TAP_ON_MUTE_BTN', index=2, number=2,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=81,
  serialized_end=187,
)
_sym_db.RegisterEnumDescriptor(_LVMDUSERACTIONHEADUNIT)

LVMDUserActionHeadUnit = enum_type_wrapper.EnumTypeWrapper(_LVMDUSERACTIONHEADUNIT)
LVMD_NO_USER_ACTION = 0
LVMD_TAP_ON_ACT_DEACT_BTN = 1
LVMD_TAP_ON_MUTE_BTN = 2


DESCRIPTOR.enum_types_by_name['LVMDUserActionHeadUnit'] = _LVMDUSERACTIONHEADUNIT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)