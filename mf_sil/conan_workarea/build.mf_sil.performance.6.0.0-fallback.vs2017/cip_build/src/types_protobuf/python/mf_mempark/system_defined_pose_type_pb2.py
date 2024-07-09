# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/system_defined_pose_type.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/system_defined_pose_type.proto',
  package='pb.mf_mempark.system_defined_pose_type',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n)mf_mempark/system_defined_pose_type.proto\x12&pb.mf_mempark.system_defined_pose_type*\x8f\x02\n\x15SystemDefinedPoseType\x12\x1f\n\x1bSYSTEM_DEF_TYPE_CURRENT_EGO\x10\x00\x12\x17\n\x13SYSTEM_DEF_TYPE_PAR\x10\x01\x12\x1c\n\x18SYSTEM_DEF_TYPE_PERP_BWD\x10\x02\x12\x1c\n\x18SYSTEM_DEF_TYPE_PERP_FWD\x10\x03\x12\x1e\n\x1aSYSTEM_DEF_TYPE_FOLLOW_BWD\x10\x04\x12\x1e\n\x1aSYSTEM_DEF_TYPE_FOLLOW_FWD\x10\x05\x12\x1d\n\x19SYSTEM_DEF_TYPE_ANGLE_BWD\x10\x06\x12!\n\x1dMAX_NUM_SYSTEM_DEF_POSE_TYPES\x10\x07'
)

_SYSTEMDEFINEDPOSETYPE = _descriptor.EnumDescriptor(
  name='SystemDefinedPoseType',
  full_name='pb.mf_mempark.system_defined_pose_type.SystemDefinedPoseType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='SYSTEM_DEF_TYPE_CURRENT_EGO', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SYSTEM_DEF_TYPE_PAR', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SYSTEM_DEF_TYPE_PERP_BWD', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SYSTEM_DEF_TYPE_PERP_FWD', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SYSTEM_DEF_TYPE_FOLLOW_BWD', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SYSTEM_DEF_TYPE_FOLLOW_FWD', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SYSTEM_DEF_TYPE_ANGLE_BWD', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MAX_NUM_SYSTEM_DEF_POSE_TYPES', index=7, number=7,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=86,
  serialized_end=357,
)
_sym_db.RegisterEnumDescriptor(_SYSTEMDEFINEDPOSETYPE)

SystemDefinedPoseType = enum_type_wrapper.EnumTypeWrapper(_SYSTEMDEFINEDPOSETYPE)
SYSTEM_DEF_TYPE_CURRENT_EGO = 0
SYSTEM_DEF_TYPE_PAR = 1
SYSTEM_DEF_TYPE_PERP_BWD = 2
SYSTEM_DEF_TYPE_PERP_FWD = 3
SYSTEM_DEF_TYPE_FOLLOW_BWD = 4
SYSTEM_DEF_TYPE_FOLLOW_FWD = 5
SYSTEM_DEF_TYPE_ANGLE_BWD = 6
MAX_NUM_SYSTEM_DEF_POSE_TYPES = 7


DESCRIPTOR.enum_types_by_name['SystemDefinedPoseType'] = _SYSTEMDEFINEDPOSETYPE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
