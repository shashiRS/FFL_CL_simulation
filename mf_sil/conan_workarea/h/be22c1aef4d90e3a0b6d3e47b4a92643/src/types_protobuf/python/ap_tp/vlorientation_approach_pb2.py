# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_tp/vlorientation_approach.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_tp/vlorientation_approach.proto',
  package='pb.ap_tp.vlorientation_approach',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\"ap_tp/vlorientation_approach.proto\x12\x1fpb.ap_tp.vlorientation_approach*\xa7\x01\n\x15VLOrientationApproach\x12\x18\n\x14\x46RONT_VL_ORIENTATION\x10\x00\x12\x17\n\x13REAR_VL_ORIENTATION\x10\x01\x12\x1a\n\x16VL_ORIENTATION_AVERAGE\x10\x02\x12\x1d\n\x19USE_BOTH_VL_PTS_NEAR_SLOT\x10\x03\x12 \n\x1cMAX_NUM_VL_ORIENT_APPROACHES\x10\x04'
)

_VLORIENTATIONAPPROACH = _descriptor.EnumDescriptor(
  name='VLOrientationApproach',
  full_name='pb.ap_tp.vlorientation_approach.VLOrientationApproach',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='FRONT_VL_ORIENTATION', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REAR_VL_ORIENTATION', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VL_ORIENTATION_AVERAGE', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='USE_BOTH_VL_PTS_NEAR_SLOT', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MAX_NUM_VL_ORIENT_APPROACHES', index=4, number=4,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=72,
  serialized_end=239,
)
_sym_db.RegisterEnumDescriptor(_VLORIENTATIONAPPROACH)

VLOrientationApproach = enum_type_wrapper.EnumTypeWrapper(_VLORIENTATIONAPPROACH)
FRONT_VL_ORIENTATION = 0
REAR_VL_ORIENTATION = 1
VL_ORIENTATION_AVERAGE = 2
USE_BOTH_VL_PTS_NEAR_SLOT = 3
MAX_NUM_VL_ORIENT_APPROACHES = 4


DESCRIPTOR.enum_types_by_name['VLOrientationApproach'] = _VLORIENTATIONAPPROACH
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
