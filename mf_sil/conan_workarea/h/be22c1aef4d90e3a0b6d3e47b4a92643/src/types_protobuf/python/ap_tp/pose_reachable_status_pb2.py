# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_tp/pose_reachable_status.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_tp/pose_reachable_status.proto',
  package='pb.ap_tp.pose_reachable_status',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n!ap_tp/pose_reachable_status.proto\x12\x1epb.ap_tp.pose_reachable_status*\xce\x01\n\x13PoseReachableStatus\x12\x10\n\x0cTP_NOT_VALID\x10\x00\x12\x14\n\x10TP_NOT_REACHABLE\x10\x01\x12\x16\n\x12TP_FULLY_REACHABLE\x10\x02\x12\x1a\n\x16TP_SAFE_ZONE_REACHABLE\x10\x03\x12\x1b\n\x17TP_MANUAL_FWD_REACHABLE\x10\x04\x12\x1b\n\x17TP_MANUAL_BWD_REACHABLE\x10\x05\x12!\n\x1dMAX_NUM_POSE_REACHABLE_STATUS\x10\x06'
)

_POSEREACHABLESTATUS = _descriptor.EnumDescriptor(
  name='PoseReachableStatus',
  full_name='pb.ap_tp.pose_reachable_status.PoseReachableStatus',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='TP_NOT_VALID', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TP_NOT_REACHABLE', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TP_FULLY_REACHABLE', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TP_SAFE_ZONE_REACHABLE', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TP_MANUAL_FWD_REACHABLE', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TP_MANUAL_BWD_REACHABLE', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MAX_NUM_POSE_REACHABLE_STATUS', index=6, number=6,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=70,
  serialized_end=276,
)
_sym_db.RegisterEnumDescriptor(_POSEREACHABLESTATUS)

PoseReachableStatus = enum_type_wrapper.EnumTypeWrapper(_POSEREACHABLESTATUS)
TP_NOT_VALID = 0
TP_NOT_REACHABLE = 1
TP_FULLY_REACHABLE = 2
TP_SAFE_ZONE_REACHABLE = 3
TP_MANUAL_FWD_REACHABLE = 4
TP_MANUAL_BWD_REACHABLE = 5
MAX_NUM_POSE_REACHABLE_STATUS = 6


DESCRIPTOR.enum_types_by_name['PoseReachableStatus'] = _POSEREACHABLESTATUS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)