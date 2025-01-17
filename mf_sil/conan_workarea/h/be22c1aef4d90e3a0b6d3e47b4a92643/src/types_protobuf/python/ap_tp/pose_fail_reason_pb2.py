# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_tp/pose_fail_reason.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_tp/pose_fail_reason.proto',
  package='pb.ap_tp.pose_fail_reason',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1c\x61p_tp/pose_fail_reason.proto\x12\x19pb.ap_tp.pose_fail_reason*\x9a\x02\n\x0ePoseFailReason\x12\x13\n\x0fTAPOSD_PFR_NONE\x10\x00\x12+\n\'TAPOSD_PFR_PARKING_BOX_WIDTH_TOO_NARROW\x10\x01\x12+\n\'TAPOSD_PFR_PARKING_BOX_LENGTH_TOO_SHORT\x10\x02\x12\x1e\n\x1aTAPOSD_PFR_MAXBOX_EXCEEDED\x10\x03\x12\x1e\n\x1aTAPOSD_PFR_WHEEL_COLLISION\x10\x04\x12$\n TAPOSD_PFR_HIGH_OBJECT_COLLISION\x10\x05\x12\x16\n\x12TAPOSD_PFR_UNKNOWN\x10\x06\x12\x1b\n\x17MAX_NUM_POSE_FAIL_TYPES\x10\x07'
)

_POSEFAILREASON = _descriptor.EnumDescriptor(
  name='PoseFailReason',
  full_name='pb.ap_tp.pose_fail_reason.PoseFailReason',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='TAPOSD_PFR_NONE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TAPOSD_PFR_PARKING_BOX_WIDTH_TOO_NARROW', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TAPOSD_PFR_PARKING_BOX_LENGTH_TOO_SHORT', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TAPOSD_PFR_MAXBOX_EXCEEDED', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TAPOSD_PFR_WHEEL_COLLISION', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TAPOSD_PFR_HIGH_OBJECT_COLLISION', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TAPOSD_PFR_UNKNOWN', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MAX_NUM_POSE_FAIL_TYPES', index=7, number=7,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=60,
  serialized_end=342,
)
_sym_db.RegisterEnumDescriptor(_POSEFAILREASON)

PoseFailReason = enum_type_wrapper.EnumTypeWrapper(_POSEFAILREASON)
TAPOSD_PFR_NONE = 0
TAPOSD_PFR_PARKING_BOX_WIDTH_TOO_NARROW = 1
TAPOSD_PFR_PARKING_BOX_LENGTH_TOO_SHORT = 2
TAPOSD_PFR_MAXBOX_EXCEEDED = 3
TAPOSD_PFR_WHEEL_COLLISION = 4
TAPOSD_PFR_HIGH_OBJECT_COLLISION = 5
TAPOSD_PFR_UNKNOWN = 6
MAX_NUM_POSE_FAIL_TYPES = 7


DESCRIPTOR.enum_types_by_name['PoseFailReason'] = _POSEFAILREASON
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
