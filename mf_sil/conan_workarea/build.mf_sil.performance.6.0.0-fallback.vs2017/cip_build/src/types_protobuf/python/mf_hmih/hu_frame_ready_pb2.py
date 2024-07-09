# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/hu_frame_ready.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/hu_frame_ready.proto',
  package='pb.mf_hmih.hu_frame_ready',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1cmf_hmih/hu_frame_ready.proto\x12\x19pb.mf_hmih.hu_frame_ready*\xc4\x02\n\x0cHuFrameReady\x12\x0b\n\x07NO_USED\x10\x00\x12\x11\n\rIN_TRANSITION\x10\x01\x12\x0f\n\x0b\x46RAME_ERROR\x10\x02\x12\x13\n\x0f\x45\x41RLY_RVC_READY\x10\x03\x12\r\n\tAPA_READY\x10\x04\x12\x11\n\rPARK_IN_READY\x10\x05\x12\x12\n\x0ePARK_OUT_READY\x10\x06\x12\x18\n\x14GARAGE_PARKING_READY\x10\x07\x12\x18\n\x14REVERSE_ASSIST_READY\x10\x08\x12\x0e\n\nRCTA_READY\x10\t\x12\r\n\tBVM_READY\x10\n\x12\x14\n\x10TWO_D_VIEW_READY\x10\x0b\x12\x16\n\x12THREE_D_VIEW_READY\x10\x0c\x12\x1b\n\x17\x46RONT_CORNER_VIEW_READY\x10\r\x12\x1a\n\x16REAR_CORNER_VIEW_READY\x10\x0e'
)

_HUFRAMEREADY = _descriptor.EnumDescriptor(
  name='HuFrameReady',
  full_name='pb.mf_hmih.hu_frame_ready.HuFrameReady',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NO_USED', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='IN_TRANSITION', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FRAME_ERROR', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='EARLY_RVC_READY', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='APA_READY', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_IN_READY', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_OUT_READY', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GARAGE_PARKING_READY', index=7, number=7,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REVERSE_ASSIST_READY', index=8, number=8,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RCTA_READY', index=9, number=9,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BVM_READY', index=10, number=10,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TWO_D_VIEW_READY', index=11, number=11,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='THREE_D_VIEW_READY', index=12, number=12,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FRONT_CORNER_VIEW_READY', index=13, number=13,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REAR_CORNER_VIEW_READY', index=14, number=14,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=60,
  serialized_end=384,
)
_sym_db.RegisterEnumDescriptor(_HUFRAMEREADY)

HuFrameReady = enum_type_wrapper.EnumTypeWrapper(_HUFRAMEREADY)
NO_USED = 0
IN_TRANSITION = 1
FRAME_ERROR = 2
EARLY_RVC_READY = 3
APA_READY = 4
PARK_IN_READY = 5
PARK_OUT_READY = 6
GARAGE_PARKING_READY = 7
REVERSE_ASSIST_READY = 8
RCTA_READY = 9
BVM_READY = 10
TWO_D_VIEW_READY = 11
THREE_D_VIEW_READY = 12
FRONT_CORNER_VIEW_READY = 13
REAR_CORNER_VIEW_READY = 14


DESCRIPTOR.enum_types_by_name['HuFrameReady'] = _HUFRAMEREADY
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)