# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/memorized_parking_status.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/memorized_parking_status.proto',
  package='pb.mf_mempark.memorized_parking_status',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n)mf_mempark/memorized_parking_status.proto\x12&pb.mf_mempark.memorized_parking_status*\xa3\x01\n\x16MemorizedParkingStatus\x12 \n\x1cMEMORIZED_PARKING_STATE_INIT\x10\x00\x12 \n\x1cMEMORIZED_PARKING_STATE_IDLE\x10\x01\x12\"\n\x1eMEMORIZED_PARKING_STATE_ACTIVE\x10\x02\x12!\n\x1dMEMORIZED_PARKING_STATE_ERROR\x10\x03'
)

_MEMORIZEDPARKINGSTATUS = _descriptor.EnumDescriptor(
  name='MemorizedParkingStatus',
  full_name='pb.mf_mempark.memorized_parking_status.MemorizedParkingStatus',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='MEMORIZED_PARKING_STATE_INIT', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MEMORIZED_PARKING_STATE_IDLE', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MEMORIZED_PARKING_STATE_ACTIVE', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MEMORIZED_PARKING_STATE_ERROR', index=3, number=3,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=86,
  serialized_end=249,
)
_sym_db.RegisterEnumDescriptor(_MEMORIZEDPARKINGSTATUS)

MemorizedParkingStatus = enum_type_wrapper.EnumTypeWrapper(_MEMORIZEDPARKINGSTATUS)
MEMORIZED_PARKING_STATE_INIT = 0
MEMORIZED_PARKING_STATE_IDLE = 1
MEMORIZED_PARKING_STATE_ACTIVE = 2
MEMORIZED_PARKING_STATE_ERROR = 3


DESCRIPTOR.enum_types_by_name['MemorizedParkingStatus'] = _MEMORIZEDPARKINGSTATUS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
