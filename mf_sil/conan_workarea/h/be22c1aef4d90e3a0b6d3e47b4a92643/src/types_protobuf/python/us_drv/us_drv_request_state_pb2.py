# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_drv/us_drv_request_state.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_drv/us_drv_request_state.proto',
  package='pb.us_drv.us_drv_request_state',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n!us_drv/us_drv_request_state.proto\x12\x1epb.us_drv.us_drv_request_state*\xbb\x01\n\x11UsDrvRequestState\x12\x13\n\x0fUS_REQUEST_NONE\x10\x00\x12\x18\n\x14US_REQUEST_INITIATED\x10\x01\x12\x16\n\x12US_REQUEST_PENDING\x10\x02\x12\x16\n\x12US_REQUEST_SUCCESS\x10\x03\x12\x15\n\x11US_REQUEST_FAILED\x10\x04\x12\x18\n\x14US_REQUEST_CANCELLED\x10\x05\x12\x16\n\x12US_REQUEST_TIMEOUT\x10\x06'
)

_USDRVREQUESTSTATE = _descriptor.EnumDescriptor(
  name='UsDrvRequestState',
  full_name='pb.us_drv.us_drv_request_state.UsDrvRequestState',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='US_REQUEST_NONE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_REQUEST_INITIATED', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_REQUEST_PENDING', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_REQUEST_SUCCESS', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_REQUEST_FAILED', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_REQUEST_CANCELLED', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_REQUEST_TIMEOUT', index=6, number=6,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=70,
  serialized_end=257,
)
_sym_db.RegisterEnumDescriptor(_USDRVREQUESTSTATE)

UsDrvRequestState = enum_type_wrapper.EnumTypeWrapper(_USDRVREQUESTSTATE)
US_REQUEST_NONE = 0
US_REQUEST_INITIATED = 1
US_REQUEST_PENDING = 2
US_REQUEST_SUCCESS = 3
US_REQUEST_FAILED = 4
US_REQUEST_CANCELLED = 5
US_REQUEST_TIMEOUT = 6


DESCRIPTOR.enum_types_by_name['UsDrvRequestState'] = _USDRVREQUESTSTATE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)