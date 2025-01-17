# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_psm_app/std_request.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_psm_app/std_request.proto',
  package='pb.ap_psm_app.std_request',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1c\x61p_psm_app/std_request.proto\x12\x19pb.ap_psm_app.std_request*\xd3\x01\n\nStdRequest\x12\x16\n\x12STD_REQ_NO_REQUEST\x10\x00\x12\x10\n\x0cSTD_REQ_INIT\x10\x01\x12\x10\n\x0cSTD_REQ_SCAN\x10\x02\x12\x13\n\x0fSTD_REQ_PREPARE\x10\x03\x12\x13\n\x0fSTD_REQ_DEGRADE\x10\x04\x12\x11\n\rSTD_REQ_START\x10\x05\x12\x11\n\rSTD_REQ_PAUSE\x10\x06\x12\x12\n\x0eSTD_REQ_SECURE\x10\x07\x12\x12\n\x0eSTD_REQ_FINISH\x10\x08\x12\x11\n\rSTD_REQ_ERROR\x10\t'
)

_STDREQUEST = _descriptor.EnumDescriptor(
  name='StdRequest',
  full_name='pb.ap_psm_app.std_request.StdRequest',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='STD_REQ_NO_REQUEST', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STD_REQ_INIT', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STD_REQ_SCAN', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STD_REQ_PREPARE', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STD_REQ_DEGRADE', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STD_REQ_START', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STD_REQ_PAUSE', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STD_REQ_SECURE', index=7, number=7,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STD_REQ_FINISH', index=8, number=8,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STD_REQ_ERROR', index=9, number=9,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=60,
  serialized_end=271,
)
_sym_db.RegisterEnumDescriptor(_STDREQUEST)

StdRequest = enum_type_wrapper.EnumTypeWrapper(_STDREQUEST)
STD_REQ_NO_REQUEST = 0
STD_REQ_INIT = 1
STD_REQ_SCAN = 2
STD_REQ_PREPARE = 3
STD_REQ_DEGRADE = 4
STD_REQ_START = 5
STD_REQ_PAUSE = 6
STD_REQ_SECURE = 7
STD_REQ_FINISH = 8
STD_REQ_ERROR = 9


DESCRIPTOR.enum_types_by_name['StdRequest'] = _STDREQUEST
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
