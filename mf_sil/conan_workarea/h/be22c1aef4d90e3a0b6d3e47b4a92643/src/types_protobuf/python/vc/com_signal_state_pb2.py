# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: vc/com_signal_state.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='vc/com_signal_state.proto',
  package='pb.vc.com_signal_state',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x19vc/com_signal_state.proto\x12\x16pb.vc.com_signal_state*\x8a\x01\n\x0e\x43omSignalState\x12\x14\n\x10\x43OMSIGSTATE_INIT\x10\x00\x12\x15\n\x11\x43OMSIGSTATE_VALID\x10\x01\x12\x17\n\x13\x43OMSIGSTATE_INVALID\x10\x02\x12\x17\n\x13\x43OMSIGSTATE_TIMEOUT\x10\x03\x12\x15\n\x11\x43OMSIGSTATE_ERROR\x10\x00\x1a\x02\x10\x01'
)

_COMSIGNALSTATE = _descriptor.EnumDescriptor(
  name='ComSignalState',
  full_name='pb.vc.com_signal_state.ComSignalState',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='COMSIGSTATE_INIT', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COMSIGSTATE_VALID', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COMSIGSTATE_INVALID', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COMSIGSTATE_TIMEOUT', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COMSIGSTATE_ERROR', index=4, number=0,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=b'\020\001',
  serialized_start=54,
  serialized_end=192,
)
_sym_db.RegisterEnumDescriptor(_COMSIGNALSTATE)

ComSignalState = enum_type_wrapper.EnumTypeWrapper(_COMSIGNALSTATE)
COMSIGSTATE_INIT = 0
COMSIGSTATE_VALID = 1
COMSIGSTATE_INVALID = 2
COMSIGSTATE_TIMEOUT = 3
COMSIGSTATE_ERROR = 0


DESCRIPTOR.enum_types_by_name['ComSignalState'] = _COMSIGNALSTATE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


_COMSIGNALSTATE._options = None
# @@protoc_insertion_point(module_scope)
