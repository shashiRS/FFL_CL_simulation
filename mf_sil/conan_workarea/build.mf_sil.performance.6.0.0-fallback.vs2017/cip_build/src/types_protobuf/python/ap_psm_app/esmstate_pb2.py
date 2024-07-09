# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_psm_app/esmstate.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_psm_app/esmstate.proto',
  package='pb.ap_psm_app.esmstate',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x19\x61p_psm_app/esmstate.proto\x12\x16pb.ap_psm_app.esmstate*d\n\x08\x45SMState\x12\x10\n\x0c\x45SM_INACTIVE\x10\x00\x12\x10\n\x0c\x45SM_NO_ERROR\x10\x01\x12\x18\n\x14\x45SM_REVERSIBLE_ERROR\x10\x02\x12\x1a\n\x16\x45SM_IRREVERSIBLE_ERROR\x10\x03'
)

_ESMSTATE = _descriptor.EnumDescriptor(
  name='ESMState',
  full_name='pb.ap_psm_app.esmstate.ESMState',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='ESM_INACTIVE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ESM_NO_ERROR', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ESM_REVERSIBLE_ERROR', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ESM_IRREVERSIBLE_ERROR', index=3, number=3,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=53,
  serialized_end=153,
)
_sym_db.RegisterEnumDescriptor(_ESMSTATE)

ESMState = enum_type_wrapper.EnumTypeWrapper(_ESMSTATE)
ESM_INACTIVE = 0
ESM_NO_ERROR = 1
ESM_REVERSIBLE_ERROR = 2
ESM_IRREVERSIBLE_ERROR = 3


DESCRIPTOR.enum_types_by_name['ESMState'] = _ESMSTATE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
