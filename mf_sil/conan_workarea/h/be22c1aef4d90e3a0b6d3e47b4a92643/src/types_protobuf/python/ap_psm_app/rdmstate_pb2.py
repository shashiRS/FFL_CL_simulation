# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_psm_app/rdmstate.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_psm_app/rdmstate.proto',
  package='pb.ap_psm_app.rdmstate',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x19\x61p_psm_app/rdmstate.proto\x12\x16pb.ap_psm_app.rdmstate*D\n\x08RDMState\x12\x10\n\x0cRDM_INACTIVE\x10\x00\x12\x14\n\x10RDM_OUT_OF_RANGE\x10\x01\x12\x10\n\x0cRDM_IN_RANGE\x10\x02'
)

_RDMSTATE = _descriptor.EnumDescriptor(
  name='RDMState',
  full_name='pb.ap_psm_app.rdmstate.RDMState',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='RDM_INACTIVE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RDM_OUT_OF_RANGE', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RDM_IN_RANGE', index=2, number=2,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=53,
  serialized_end=121,
)
_sym_db.RegisterEnumDescriptor(_RDMSTATE)

RDMState = enum_type_wrapper.EnumTypeWrapper(_RDMSTATE)
RDM_INACTIVE = 0
RDM_OUT_OF_RANGE = 1
RDM_IN_RANGE = 2


DESCRIPTOR.enum_types_by_name['RDMState'] = _RDMSTATE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
