# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_vehstatesigprovider/escstate.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_vehstatesigprovider/escstate.proto',
  package='pb.ap_vehstatesigprovider.escstate',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n%ap_vehstatesigprovider/escstate.proto\x12\"pb.ap_vehstatesigprovider.escstate*;\n\x08\x45SCState\x12\x10\n\x0c\x45SC_INACTIVE\x10\x00\x12\x0e\n\nESC_ACTIVE\x10\x01\x12\r\n\tESC_ERROR\x10\x02'
)

_ESCSTATE = _descriptor.EnumDescriptor(
  name='ESCState',
  full_name='pb.ap_vehstatesigprovider.escstate.ESCState',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='ESC_INACTIVE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ESC_ACTIVE', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ESC_ERROR', index=2, number=2,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=77,
  serialized_end=136,
)
_sym_db.RegisterEnumDescriptor(_ESCSTATE)

ESCState = enum_type_wrapper.EnumTypeWrapper(_ESCSTATE)
ESC_INACTIVE = 0
ESC_ACTIVE = 1
ESC_ERROR = 2


DESCRIPTOR.enum_types_by_name['ESCState'] = _ESCSTATE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
