# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_vehstatesigprovider/convertible_top_state.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_vehstatesigprovider/convertible_top_state.proto',
  package='pb.ap_vehstatesigprovider.convertible_top_state',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n2ap_vehstatesigprovider/convertible_top_state.proto\x12/pb.ap_vehstatesigprovider.convertible_top_state*G\n\x13\x43onvertibleTopState\x12\x0f\n\x0bLOCKED_OPEN\x10\x00\x12\x11\n\rLOCKED_CLOSED\x10\x01\x12\x0c\n\x08UNLOCKED\x10\x02'
)

_CONVERTIBLETOPSTATE = _descriptor.EnumDescriptor(
  name='ConvertibleTopState',
  full_name='pb.ap_vehstatesigprovider.convertible_top_state.ConvertibleTopState',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='LOCKED_OPEN', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LOCKED_CLOSED', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UNLOCKED', index=2, number=2,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=103,
  serialized_end=174,
)
_sym_db.RegisterEnumDescriptor(_CONVERTIBLETOPSTATE)

ConvertibleTopState = enum_type_wrapper.EnumTypeWrapper(_CONVERTIBLETOPSTATE)
LOCKED_OPEN = 0
LOCKED_CLOSED = 1
UNLOCKED = 2


DESCRIPTOR.enum_types_by_name['ConvertibleTopState'] = _CONVERTIBLETOPSTATE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
