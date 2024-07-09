# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_lsca/lsca_tube_marking.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_lsca/lsca_tube_marking.proto',
  package='pb.mf_lsca.lsca_tube_marking',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1fmf_lsca/lsca_tube_marking.proto\x12\x1cpb.mf_lsca.lsca_tube_marking*\x81\x01\n\x11LSCA_TUBE_MARKING\x12\x11\n\rLSCA_TUBE_OFF\x10\x00\x12\x12\n\x0eLSCA_TUBE_LEFT\x10\x01\x12\x13\n\x0fLSCA_TUBE_RIGHT\x10\x02\x12\x1c\n\x18LSCA_TUBE_LEFT_AND_RIGHT\x10\x03\x12\x12\n\x0eLSCA_TUBE_FULL\x10\x04'
)

_LSCA_TUBE_MARKING = _descriptor.EnumDescriptor(
  name='LSCA_TUBE_MARKING',
  full_name='pb.mf_lsca.lsca_tube_marking.LSCA_TUBE_MARKING',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='LSCA_TUBE_OFF', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LSCA_TUBE_LEFT', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LSCA_TUBE_RIGHT', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LSCA_TUBE_LEFT_AND_RIGHT', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LSCA_TUBE_FULL', index=4, number=4,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=66,
  serialized_end=195,
)
_sym_db.RegisterEnumDescriptor(_LSCA_TUBE_MARKING)

LSCA_TUBE_MARKING = enum_type_wrapper.EnumTypeWrapper(_LSCA_TUBE_MARKING)
LSCA_TUBE_OFF = 0
LSCA_TUBE_LEFT = 1
LSCA_TUBE_RIGHT = 2
LSCA_TUBE_LEFT_AND_RIGHT = 3
LSCA_TUBE_FULL = 4


DESCRIPTOR.enum_types_by_name['LSCA_TUBE_MARKING'] = _LSCA_TUBE_MARKING
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
