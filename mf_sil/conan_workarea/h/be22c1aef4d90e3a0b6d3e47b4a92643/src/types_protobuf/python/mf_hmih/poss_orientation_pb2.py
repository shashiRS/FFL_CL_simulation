# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/poss_orientation.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/poss_orientation.proto',
  package='pb.mf_hmih.poss_orientation',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1emf_hmih/poss_orientation.proto\x12\x1bpb.mf_hmih.poss_orientation*?\n\x0fPossOrientation\x12\x14\n\x10POSS_ORI_CERTAIN\x10\x00\x12\x16\n\x12POSS_ORI_UNCERTAIN\x10\x01'
)

_POSSORIENTATION = _descriptor.EnumDescriptor(
  name='PossOrientation',
  full_name='pb.mf_hmih.poss_orientation.PossOrientation',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='POSS_ORI_CERTAIN', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='POSS_ORI_UNCERTAIN', index=1, number=1,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=63,
  serialized_end=126,
)
_sym_db.RegisterEnumDescriptor(_POSSORIENTATION)

PossOrientation = enum_type_wrapper.EnumTypeWrapper(_POSSORIENTATION)
POSS_ORI_CERTAIN = 0
POSS_ORI_UNCERTAIN = 1


DESCRIPTOR.enum_types_by_name['PossOrientation'] = _POSSORIENTATION
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
