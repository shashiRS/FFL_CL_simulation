# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/hu_ra_sub_feature_state.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/hu_ra_sub_feature_state.proto',
  package='pb.mf_hmih.hu_ra_sub_feature_state',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n%mf_hmih/hu_ra_sub_feature_state.proto\x12\"pb.mf_hmih.hu_ra_sub_feature_state*<\n\x13HuRaSubFeatureState\x12\n\n\x06UNUSED\x10\x00\x12\x19\n\x15REVERSE_ASSIST_ACTIVE\x10\x01'
)

_HURASUBFEATURESTATE = _descriptor.EnumDescriptor(
  name='HuRaSubFeatureState',
  full_name='pb.mf_hmih.hu_ra_sub_feature_state.HuRaSubFeatureState',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNUSED', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REVERSE_ASSIST_ACTIVE', index=1, number=1,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=77,
  serialized_end=137,
)
_sym_db.RegisterEnumDescriptor(_HURASUBFEATURESTATE)

HuRaSubFeatureState = enum_type_wrapper.EnumTypeWrapper(_HURASUBFEATURESTATE)
UNUSED = 0
REVERSE_ASSIST_ACTIVE = 1


DESCRIPTOR.enum_types_by_name['HuRaSubFeatureState'] = _HURASUBFEATURESTATE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
