# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/hu_garage_park_sub_feature_state.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/hu_garage_park_sub_feature_state.proto',
  package='pb.mf_hmih.hu_garage_park_sub_feature_state',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n.mf_hmih/hu_garage_park_sub_feature_state.proto\x12+pb.mf_hmih.hu_garage_park_sub_feature_state*=\n\x1bHuGarageParkSubFeatureState\x12\n\n\x06UNUSED\x10\x00\x12\x12\n\x0eGARAGE_PARKING\x10\x01'
)

_HUGARAGEPARKSUBFEATURESTATE = _descriptor.EnumDescriptor(
  name='HuGarageParkSubFeatureState',
  full_name='pb.mf_hmih.hu_garage_park_sub_feature_state.HuGarageParkSubFeatureState',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNUSED', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GARAGE_PARKING', index=1, number=1,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=95,
  serialized_end=156,
)
_sym_db.RegisterEnumDescriptor(_HUGARAGEPARKSUBFEATURESTATE)

HuGarageParkSubFeatureState = enum_type_wrapper.EnumTypeWrapper(_HUGARAGEPARKSUBFEATURESTATE)
UNUSED = 0
GARAGE_PARKING = 1


DESCRIPTOR.enum_types_by_name['HuGarageParkSubFeatureState'] = _HUGARAGEPARKSUBFEATURESTATE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)