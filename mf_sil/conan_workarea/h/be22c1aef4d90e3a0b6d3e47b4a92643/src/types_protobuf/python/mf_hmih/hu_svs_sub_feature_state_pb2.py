# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/hu_svs_sub_feature_state.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/hu_svs_sub_feature_state.proto',
  package='pb.mf_hmih.hu_svs_sub_feature_state',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n&mf_hmih/hu_svs_sub_feature_state.proto\x12#pb.mf_hmih.hu_svs_sub_feature_state*\x81\x01\n\x14HuSVsSubFeatureState\x12\n\n\x06UNUSED\x10\x00\x12\x11\n\rSVS_2D_ACTIVE\x10\x01\x12\x11\n\rSVS_3D_ACTIVE\x10\x02\x12\x1b\n\x17SVS_FRONT_CORNER_ACTIVE\x10\x03\x12\x1a\n\x16SVS_REAR_CORNER_ACTIVE\x10\x04'
)

_HUSVSSUBFEATURESTATE = _descriptor.EnumDescriptor(
  name='HuSVsSubFeatureState',
  full_name='pb.mf_hmih.hu_svs_sub_feature_state.HuSVsSubFeatureState',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNUSED', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SVS_2D_ACTIVE', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SVS_3D_ACTIVE', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SVS_FRONT_CORNER_ACTIVE', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SVS_REAR_CORNER_ACTIVE', index=4, number=4,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=80,
  serialized_end=209,
)
_sym_db.RegisterEnumDescriptor(_HUSVSSUBFEATURESTATE)

HuSVsSubFeatureState = enum_type_wrapper.EnumTypeWrapper(_HUSVSSUBFEATURESTATE)
UNUSED = 0
SVS_2D_ACTIVE = 1
SVS_3D_ACTIVE = 2
SVS_FRONT_CORNER_ACTIVE = 3
SVS_REAR_CORNER_ACTIVE = 4


DESCRIPTOR.enum_types_by_name['HuSVsSubFeatureState'] = _HUSVSSUBFEATURESTATE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
