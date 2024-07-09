# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_manager/mfmdriving_resistance_type.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_manager/mfmdriving_resistance_type.proto',
  package='pb.mf_manager.mfmdriving_resistance_type',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n+mf_manager/mfmdriving_resistance_type.proto\x12(pb.mf_manager.mfmdriving_resistance_type*\xc2\x01\n\x18MFMDrivingResistanceType\x12\x0c\n\x08MFM_NONE\x10\x00\x12\x13\n\x0fMFM_FALLING_LOW\x10\x01\x12\x16\n\x12MFM_FALLING_MEDIUM\x10\x02\x12\x14\n\x10MFM_FALLING_HIGH\x10\x03\x12\x12\n\x0eMFM_RISING_LOW\x10\x04\x12\x15\n\x11MFM_RISING_MEDIUM\x10\x05\x12\x13\n\x0fMFM_RISING_HIGH\x10\x06\x12\x15\n\x11MFM_WHEEL_STOPPER\x10\x07'
)

_MFMDRIVINGRESISTANCETYPE = _descriptor.EnumDescriptor(
  name='MFMDrivingResistanceType',
  full_name='pb.mf_manager.mfmdriving_resistance_type.MFMDrivingResistanceType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='MFM_NONE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MFM_FALLING_LOW', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MFM_FALLING_MEDIUM', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MFM_FALLING_HIGH', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MFM_RISING_LOW', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MFM_RISING_MEDIUM', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MFM_RISING_HIGH', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MFM_WHEEL_STOPPER', index=7, number=7,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=90,
  serialized_end=284,
)
_sym_db.RegisterEnumDescriptor(_MFMDRIVINGRESISTANCETYPE)

MFMDrivingResistanceType = enum_type_wrapper.EnumTypeWrapper(_MFMDRIVINGRESISTANCETYPE)
MFM_NONE = 0
MFM_FALLING_LOW = 1
MFM_FALLING_MEDIUM = 2
MFM_FALLING_HIGH = 3
MFM_RISING_LOW = 4
MFM_RISING_MEDIUM = 5
MFM_RISING_HIGH = 6
MFM_WHEEL_STOPPER = 7


DESCRIPTOR.enum_types_by_name['MFMDrivingResistanceType'] = _MFMDRIVINGRESISTANCETYPE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)