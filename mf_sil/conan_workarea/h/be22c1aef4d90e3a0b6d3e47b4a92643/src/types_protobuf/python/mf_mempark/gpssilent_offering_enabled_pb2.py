# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/gpssilent_offering_enabled.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/gpssilent_offering_enabled.proto',
  package='pb.mf_mempark.gpssilent_offering_enabled',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n+mf_mempark/gpssilent_offering_enabled.proto\x12(pb.mf_mempark.gpssilent_offering_enabled*I\n\x18GPSSilentOfferingEnabled\x12\x15\n\x11GPS_OFFER_ENABLED\x10\x00\x12\x16\n\x12GPS_OFFER_DISABLED\x10\x01'
)

_GPSSILENTOFFERINGENABLED = _descriptor.EnumDescriptor(
  name='GPSSilentOfferingEnabled',
  full_name='pb.mf_mempark.gpssilent_offering_enabled.GPSSilentOfferingEnabled',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='GPS_OFFER_ENABLED', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GPS_OFFER_DISABLED', index=1, number=1,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=89,
  serialized_end=162,
)
_sym_db.RegisterEnumDescriptor(_GPSSILENTOFFERINGENABLED)

GPSSilentOfferingEnabled = enum_type_wrapper.EnumTypeWrapper(_GPSSILENTOFFERINGENABLED)
GPS_OFFER_ENABLED = 0
GPS_OFFER_DISABLED = 1


DESCRIPTOR.enum_types_by_name['GPSSilentOfferingEnabled'] = _GPSSILENTOFFERINGENABLED
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)