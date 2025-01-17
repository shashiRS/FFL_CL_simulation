# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_psm_app/garage_parking.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_psm_app/garage_parking.proto',
  package='pb.ap_psm_app.garage_parking',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1f\x61p_psm_app/garage_parking.proto\x12\x1cpb.ap_psm_app.garage_parking*\x9d\x01\n\rGarageParking\x12\x14\n\x10GP_NOT_AVAILABLE\x10\x00\x12\x12\n\x0eGP_IN_SCAN_FWD\x10\x01\x12\x12\n\x0eGP_IN_SCAN_BWD\x10\x02\x12\x0e\n\nGP_OUT_FWD\x10\x03\x12\x0e\n\nGP_OUT_BWD\x10\x04\x12\x16\n\x12GP_IN_DETECTED_FWD\x10\x05\x12\x16\n\x12GP_IN_DETECTED_BWD\x10\x06'
)

_GARAGEPARKING = _descriptor.EnumDescriptor(
  name='GarageParking',
  full_name='pb.ap_psm_app.garage_parking.GarageParking',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='GP_NOT_AVAILABLE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GP_IN_SCAN_FWD', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GP_IN_SCAN_BWD', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GP_OUT_FWD', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GP_OUT_BWD', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GP_IN_DETECTED_FWD', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GP_IN_DETECTED_BWD', index=6, number=6,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=66,
  serialized_end=223,
)
_sym_db.RegisterEnumDescriptor(_GARAGEPARKING)

GarageParking = enum_type_wrapper.EnumTypeWrapper(_GARAGEPARKING)
GP_NOT_AVAILABLE = 0
GP_IN_SCAN_FWD = 1
GP_IN_SCAN_BWD = 2
GP_OUT_FWD = 3
GP_OUT_BWD = 4
GP_IN_DETECTED_FWD = 5
GP_IN_DETECTED_BWD = 6


DESCRIPTOR.enum_types_by_name['GarageParking'] = _GARAGEPARKING
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
