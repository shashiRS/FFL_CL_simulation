# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_vehstatesigprovider/accstatus.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_vehstatesigprovider/accstatus.proto',
  package='pb.ap_vehstatesigprovider.accstatus',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n&ap_vehstatesigprovider/accstatus.proto\x12#pb.ap_vehstatesigprovider.accstatus*\xa1\x01\n\tACCStatus\x12\x0b\n\x07\x41\x43\x43_OFF\x10\x00\x12\x0c\n\x08\x41\x43\x43_INIT\x10\x01\x12\x0f\n\x0b\x41\x43\x43_STANDBY\x10\x02\x12\x0e\n\nACC_ACTIVE\x10\x03\x12\x10\n\x0c\x41\x43\x43_OVERRIDE\x10\x04\x12\x10\n\x0c\x41\x43\x43_TURN_OFF\x10\x05\x12\x18\n\x14\x41\x43\x43_ERROR_REVERSIBLE\x10\x06\x12\x1a\n\x16\x41\x43\x43_ERROR_IRREVERSIBLE\x10\x07'
)

_ACCSTATUS = _descriptor.EnumDescriptor(
  name='ACCStatus',
  full_name='pb.ap_vehstatesigprovider.accstatus.ACCStatus',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='ACC_OFF', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ACC_INIT', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ACC_STANDBY', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ACC_ACTIVE', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ACC_OVERRIDE', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ACC_TURN_OFF', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ACC_ERROR_REVERSIBLE', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ACC_ERROR_IRREVERSIBLE', index=7, number=7,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=80,
  serialized_end=241,
)
_sym_db.RegisterEnumDescriptor(_ACCSTATUS)

ACCStatus = enum_type_wrapper.EnumTypeWrapper(_ACCSTATUS)
ACC_OFF = 0
ACC_INIT = 1
ACC_STANDBY = 2
ACC_ACTIVE = 3
ACC_OVERRIDE = 4
ACC_TURN_OFF = 5
ACC_ERROR_REVERSIBLE = 6
ACC_ERROR_IRREVERSIBLE = 7


DESCRIPTOR.enum_types_by_name['ACCStatus'] = _ACCSTATUS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
