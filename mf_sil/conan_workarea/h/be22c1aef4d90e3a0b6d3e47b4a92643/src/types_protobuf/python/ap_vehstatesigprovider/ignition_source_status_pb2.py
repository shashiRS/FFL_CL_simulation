# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_vehstatesigprovider/ignition_source_status.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_vehstatesigprovider/ignition_source_status.proto',
  package='pb.ap_vehstatesigprovider.ignition_source_status',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n3ap_vehstatesigprovider/ignition_source_status.proto\x12\x30pb.ap_vehstatesigprovider.ignition_source_status*I\n\x14IgnitionSourceStatus\x12\n\n\x06\x44RIVER\x10\x00\x12\n\n\x06REMOTE\x10\x01\x12\x0c\n\x08RESERVED\x10\x02\x12\x0b\n\x07INVALID\x10\x03'
)

_IGNITIONSOURCESTATUS = _descriptor.EnumDescriptor(
  name='IgnitionSourceStatus',
  full_name='pb.ap_vehstatesigprovider.ignition_source_status.IgnitionSourceStatus',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='DRIVER', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REMOTE', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RESERVED', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INVALID', index=3, number=3,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=105,
  serialized_end=178,
)
_sym_db.RegisterEnumDescriptor(_IGNITIONSOURCESTATUS)

IgnitionSourceStatus = enum_type_wrapper.EnumTypeWrapper(_IGNITIONSOURCESTATUS)
DRIVER = 0
REMOTE = 1
RESERVED = 2
INVALID = 3


DESCRIPTOR.enum_types_by_name['IgnitionSourceStatus'] = _IGNITIONSOURCESTATUS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
