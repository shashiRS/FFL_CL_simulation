# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_psm/driver_in_out_request_type.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_psm/driver_in_out_request_type.proto',
  package='pb.ap_psm.driver_in_out_request_type',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\'ap_psm/driver_in_out_request_type.proto\x12$pb.ap_psm.driver_in_out_request_type*O\n\x16\x44riverInOutRequestType\x12\x19\n\x15PSM_DRIVER_IN_REQUEST\x10\x00\x12\x1a\n\x16PSM_DRIVER_OUT_REQUEST\x10\x01'
)

_DRIVERINOUTREQUESTTYPE = _descriptor.EnumDescriptor(
  name='DriverInOutRequestType',
  full_name='pb.ap_psm.driver_in_out_request_type.DriverInOutRequestType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='PSM_DRIVER_IN_REQUEST', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PSM_DRIVER_OUT_REQUEST', index=1, number=1,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=81,
  serialized_end=160,
)
_sym_db.RegisterEnumDescriptor(_DRIVERINOUTREQUESTTYPE)

DriverInOutRequestType = enum_type_wrapper.EnumTypeWrapper(_DRIVERINOUTREQUESTTYPE)
PSM_DRIVER_IN_REQUEST = 0
PSM_DRIVER_OUT_REQUEST = 1


DESCRIPTOR.enum_types_by_name['DriverInOutRequestType'] = _DRIVERINOUTREQUESTTYPE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)