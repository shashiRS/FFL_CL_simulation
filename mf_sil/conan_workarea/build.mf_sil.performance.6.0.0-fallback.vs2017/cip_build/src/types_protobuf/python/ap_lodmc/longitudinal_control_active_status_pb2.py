# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_lodmc/longitudinal_control_active_status.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_lodmc/longitudinal_control_active_status.proto',
  package='pb.ap_lodmc.longitudinal_control_active_status',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n1ap_lodmc/longitudinal_control_active_status.proto\x12.pb.ap_lodmc.longitudinal_control_active_status*\xcc\x01\n\x1flongitudinalControlActiveStatus\x12\x1d\n\x19LODMC_NO_ACTIVE_HANDSHAKE\x10\x00\x12\x1e\n\x1aLODMC_AUP_HANDSHAKE_ACTIVE\x10\x01\x12\x1f\n\x1bLODMC_LSCA_HANDSHAKE_ACTIVE\x10\x02\x12)\n%LODMC_REMOTE_PARKING_HANDSHAKE_ACTIVE\x10\x03\x12\x1e\n\x1aLODMC_MSP_HANDSHAKE_ACTIVE\x10\x04'
)

_LONGITUDINALCONTROLACTIVESTATUS = _descriptor.EnumDescriptor(
  name='longitudinalControlActiveStatus',
  full_name='pb.ap_lodmc.longitudinal_control_active_status.longitudinalControlActiveStatus',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='LODMC_NO_ACTIVE_HANDSHAKE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LODMC_AUP_HANDSHAKE_ACTIVE', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LODMC_LSCA_HANDSHAKE_ACTIVE', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LODMC_REMOTE_PARKING_HANDSHAKE_ACTIVE', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LODMC_MSP_HANDSHAKE_ACTIVE', index=4, number=4,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=102,
  serialized_end=306,
)
_sym_db.RegisterEnumDescriptor(_LONGITUDINALCONTROLACTIVESTATUS)

longitudinalControlActiveStatus = enum_type_wrapper.EnumTypeWrapper(_LONGITUDINALCONTROLACTIVESTATUS)
LODMC_NO_ACTIVE_HANDSHAKE = 0
LODMC_AUP_HANDSHAKE_ACTIVE = 1
LODMC_LSCA_HANDSHAKE_ACTIVE = 2
LODMC_REMOTE_PARKING_HANDSHAKE_ACTIVE = 3
LODMC_MSP_HANDSHAKE_ACTIVE = 4


DESCRIPTOR.enum_types_by_name['longitudinalControlActiveStatus'] = _LONGITUDINALCONTROLACTIVESTATUS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)