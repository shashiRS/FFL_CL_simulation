# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_manager/lo_ctrl_request_source.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_manager/lo_ctrl_request_source.proto',
  package='pb.mf_manager.lo_ctrl_request_source',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\'mf_manager/lo_ctrl_request_source.proto\x12$pb.mf_manager.lo_ctrl_request_source*\xdf\x01\n\x13loCtrlRequestSource\x12\x1f\n\x1bLODMC_REQ_SRC_NO_REQEUESTER\x10\x00\x12\x15\n\x11LODMC_REQ_SRC_AUP\x10\x01\x12!\n\x1dLODMC_REQ_SRC_AUP_WITH_REMOTE\x10\x06\x12\x16\n\x12LODMC_REQ_SRC_LSCA\x10\x02\x12$\n LODMC_REQ_SRC_REMOTE_MANEUVERING\x10\x03\x12\x15\n\x11LODMC_REQ_SRC_MSP\x10\x04\x12\x18\n\x14LODMC_REQ_SRC_SAFBAR\x10\x05'
)

_LOCTRLREQUESTSOURCE = _descriptor.EnumDescriptor(
  name='loCtrlRequestSource',
  full_name='pb.mf_manager.lo_ctrl_request_source.loCtrlRequestSource',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='LODMC_REQ_SRC_NO_REQEUESTER', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LODMC_REQ_SRC_AUP', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LODMC_REQ_SRC_AUP_WITH_REMOTE', index=2, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LODMC_REQ_SRC_LSCA', index=3, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LODMC_REQ_SRC_REMOTE_MANEUVERING', index=4, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LODMC_REQ_SRC_MSP', index=5, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LODMC_REQ_SRC_SAFBAR', index=6, number=5,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=82,
  serialized_end=305,
)
_sym_db.RegisterEnumDescriptor(_LOCTRLREQUESTSOURCE)

loCtrlRequestSource = enum_type_wrapper.EnumTypeWrapper(_LOCTRLREQUESTSOURCE)
LODMC_REQ_SRC_NO_REQEUESTER = 0
LODMC_REQ_SRC_AUP = 1
LODMC_REQ_SRC_AUP_WITH_REMOTE = 6
LODMC_REQ_SRC_LSCA = 2
LODMC_REQ_SRC_REMOTE_MANEUVERING = 3
LODMC_REQ_SRC_MSP = 4
LODMC_REQ_SRC_SAFBAR = 5


DESCRIPTOR.enum_types_by_name['loCtrlRequestSource'] = _LOCTRLREQUESTSOURCE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
