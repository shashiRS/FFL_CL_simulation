# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_drvwarnsm/reduce_to_mute_req.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_drvwarnsm/reduce_to_mute_req.proto',
  package='pb.mf_drvwarnsm.reduce_to_mute_req',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n%mf_drvwarnsm/reduce_to_mute_req.proto\x12\"pb.mf_drvwarnsm.reduce_to_mute_req*\x8e\x01\n\x0fReduceToMuteReq\x12\x13\n\x0fPDW_REDUCE_NONE\x10\x00\x12\x13\n\x0fPDW_REDUCE_LVL1\x10\x01\x12\x13\n\x0fPDW_REDUCE_LVL2\x10\x02\x12\x13\n\x0fPDW_REDUCE_LVL3\x10\x03\x12\x13\n\x0fPDW_REDUCE_LVL4\x10\x04\x12\x12\n\x0ePDW_NUM_LEVELS\x10\x05'
)

_REDUCETOMUTEREQ = _descriptor.EnumDescriptor(
  name='ReduceToMuteReq',
  full_name='pb.mf_drvwarnsm.reduce_to_mute_req.ReduceToMuteReq',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='PDW_REDUCE_NONE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PDW_REDUCE_LVL1', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PDW_REDUCE_LVL2', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PDW_REDUCE_LVL3', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PDW_REDUCE_LVL4', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PDW_NUM_LEVELS', index=5, number=5,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=78,
  serialized_end=220,
)
_sym_db.RegisterEnumDescriptor(_REDUCETOMUTEREQ)

ReduceToMuteReq = enum_type_wrapper.EnumTypeWrapper(_REDUCETOMUTEREQ)
PDW_REDUCE_NONE = 0
PDW_REDUCE_LVL1 = 1
PDW_REDUCE_LVL2 = 2
PDW_REDUCE_LVL3 = 3
PDW_REDUCE_LVL4 = 4
PDW_NUM_LEVELS = 5


DESCRIPTOR.enum_types_by_name['ReduceToMuteReq'] = _REDUCETOMUTEREQ
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
