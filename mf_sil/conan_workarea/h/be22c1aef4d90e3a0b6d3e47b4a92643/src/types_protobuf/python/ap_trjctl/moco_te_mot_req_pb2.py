# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_trjctl/moco_te_mot_req.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_trjctl/moco_te_mot_req.proto',
  package='pb.ap_trjctl.moco_te_mot_req',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1f\x61p_trjctl/moco_te_mot_req.proto\x12\x1cpb.ap_trjctl.moco_te_mot_req*n\n\x0eMOCO_te_MotReq\x12$\n MOCO_MOT_REQ_SUPPRESS_STANDSTILL\x10\x00\x12\x1b\n\x17MOCO_MOT_REQ_STANDSTILL\x10\x01\x12\x19\n\x15MOCO_MOT_REQ_DRIVEOFF\x10\x02'
)

_MOCO_TE_MOTREQ = _descriptor.EnumDescriptor(
  name='MOCO_te_MotReq',
  full_name='pb.ap_trjctl.moco_te_mot_req.MOCO_te_MotReq',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='MOCO_MOT_REQ_SUPPRESS_STANDSTILL', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MOCO_MOT_REQ_STANDSTILL', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MOCO_MOT_REQ_DRIVEOFF', index=2, number=2,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=65,
  serialized_end=175,
)
_sym_db.RegisterEnumDescriptor(_MOCO_TE_MOTREQ)

MOCO_te_MotReq = enum_type_wrapper.EnumTypeWrapper(_MOCO_TE_MOTREQ)
MOCO_MOT_REQ_SUPPRESS_STANDSTILL = 0
MOCO_MOT_REQ_STANDSTILL = 1
MOCO_MOT_REQ_DRIVEOFF = 2


DESCRIPTOR.enum_types_by_name['MOCO_te_MotReq'] = _MOCO_TE_MOTREQ
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
