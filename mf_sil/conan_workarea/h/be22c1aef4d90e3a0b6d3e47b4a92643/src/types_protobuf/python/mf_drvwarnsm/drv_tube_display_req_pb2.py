# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_drvwarnsm/drv_tube_display_req.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_drvwarnsm/drv_tube_display_req.proto',
  package='pb.mf_drvwarnsm.drv_tube_display_req',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\'mf_drvwarnsm/drv_tube_display_req.proto\x12$pb.mf_drvwarnsm.drv_tube_display_req*e\n\x11\x44rvTubeDisplayReq\x12\x19\n\x15PDW_DRV_TUBE_REQ_NONE\x10\x00\x12\x1a\n\x16PDW_DRV_TUBE_REQ_FRONT\x10\x01\x12\x19\n\x15PDW_DRV_TUBE_REQ_REAR\x10\x02'
)

_DRVTUBEDISPLAYREQ = _descriptor.EnumDescriptor(
  name='DrvTubeDisplayReq',
  full_name='pb.mf_drvwarnsm.drv_tube_display_req.DrvTubeDisplayReq',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='PDW_DRV_TUBE_REQ_NONE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PDW_DRV_TUBE_REQ_FRONT', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PDW_DRV_TUBE_REQ_REAR', index=2, number=2,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=81,
  serialized_end=182,
)
_sym_db.RegisterEnumDescriptor(_DRVTUBEDISPLAYREQ)

DrvTubeDisplayReq = enum_type_wrapper.EnumTypeWrapper(_DRVTUBEDISPLAYREQ)
PDW_DRV_TUBE_REQ_NONE = 0
PDW_DRV_TUBE_REQ_FRONT = 1
PDW_DRV_TUBE_REQ_REAR = 2


DESCRIPTOR.enum_types_by_name['DrvTubeDisplayReq'] = _DRVTUBEDISPLAYREQ
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
