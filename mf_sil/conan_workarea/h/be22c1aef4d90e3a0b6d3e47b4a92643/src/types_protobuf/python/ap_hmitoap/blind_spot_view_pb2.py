# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_hmitoap/blind_spot_view.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_hmitoap/blind_spot_view.proto',
  package='pb.ap_hmitoap.blind_spot_view',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n ap_hmitoap/blind_spot_view.proto\x12\x1dpb.ap_hmitoap.blind_spot_view*N\n\rBlindSpotView\x12\x1e\n\x1a\x42LIND_SPOT_VIEW_ACTIVE_HMI\x10\x00\x12\x1d\n\x19\x42LIND_SPOT_VIEW_ACTIVE_IC\x10\x01'
)

_BLINDSPOTVIEW = _descriptor.EnumDescriptor(
  name='BlindSpotView',
  full_name='pb.ap_hmitoap.blind_spot_view.BlindSpotView',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='BLIND_SPOT_VIEW_ACTIVE_HMI', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BLIND_SPOT_VIEW_ACTIVE_IC', index=1, number=1,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=67,
  serialized_end=145,
)
_sym_db.RegisterEnumDescriptor(_BLINDSPOTVIEW)

BlindSpotView = enum_type_wrapper.EnumTypeWrapper(_BLINDSPOTVIEW)
BLIND_SPOT_VIEW_ACTIVE_HMI = 0
BLIND_SPOT_VIEW_ACTIVE_IC = 1


DESCRIPTOR.enum_types_by_name['BlindSpotView'] = _BLINDSPOTVIEW
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)