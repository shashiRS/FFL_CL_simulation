# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_tp/pos_def_approach.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_tp/pos_def_approach.proto',
  package='pb.ap_tp.pos_def_approach',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1c\x61p_tp/pos_def_approach.proto\x12\x19pb.ap_tp.pos_def_approach*j\n\x0ePosDefApproach\x12\x16\n\x12\x41TTACH_EGO_TO_EDGE\x10\x00\x12\r\n\tCENTERING\x10\x01\x12\x17\n\x13ROAD_SIDE_CENTERING\x10\x02\x12\x18\n\x14SHORT_SIDE_CENTERING\x10\x03'
)

_POSDEFAPPROACH = _descriptor.EnumDescriptor(
  name='PosDefApproach',
  full_name='pb.ap_tp.pos_def_approach.PosDefApproach',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='ATTACH_EGO_TO_EDGE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CENTERING', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ROAD_SIDE_CENTERING', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SHORT_SIDE_CENTERING', index=3, number=3,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=59,
  serialized_end=165,
)
_sym_db.RegisterEnumDescriptor(_POSDEFAPPROACH)

PosDefApproach = enum_type_wrapper.EnumTypeWrapper(_POSDEFAPPROACH)
ATTACH_EGO_TO_EDGE = 0
CENTERING = 1
ROAD_SIDE_CENTERING = 2
SHORT_SIDE_CENTERING = 3


DESCRIPTOR.enum_types_by_name['PosDefApproach'] = _POSDEFAPPROACH
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)