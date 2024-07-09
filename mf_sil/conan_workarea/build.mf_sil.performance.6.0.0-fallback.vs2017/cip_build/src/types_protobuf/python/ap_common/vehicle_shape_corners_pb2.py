# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_common/vehicle_shape_corners.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_common/vehicle_shape_corners.proto',
  package='pb.ap_common.vehicle_shape_corners',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n%ap_common/vehicle_shape_corners.proto\x12\"pb.ap_common.vehicle_shape_corners*\x8e\x01\n\x13VehicleShapeCorners\x12\x0e\n\nFRONT_LEFT\x10\x00\x12\r\n\tAXIS_LEFT\x10\x01\x12\r\n\tREAR_LEFT\x10\x02\x12\x0e\n\nREAR_RIGHT\x10\x03\x12\x0e\n\nAXIS_RIGHT\x10\x04\x12\x0f\n\x0b\x46RONT_RIGHT\x10\x05\x12\x18\n\x14NUM_RELEVANT_CORNERS\x10\x06'
)

_VEHICLESHAPECORNERS = _descriptor.EnumDescriptor(
  name='VehicleShapeCorners',
  full_name='pb.ap_common.vehicle_shape_corners.VehicleShapeCorners',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='FRONT_LEFT', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='AXIS_LEFT', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REAR_LEFT', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REAR_RIGHT', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='AXIS_RIGHT', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FRONT_RIGHT', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NUM_RELEVANT_CORNERS', index=6, number=6,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=78,
  serialized_end=220,
)
_sym_db.RegisterEnumDescriptor(_VEHICLESHAPECORNERS)

VehicleShapeCorners = enum_type_wrapper.EnumTypeWrapper(_VEHICLESHAPECORNERS)
FRONT_LEFT = 0
AXIS_LEFT = 1
REAR_LEFT = 2
REAR_RIGHT = 3
AXIS_RIGHT = 4
FRONT_RIGHT = 5
NUM_RELEVANT_CORNERS = 6


DESCRIPTOR.enum_types_by_name['VehicleShapeCorners'] = _VEHICLESHAPECORNERS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
