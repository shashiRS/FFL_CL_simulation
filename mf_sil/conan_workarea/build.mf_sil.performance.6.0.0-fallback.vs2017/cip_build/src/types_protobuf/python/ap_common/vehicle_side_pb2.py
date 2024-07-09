# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_common/vehicle_side.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_common/vehicle_side.proto',
  package='pb.ap_common.vehicle_side',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1c\x61p_common/vehicle_side.proto\x12\x19pb.ap_common.vehicle_side*v\n\x0bVehicleSide\x12\x11\n\rVEHICLE_FRONT\x10\x00\x12\x15\n\x11VEHICLE_LEFT_SIDE\x10\x01\x12\x10\n\x0cVEHICLE_REAR\x10\x02\x12\x16\n\x12VEHICLE_RIGHT_SIDE\x10\x03\x12\x13\n\x0fVEHICLE_NO_SIDE\x10\x04'
)

_VEHICLESIDE = _descriptor.EnumDescriptor(
  name='VehicleSide',
  full_name='pb.ap_common.vehicle_side.VehicleSide',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='VEHICLE_FRONT', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VEHICLE_LEFT_SIDE', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VEHICLE_REAR', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VEHICLE_RIGHT_SIDE', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VEHICLE_NO_SIDE', index=4, number=4,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=59,
  serialized_end=177,
)
_sym_db.RegisterEnumDescriptor(_VEHICLESIDE)

VehicleSide = enum_type_wrapper.EnumTypeWrapper(_VEHICLESIDE)
VEHICLE_FRONT = 0
VEHICLE_LEFT_SIDE = 1
VEHICLE_REAR = 2
VEHICLE_RIGHT_SIDE = 3
VEHICLE_NO_SIDE = 4


DESCRIPTOR.enum_types_by_name['VehicleSide'] = _VEHICLESIDE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
