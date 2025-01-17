# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_processing/us_processing_object_type.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_processing/us_processing_object_type.proto',
  package='pb.us_processing.us_processing_object_type',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n-us_processing/us_processing_object_type.proto\x12*pb.us_processing.us_processing_object_type*\xe9\x01\n\x16UsProcessingObjectType\x12!\n\x1dUS_PROCESSING_OBJ_TYPE_UNKOWN\x10\x00\x12\x1f\n\x1bUS_PROCESSING_OBJ_TYPE_POLE\x10\x01\x12\x1f\n\x1bUS_PROCESSING_OBJ_TYPE_WALL\x10\x02\x12\x1f\n\x1bUS_PROCESSING_OBJ_TYPE_CURB\x10\x03\x12\"\n\x1eUS_PROCESSING_OBJ_TYPE_VEHICLE\x10\x04\x12%\n!US_PROCESSING_OBJ_TYPE_PEDESTRIAN\x10\x05'
)

_USPROCESSINGOBJECTTYPE = _descriptor.EnumDescriptor(
  name='UsProcessingObjectType',
  full_name='pb.us_processing.us_processing_object_type.UsProcessingObjectType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='US_PROCESSING_OBJ_TYPE_UNKOWN', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_PROCESSING_OBJ_TYPE_POLE', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_PROCESSING_OBJ_TYPE_WALL', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_PROCESSING_OBJ_TYPE_CURB', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_PROCESSING_OBJ_TYPE_VEHICLE', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_PROCESSING_OBJ_TYPE_PEDESTRIAN', index=5, number=5,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=94,
  serialized_end=327,
)
_sym_db.RegisterEnumDescriptor(_USPROCESSINGOBJECTTYPE)

UsProcessingObjectType = enum_type_wrapper.EnumTypeWrapper(_USPROCESSINGOBJECTTYPE)
US_PROCESSING_OBJ_TYPE_UNKOWN = 0
US_PROCESSING_OBJ_TYPE_POLE = 1
US_PROCESSING_OBJ_TYPE_WALL = 2
US_PROCESSING_OBJ_TYPE_CURB = 3
US_PROCESSING_OBJ_TYPE_VEHICLE = 4
US_PROCESSING_OBJ_TYPE_PEDESTRIAN = 5


DESCRIPTOR.enum_types_by_name['UsProcessingObjectType'] = _USPROCESSINGOBJECTTYPE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
