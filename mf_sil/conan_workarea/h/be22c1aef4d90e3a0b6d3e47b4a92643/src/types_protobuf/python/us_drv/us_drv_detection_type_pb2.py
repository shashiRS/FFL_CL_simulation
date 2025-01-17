# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_drv/us_drv_detection_type.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_drv/us_drv_detection_type.proto',
  package='pb.us_drv.us_drv_detection_type',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\"us_drv/us_drv_detection_type.proto\x12\x1fpb.us_drv.us_drv_detection_type*\xba\x02\n\x12UsDrvDetectionType\x12 \n\x1cUS_DETECTION_CONFIDENCE_MASK\x10\x0f\x12\x1c\n\x18US_DETECTION_NORMAL_PATH\x10\x00\x12 \n\x1cUS_DETECTION_ADVANCED_PATH_1\x10\x10\x12 \n\x1cUS_DETECTION_ADVANCED_PATH_2\x10 \x12\x19\n\x15US_DETECTION_NFD_PATH\x10\x30\x12!\n\x1dUS_DETECTION_AATG_THRESHOLD_1\x10@\x12!\n\x1dUS_DETECTION_AATG_THRESHOLD_2\x10P\x12\"\n\x1dUS_DETECTION_FIRING_TIMESTAMP\x10\x80\x01\x12\x1b\n\x16US_DETECTION_TYPE_MASK\x10\xf0\x01'
)

_USDRVDETECTIONTYPE = _descriptor.EnumDescriptor(
  name='UsDrvDetectionType',
  full_name='pb.us_drv.us_drv_detection_type.UsDrvDetectionType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='US_DETECTION_CONFIDENCE_MASK', index=0, number=15,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_DETECTION_NORMAL_PATH', index=1, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_DETECTION_ADVANCED_PATH_1', index=2, number=16,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_DETECTION_ADVANCED_PATH_2', index=3, number=32,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_DETECTION_NFD_PATH', index=4, number=48,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_DETECTION_AATG_THRESHOLD_1', index=5, number=64,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_DETECTION_AATG_THRESHOLD_2', index=6, number=80,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_DETECTION_FIRING_TIMESTAMP', index=7, number=128,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='US_DETECTION_TYPE_MASK', index=8, number=240,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=72,
  serialized_end=386,
)
_sym_db.RegisterEnumDescriptor(_USDRVDETECTIONTYPE)

UsDrvDetectionType = enum_type_wrapper.EnumTypeWrapper(_USDRVDETECTIONTYPE)
US_DETECTION_CONFIDENCE_MASK = 15
US_DETECTION_NORMAL_PATH = 0
US_DETECTION_ADVANCED_PATH_1 = 16
US_DETECTION_ADVANCED_PATH_2 = 32
US_DETECTION_NFD_PATH = 48
US_DETECTION_AATG_THRESHOLD_1 = 64
US_DETECTION_AATG_THRESHOLD_2 = 80
US_DETECTION_FIRING_TIMESTAMP = 128
US_DETECTION_TYPE_MASK = 240


DESCRIPTOR.enum_types_by_name['UsDrvDetectionType'] = _USDRVDETECTIONTYPE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
