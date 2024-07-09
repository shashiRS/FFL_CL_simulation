# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_em/us_em_detection_zone_cfg.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_em/us_em_detection_zone_cfg.proto',
  package='pb.us_em.us_em_detection_zone_cfg',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n$us_em/us_em_detection_zone_cfg.proto\x12!pb.us_em.us_em_detection_zone_cfg\"\xf6\x01\n\x14UsEmDetectionZoneCfg\x12\x1a\n\x11\x64\x65tZoneFrontLeftX\x18\xf0\x1e \x01(\x02\x12\x1a\n\x11\x64\x65tZoneFrontLeftY\x18\xd1\x1e \x01(\x02\x12\x1b\n\x12\x64\x65tZoneFrontRightX\x18\xf8\x1f \x01(\x02\x12\x1b\n\x12\x64\x65tZoneFrontRightY\x18\xd9\x1f \x01(\x02\x12\x19\n\x10\x64\x65tZoneBackLeftX\x18\xa0\t \x01(\x02\x12\x19\n\x10\x64\x65tZoneBackLeftY\x18\x81\t \x01(\x02\x12\x1a\n\x11\x64\x65tZoneBackRightX\x18\x8c\x03 \x01(\x02\x12\x1a\n\x11\x64\x65tZoneBackRightY\x18\xad\x03 \x01(\x02\"i\n\x1fUsEmDetectionZoneCfg_array_port\x12\x46\n\x04\x64\x61ta\x18\xbf\x17 \x03(\x0b\x32\x37.pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg'
)




_USEMDETECTIONZONECFG = _descriptor.Descriptor(
  name='UsEmDetectionZoneCfg',
  full_name='pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='detZoneFrontLeftX', full_name='pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneFrontLeftX', index=0,
      number=3952, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='detZoneFrontLeftY', full_name='pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneFrontLeftY', index=1,
      number=3921, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='detZoneFrontRightX', full_name='pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneFrontRightX', index=2,
      number=4088, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='detZoneFrontRightY', full_name='pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneFrontRightY', index=3,
      number=4057, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='detZoneBackLeftX', full_name='pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneBackLeftX', index=4,
      number=1184, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='detZoneBackLeftY', full_name='pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneBackLeftY', index=5,
      number=1153, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='detZoneBackRightX', full_name='pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneBackRightX', index=6,
      number=396, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='detZoneBackRightY', full_name='pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneBackRightY', index=7,
      number=429, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=76,
  serialized_end=322,
)


_USEMDETECTIONZONECFG_ARRAY_PORT = _descriptor.Descriptor(
  name='UsEmDetectionZoneCfg_array_port',
  full_name='pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg_array_port.data', index=0,
      number=3007, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=324,
  serialized_end=429,
)

_USEMDETECTIONZONECFG_ARRAY_PORT.fields_by_name['data'].message_type = _USEMDETECTIONZONECFG
DESCRIPTOR.message_types_by_name['UsEmDetectionZoneCfg'] = _USEMDETECTIONZONECFG
DESCRIPTOR.message_types_by_name['UsEmDetectionZoneCfg_array_port'] = _USEMDETECTIONZONECFG_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsEmDetectionZoneCfg = _reflection.GeneratedProtocolMessageType('UsEmDetectionZoneCfg', (_message.Message,), {
  'DESCRIPTOR' : _USEMDETECTIONZONECFG,
  '__module__' : 'us_em.us_em_detection_zone_cfg_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg)
  })
_sym_db.RegisterMessage(UsEmDetectionZoneCfg)

UsEmDetectionZoneCfg_array_port = _reflection.GeneratedProtocolMessageType('UsEmDetectionZoneCfg_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USEMDETECTIONZONECFG_ARRAY_PORT,
  '__module__' : 'us_em.us_em_detection_zone_cfg_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg_array_port)
  })
_sym_db.RegisterMessage(UsEmDetectionZoneCfg_array_port)


# @@protoc_insertion_point(module_scope)
