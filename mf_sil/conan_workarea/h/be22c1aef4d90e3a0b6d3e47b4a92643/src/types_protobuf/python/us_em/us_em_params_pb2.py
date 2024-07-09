# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_em/us_em_params.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from us_em import us_em_neighboring_filter_cfg_intern_pb2 as us__em_dot_us__em__neighboring__filter__cfg__intern__pb2
from us_em import us_em_detection_zone_cfg_pb2 as us__em_dot_us__em__detection__zone__cfg__pb2
from us_em import us_em_sensor_parameters_intern_pb2 as us__em_dot_us__em__sensor__parameters__intern__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_em/us_em_params.proto',
  package='pb.us_em.us_em_params',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x18us_em/us_em_params.proto\x12\x15pb.us_em.us_em_params\x1a\x17\x65\x63o/signal_header.proto\x1a/us_em/us_em_neighboring_filter_cfg_intern.proto\x1a$us_em/us_em_detection_zone_cfg.proto\x1a*us_em/us_em_sensor_parameters_intern.proto\"\xf0\x02\n\nUsEmParams\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x61\n\nuspcParams\x18\xda\r \x01(\x0b\x32L.pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern\x12N\n\rdetZoneParams\x18Y \x01(\x0b\x32\x37.pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg\x12\\\n\x0eusSensorParams\x18\xa1\x18 \x01(\x0b\x32\x43.pb.us_em.us_em_sensor_parameters_intern.UsEmSensorParametersIntern\"I\n\x15UsEmParams_array_port\x12\x30\n\x04\x64\x61ta\x18\xe5\x11 \x03(\x0b\x32!.pb.us_em.us_em_params.UsEmParams'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,us__em_dot_us__em__neighboring__filter__cfg__intern__pb2.DESCRIPTOR,us__em_dot_us__em__detection__zone__cfg__pb2.DESCRIPTOR,us__em_dot_us__em__sensor__parameters__intern__pb2.DESCRIPTOR,])




_USEMPARAMS = _descriptor.Descriptor(
  name='UsEmParams',
  full_name='pb.us_em.us_em_params.UsEmParams',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.us_em.us_em_params.UsEmParams.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.us_em.us_em_params.UsEmParams.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='uspcParams', full_name='pb.us_em.us_em_params.UsEmParams.uspcParams', index=2,
      number=1754, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='detZoneParams', full_name='pb.us_em.us_em_params.UsEmParams.detZoneParams', index=3,
      number=89, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='usSensorParams', full_name='pb.us_em.us_em_params.UsEmParams.usSensorParams', index=4,
      number=3105, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
  serialized_start=208,
  serialized_end=576,
)


_USEMPARAMS_ARRAY_PORT = _descriptor.Descriptor(
  name='UsEmParams_array_port',
  full_name='pb.us_em.us_em_params.UsEmParams_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_em.us_em_params.UsEmParams_array_port.data', index=0,
      number=2277, type=11, cpp_type=10, label=3,
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
  serialized_start=578,
  serialized_end=651,
)

_USEMPARAMS.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_USEMPARAMS.fields_by_name['uspcParams'].message_type = us__em_dot_us__em__neighboring__filter__cfg__intern__pb2._USEMNEIGHBORINGFILTERCFGINTERN
_USEMPARAMS.fields_by_name['detZoneParams'].message_type = us__em_dot_us__em__detection__zone__cfg__pb2._USEMDETECTIONZONECFG
_USEMPARAMS.fields_by_name['usSensorParams'].message_type = us__em_dot_us__em__sensor__parameters__intern__pb2._USEMSENSORPARAMETERSINTERN
_USEMPARAMS_ARRAY_PORT.fields_by_name['data'].message_type = _USEMPARAMS
DESCRIPTOR.message_types_by_name['UsEmParams'] = _USEMPARAMS
DESCRIPTOR.message_types_by_name['UsEmParams_array_port'] = _USEMPARAMS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsEmParams = _reflection.GeneratedProtocolMessageType('UsEmParams', (_message.Message,), {
  'DESCRIPTOR' : _USEMPARAMS,
  '__module__' : 'us_em.us_em_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_em.us_em_params.UsEmParams)
  })
_sym_db.RegisterMessage(UsEmParams)

UsEmParams_array_port = _reflection.GeneratedProtocolMessageType('UsEmParams_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USEMPARAMS_ARRAY_PORT,
  '__module__' : 'us_em.us_em_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_em.us_em_params.UsEmParams_array_port)
  })
_sym_db.RegisterMessage(UsEmParams_array_port)


# @@protoc_insertion_point(module_scope)
