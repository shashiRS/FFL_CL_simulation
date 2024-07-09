# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_processing/us_processing_sensor_parameters.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from us_processing import us_processing_sensor_parameter_pb2 as us__processing_dot_us__processing__sensor__parameter__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_processing/us_processing_sensor_parameters.proto',
  package='pb.us_processing.us_processing_sensor_parameters',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n3us_processing/us_processing_sensor_parameters.proto\x12\x30pb.us_processing.us_processing_sensor_parameters\x1a\x32us_processing/us_processing_sensor_parameter.proto\"\xa5\x01\n\x1cUsProcessingSensorParameters\x12\x1d\n\x14sensorParameterCount\x18\xfc\x15 \x01(\r\x12\x66\n\x0fsensorParameter\x18\xc2\n \x03(\x0b\x32L.pb.us_processing.us_processing_sensor_parameter.UsProcessingSensorParameter\"\x88\x01\n\'UsProcessingSensorParameters_array_port\x12]\n\x04\x64\x61ta\x18\xda\x03 \x03(\x0b\x32N.pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters'
  ,
  dependencies=[us__processing_dot_us__processing__sensor__parameter__pb2.DESCRIPTOR,])




_USPROCESSINGSENSORPARAMETERS = _descriptor.Descriptor(
  name='UsProcessingSensorParameters',
  full_name='pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='sensorParameterCount', full_name='pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters.sensorParameterCount', index=0,
      number=2812, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sensorParameter', full_name='pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters.sensorParameter', index=1,
      number=1346, type=11, cpp_type=10, label=3,
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
  serialized_start=158,
  serialized_end=323,
)


_USPROCESSINGSENSORPARAMETERS_ARRAY_PORT = _descriptor.Descriptor(
  name='UsProcessingSensorParameters_array_port',
  full_name='pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port.data', index=0,
      number=474, type=11, cpp_type=10, label=3,
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
  serialized_start=326,
  serialized_end=462,
)

_USPROCESSINGSENSORPARAMETERS.fields_by_name['sensorParameter'].message_type = us__processing_dot_us__processing__sensor__parameter__pb2._USPROCESSINGSENSORPARAMETER
_USPROCESSINGSENSORPARAMETERS_ARRAY_PORT.fields_by_name['data'].message_type = _USPROCESSINGSENSORPARAMETERS
DESCRIPTOR.message_types_by_name['UsProcessingSensorParameters'] = _USPROCESSINGSENSORPARAMETERS
DESCRIPTOR.message_types_by_name['UsProcessingSensorParameters_array_port'] = _USPROCESSINGSENSORPARAMETERS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsProcessingSensorParameters = _reflection.GeneratedProtocolMessageType('UsProcessingSensorParameters', (_message.Message,), {
  'DESCRIPTOR' : _USPROCESSINGSENSORPARAMETERS,
  '__module__' : 'us_processing.us_processing_sensor_parameters_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters)
  })
_sym_db.RegisterMessage(UsProcessingSensorParameters)

UsProcessingSensorParameters_array_port = _reflection.GeneratedProtocolMessageType('UsProcessingSensorParameters_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USPROCESSINGSENSORPARAMETERS_ARRAY_PORT,
  '__module__' : 'us_processing.us_processing_sensor_parameters_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters_array_port)
  })
_sym_db.RegisterMessage(UsProcessingSensorParameters_array_port)


# @@protoc_insertion_point(module_scope)
