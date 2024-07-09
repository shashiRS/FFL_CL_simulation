# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_drv/us_drv_sensor_request.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from us_drv import us_drv_request_type_pb2 as us__drv_dot_us__drv__request__type__pb2
from us_drv import us_drv_request_state_pb2 as us__drv_dot_us__drv__request__state__pb2
from us_drv import us_drv_request_meas_mode_pb2 as us__drv_dot_us__drv__request__meas__mode__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_drv/us_drv_sensor_request.proto',
  package='pb.us_drv.us_drv_sensor_request',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\"us_drv/us_drv_sensor_request.proto\x12\x1fpb.us_drv.us_drv_sensor_request\x1a us_drv/us_drv_request_type.proto\x1a!us_drv/us_drv_request_state.proto\x1a%us_drv/us_drv_request_meas_mode.proto\"\xe4\x01\n\x12UsDrvSensorRequest\x12>\n\x04type\x18\xde\x11 \x01(\x0e\x32/.pb.us_drv.us_drv_request_type.UsDrvRequestType\x12\x41\n\x05state\x18\xaf\x14 \x01(\x0e\x32\x31.pb.us_drv.us_drv_request_state.UsDrvRequestState\x12K\n\x08measMode\x18\x97\x07 \x01(\x0e\x32\x38.pb.us_drv.us_drv_request_meas_mode.UsDrvRequestMeasMode\"c\n\x1dUsDrvSensorRequest_array_port\x12\x42\n\x04\x64\x61ta\x18\xdb\x17 \x03(\x0b\x32\x33.pb.us_drv.us_drv_sensor_request.UsDrvSensorRequest'
  ,
  dependencies=[us__drv_dot_us__drv__request__type__pb2.DESCRIPTOR,us__drv_dot_us__drv__request__state__pb2.DESCRIPTOR,us__drv_dot_us__drv__request__meas__mode__pb2.DESCRIPTOR,])




_USDRVSENSORREQUEST = _descriptor.Descriptor(
  name='UsDrvSensorRequest',
  full_name='pb.us_drv.us_drv_sensor_request.UsDrvSensorRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='type', full_name='pb.us_drv.us_drv_sensor_request.UsDrvSensorRequest.type', index=0,
      number=2270, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='state', full_name='pb.us_drv.us_drv_sensor_request.UsDrvSensorRequest.state', index=1,
      number=2607, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='measMode', full_name='pb.us_drv.us_drv_sensor_request.UsDrvSensorRequest.measMode', index=2,
      number=919, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
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
  serialized_start=180,
  serialized_end=408,
)


_USDRVSENSORREQUEST_ARRAY_PORT = _descriptor.Descriptor(
  name='UsDrvSensorRequest_array_port',
  full_name='pb.us_drv.us_drv_sensor_request.UsDrvSensorRequest_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_drv.us_drv_sensor_request.UsDrvSensorRequest_array_port.data', index=0,
      number=3035, type=11, cpp_type=10, label=3,
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
  serialized_start=410,
  serialized_end=509,
)

_USDRVSENSORREQUEST.fields_by_name['type'].enum_type = us__drv_dot_us__drv__request__type__pb2._USDRVREQUESTTYPE
_USDRVSENSORREQUEST.fields_by_name['state'].enum_type = us__drv_dot_us__drv__request__state__pb2._USDRVREQUESTSTATE
_USDRVSENSORREQUEST.fields_by_name['measMode'].enum_type = us__drv_dot_us__drv__request__meas__mode__pb2._USDRVREQUESTMEASMODE
_USDRVSENSORREQUEST_ARRAY_PORT.fields_by_name['data'].message_type = _USDRVSENSORREQUEST
DESCRIPTOR.message_types_by_name['UsDrvSensorRequest'] = _USDRVSENSORREQUEST
DESCRIPTOR.message_types_by_name['UsDrvSensorRequest_array_port'] = _USDRVSENSORREQUEST_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsDrvSensorRequest = _reflection.GeneratedProtocolMessageType('UsDrvSensorRequest', (_message.Message,), {
  'DESCRIPTOR' : _USDRVSENSORREQUEST,
  '__module__' : 'us_drv.us_drv_sensor_request_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_sensor_request.UsDrvSensorRequest)
  })
_sym_db.RegisterMessage(UsDrvSensorRequest)

UsDrvSensorRequest_array_port = _reflection.GeneratedProtocolMessageType('UsDrvSensorRequest_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USDRVSENSORREQUEST_ARRAY_PORT,
  '__module__' : 'us_drv.us_drv_sensor_request_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_sensor_request.UsDrvSensorRequest_array_port)
  })
_sym_db.RegisterMessage(UsDrvSensorRequest_array_port)


# @@protoc_insertion_point(module_scope)