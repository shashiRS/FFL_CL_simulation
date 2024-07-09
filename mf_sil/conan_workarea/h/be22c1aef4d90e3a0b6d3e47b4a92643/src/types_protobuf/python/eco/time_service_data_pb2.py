# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: eco/time_service_data.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='eco/time_service_data.proto',
  package='pb.eco.time_service_data',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1b\x65\x63o/time_service_data.proto\x12\x18pb.eco.time_service_data\x1a\x17\x65\x63o/signal_header.proto\"z\n\x0fTimeServiceData\x12\x36\n\tsigHeader\x18\xaa\x0b \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x13\n\ncycleStart\x18\x8b\x1e \x01(\x04\x12\x1a\n\x11\x65stimatedCycleEnd\x18\xf4\x03 \x01(\x04\"V\n\x1aTimeServiceData_array_port\x12\x38\n\x04\x64\x61ta\x18\xd9\x1a \x03(\x0b\x32).pb.eco.time_service_data.TimeServiceData'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,])




_TIMESERVICEDATA = _descriptor.Descriptor(
  name='TimeServiceData',
  full_name='pb.eco.time_service_data.TimeServiceData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='sigHeader', full_name='pb.eco.time_service_data.TimeServiceData.sigHeader', index=0,
      number=1450, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='cycleStart', full_name='pb.eco.time_service_data.TimeServiceData.cycleStart', index=1,
      number=3851, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='estimatedCycleEnd', full_name='pb.eco.time_service_data.TimeServiceData.estimatedCycleEnd', index=2,
      number=500, type=4, cpp_type=4, label=1,
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
  serialized_start=82,
  serialized_end=204,
)


_TIMESERVICEDATA_ARRAY_PORT = _descriptor.Descriptor(
  name='TimeServiceData_array_port',
  full_name='pb.eco.time_service_data.TimeServiceData_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.eco.time_service_data.TimeServiceData_array_port.data', index=0,
      number=3417, type=11, cpp_type=10, label=3,
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
  serialized_start=206,
  serialized_end=292,
)

_TIMESERVICEDATA.fields_by_name['sigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_TIMESERVICEDATA_ARRAY_PORT.fields_by_name['data'].message_type = _TIMESERVICEDATA
DESCRIPTOR.message_types_by_name['TimeServiceData'] = _TIMESERVICEDATA
DESCRIPTOR.message_types_by_name['TimeServiceData_array_port'] = _TIMESERVICEDATA_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TimeServiceData = _reflection.GeneratedProtocolMessageType('TimeServiceData', (_message.Message,), {
  'DESCRIPTOR' : _TIMESERVICEDATA,
  '__module__' : 'eco.time_service_data_pb2'
  # @@protoc_insertion_point(class_scope:pb.eco.time_service_data.TimeServiceData)
  })
_sym_db.RegisterMessage(TimeServiceData)

TimeServiceData_array_port = _reflection.GeneratedProtocolMessageType('TimeServiceData_array_port', (_message.Message,), {
  'DESCRIPTOR' : _TIMESERVICEDATA_ARRAY_PORT,
  '__module__' : 'eco.time_service_data_pb2'
  # @@protoc_insertion_point(class_scope:pb.eco.time_service_data.TimeServiceData_array_port)
  })
_sym_db.RegisterMessage(TimeServiceData_array_port)


# @@protoc_insertion_point(module_scope)
