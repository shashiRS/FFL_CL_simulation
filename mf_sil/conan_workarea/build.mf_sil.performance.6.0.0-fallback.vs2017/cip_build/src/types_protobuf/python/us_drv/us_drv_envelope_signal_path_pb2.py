# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_drv/us_drv_envelope_signal_path.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_drv/us_drv_envelope_signal_path.proto',
  package='pb.us_drv.us_drv_envelope_signal_path',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n(us_drv/us_drv_envelope_signal_path.proto\x12%pb.us_drv.us_drv_envelope_signal_path\"\x9c\x01\n\x17UsDrvEnvelopeSignalPath\x12\x13\n\nrxSensorId\x18\xd8\r \x01(\r\x12\x13\n\ntxSensorId\x18\xe7\n \x01(\r\x12\x15\n\x0csignalPathId\x18\xa9\x0c \x01(\r\x12\x14\n\x0bstartSample\x18\xe5\x1e \x01(\r\x12\x13\n\nnumSamples\x18\xe1\x07 \x01(\r\x12\x15\n\x0ctimestamp_us\x18\x98\x1e \x01(\x04\"s\n\"UsDrvEnvelopeSignalPath_array_port\x12M\n\x04\x64\x61ta\x18\x87\x01 \x03(\x0b\x32>.pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath'
)




_USDRVENVELOPESIGNALPATH = _descriptor.Descriptor(
  name='UsDrvEnvelopeSignalPath',
  full_name='pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='rxSensorId', full_name='pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath.rxSensorId', index=0,
      number=1752, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='txSensorId', full_name='pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath.txSensorId', index=1,
      number=1383, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='signalPathId', full_name='pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath.signalPathId', index=2,
      number=1577, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='startSample', full_name='pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath.startSample', index=3,
      number=3941, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numSamples', full_name='pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath.numSamples', index=4,
      number=993, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='timestamp_us', full_name='pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath.timestamp_us', index=5,
      number=3864, type=4, cpp_type=4, label=1,
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
  serialized_start=84,
  serialized_end=240,
)


_USDRVENVELOPESIGNALPATH_ARRAY_PORT = _descriptor.Descriptor(
  name='UsDrvEnvelopeSignalPath_array_port',
  full_name='pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath_array_port.data', index=0,
      number=135, type=11, cpp_type=10, label=3,
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
  serialized_start=242,
  serialized_end=357,
)

_USDRVENVELOPESIGNALPATH_ARRAY_PORT.fields_by_name['data'].message_type = _USDRVENVELOPESIGNALPATH
DESCRIPTOR.message_types_by_name['UsDrvEnvelopeSignalPath'] = _USDRVENVELOPESIGNALPATH
DESCRIPTOR.message_types_by_name['UsDrvEnvelopeSignalPath_array_port'] = _USDRVENVELOPESIGNALPATH_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsDrvEnvelopeSignalPath = _reflection.GeneratedProtocolMessageType('UsDrvEnvelopeSignalPath', (_message.Message,), {
  'DESCRIPTOR' : _USDRVENVELOPESIGNALPATH,
  '__module__' : 'us_drv.us_drv_envelope_signal_path_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath)
  })
_sym_db.RegisterMessage(UsDrvEnvelopeSignalPath)

UsDrvEnvelopeSignalPath_array_port = _reflection.GeneratedProtocolMessageType('UsDrvEnvelopeSignalPath_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USDRVENVELOPESIGNALPATH_ARRAY_PORT,
  '__module__' : 'us_drv.us_drv_envelope_signal_path_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath_array_port)
  })
_sym_db.RegisterMessage(UsDrvEnvelopeSignalPath_array_port)


# @@protoc_insertion_point(module_scope)