# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_processing/us_processing_diag_output.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from us_processing import us_processing_sensor_status_pb2 as us__processing_dot_us__processing__sensor__status__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_processing/us_processing_diag_output.proto',
  package='pb.us_processing.us_processing_diag_output',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n-us_processing/us_processing_diag_output.proto\x12*pb.us_processing.us_processing_diag_output\x1a\x17\x65\x63o/signal_header.proto\x1a/us_processing/us_processing_sensor_status.proto\"\xca\x01\n\x16UsProcessingDiagOutput\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12]\n\x0csensorStatus\x18\x93\x03 \x03(\x0e\x32\x46.pb.us_processing.us_processing_sensor_status.UsProcessingSensorStatus\"v\n!UsProcessingDiagOutput_array_port\x12Q\n\x04\x64\x61ta\x18\xa7\x14 \x03(\x0b\x32\x42.pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,us__processing_dot_us__processing__sensor__status__pb2.DESCRIPTOR,])




_USPROCESSINGDIAGOUTPUT = _descriptor.Descriptor(
  name='UsProcessingDiagOutput',
  full_name='pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sensorStatus', full_name='pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput.sensorStatus', index=2,
      number=403, type=14, cpp_type=8, label=3,
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
  serialized_start=168,
  serialized_end=370,
)


_USPROCESSINGDIAGOUTPUT_ARRAY_PORT = _descriptor.Descriptor(
  name='UsProcessingDiagOutput_array_port',
  full_name='pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput_array_port.data', index=0,
      number=2599, type=11, cpp_type=10, label=3,
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
  serialized_start=372,
  serialized_end=490,
)

_USPROCESSINGDIAGOUTPUT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_USPROCESSINGDIAGOUTPUT.fields_by_name['sensorStatus'].enum_type = us__processing_dot_us__processing__sensor__status__pb2._USPROCESSINGSENSORSTATUS
_USPROCESSINGDIAGOUTPUT_ARRAY_PORT.fields_by_name['data'].message_type = _USPROCESSINGDIAGOUTPUT
DESCRIPTOR.message_types_by_name['UsProcessingDiagOutput'] = _USPROCESSINGDIAGOUTPUT
DESCRIPTOR.message_types_by_name['UsProcessingDiagOutput_array_port'] = _USPROCESSINGDIAGOUTPUT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsProcessingDiagOutput = _reflection.GeneratedProtocolMessageType('UsProcessingDiagOutput', (_message.Message,), {
  'DESCRIPTOR' : _USPROCESSINGDIAGOUTPUT,
  '__module__' : 'us_processing.us_processing_diag_output_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput)
  })
_sym_db.RegisterMessage(UsProcessingDiagOutput)

UsProcessingDiagOutput_array_port = _reflection.GeneratedProtocolMessageType('UsProcessingDiagOutput_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USPROCESSINGDIAGOUTPUT_ARRAY_PORT,
  '__module__' : 'us_processing.us_processing_diag_output_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_diag_output.UsProcessingDiagOutput_array_port)
  })
_sym_db.RegisterMessage(UsProcessingDiagOutput_array_port)


# @@protoc_insertion_point(module_scope)