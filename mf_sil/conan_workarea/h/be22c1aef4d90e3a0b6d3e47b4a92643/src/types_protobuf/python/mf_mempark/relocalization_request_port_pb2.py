# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/relocalization_request_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from mf_mempark import redetection_request_pb2 as mf__mempark_dot_redetection__request__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/relocalization_request_port.proto',
  package='pb.mf_mempark.relocalization_request_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n,mf_mempark/relocalization_request_port.proto\x12)pb.mf_mempark.relocalization_request_port\x1a\x17\x65\x63o/signal_header.proto\x1a$mf_mempark/redetection_request.proto\"\xea\x01\n\x19RelocalizationRequestPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x1c\n\x13numValidRequests_nu\x18\xdc\x08 \x01(\r\x12\\\n\x1crequestedRelocalizationSlots\x18\xca\n \x03(\x0b\x32\x35.pb.mf_mempark.redetection_request.RedetectionRequest\"{\n$RelocalizationRequestPort_array_port\x12S\n\x04\x64\x61ta\x18\xda\x15 \x03(\x0b\x32\x44.pb.mf_mempark.relocalization_request_port.RelocalizationRequestPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,mf__mempark_dot_redetection__request__pb2.DESCRIPTOR,])




_RELOCALIZATIONREQUESTPORT = _descriptor.Descriptor(
  name='RelocalizationRequestPort',
  full_name='pb.mf_mempark.relocalization_request_port.RelocalizationRequestPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.mf_mempark.relocalization_request_port.RelocalizationRequestPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.mf_mempark.relocalization_request_port.RelocalizationRequestPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numValidRequests_nu', full_name='pb.mf_mempark.relocalization_request_port.RelocalizationRequestPort.numValidRequests_nu', index=2,
      number=1116, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='requestedRelocalizationSlots', full_name='pb.mf_mempark.relocalization_request_port.RelocalizationRequestPort.requestedRelocalizationSlots', index=3,
      number=1354, type=11, cpp_type=10, label=3,
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
  serialized_start=155,
  serialized_end=389,
)


_RELOCALIZATIONREQUESTPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='RelocalizationRequestPort_array_port',
  full_name='pb.mf_mempark.relocalization_request_port.RelocalizationRequestPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_mempark.relocalization_request_port.RelocalizationRequestPort_array_port.data', index=0,
      number=2778, type=11, cpp_type=10, label=3,
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
  serialized_start=391,
  serialized_end=514,
)

_RELOCALIZATIONREQUESTPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_RELOCALIZATIONREQUESTPORT.fields_by_name['requestedRelocalizationSlots'].message_type = mf__mempark_dot_redetection__request__pb2._REDETECTIONREQUEST
_RELOCALIZATIONREQUESTPORT_ARRAY_PORT.fields_by_name['data'].message_type = _RELOCALIZATIONREQUESTPORT
DESCRIPTOR.message_types_by_name['RelocalizationRequestPort'] = _RELOCALIZATIONREQUESTPORT
DESCRIPTOR.message_types_by_name['RelocalizationRequestPort_array_port'] = _RELOCALIZATIONREQUESTPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

RelocalizationRequestPort = _reflection.GeneratedProtocolMessageType('RelocalizationRequestPort', (_message.Message,), {
  'DESCRIPTOR' : _RELOCALIZATIONREQUESTPORT,
  '__module__' : 'mf_mempark.relocalization_request_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.relocalization_request_port.RelocalizationRequestPort)
  })
_sym_db.RegisterMessage(RelocalizationRequestPort)

RelocalizationRequestPort_array_port = _reflection.GeneratedProtocolMessageType('RelocalizationRequestPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _RELOCALIZATIONREQUESTPORT_ARRAY_PORT,
  '__module__' : 'mf_mempark.relocalization_request_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.relocalization_request_port.RelocalizationRequestPort_array_port)
  })
_sym_db.RegisterMessage(RelocalizationRequestPort_array_port)


# @@protoc_insertion_point(module_scope)
