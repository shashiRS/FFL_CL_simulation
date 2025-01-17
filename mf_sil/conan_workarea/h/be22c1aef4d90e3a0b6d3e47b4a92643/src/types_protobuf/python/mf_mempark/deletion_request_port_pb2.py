# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/deletion_request_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/deletion_request_port.proto',
  package='pb.mf_mempark.deletion_request_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n&mf_mempark/deletion_request_port.proto\x12#pb.mf_mempark.deletion_request_port\x1a\x17\x65\x63o/signal_header.proto\"x\n\x13\x44\x65letionRequestPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x0e\n\x05mapID\x18\xd8\r \x01(\r\"i\n\x1e\x44\x65letionRequestPort_array_port\x12G\n\x04\x64\x61ta\x18\xd6\x0f \x03(\x0b\x32\x38.pb.mf_mempark.deletion_request_port.DeletionRequestPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,])




_DELETIONREQUESTPORT = _descriptor.Descriptor(
  name='DeletionRequestPort',
  full_name='pb.mf_mempark.deletion_request_port.DeletionRequestPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.mf_mempark.deletion_request_port.DeletionRequestPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.mf_mempark.deletion_request_port.DeletionRequestPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='mapID', full_name='pb.mf_mempark.deletion_request_port.DeletionRequestPort.mapID', index=2,
      number=1752, type=13, cpp_type=3, label=1,
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
  serialized_start=104,
  serialized_end=224,
)


_DELETIONREQUESTPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='DeletionRequestPort_array_port',
  full_name='pb.mf_mempark.deletion_request_port.DeletionRequestPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_mempark.deletion_request_port.DeletionRequestPort_array_port.data', index=0,
      number=2006, type=11, cpp_type=10, label=3,
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
  serialized_start=226,
  serialized_end=331,
)

_DELETIONREQUESTPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_DELETIONREQUESTPORT_ARRAY_PORT.fields_by_name['data'].message_type = _DELETIONREQUESTPORT
DESCRIPTOR.message_types_by_name['DeletionRequestPort'] = _DELETIONREQUESTPORT
DESCRIPTOR.message_types_by_name['DeletionRequestPort_array_port'] = _DELETIONREQUESTPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

DeletionRequestPort = _reflection.GeneratedProtocolMessageType('DeletionRequestPort', (_message.Message,), {
  'DESCRIPTOR' : _DELETIONREQUESTPORT,
  '__module__' : 'mf_mempark.deletion_request_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.deletion_request_port.DeletionRequestPort)
  })
_sym_db.RegisterMessage(DeletionRequestPort)

DeletionRequestPort_array_port = _reflection.GeneratedProtocolMessageType('DeletionRequestPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _DELETIONREQUESTPORT_ARRAY_PORT,
  '__module__' : 'mf_mempark.deletion_request_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.deletion_request_port.DeletionRequestPort_array_port)
  })
_sym_db.RegisterMessage(DeletionRequestPort_array_port)


# @@protoc_insertion_point(module_scope)
