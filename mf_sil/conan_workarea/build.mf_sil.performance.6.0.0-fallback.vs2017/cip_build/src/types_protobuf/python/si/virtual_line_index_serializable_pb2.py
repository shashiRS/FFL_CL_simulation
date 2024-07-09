# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/virtual_line_index_serializable.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/virtual_line_index_serializable.proto',
  package='pb.si.virtual_line_index_serializable',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n(si/virtual_line_index_serializable.proto\x12%pb.si.virtual_line_index_serializable\"C\n\x1cVirtualLineIndexSerializable\x12\x13\n\nactualSize\x18\xc1\x17 \x01(\r\x12\x0e\n\x05\x61rray\x18\xb7\x1c \x03(\r\"}\n\'VirtualLineIndexSerializable_array_port\x12R\n\x04\x64\x61ta\x18\xc2\x0e \x03(\x0b\x32\x43.pb.si.virtual_line_index_serializable.VirtualLineIndexSerializable'
)




_VIRTUALLINEINDEXSERIALIZABLE = _descriptor.Descriptor(
  name='VirtualLineIndexSerializable',
  full_name='pb.si.virtual_line_index_serializable.VirtualLineIndexSerializable',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='actualSize', full_name='pb.si.virtual_line_index_serializable.VirtualLineIndexSerializable.actualSize', index=0,
      number=3009, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='array', full_name='pb.si.virtual_line_index_serializable.VirtualLineIndexSerializable.array', index=1,
      number=3639, type=13, cpp_type=3, label=3,
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
  serialized_start=83,
  serialized_end=150,
)


_VIRTUALLINEINDEXSERIALIZABLE_ARRAY_PORT = _descriptor.Descriptor(
  name='VirtualLineIndexSerializable_array_port',
  full_name='pb.si.virtual_line_index_serializable.VirtualLineIndexSerializable_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.virtual_line_index_serializable.VirtualLineIndexSerializable_array_port.data', index=0,
      number=1858, type=11, cpp_type=10, label=3,
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
  serialized_start=152,
  serialized_end=277,
)

_VIRTUALLINEINDEXSERIALIZABLE_ARRAY_PORT.fields_by_name['data'].message_type = _VIRTUALLINEINDEXSERIALIZABLE
DESCRIPTOR.message_types_by_name['VirtualLineIndexSerializable'] = _VIRTUALLINEINDEXSERIALIZABLE
DESCRIPTOR.message_types_by_name['VirtualLineIndexSerializable_array_port'] = _VIRTUALLINEINDEXSERIALIZABLE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

VirtualLineIndexSerializable = _reflection.GeneratedProtocolMessageType('VirtualLineIndexSerializable', (_message.Message,), {
  'DESCRIPTOR' : _VIRTUALLINEINDEXSERIALIZABLE,
  '__module__' : 'si.virtual_line_index_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.virtual_line_index_serializable.VirtualLineIndexSerializable)
  })
_sym_db.RegisterMessage(VirtualLineIndexSerializable)

VirtualLineIndexSerializable_array_port = _reflection.GeneratedProtocolMessageType('VirtualLineIndexSerializable_array_port', (_message.Message,), {
  'DESCRIPTOR' : _VIRTUALLINEINDEXSERIALIZABLE_ARRAY_PORT,
  '__module__' : 'si.virtual_line_index_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.virtual_line_index_serializable.VirtualLineIndexSerializable_array_port)
  })
_sym_db.RegisterMessage(VirtualLineIndexSerializable_array_port)


# @@protoc_insertion_point(module_scope)