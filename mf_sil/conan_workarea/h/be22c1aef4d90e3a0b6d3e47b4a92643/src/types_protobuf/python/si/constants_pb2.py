# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/constants.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/constants.proto',
  package='pb.si.constants',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x12si/constants.proto\x12\x0fpb.si.constants\"5\n\tConstants\x12(\n\x1fMAX_NUM_OF_DIAGNOSTIC_EVENT_IDS\x18\xa2\x04 \x01(\r\"A\n\x14\x43onstants_array_port\x12)\n\x04\x64\x61ta\x18\x99\x0c \x03(\x0b\x32\x1a.pb.si.constants.Constants'
)




_CONSTANTS = _descriptor.Descriptor(
  name='Constants',
  full_name='pb.si.constants.Constants',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='MAX_NUM_OF_DIAGNOSTIC_EVENT_IDS', full_name='pb.si.constants.Constants.MAX_NUM_OF_DIAGNOSTIC_EVENT_IDS', index=0,
      number=546, type=13, cpp_type=3, label=1,
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
  serialized_start=39,
  serialized_end=92,
)


_CONSTANTS_ARRAY_PORT = _descriptor.Descriptor(
  name='Constants_array_port',
  full_name='pb.si.constants.Constants_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.constants.Constants_array_port.data', index=0,
      number=1561, type=11, cpp_type=10, label=3,
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
  serialized_start=94,
  serialized_end=159,
)

_CONSTANTS_ARRAY_PORT.fields_by_name['data'].message_type = _CONSTANTS
DESCRIPTOR.message_types_by_name['Constants'] = _CONSTANTS
DESCRIPTOR.message_types_by_name['Constants_array_port'] = _CONSTANTS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Constants = _reflection.GeneratedProtocolMessageType('Constants', (_message.Message,), {
  'DESCRIPTOR' : _CONSTANTS,
  '__module__' : 'si.constants_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.constants.Constants)
  })
_sym_db.RegisterMessage(Constants)

Constants_array_port = _reflection.GeneratedProtocolMessageType('Constants_array_port', (_message.Message,), {
  'DESCRIPTOR' : _CONSTANTS_ARRAY_PORT,
  '__module__' : 'si.constants_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.constants.Constants_array_port)
  })
_sym_db.RegisterMessage(Constants_array_port)


# @@protoc_insertion_point(module_scope)