# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_lsca/lsca_general_debug_data_serializable.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_lsca/lsca_general_debug_data_serializable.proto',
  package='pb.mf_lsca.lsca_general_debug_data_serializable',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n2mf_lsca/lsca_general_debug_data_serializable.proto\x12/pb.mf_lsca.lsca_general_debug_data_serializable\"=\n LscaGeneralDebugDataSerializable\x12\x19\n\x10gasPedalOverride\x18\xd0\x13 \x01(\x08\"\x8f\x01\n+LscaGeneralDebugDataSerializable_array_port\x12`\n\x04\x64\x61ta\x18\x9f\x0c \x03(\x0b\x32Q.pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable'
)




_LSCAGENERALDEBUGDATASERIALIZABLE = _descriptor.Descriptor(
  name='LscaGeneralDebugDataSerializable',
  full_name='pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='gasPedalOverride', full_name='pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable.gasPedalOverride', index=0,
      number=2512, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
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
  serialized_start=103,
  serialized_end=164,
)


_LSCAGENERALDEBUGDATASERIALIZABLE_ARRAY_PORT = _descriptor.Descriptor(
  name='LscaGeneralDebugDataSerializable_array_port',
  full_name='pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable_array_port.data', index=0,
      number=1567, type=11, cpp_type=10, label=3,
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
  serialized_start=167,
  serialized_end=310,
)

_LSCAGENERALDEBUGDATASERIALIZABLE_ARRAY_PORT.fields_by_name['data'].message_type = _LSCAGENERALDEBUGDATASERIALIZABLE
DESCRIPTOR.message_types_by_name['LscaGeneralDebugDataSerializable'] = _LSCAGENERALDEBUGDATASERIALIZABLE
DESCRIPTOR.message_types_by_name['LscaGeneralDebugDataSerializable_array_port'] = _LSCAGENERALDEBUGDATASERIALIZABLE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LscaGeneralDebugDataSerializable = _reflection.GeneratedProtocolMessageType('LscaGeneralDebugDataSerializable', (_message.Message,), {
  'DESCRIPTOR' : _LSCAGENERALDEBUGDATASERIALIZABLE,
  '__module__' : 'mf_lsca.lsca_general_debug_data_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable)
  })
_sym_db.RegisterMessage(LscaGeneralDebugDataSerializable)

LscaGeneralDebugDataSerializable_array_port = _reflection.GeneratedProtocolMessageType('LscaGeneralDebugDataSerializable_array_port', (_message.Message,), {
  'DESCRIPTOR' : _LSCAGENERALDEBUGDATASERIALIZABLE_ARRAY_PORT,
  '__module__' : 'mf_lsca.lsca_general_debug_data_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_general_debug_data_serializable.LscaGeneralDebugDataSerializable_array_port)
  })
_sym_db.RegisterMessage(LscaGeneralDebugDataSerializable_array_port)


# @@protoc_insertion_point(module_scope)
