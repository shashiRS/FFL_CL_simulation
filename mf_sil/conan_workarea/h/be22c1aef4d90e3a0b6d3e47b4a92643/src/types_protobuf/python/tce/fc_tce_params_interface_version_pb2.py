# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: tce/fc_tce_params_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='tce/fc_tce_params_interface_version.proto',
  package='pb.tce.fc_tce_params_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n)tce/fc_tce_params_interface_version.proto\x12&pb.tce.fc_tce_params_interface_version\"@\n\x1e\x46\x43_TCE_Params_InterfaceVersion\x12\x1e\n\x15\x46\x43_TCE_Params_VERSION\x18\x8b\x15 \x01(\r\"\x82\x01\n)FC_TCE_Params_InterfaceVersion_array_port\x12U\n\x04\x64\x61ta\x18\xeb\x16 \x03(\x0b\x32\x46.pb.tce.fc_tce_params_interface_version.FC_TCE_Params_InterfaceVersion'
)




_FC_TCE_PARAMS_INTERFACEVERSION = _descriptor.Descriptor(
  name='FC_TCE_Params_InterfaceVersion',
  full_name='pb.tce.fc_tce_params_interface_version.FC_TCE_Params_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='FC_TCE_Params_VERSION', full_name='pb.tce.fc_tce_params_interface_version.FC_TCE_Params_InterfaceVersion.FC_TCE_Params_VERSION', index=0,
      number=2699, type=13, cpp_type=3, label=1,
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
  serialized_start=85,
  serialized_end=149,
)


_FC_TCE_PARAMS_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='FC_TCE_Params_InterfaceVersion_array_port',
  full_name='pb.tce.fc_tce_params_interface_version.FC_TCE_Params_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.tce.fc_tce_params_interface_version.FC_TCE_Params_InterfaceVersion_array_port.data', index=0,
      number=2923, type=11, cpp_type=10, label=3,
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
  serialized_end=282,
)

_FC_TCE_PARAMS_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _FC_TCE_PARAMS_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['FC_TCE_Params_InterfaceVersion'] = _FC_TCE_PARAMS_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['FC_TCE_Params_InterfaceVersion_array_port'] = _FC_TCE_PARAMS_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FC_TCE_Params_InterfaceVersion = _reflection.GeneratedProtocolMessageType('FC_TCE_Params_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _FC_TCE_PARAMS_INTERFACEVERSION,
  '__module__' : 'tce.fc_tce_params_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.tce.fc_tce_params_interface_version.FC_TCE_Params_InterfaceVersion)
  })
_sym_db.RegisterMessage(FC_TCE_Params_InterfaceVersion)

FC_TCE_Params_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('FC_TCE_Params_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _FC_TCE_PARAMS_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'tce.fc_tce_params_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.tce.fc_tce_params_interface_version.FC_TCE_Params_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(FC_TCE_Params_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
