# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_tp/fc_trjpla_params_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_tp/fc_trjpla_params_interface_version.proto',
  package='pb.ap_tp.fc_trjpla_params_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n.ap_tp/fc_trjpla_params_interface_version.proto\x12+pb.ap_tp.fc_trjpla_params_interface_version\"F\n!FC_TRJPLA_Params_InterfaceVersion\x12!\n\x18\x46\x43_TRJPLA_Params_VERSION\x18\xd4\x07 \x01(\r\"\x8d\x01\n,FC_TRJPLA_Params_InterfaceVersion_array_port\x12]\n\x04\x64\x61ta\x18\x8f\x04 \x03(\x0b\x32N.pb.ap_tp.fc_trjpla_params_interface_version.FC_TRJPLA_Params_InterfaceVersion'
)




_FC_TRJPLA_PARAMS_INTERFACEVERSION = _descriptor.Descriptor(
  name='FC_TRJPLA_Params_InterfaceVersion',
  full_name='pb.ap_tp.fc_trjpla_params_interface_version.FC_TRJPLA_Params_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='FC_TRJPLA_Params_VERSION', full_name='pb.ap_tp.fc_trjpla_params_interface_version.FC_TRJPLA_Params_InterfaceVersion.FC_TRJPLA_Params_VERSION', index=0,
      number=980, type=13, cpp_type=3, label=1,
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
  serialized_start=95,
  serialized_end=165,
)


_FC_TRJPLA_PARAMS_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='FC_TRJPLA_Params_InterfaceVersion_array_port',
  full_name='pb.ap_tp.fc_trjpla_params_interface_version.FC_TRJPLA_Params_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_tp.fc_trjpla_params_interface_version.FC_TRJPLA_Params_InterfaceVersion_array_port.data', index=0,
      number=527, type=11, cpp_type=10, label=3,
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
  serialized_end=309,
)

_FC_TRJPLA_PARAMS_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _FC_TRJPLA_PARAMS_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['FC_TRJPLA_Params_InterfaceVersion'] = _FC_TRJPLA_PARAMS_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['FC_TRJPLA_Params_InterfaceVersion_array_port'] = _FC_TRJPLA_PARAMS_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FC_TRJPLA_Params_InterfaceVersion = _reflection.GeneratedProtocolMessageType('FC_TRJPLA_Params_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _FC_TRJPLA_PARAMS_INTERFACEVERSION,
  '__module__' : 'ap_tp.fc_trjpla_params_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.fc_trjpla_params_interface_version.FC_TRJPLA_Params_InterfaceVersion)
  })
_sym_db.RegisterMessage(FC_TRJPLA_Params_InterfaceVersion)

FC_TRJPLA_Params_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('FC_TRJPLA_Params_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _FC_TRJPLA_PARAMS_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'ap_tp.fc_trjpla_params_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.fc_trjpla_params_interface_version.FC_TRJPLA_Params_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(FC_TRJPLA_Params_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
