# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_em/us_env_model_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_em/us_env_model_port_interface_version.proto',
  package='pb.us_em.us_env_model_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n/us_em/us_env_model_port_interface_version.proto\x12,pb.us_em.us_env_model_port_interface_version\"B\n\x1fUsEnvModelPort_InterfaceVersion\x12\x1f\n\x16UsEnvModelPort_VERSION\x18\xbc\x19 \x01(\r\"\x89\x01\n*UsEnvModelPort_InterfaceVersion_array_port\x12[\n\x04\x64\x61ta\x18\x1c \x03(\x0b\x32M.pb.us_em.us_env_model_port_interface_version.UsEnvModelPort_InterfaceVersion'
)




_USENVMODELPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='UsEnvModelPort_InterfaceVersion',
  full_name='pb.us_em.us_env_model_port_interface_version.UsEnvModelPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='UsEnvModelPort_VERSION', full_name='pb.us_em.us_env_model_port_interface_version.UsEnvModelPort_InterfaceVersion.UsEnvModelPort_VERSION', index=0,
      number=3260, type=13, cpp_type=3, label=1,
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
  serialized_start=97,
  serialized_end=163,
)


_USENVMODELPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='UsEnvModelPort_InterfaceVersion_array_port',
  full_name='pb.us_em.us_env_model_port_interface_version.UsEnvModelPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_em.us_env_model_port_interface_version.UsEnvModelPort_InterfaceVersion_array_port.data', index=0,
      number=28, type=11, cpp_type=10, label=3,
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
  serialized_start=166,
  serialized_end=303,
)

_USENVMODELPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _USENVMODELPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['UsEnvModelPort_InterfaceVersion'] = _USENVMODELPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['UsEnvModelPort_InterfaceVersion_array_port'] = _USENVMODELPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsEnvModelPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('UsEnvModelPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _USENVMODELPORT_INTERFACEVERSION,
  '__module__' : 'us_em.us_env_model_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_em.us_env_model_port_interface_version.UsEnvModelPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(UsEnvModelPort_InterfaceVersion)

UsEnvModelPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('UsEnvModelPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USENVMODELPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'us_em.us_env_model_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_em.us_env_model_port_interface_version.UsEnvModelPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(UsEnvModelPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)