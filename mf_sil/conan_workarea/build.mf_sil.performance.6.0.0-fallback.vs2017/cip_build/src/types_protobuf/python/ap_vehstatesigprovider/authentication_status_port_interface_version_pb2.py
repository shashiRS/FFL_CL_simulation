# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_vehstatesigprovider/authentication_status_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_vehstatesigprovider/authentication_status_port_interface_version.proto',
  package='pb.ap_vehstatesigprovider.authentication_status_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\nIap_vehstatesigprovider/authentication_status_port_interface_version.proto\x12\x46pb.ap_vehstatesigprovider.authentication_status_port_interface_version\"V\n)AuthenticationStatusPort_InterfaceVersion\x12)\n AuthenticationStatusPort_VERSION\x18\xef\x03 \x01(\r\"\xb9\x01\n4AuthenticationStatusPort_InterfaceVersion_array_port\x12\x80\x01\n\x04\x64\x61ta\x18\xe2\n \x03(\x0b\x32q.pb.ap_vehstatesigprovider.authentication_status_port_interface_version.AuthenticationStatusPort_InterfaceVersion'
)




_AUTHENTICATIONSTATUSPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='AuthenticationStatusPort_InterfaceVersion',
  full_name='pb.ap_vehstatesigprovider.authentication_status_port_interface_version.AuthenticationStatusPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='AuthenticationStatusPort_VERSION', full_name='pb.ap_vehstatesigprovider.authentication_status_port_interface_version.AuthenticationStatusPort_InterfaceVersion.AuthenticationStatusPort_VERSION', index=0,
      number=495, type=13, cpp_type=3, label=1,
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
  serialized_start=149,
  serialized_end=235,
)


_AUTHENTICATIONSTATUSPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='AuthenticationStatusPort_InterfaceVersion_array_port',
  full_name='pb.ap_vehstatesigprovider.authentication_status_port_interface_version.AuthenticationStatusPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_vehstatesigprovider.authentication_status_port_interface_version.AuthenticationStatusPort_InterfaceVersion_array_port.data', index=0,
      number=1378, type=11, cpp_type=10, label=3,
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
  serialized_start=238,
  serialized_end=423,
)

_AUTHENTICATIONSTATUSPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _AUTHENTICATIONSTATUSPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['AuthenticationStatusPort_InterfaceVersion'] = _AUTHENTICATIONSTATUSPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['AuthenticationStatusPort_InterfaceVersion_array_port'] = _AUTHENTICATIONSTATUSPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

AuthenticationStatusPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('AuthenticationStatusPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _AUTHENTICATIONSTATUSPORT_INTERFACEVERSION,
  '__module__' : 'ap_vehstatesigprovider.authentication_status_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.authentication_status_port_interface_version.AuthenticationStatusPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(AuthenticationStatusPort_InterfaceVersion)

AuthenticationStatusPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('AuthenticationStatusPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _AUTHENTICATIONSTATUSPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'ap_vehstatesigprovider.authentication_status_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.authentication_status_port_interface_version.AuthenticationStatusPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(AuthenticationStatusPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)