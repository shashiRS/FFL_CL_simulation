# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/hmigeneral_input_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/hmigeneral_input_port_interface_version.proto',
  package='pb.mf_hmih.hmigeneral_input_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n5mf_hmih/hmigeneral_input_port_interface_version.proto\x12\x32pb.mf_hmih.hmigeneral_input_port_interface_version\"L\n$HMIGeneralInputPort_InterfaceVersion\x12$\n\x1bHMIGeneralInputPort_VERSION\x18\xa2\x0f \x01(\r\"\x9a\x01\n/HMIGeneralInputPort_InterfaceVersion_array_port\x12g\n\x04\x64\x61ta\x18\xb9\x15 \x03(\x0b\x32X.pb.mf_hmih.hmigeneral_input_port_interface_version.HMIGeneralInputPort_InterfaceVersion'
)




_HMIGENERALINPUTPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='HMIGeneralInputPort_InterfaceVersion',
  full_name='pb.mf_hmih.hmigeneral_input_port_interface_version.HMIGeneralInputPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='HMIGeneralInputPort_VERSION', full_name='pb.mf_hmih.hmigeneral_input_port_interface_version.HMIGeneralInputPort_InterfaceVersion.HMIGeneralInputPort_VERSION', index=0,
      number=1954, type=13, cpp_type=3, label=1,
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
  serialized_start=109,
  serialized_end=185,
)


_HMIGENERALINPUTPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='HMIGeneralInputPort_InterfaceVersion_array_port',
  full_name='pb.mf_hmih.hmigeneral_input_port_interface_version.HMIGeneralInputPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_hmih.hmigeneral_input_port_interface_version.HMIGeneralInputPort_InterfaceVersion_array_port.data', index=0,
      number=2745, type=11, cpp_type=10, label=3,
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
  serialized_start=188,
  serialized_end=342,
)

_HMIGENERALINPUTPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _HMIGENERALINPUTPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['HMIGeneralInputPort_InterfaceVersion'] = _HMIGENERALINPUTPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['HMIGeneralInputPort_InterfaceVersion_array_port'] = _HMIGENERALINPUTPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

HMIGeneralInputPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('HMIGeneralInputPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _HMIGENERALINPUTPORT_INTERFACEVERSION,
  '__module__' : 'mf_hmih.hmigeneral_input_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.hmigeneral_input_port_interface_version.HMIGeneralInputPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(HMIGeneralInputPort_InterfaceVersion)

HMIGeneralInputPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('HMIGeneralInputPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _HMIGENERALINPUTPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'mf_hmih.hmigeneral_input_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.hmigeneral_input_port_interface_version.HMIGeneralInputPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(HMIGeneralInputPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
