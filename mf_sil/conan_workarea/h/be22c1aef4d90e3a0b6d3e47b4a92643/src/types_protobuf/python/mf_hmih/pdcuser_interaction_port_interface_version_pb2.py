# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/pdcuser_interaction_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/pdcuser_interaction_port_interface_version.proto',
  package='pb.mf_hmih.pdcuser_interaction_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n8mf_hmih/pdcuser_interaction_port_interface_version.proto\x12\x35pb.mf_hmih.pdcuser_interaction_port_interface_version\"R\n\'PDCUserInteractionPort_InterfaceVersion\x12\'\n\x1ePDCUserInteractionPort_VERSION\x18\xb1\x15 \x01(\r\"\xa2\x01\n2PDCUserInteractionPort_InterfaceVersion_array_port\x12l\n\x04\x64\x61ta\x18w \x03(\x0b\x32^.pb.mf_hmih.pdcuser_interaction_port_interface_version.PDCUserInteractionPort_InterfaceVersion'
)




_PDCUSERINTERACTIONPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='PDCUserInteractionPort_InterfaceVersion',
  full_name='pb.mf_hmih.pdcuser_interaction_port_interface_version.PDCUserInteractionPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='PDCUserInteractionPort_VERSION', full_name='pb.mf_hmih.pdcuser_interaction_port_interface_version.PDCUserInteractionPort_InterfaceVersion.PDCUserInteractionPort_VERSION', index=0,
      number=2737, type=13, cpp_type=3, label=1,
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
  serialized_start=115,
  serialized_end=197,
)


_PDCUSERINTERACTIONPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='PDCUserInteractionPort_InterfaceVersion_array_port',
  full_name='pb.mf_hmih.pdcuser_interaction_port_interface_version.PDCUserInteractionPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_hmih.pdcuser_interaction_port_interface_version.PDCUserInteractionPort_InterfaceVersion_array_port.data', index=0,
      number=119, type=11, cpp_type=10, label=3,
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
  serialized_start=200,
  serialized_end=362,
)

_PDCUSERINTERACTIONPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _PDCUSERINTERACTIONPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['PDCUserInteractionPort_InterfaceVersion'] = _PDCUSERINTERACTIONPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['PDCUserInteractionPort_InterfaceVersion_array_port'] = _PDCUSERINTERACTIONPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PDCUserInteractionPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('PDCUserInteractionPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _PDCUSERINTERACTIONPORT_INTERFACEVERSION,
  '__module__' : 'mf_hmih.pdcuser_interaction_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.pdcuser_interaction_port_interface_version.PDCUserInteractionPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(PDCUserInteractionPort_InterfaceVersion)

PDCUserInteractionPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('PDCUserInteractionPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _PDCUSERINTERACTIONPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'mf_hmih.pdcuser_interaction_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.pdcuser_interaction_port_interface_version.PDCUserInteractionPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(PDCUserInteractionPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
