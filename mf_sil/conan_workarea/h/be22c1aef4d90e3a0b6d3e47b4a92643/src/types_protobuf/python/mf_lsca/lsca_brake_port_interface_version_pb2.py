# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_lsca/lsca_brake_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_lsca/lsca_brake_port_interface_version.proto',
  package='pb.mf_lsca.lsca_brake_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n/mf_lsca/lsca_brake_port_interface_version.proto\x12,pb.mf_lsca.lsca_brake_port_interface_version\"@\n\x1eLscaBrakePort_InterfaceVersion\x12\x1e\n\x15LscaBrakePort_VERSION\x18\x87\x07 \x01(\r\"\x88\x01\n)LscaBrakePort_InterfaceVersion_array_port\x12[\n\x04\x64\x61ta\x18\x8e\x03 \x03(\x0b\x32L.pb.mf_lsca.lsca_brake_port_interface_version.LscaBrakePort_InterfaceVersion'
)




_LSCABRAKEPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='LscaBrakePort_InterfaceVersion',
  full_name='pb.mf_lsca.lsca_brake_port_interface_version.LscaBrakePort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='LscaBrakePort_VERSION', full_name='pb.mf_lsca.lsca_brake_port_interface_version.LscaBrakePort_InterfaceVersion.LscaBrakePort_VERSION', index=0,
      number=903, type=13, cpp_type=3, label=1,
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
  serialized_end=161,
)


_LSCABRAKEPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='LscaBrakePort_InterfaceVersion_array_port',
  full_name='pb.mf_lsca.lsca_brake_port_interface_version.LscaBrakePort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_lsca.lsca_brake_port_interface_version.LscaBrakePort_InterfaceVersion_array_port.data', index=0,
      number=398, type=11, cpp_type=10, label=3,
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
  serialized_start=164,
  serialized_end=300,
)

_LSCABRAKEPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _LSCABRAKEPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['LscaBrakePort_InterfaceVersion'] = _LSCABRAKEPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['LscaBrakePort_InterfaceVersion_array_port'] = _LSCABRAKEPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LscaBrakePort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('LscaBrakePort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _LSCABRAKEPORT_INTERFACEVERSION,
  '__module__' : 'mf_lsca.lsca_brake_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_brake_port_interface_version.LscaBrakePort_InterfaceVersion)
  })
_sym_db.RegisterMessage(LscaBrakePort_InterfaceVersion)

LscaBrakePort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('LscaBrakePort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _LSCABRAKEPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'mf_lsca.lsca_brake_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_brake_port_interface_version.LscaBrakePort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(LscaBrakePort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
