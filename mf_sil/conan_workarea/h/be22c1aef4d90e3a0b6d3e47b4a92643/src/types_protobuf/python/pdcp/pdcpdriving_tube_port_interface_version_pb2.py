# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: pdcp/pdcpdriving_tube_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='pdcp/pdcpdriving_tube_port_interface_version.proto',
  package='pb.pdcp.pdcpdriving_tube_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n2pdcp/pdcpdriving_tube_port_interface_version.proto\x12/pb.pdcp.pdcpdriving_tube_port_interface_version\"L\n$PDCPDrivingTubePort_InterfaceVersion\x12$\n\x1bPDCPDrivingTubePort_VERSION\x18\xb4\x08 \x01(\r\"\x97\x01\n/PDCPDrivingTubePort_InterfaceVersion_array_port\x12\x64\n\x04\x64\x61ta\x18\xda\x1f \x03(\x0b\x32U.pb.pdcp.pdcpdriving_tube_port_interface_version.PDCPDrivingTubePort_InterfaceVersion'
)




_PDCPDRIVINGTUBEPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='PDCPDrivingTubePort_InterfaceVersion',
  full_name='pb.pdcp.pdcpdriving_tube_port_interface_version.PDCPDrivingTubePort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='PDCPDrivingTubePort_VERSION', full_name='pb.pdcp.pdcpdriving_tube_port_interface_version.PDCPDrivingTubePort_InterfaceVersion.PDCPDrivingTubePort_VERSION', index=0,
      number=1076, type=13, cpp_type=3, label=1,
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
  serialized_start=103,
  serialized_end=179,
)


_PDCPDRIVINGTUBEPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='PDCPDrivingTubePort_InterfaceVersion_array_port',
  full_name='pb.pdcp.pdcpdriving_tube_port_interface_version.PDCPDrivingTubePort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.pdcp.pdcpdriving_tube_port_interface_version.PDCPDrivingTubePort_InterfaceVersion_array_port.data', index=0,
      number=4058, type=11, cpp_type=10, label=3,
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
  serialized_start=182,
  serialized_end=333,
)

_PDCPDRIVINGTUBEPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _PDCPDRIVINGTUBEPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['PDCPDrivingTubePort_InterfaceVersion'] = _PDCPDRIVINGTUBEPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['PDCPDrivingTubePort_InterfaceVersion_array_port'] = _PDCPDRIVINGTUBEPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PDCPDrivingTubePort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('PDCPDrivingTubePort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _PDCPDRIVINGTUBEPORT_INTERFACEVERSION,
  '__module__' : 'pdcp.pdcpdriving_tube_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.pdcp.pdcpdriving_tube_port_interface_version.PDCPDrivingTubePort_InterfaceVersion)
  })
_sym_db.RegisterMessage(PDCPDrivingTubePort_InterfaceVersion)

PDCPDrivingTubePort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('PDCPDrivingTubePort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _PDCPDRIVINGTUBEPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'pdcp.pdcpdriving_tube_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.pdcp.pdcpdriving_tube_port_interface_version.PDCPDrivingTubePort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(PDCPDrivingTubePort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)