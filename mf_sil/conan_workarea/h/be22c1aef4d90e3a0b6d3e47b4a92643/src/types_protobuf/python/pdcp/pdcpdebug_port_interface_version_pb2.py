# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: pdcp/pdcpdebug_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='pdcp/pdcpdebug_port_interface_version.proto',
  package='pb.pdcp.pdcpdebug_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n+pdcp/pdcpdebug_port_interface_version.proto\x12(pb.pdcp.pdcpdebug_port_interface_version\"@\n\x1ePDCPDebugPort_InterfaceVersion\x12\x1e\n\x15PDCPDebugPort_VERSION\x18\x81\x02 \x01(\r\"\x84\x01\n)PDCPDebugPort_InterfaceVersion_array_port\x12W\n\x04\x64\x61ta\x18\xd6\x16 \x03(\x0b\x32H.pb.pdcp.pdcpdebug_port_interface_version.PDCPDebugPort_InterfaceVersion'
)




_PDCPDEBUGPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='PDCPDebugPort_InterfaceVersion',
  full_name='pb.pdcp.pdcpdebug_port_interface_version.PDCPDebugPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='PDCPDebugPort_VERSION', full_name='pb.pdcp.pdcpdebug_port_interface_version.PDCPDebugPort_InterfaceVersion.PDCPDebugPort_VERSION', index=0,
      number=257, type=13, cpp_type=3, label=1,
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
  serialized_start=89,
  serialized_end=153,
)


_PDCPDEBUGPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='PDCPDebugPort_InterfaceVersion_array_port',
  full_name='pb.pdcp.pdcpdebug_port_interface_version.PDCPDebugPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.pdcp.pdcpdebug_port_interface_version.PDCPDebugPort_InterfaceVersion_array_port.data', index=0,
      number=2902, type=11, cpp_type=10, label=3,
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
  serialized_start=156,
  serialized_end=288,
)

_PDCPDEBUGPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _PDCPDEBUGPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['PDCPDebugPort_InterfaceVersion'] = _PDCPDEBUGPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['PDCPDebugPort_InterfaceVersion_array_port'] = _PDCPDEBUGPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PDCPDebugPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('PDCPDebugPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _PDCPDEBUGPORT_INTERFACEVERSION,
  '__module__' : 'pdcp.pdcpdebug_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.pdcp.pdcpdebug_port_interface_version.PDCPDebugPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(PDCPDebugPort_InterfaceVersion)

PDCPDebugPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('PDCPDebugPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _PDCPDEBUGPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'pdcp.pdcpdebug_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.pdcp.pdcpdebug_port_interface_version.PDCPDebugPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(PDCPDebugPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
