# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_whlprotectproc/whpproc_debug_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_whlprotectproc/whpproc_debug_port_interface_version.proto',
  package='pb.mf_whlprotectproc.whpproc_debug_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n<mf_whlprotectproc/whpproc_debug_port_interface_version.proto\x12\x39pb.mf_whlprotectproc.whpproc_debug_port_interface_version\"E\n!WHPProcDebugPort_InterfaceVersion\x12 \n\x18WHPProcDebugPort_VERSION\x18\x42 \x01(\r\"\x9b\x01\n,WHPProcDebugPort_InterfaceVersion_array_port\x12k\n\x04\x64\x61ta\x18\x94\x07 \x03(\x0b\x32\\.pb.mf_whlprotectproc.whpproc_debug_port_interface_version.WHPProcDebugPort_InterfaceVersion'
)




_WHPPROCDEBUGPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='WHPProcDebugPort_InterfaceVersion',
  full_name='pb.mf_whlprotectproc.whpproc_debug_port_interface_version.WHPProcDebugPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='WHPProcDebugPort_VERSION', full_name='pb.mf_whlprotectproc.whpproc_debug_port_interface_version.WHPProcDebugPort_InterfaceVersion.WHPProcDebugPort_VERSION', index=0,
      number=66, type=13, cpp_type=3, label=1,
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
  serialized_start=123,
  serialized_end=192,
)


_WHPPROCDEBUGPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='WHPProcDebugPort_InterfaceVersion_array_port',
  full_name='pb.mf_whlprotectproc.whpproc_debug_port_interface_version.WHPProcDebugPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_whlprotectproc.whpproc_debug_port_interface_version.WHPProcDebugPort_InterfaceVersion_array_port.data', index=0,
      number=916, type=11, cpp_type=10, label=3,
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
  serialized_start=195,
  serialized_end=350,
)

_WHPPROCDEBUGPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _WHPPROCDEBUGPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['WHPProcDebugPort_InterfaceVersion'] = _WHPPROCDEBUGPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['WHPProcDebugPort_InterfaceVersion_array_port'] = _WHPPROCDEBUGPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

WHPProcDebugPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('WHPProcDebugPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _WHPPROCDEBUGPORT_INTERFACEVERSION,
  '__module__' : 'mf_whlprotectproc.whpproc_debug_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_whlprotectproc.whpproc_debug_port_interface_version.WHPProcDebugPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(WHPProcDebugPort_InterfaceVersion)

WHPProcDebugPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('WHPProcDebugPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _WHPPROCDEBUGPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'mf_whlprotectproc.whpproc_debug_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_whlprotectproc.whpproc_debug_port_interface_version.WHPProcDebugPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(WHPProcDebugPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
