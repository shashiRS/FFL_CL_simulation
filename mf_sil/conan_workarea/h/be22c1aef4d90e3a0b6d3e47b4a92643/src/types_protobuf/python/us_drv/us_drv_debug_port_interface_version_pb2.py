# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_drv/us_drv_debug_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_drv/us_drv_debug_port_interface_version.proto',
  package='pb.us_drv.us_drv_debug_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n0us_drv/us_drv_debug_port_interface_version.proto\x12-pb.us_drv.us_drv_debug_port_interface_version\"B\n\x1fUsDrvDebugPort_InterfaceVersion\x12\x1f\n\x16UsDrvDebugPort_VERSION\x18\x8e\x06 \x01(\r\"\x8b\x01\n*UsDrvDebugPort_InterfaceVersion_array_port\x12]\n\x04\x64\x61ta\x18\xfe\x1b \x03(\x0b\x32N.pb.us_drv.us_drv_debug_port_interface_version.UsDrvDebugPort_InterfaceVersion'
)




_USDRVDEBUGPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='UsDrvDebugPort_InterfaceVersion',
  full_name='pb.us_drv.us_drv_debug_port_interface_version.UsDrvDebugPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='UsDrvDebugPort_VERSION', full_name='pb.us_drv.us_drv_debug_port_interface_version.UsDrvDebugPort_InterfaceVersion.UsDrvDebugPort_VERSION', index=0,
      number=782, type=13, cpp_type=3, label=1,
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
  serialized_start=99,
  serialized_end=165,
)


_USDRVDEBUGPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='UsDrvDebugPort_InterfaceVersion_array_port',
  full_name='pb.us_drv.us_drv_debug_port_interface_version.UsDrvDebugPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_drv.us_drv_debug_port_interface_version.UsDrvDebugPort_InterfaceVersion_array_port.data', index=0,
      number=3582, type=11, cpp_type=10, label=3,
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
  serialized_end=307,
)

_USDRVDEBUGPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _USDRVDEBUGPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['UsDrvDebugPort_InterfaceVersion'] = _USDRVDEBUGPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['UsDrvDebugPort_InterfaceVersion_array_port'] = _USDRVDEBUGPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsDrvDebugPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('UsDrvDebugPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _USDRVDEBUGPORT_INTERFACEVERSION,
  '__module__' : 'us_drv.us_drv_debug_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_debug_port_interface_version.UsDrvDebugPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(UsDrvDebugPort_InterfaceVersion)

UsDrvDebugPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('UsDrvDebugPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USDRVDEBUGPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'us_drv.us_drv_debug_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_debug_port_interface_version.UsDrvDebugPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(UsDrvDebugPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
