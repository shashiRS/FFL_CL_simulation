# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: avga_swc/avga_supervision_request_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='avga_swc/avga_supervision_request_interface_version.proto',
  package='pb.avga_swc.avga_supervision_request_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n9avga_swc/avga_supervision_request_interface_version.proto\x12\x36pb.avga_swc.avga_supervision_request_interface_version\"T\n(AVGA_SupervisionRequest_InterfaceVersion\x12(\n\x1f\x41VGA_SupervisionRequest_VERSION\x18\xa7\x16 \x01(\r\"\xa6\x01\n3AVGA_SupervisionRequest_InterfaceVersion_array_port\x12o\n\x04\x64\x61ta\x18\xf9\x18 \x03(\x0b\x32`.pb.avga_swc.avga_supervision_request_interface_version.AVGA_SupervisionRequest_InterfaceVersion'
)




_AVGA_SUPERVISIONREQUEST_INTERFACEVERSION = _descriptor.Descriptor(
  name='AVGA_SupervisionRequest_InterfaceVersion',
  full_name='pb.avga_swc.avga_supervision_request_interface_version.AVGA_SupervisionRequest_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='AVGA_SupervisionRequest_VERSION', full_name='pb.avga_swc.avga_supervision_request_interface_version.AVGA_SupervisionRequest_InterfaceVersion.AVGA_SupervisionRequest_VERSION', index=0,
      number=2855, type=13, cpp_type=3, label=1,
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
  serialized_start=117,
  serialized_end=201,
)


_AVGA_SUPERVISIONREQUEST_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='AVGA_SupervisionRequest_InterfaceVersion_array_port',
  full_name='pb.avga_swc.avga_supervision_request_interface_version.AVGA_SupervisionRequest_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.avga_swc.avga_supervision_request_interface_version.AVGA_SupervisionRequest_InterfaceVersion_array_port.data', index=0,
      number=3193, type=11, cpp_type=10, label=3,
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
  serialized_start=204,
  serialized_end=370,
)

_AVGA_SUPERVISIONREQUEST_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _AVGA_SUPERVISIONREQUEST_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['AVGA_SupervisionRequest_InterfaceVersion'] = _AVGA_SUPERVISIONREQUEST_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['AVGA_SupervisionRequest_InterfaceVersion_array_port'] = _AVGA_SUPERVISIONREQUEST_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

AVGA_SupervisionRequest_InterfaceVersion = _reflection.GeneratedProtocolMessageType('AVGA_SupervisionRequest_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _AVGA_SUPERVISIONREQUEST_INTERFACEVERSION,
  '__module__' : 'avga_swc.avga_supervision_request_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.avga_swc.avga_supervision_request_interface_version.AVGA_SupervisionRequest_InterfaceVersion)
  })
_sym_db.RegisterMessage(AVGA_SupervisionRequest_InterfaceVersion)

AVGA_SupervisionRequest_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('AVGA_SupervisionRequest_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _AVGA_SUPERVISIONREQUEST_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'avga_swc.avga_supervision_request_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.avga_swc.avga_supervision_request_interface_version.AVGA_SupervisionRequest_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(AVGA_SupervisionRequest_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)