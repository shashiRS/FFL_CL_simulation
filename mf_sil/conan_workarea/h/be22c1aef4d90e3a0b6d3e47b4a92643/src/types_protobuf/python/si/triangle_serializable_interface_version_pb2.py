# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/triangle_serializable_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/triangle_serializable_interface_version.proto',
  package='pb.si.triangle_serializable_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n0si/triangle_serializable_interface_version.proto\x12-pb.si.triangle_serializable_interface_version\"N\n%TriangleSerializable_InterfaceVersion\x12%\n\x1cTriangleSerializable_VERSION\x18\xcb\x0b \x01(\r\"\x97\x01\n0TriangleSerializable_InterfaceVersion_array_port\x12\x63\n\x04\x64\x61ta\x18\xc5\x08 \x03(\x0b\x32T.pb.si.triangle_serializable_interface_version.TriangleSerializable_InterfaceVersion'
)




_TRIANGLESERIALIZABLE_INTERFACEVERSION = _descriptor.Descriptor(
  name='TriangleSerializable_InterfaceVersion',
  full_name='pb.si.triangle_serializable_interface_version.TriangleSerializable_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='TriangleSerializable_VERSION', full_name='pb.si.triangle_serializable_interface_version.TriangleSerializable_InterfaceVersion.TriangleSerializable_VERSION', index=0,
      number=1483, type=13, cpp_type=3, label=1,
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
  serialized_end=177,
)


_TRIANGLESERIALIZABLE_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='TriangleSerializable_InterfaceVersion_array_port',
  full_name='pb.si.triangle_serializable_interface_version.TriangleSerializable_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.triangle_serializable_interface_version.TriangleSerializable_InterfaceVersion_array_port.data', index=0,
      number=1093, type=11, cpp_type=10, label=3,
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
  serialized_start=180,
  serialized_end=331,
)

_TRIANGLESERIALIZABLE_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _TRIANGLESERIALIZABLE_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['TriangleSerializable_InterfaceVersion'] = _TRIANGLESERIALIZABLE_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['TriangleSerializable_InterfaceVersion_array_port'] = _TRIANGLESERIALIZABLE_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TriangleSerializable_InterfaceVersion = _reflection.GeneratedProtocolMessageType('TriangleSerializable_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _TRIANGLESERIALIZABLE_INTERFACEVERSION,
  '__module__' : 'si.triangle_serializable_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.triangle_serializable_interface_version.TriangleSerializable_InterfaceVersion)
  })
_sym_db.RegisterMessage(TriangleSerializable_InterfaceVersion)

TriangleSerializable_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('TriangleSerializable_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _TRIANGLESERIALIZABLE_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'si.triangle_serializable_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.triangle_serializable_interface_version.TriangleSerializable_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(TriangleSerializable_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
