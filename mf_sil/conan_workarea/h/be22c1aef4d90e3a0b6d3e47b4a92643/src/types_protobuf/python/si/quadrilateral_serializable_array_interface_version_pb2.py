# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/quadrilateral_serializable_array_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/quadrilateral_serializable_array_interface_version.proto',
  package='pb.si.quadrilateral_serializable_array_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n;si/quadrilateral_serializable_array_interface_version.proto\x12\x38pb.si.quadrilateral_serializable_array_interface_version\"b\n/QuadrilateralSerializableArray_InterfaceVersion\x12/\n&QuadrilateralSerializableArray_VERSION\x18\x9a\x11 \x01(\r\"\xb6\x01\n:QuadrilateralSerializableArray_InterfaceVersion_array_port\x12x\n\x04\x64\x61ta\x18\x93\x1e \x03(\x0b\x32i.pb.si.quadrilateral_serializable_array_interface_version.QuadrilateralSerializableArray_InterfaceVersion'
)




_QUADRILATERALSERIALIZABLEARRAY_INTERFACEVERSION = _descriptor.Descriptor(
  name='QuadrilateralSerializableArray_InterfaceVersion',
  full_name='pb.si.quadrilateral_serializable_array_interface_version.QuadrilateralSerializableArray_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='QuadrilateralSerializableArray_VERSION', full_name='pb.si.quadrilateral_serializable_array_interface_version.QuadrilateralSerializableArray_InterfaceVersion.QuadrilateralSerializableArray_VERSION', index=0,
      number=2202, type=13, cpp_type=3, label=1,
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
  serialized_start=121,
  serialized_end=219,
)


_QUADRILATERALSERIALIZABLEARRAY_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='QuadrilateralSerializableArray_InterfaceVersion_array_port',
  full_name='pb.si.quadrilateral_serializable_array_interface_version.QuadrilateralSerializableArray_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.quadrilateral_serializable_array_interface_version.QuadrilateralSerializableArray_InterfaceVersion_array_port.data', index=0,
      number=3859, type=11, cpp_type=10, label=3,
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
  serialized_start=222,
  serialized_end=404,
)

_QUADRILATERALSERIALIZABLEARRAY_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _QUADRILATERALSERIALIZABLEARRAY_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['QuadrilateralSerializableArray_InterfaceVersion'] = _QUADRILATERALSERIALIZABLEARRAY_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['QuadrilateralSerializableArray_InterfaceVersion_array_port'] = _QUADRILATERALSERIALIZABLEARRAY_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

QuadrilateralSerializableArray_InterfaceVersion = _reflection.GeneratedProtocolMessageType('QuadrilateralSerializableArray_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _QUADRILATERALSERIALIZABLEARRAY_INTERFACEVERSION,
  '__module__' : 'si.quadrilateral_serializable_array_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.quadrilateral_serializable_array_interface_version.QuadrilateralSerializableArray_InterfaceVersion)
  })
_sym_db.RegisterMessage(QuadrilateralSerializableArray_InterfaceVersion)

QuadrilateralSerializableArray_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('QuadrilateralSerializableArray_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _QUADRILATERALSERIALIZABLEARRAY_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'si.quadrilateral_serializable_array_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.quadrilateral_serializable_array_interface_version.QuadrilateralSerializableArray_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(QuadrilateralSerializableArray_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
