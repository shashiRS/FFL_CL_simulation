# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/line_segment_serializable_array_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/line_segment_serializable_array_interface_version.proto',
  package='pb.si.line_segment_serializable_array_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n:si/line_segment_serializable_array_interface_version.proto\x12\x37pb.si.line_segment_serializable_array_interface_version\"^\n-LineSegmentSerializableArray_InterfaceVersion\x12-\n$LineSegmentSerializableArray_VERSION\x18\xd6\x06 \x01(\r\"\xb1\x01\n8LineSegmentSerializableArray_InterfaceVersion_array_port\x12u\n\x04\x64\x61ta\x18\xdf\x06 \x03(\x0b\x32\x66.pb.si.line_segment_serializable_array_interface_version.LineSegmentSerializableArray_InterfaceVersion'
)




_LINESEGMENTSERIALIZABLEARRAY_INTERFACEVERSION = _descriptor.Descriptor(
  name='LineSegmentSerializableArray_InterfaceVersion',
  full_name='pb.si.line_segment_serializable_array_interface_version.LineSegmentSerializableArray_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='LineSegmentSerializableArray_VERSION', full_name='pb.si.line_segment_serializable_array_interface_version.LineSegmentSerializableArray_InterfaceVersion.LineSegmentSerializableArray_VERSION', index=0,
      number=854, type=13, cpp_type=3, label=1,
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
  serialized_start=119,
  serialized_end=213,
)


_LINESEGMENTSERIALIZABLEARRAY_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='LineSegmentSerializableArray_InterfaceVersion_array_port',
  full_name='pb.si.line_segment_serializable_array_interface_version.LineSegmentSerializableArray_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.line_segment_serializable_array_interface_version.LineSegmentSerializableArray_InterfaceVersion_array_port.data', index=0,
      number=863, type=11, cpp_type=10, label=3,
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
  serialized_start=216,
  serialized_end=393,
)

_LINESEGMENTSERIALIZABLEARRAY_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _LINESEGMENTSERIALIZABLEARRAY_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['LineSegmentSerializableArray_InterfaceVersion'] = _LINESEGMENTSERIALIZABLEARRAY_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['LineSegmentSerializableArray_InterfaceVersion_array_port'] = _LINESEGMENTSERIALIZABLEARRAY_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LineSegmentSerializableArray_InterfaceVersion = _reflection.GeneratedProtocolMessageType('LineSegmentSerializableArray_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _LINESEGMENTSERIALIZABLEARRAY_INTERFACEVERSION,
  '__module__' : 'si.line_segment_serializable_array_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.line_segment_serializable_array_interface_version.LineSegmentSerializableArray_InterfaceVersion)
  })
_sym_db.RegisterMessage(LineSegmentSerializableArray_InterfaceVersion)

LineSegmentSerializableArray_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('LineSegmentSerializableArray_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _LINESEGMENTSERIALIZABLEARRAY_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'si.line_segment_serializable_array_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.line_segment_serializable_array_interface_version.LineSegmentSerializableArray_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(LineSegmentSerializableArray_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
