# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/triangle_expansion_serializable.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from si import triangle_serializable_array_pb2 as si_dot_triangle__serializable__array__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/triangle_expansion_serializable.proto',
  package='pb.si.triangle_expansion_serializable',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n(si/triangle_expansion_serializable.proto\x12%pb.si.triangle_expansion_serializable\x1a$si/triangle_serializable_array.proto\"\x82\x01\n\x1dTriangleExpansionSerializable\x12\x13\n\nactualSize\x18\xc1\x17 \x01(\r\x12L\n\x05\x61rray\x18\xb6\x05 \x03(\x0b\x32<.pb.si.triangle_serializable_array.TriangleSerializableArray\"\x7f\n(TriangleExpansionSerializable_array_port\x12S\n\x04\x64\x61ta\x18\x87\x12 \x03(\x0b\x32\x44.pb.si.triangle_expansion_serializable.TriangleExpansionSerializable'
  ,
  dependencies=[si_dot_triangle__serializable__array__pb2.DESCRIPTOR,])




_TRIANGLEEXPANSIONSERIALIZABLE = _descriptor.Descriptor(
  name='TriangleExpansionSerializable',
  full_name='pb.si.triangle_expansion_serializable.TriangleExpansionSerializable',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='actualSize', full_name='pb.si.triangle_expansion_serializable.TriangleExpansionSerializable.actualSize', index=0,
      number=3009, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='array', full_name='pb.si.triangle_expansion_serializable.TriangleExpansionSerializable.array', index=1,
      number=694, type=11, cpp_type=10, label=3,
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
  serialized_start=122,
  serialized_end=252,
)


_TRIANGLEEXPANSIONSERIALIZABLE_ARRAY_PORT = _descriptor.Descriptor(
  name='TriangleExpansionSerializable_array_port',
  full_name='pb.si.triangle_expansion_serializable.TriangleExpansionSerializable_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.triangle_expansion_serializable.TriangleExpansionSerializable_array_port.data', index=0,
      number=2311, type=11, cpp_type=10, label=3,
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
  serialized_start=254,
  serialized_end=381,
)

_TRIANGLEEXPANSIONSERIALIZABLE.fields_by_name['array'].message_type = si_dot_triangle__serializable__array__pb2._TRIANGLESERIALIZABLEARRAY
_TRIANGLEEXPANSIONSERIALIZABLE_ARRAY_PORT.fields_by_name['data'].message_type = _TRIANGLEEXPANSIONSERIALIZABLE
DESCRIPTOR.message_types_by_name['TriangleExpansionSerializable'] = _TRIANGLEEXPANSIONSERIALIZABLE
DESCRIPTOR.message_types_by_name['TriangleExpansionSerializable_array_port'] = _TRIANGLEEXPANSIONSERIALIZABLE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TriangleExpansionSerializable = _reflection.GeneratedProtocolMessageType('TriangleExpansionSerializable', (_message.Message,), {
  'DESCRIPTOR' : _TRIANGLEEXPANSIONSERIALIZABLE,
  '__module__' : 'si.triangle_expansion_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.triangle_expansion_serializable.TriangleExpansionSerializable)
  })
_sym_db.RegisterMessage(TriangleExpansionSerializable)

TriangleExpansionSerializable_array_port = _reflection.GeneratedProtocolMessageType('TriangleExpansionSerializable_array_port', (_message.Message,), {
  'DESCRIPTOR' : _TRIANGLEEXPANSIONSERIALIZABLE_ARRAY_PORT,
  '__module__' : 'si.triangle_expansion_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.triangle_expansion_serializable.TriangleExpansionSerializable_array_port)
  })
_sym_db.RegisterMessage(TriangleExpansionSerializable_array_port)


# @@protoc_insertion_point(module_scope)
