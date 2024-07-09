# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/quadrilateral_expansion_serializable.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from si import quadrilateral_serializable_array_pb2 as si_dot_quadrilateral__serializable__array__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/quadrilateral_expansion_serializable.proto',
  package='pb.si.quadrilateral_expansion_serializable',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n-si/quadrilateral_expansion_serializable.proto\x12*pb.si.quadrilateral_expansion_serializable\x1a)si/quadrilateral_serializable_array.proto\"\x91\x01\n\"QuadrilateralExpansionSerializable\x12\x13\n\nactualSize\x18\xc1\x17 \x01(\r\x12V\n\x05\x61rray\x18\x8e\x1e \x03(\x0b\x32\x46.pb.si.quadrilateral_serializable_array.QuadrilateralSerializableArray\"\x8e\x01\n-QuadrilateralExpansionSerializable_array_port\x12]\n\x04\x64\x61ta\x18\x91\x18 \x03(\x0b\x32N.pb.si.quadrilateral_expansion_serializable.QuadrilateralExpansionSerializable'
  ,
  dependencies=[si_dot_quadrilateral__serializable__array__pb2.DESCRIPTOR,])




_QUADRILATERALEXPANSIONSERIALIZABLE = _descriptor.Descriptor(
  name='QuadrilateralExpansionSerializable',
  full_name='pb.si.quadrilateral_expansion_serializable.QuadrilateralExpansionSerializable',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='actualSize', full_name='pb.si.quadrilateral_expansion_serializable.QuadrilateralExpansionSerializable.actualSize', index=0,
      number=3009, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='array', full_name='pb.si.quadrilateral_expansion_serializable.QuadrilateralExpansionSerializable.array', index=1,
      number=3854, type=11, cpp_type=10, label=3,
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
  serialized_start=137,
  serialized_end=282,
)


_QUADRILATERALEXPANSIONSERIALIZABLE_ARRAY_PORT = _descriptor.Descriptor(
  name='QuadrilateralExpansionSerializable_array_port',
  full_name='pb.si.quadrilateral_expansion_serializable.QuadrilateralExpansionSerializable_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.quadrilateral_expansion_serializable.QuadrilateralExpansionSerializable_array_port.data', index=0,
      number=3089, type=11, cpp_type=10, label=3,
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
  serialized_start=285,
  serialized_end=427,
)

_QUADRILATERALEXPANSIONSERIALIZABLE.fields_by_name['array'].message_type = si_dot_quadrilateral__serializable__array__pb2._QUADRILATERALSERIALIZABLEARRAY
_QUADRILATERALEXPANSIONSERIALIZABLE_ARRAY_PORT.fields_by_name['data'].message_type = _QUADRILATERALEXPANSIONSERIALIZABLE
DESCRIPTOR.message_types_by_name['QuadrilateralExpansionSerializable'] = _QUADRILATERALEXPANSIONSERIALIZABLE
DESCRIPTOR.message_types_by_name['QuadrilateralExpansionSerializable_array_port'] = _QUADRILATERALEXPANSIONSERIALIZABLE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

QuadrilateralExpansionSerializable = _reflection.GeneratedProtocolMessageType('QuadrilateralExpansionSerializable', (_message.Message,), {
  'DESCRIPTOR' : _QUADRILATERALEXPANSIONSERIALIZABLE,
  '__module__' : 'si.quadrilateral_expansion_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.quadrilateral_expansion_serializable.QuadrilateralExpansionSerializable)
  })
_sym_db.RegisterMessage(QuadrilateralExpansionSerializable)

QuadrilateralExpansionSerializable_array_port = _reflection.GeneratedProtocolMessageType('QuadrilateralExpansionSerializable_array_port', (_message.Message,), {
  'DESCRIPTOR' : _QUADRILATERALEXPANSIONSERIALIZABLE_ARRAY_PORT,
  '__module__' : 'si.quadrilateral_expansion_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.quadrilateral_expansion_serializable.QuadrilateralExpansionSerializable_array_port)
  })
_sym_db.RegisterMessage(QuadrilateralExpansionSerializable_array_port)


# @@protoc_insertion_point(module_scope)