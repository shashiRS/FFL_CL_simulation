# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/quadrilateral_serializable.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from cml import vec2_df_pod_pb2 as cml_dot_vec2__df__pod__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/quadrilateral_serializable.proto',
  package='pb.si.quadrilateral_serializable',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n#si/quadrilateral_serializable.proto\x12 pb.si.quadrilateral_serializable\x1a\x15\x63ml/vec2_df_pod.proto\"`\n\x19QuadrilateralSerializable\x12\x13\n\nactualSize\x18\xc1\x17 \x01(\r\x12.\n\x05\x61rray\x18\xb9\x18 \x03(\x0b\x32\x1e.pb.cml.vec2_df_pod.Vec2Df_POD\"r\n$QuadrilateralSerializable_array_port\x12J\n\x04\x64\x61ta\x18\xb6\x06 \x03(\x0b\x32;.pb.si.quadrilateral_serializable.QuadrilateralSerializable'
  ,
  dependencies=[cml_dot_vec2__df__pod__pb2.DESCRIPTOR,])




_QUADRILATERALSERIALIZABLE = _descriptor.Descriptor(
  name='QuadrilateralSerializable',
  full_name='pb.si.quadrilateral_serializable.QuadrilateralSerializable',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='actualSize', full_name='pb.si.quadrilateral_serializable.QuadrilateralSerializable.actualSize', index=0,
      number=3009, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='array', full_name='pb.si.quadrilateral_serializable.QuadrilateralSerializable.array', index=1,
      number=3129, type=11, cpp_type=10, label=3,
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
  serialized_start=96,
  serialized_end=192,
)


_QUADRILATERALSERIALIZABLE_ARRAY_PORT = _descriptor.Descriptor(
  name='QuadrilateralSerializable_array_port',
  full_name='pb.si.quadrilateral_serializable.QuadrilateralSerializable_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.quadrilateral_serializable.QuadrilateralSerializable_array_port.data', index=0,
      number=822, type=11, cpp_type=10, label=3,
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
  serialized_start=194,
  serialized_end=308,
)

_QUADRILATERALSERIALIZABLE.fields_by_name['array'].message_type = cml_dot_vec2__df__pod__pb2._VEC2DF_POD
_QUADRILATERALSERIALIZABLE_ARRAY_PORT.fields_by_name['data'].message_type = _QUADRILATERALSERIALIZABLE
DESCRIPTOR.message_types_by_name['QuadrilateralSerializable'] = _QUADRILATERALSERIALIZABLE
DESCRIPTOR.message_types_by_name['QuadrilateralSerializable_array_port'] = _QUADRILATERALSERIALIZABLE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

QuadrilateralSerializable = _reflection.GeneratedProtocolMessageType('QuadrilateralSerializable', (_message.Message,), {
  'DESCRIPTOR' : _QUADRILATERALSERIALIZABLE,
  '__module__' : 'si.quadrilateral_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.quadrilateral_serializable.QuadrilateralSerializable)
  })
_sym_db.RegisterMessage(QuadrilateralSerializable)

QuadrilateralSerializable_array_port = _reflection.GeneratedProtocolMessageType('QuadrilateralSerializable_array_port', (_message.Message,), {
  'DESCRIPTOR' : _QUADRILATERALSERIALIZABLE_ARRAY_PORT,
  '__module__' : 'si.quadrilateral_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.quadrilateral_serializable.QuadrilateralSerializable_array_port)
  })
_sym_db.RegisterMessage(QuadrilateralSerializable_array_port)


# @@protoc_insertion_point(module_scope)