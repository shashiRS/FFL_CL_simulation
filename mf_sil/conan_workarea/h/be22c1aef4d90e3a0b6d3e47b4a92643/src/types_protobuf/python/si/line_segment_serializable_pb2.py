# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/line_segment_serializable.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from cml import vec2_df_pod_pb2 as cml_dot_vec2__df__pod__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/line_segment_serializable.proto',
  package='pb.si.line_segment_serializable',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\"si/line_segment_serializable.proto\x12\x1fpb.si.line_segment_serializable\x1a\x15\x63ml/vec2_df_pod.proto\"^\n\x17LineSegmentSerializable\x12\x13\n\nactualSize\x18\xc1\x17 \x01(\r\x12.\n\x05\x61rray\x18\xb9\x18 \x03(\x0b\x32\x1e.pb.cml.vec2_df_pod.Vec2Df_POD\"m\n\"LineSegmentSerializable_array_port\x12G\n\x04\x64\x61ta\x18\xe6\x04 \x03(\x0b\x32\x38.pb.si.line_segment_serializable.LineSegmentSerializable'
  ,
  dependencies=[cml_dot_vec2__df__pod__pb2.DESCRIPTOR,])




_LINESEGMENTSERIALIZABLE = _descriptor.Descriptor(
  name='LineSegmentSerializable',
  full_name='pb.si.line_segment_serializable.LineSegmentSerializable',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='actualSize', full_name='pb.si.line_segment_serializable.LineSegmentSerializable.actualSize', index=0,
      number=3009, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='array', full_name='pb.si.line_segment_serializable.LineSegmentSerializable.array', index=1,
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
  serialized_start=94,
  serialized_end=188,
)


_LINESEGMENTSERIALIZABLE_ARRAY_PORT = _descriptor.Descriptor(
  name='LineSegmentSerializable_array_port',
  full_name='pb.si.line_segment_serializable.LineSegmentSerializable_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.line_segment_serializable.LineSegmentSerializable_array_port.data', index=0,
      number=614, type=11, cpp_type=10, label=3,
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
  serialized_start=190,
  serialized_end=299,
)

_LINESEGMENTSERIALIZABLE.fields_by_name['array'].message_type = cml_dot_vec2__df__pod__pb2._VEC2DF_POD
_LINESEGMENTSERIALIZABLE_ARRAY_PORT.fields_by_name['data'].message_type = _LINESEGMENTSERIALIZABLE
DESCRIPTOR.message_types_by_name['LineSegmentSerializable'] = _LINESEGMENTSERIALIZABLE
DESCRIPTOR.message_types_by_name['LineSegmentSerializable_array_port'] = _LINESEGMENTSERIALIZABLE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LineSegmentSerializable = _reflection.GeneratedProtocolMessageType('LineSegmentSerializable', (_message.Message,), {
  'DESCRIPTOR' : _LINESEGMENTSERIALIZABLE,
  '__module__' : 'si.line_segment_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.line_segment_serializable.LineSegmentSerializable)
  })
_sym_db.RegisterMessage(LineSegmentSerializable)

LineSegmentSerializable_array_port = _reflection.GeneratedProtocolMessageType('LineSegmentSerializable_array_port', (_message.Message,), {
  'DESCRIPTOR' : _LINESEGMENTSERIALIZABLE_ARRAY_PORT,
  '__module__' : 'si.line_segment_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.line_segment_serializable.LineSegmentSerializable_array_port)
  })
_sym_db.RegisterMessage(LineSegmentSerializable_array_port)


# @@protoc_insertion_point(module_scope)