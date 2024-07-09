# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/pull_corner_rect.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from si import quadrilateral_serializable_pb2 as si_dot_quadrilateral__serializable__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/pull_corner_rect.proto',
  package='pb.si.pull_corner_rect',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x19si/pull_corner_rect.proto\x12\x16pb.si.pull_corner_rect\x1a#si/quadrilateral_serializable.proto\"\xf0\x01\n\x0ePullCornerRect\x12H\n\x02\x66l\x18\xb7\x0f \x01(\x0b\x32;.pb.si.quadrilateral_serializable.QuadrilateralSerializable\x12H\n\x02\x66r\x18\xc8\x08 \x01(\x0b\x32;.pb.si.quadrilateral_serializable.QuadrilateralSerializable\x12J\n\x04\x63urb\x18\xa5\x16 \x01(\x0b\x32;.pb.si.quadrilateral_serializable.QuadrilateralSerializable\"R\n\x19PullCornerRect_array_port\x12\x35\n\x04\x64\x61ta\x18\x8a\x02 \x03(\x0b\x32&.pb.si.pull_corner_rect.PullCornerRect'
  ,
  dependencies=[si_dot_quadrilateral__serializable__pb2.DESCRIPTOR,])




_PULLCORNERRECT = _descriptor.Descriptor(
  name='PullCornerRect',
  full_name='pb.si.pull_corner_rect.PullCornerRect',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='fl', full_name='pb.si.pull_corner_rect.PullCornerRect.fl', index=0,
      number=1975, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='fr', full_name='pb.si.pull_corner_rect.PullCornerRect.fr', index=1,
      number=1096, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='curb', full_name='pb.si.pull_corner_rect.PullCornerRect.curb', index=2,
      number=2853, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
  serialized_start=91,
  serialized_end=331,
)


_PULLCORNERRECT_ARRAY_PORT = _descriptor.Descriptor(
  name='PullCornerRect_array_port',
  full_name='pb.si.pull_corner_rect.PullCornerRect_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.pull_corner_rect.PullCornerRect_array_port.data', index=0,
      number=266, type=11, cpp_type=10, label=3,
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
  serialized_start=333,
  serialized_end=415,
)

_PULLCORNERRECT.fields_by_name['fl'].message_type = si_dot_quadrilateral__serializable__pb2._QUADRILATERALSERIALIZABLE
_PULLCORNERRECT.fields_by_name['fr'].message_type = si_dot_quadrilateral__serializable__pb2._QUADRILATERALSERIALIZABLE
_PULLCORNERRECT.fields_by_name['curb'].message_type = si_dot_quadrilateral__serializable__pb2._QUADRILATERALSERIALIZABLE
_PULLCORNERRECT_ARRAY_PORT.fields_by_name['data'].message_type = _PULLCORNERRECT
DESCRIPTOR.message_types_by_name['PullCornerRect'] = _PULLCORNERRECT
DESCRIPTOR.message_types_by_name['PullCornerRect_array_port'] = _PULLCORNERRECT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PullCornerRect = _reflection.GeneratedProtocolMessageType('PullCornerRect', (_message.Message,), {
  'DESCRIPTOR' : _PULLCORNERRECT,
  '__module__' : 'si.pull_corner_rect_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.pull_corner_rect.PullCornerRect)
  })
_sym_db.RegisterMessage(PullCornerRect)

PullCornerRect_array_port = _reflection.GeneratedProtocolMessageType('PullCornerRect_array_port', (_message.Message,), {
  'DESCRIPTOR' : _PULLCORNERRECT_ARRAY_PORT,
  '__module__' : 'si.pull_corner_rect_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.pull_corner_rect.PullCornerRect_array_port)
  })
_sym_db.RegisterMessage(PullCornerRect_array_port)


# @@protoc_insertion_point(module_scope)