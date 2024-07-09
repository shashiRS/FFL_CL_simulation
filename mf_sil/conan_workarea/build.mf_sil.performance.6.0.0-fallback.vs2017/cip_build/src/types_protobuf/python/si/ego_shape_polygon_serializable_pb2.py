# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/ego_shape_polygon_serializable.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from cml import vec2_df_pod_pb2 as cml_dot_vec2__df__pod__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/ego_shape_polygon_serializable.proto',
  package='pb.si.ego_shape_polygon_serializable',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\'si/ego_shape_polygon_serializable.proto\x12$pb.si.ego_shape_polygon_serializable\x1a\x15\x63ml/vec2_df_pod.proto\"b\n\x1b\x45goShapePolygonSerializable\x12\x13\n\nactualSize\x18\xc1\x17 \x01(\r\x12.\n\x05\x61rray\x18\xb9\x18 \x03(\x0b\x32\x1e.pb.cml.vec2_df_pod.Vec2Df_POD\"z\n&EgoShapePolygonSerializable_array_port\x12P\n\x04\x64\x61ta\x18\xab\x0b \x03(\x0b\x32\x41.pb.si.ego_shape_polygon_serializable.EgoShapePolygonSerializable'
  ,
  dependencies=[cml_dot_vec2__df__pod__pb2.DESCRIPTOR,])




_EGOSHAPEPOLYGONSERIALIZABLE = _descriptor.Descriptor(
  name='EgoShapePolygonSerializable',
  full_name='pb.si.ego_shape_polygon_serializable.EgoShapePolygonSerializable',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='actualSize', full_name='pb.si.ego_shape_polygon_serializable.EgoShapePolygonSerializable.actualSize', index=0,
      number=3009, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='array', full_name='pb.si.ego_shape_polygon_serializable.EgoShapePolygonSerializable.array', index=1,
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
  serialized_start=104,
  serialized_end=202,
)


_EGOSHAPEPOLYGONSERIALIZABLE_ARRAY_PORT = _descriptor.Descriptor(
  name='EgoShapePolygonSerializable_array_port',
  full_name='pb.si.ego_shape_polygon_serializable.EgoShapePolygonSerializable_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.ego_shape_polygon_serializable.EgoShapePolygonSerializable_array_port.data', index=0,
      number=1451, type=11, cpp_type=10, label=3,
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
  serialized_end=326,
)

_EGOSHAPEPOLYGONSERIALIZABLE.fields_by_name['array'].message_type = cml_dot_vec2__df__pod__pb2._VEC2DF_POD
_EGOSHAPEPOLYGONSERIALIZABLE_ARRAY_PORT.fields_by_name['data'].message_type = _EGOSHAPEPOLYGONSERIALIZABLE
DESCRIPTOR.message_types_by_name['EgoShapePolygonSerializable'] = _EGOSHAPEPOLYGONSERIALIZABLE
DESCRIPTOR.message_types_by_name['EgoShapePolygonSerializable_array_port'] = _EGOSHAPEPOLYGONSERIALIZABLE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

EgoShapePolygonSerializable = _reflection.GeneratedProtocolMessageType('EgoShapePolygonSerializable', (_message.Message,), {
  'DESCRIPTOR' : _EGOSHAPEPOLYGONSERIALIZABLE,
  '__module__' : 'si.ego_shape_polygon_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.ego_shape_polygon_serializable.EgoShapePolygonSerializable)
  })
_sym_db.RegisterMessage(EgoShapePolygonSerializable)

EgoShapePolygonSerializable_array_port = _reflection.GeneratedProtocolMessageType('EgoShapePolygonSerializable_array_port', (_message.Message,), {
  'DESCRIPTOR' : _EGOSHAPEPOLYGONSERIALIZABLE_ARRAY_PORT,
  '__module__' : 'si.ego_shape_polygon_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.ego_shape_polygon_serializable.EgoShapePolygonSerializable_array_port)
  })
_sym_db.RegisterMessage(EgoShapePolygonSerializable_array_port)


# @@protoc_insertion_point(module_scope)