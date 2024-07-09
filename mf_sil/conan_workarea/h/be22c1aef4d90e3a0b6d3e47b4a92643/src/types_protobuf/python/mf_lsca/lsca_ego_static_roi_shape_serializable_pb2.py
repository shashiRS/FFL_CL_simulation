# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_lsca/lsca_ego_static_roi_shape_serializable.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from cml import vec2_df_pod_pb2 as cml_dot_vec2__df__pod__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_lsca/lsca_ego_static_roi_shape_serializable.proto',
  package='pb.mf_lsca.lsca_ego_static_roi_shape_serializable',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n4mf_lsca/lsca_ego_static_roi_shape_serializable.proto\x12\x31pb.mf_lsca.lsca_ego_static_roi_shape_serializable\x1a\x15\x63ml/vec2_df_pod.proto\"i\n!LscaEgoStaticRoiShapeSerializable\x12\x13\n\nactualSize\x18\xc1\x17 \x01(\r\x12/\n\x06points\x18\x8f\x1e \x03(\x0b\x32\x1e.pb.cml.vec2_df_pod.Vec2Df_POD\"\x93\x01\n,LscaEgoStaticRoiShapeSerializable_array_port\x12\x63\n\x04\x64\x61ta\x18\x99\x12 \x03(\x0b\x32T.pb.mf_lsca.lsca_ego_static_roi_shape_serializable.LscaEgoStaticRoiShapeSerializable'
  ,
  dependencies=[cml_dot_vec2__df__pod__pb2.DESCRIPTOR,])




_LSCAEGOSTATICROISHAPESERIALIZABLE = _descriptor.Descriptor(
  name='LscaEgoStaticRoiShapeSerializable',
  full_name='pb.mf_lsca.lsca_ego_static_roi_shape_serializable.LscaEgoStaticRoiShapeSerializable',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='actualSize', full_name='pb.mf_lsca.lsca_ego_static_roi_shape_serializable.LscaEgoStaticRoiShapeSerializable.actualSize', index=0,
      number=3009, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='points', full_name='pb.mf_lsca.lsca_ego_static_roi_shape_serializable.LscaEgoStaticRoiShapeSerializable.points', index=1,
      number=3855, type=11, cpp_type=10, label=3,
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
  serialized_start=130,
  serialized_end=235,
)


_LSCAEGOSTATICROISHAPESERIALIZABLE_ARRAY_PORT = _descriptor.Descriptor(
  name='LscaEgoStaticRoiShapeSerializable_array_port',
  full_name='pb.mf_lsca.lsca_ego_static_roi_shape_serializable.LscaEgoStaticRoiShapeSerializable_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_lsca.lsca_ego_static_roi_shape_serializable.LscaEgoStaticRoiShapeSerializable_array_port.data', index=0,
      number=2329, type=11, cpp_type=10, label=3,
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
  serialized_start=238,
  serialized_end=385,
)

_LSCAEGOSTATICROISHAPESERIALIZABLE.fields_by_name['points'].message_type = cml_dot_vec2__df__pod__pb2._VEC2DF_POD
_LSCAEGOSTATICROISHAPESERIALIZABLE_ARRAY_PORT.fields_by_name['data'].message_type = _LSCAEGOSTATICROISHAPESERIALIZABLE
DESCRIPTOR.message_types_by_name['LscaEgoStaticRoiShapeSerializable'] = _LSCAEGOSTATICROISHAPESERIALIZABLE
DESCRIPTOR.message_types_by_name['LscaEgoStaticRoiShapeSerializable_array_port'] = _LSCAEGOSTATICROISHAPESERIALIZABLE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LscaEgoStaticRoiShapeSerializable = _reflection.GeneratedProtocolMessageType('LscaEgoStaticRoiShapeSerializable', (_message.Message,), {
  'DESCRIPTOR' : _LSCAEGOSTATICROISHAPESERIALIZABLE,
  '__module__' : 'mf_lsca.lsca_ego_static_roi_shape_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_ego_static_roi_shape_serializable.LscaEgoStaticRoiShapeSerializable)
  })
_sym_db.RegisterMessage(LscaEgoStaticRoiShapeSerializable)

LscaEgoStaticRoiShapeSerializable_array_port = _reflection.GeneratedProtocolMessageType('LscaEgoStaticRoiShapeSerializable_array_port', (_message.Message,), {
  'DESCRIPTOR' : _LSCAEGOSTATICROISHAPESERIALIZABLE_ARRAY_PORT,
  '__module__' : 'mf_lsca.lsca_ego_static_roi_shape_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_ego_static_roi_shape_serializable.LscaEgoStaticRoiShapeSerializable_array_port)
  })
_sym_db.RegisterMessage(LscaEgoStaticRoiShapeSerializable_array_port)


# @@protoc_insertion_point(module_scope)
