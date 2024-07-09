# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_lsca/lsca_dynamic_object_prediction_shape_serializable.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from cml import vec2_df_pod_pb2 as cml_dot_vec2__df__pod__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_lsca/lsca_dynamic_object_prediction_shape_serializable.proto',
  package='pb.mf_lsca.lsca_dynamic_object_prediction_shape_serializable',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n?mf_lsca/lsca_dynamic_object_prediction_shape_serializable.proto\x12<pb.mf_lsca.lsca_dynamic_object_prediction_shape_serializable\x1a\x15\x63ml/vec2_df_pod.proto\"t\n,LscaDynamicObjectPredictionShapeSerializable\x12\x13\n\nactualSize\x18\xc1\x17 \x01(\r\x12/\n\x06points\x18\x8f\x1e \x03(\x0b\x32\x1e.pb.cml.vec2_df_pod.Vec2Df_POD\"\xb4\x01\n7LscaDynamicObjectPredictionShapeSerializable_array_port\x12y\n\x04\x64\x61ta\x18\x96\x1e \x03(\x0b\x32j.pb.mf_lsca.lsca_dynamic_object_prediction_shape_serializable.LscaDynamicObjectPredictionShapeSerializable'
  ,
  dependencies=[cml_dot_vec2__df__pod__pb2.DESCRIPTOR,])




_LSCADYNAMICOBJECTPREDICTIONSHAPESERIALIZABLE = _descriptor.Descriptor(
  name='LscaDynamicObjectPredictionShapeSerializable',
  full_name='pb.mf_lsca.lsca_dynamic_object_prediction_shape_serializable.LscaDynamicObjectPredictionShapeSerializable',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='actualSize', full_name='pb.mf_lsca.lsca_dynamic_object_prediction_shape_serializable.LscaDynamicObjectPredictionShapeSerializable.actualSize', index=0,
      number=3009, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='points', full_name='pb.mf_lsca.lsca_dynamic_object_prediction_shape_serializable.LscaDynamicObjectPredictionShapeSerializable.points', index=1,
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
  serialized_start=152,
  serialized_end=268,
)


_LSCADYNAMICOBJECTPREDICTIONSHAPESERIALIZABLE_ARRAY_PORT = _descriptor.Descriptor(
  name='LscaDynamicObjectPredictionShapeSerializable_array_port',
  full_name='pb.mf_lsca.lsca_dynamic_object_prediction_shape_serializable.LscaDynamicObjectPredictionShapeSerializable_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_lsca.lsca_dynamic_object_prediction_shape_serializable.LscaDynamicObjectPredictionShapeSerializable_array_port.data', index=0,
      number=3862, type=11, cpp_type=10, label=3,
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
  serialized_start=271,
  serialized_end=451,
)

_LSCADYNAMICOBJECTPREDICTIONSHAPESERIALIZABLE.fields_by_name['points'].message_type = cml_dot_vec2__df__pod__pb2._VEC2DF_POD
_LSCADYNAMICOBJECTPREDICTIONSHAPESERIALIZABLE_ARRAY_PORT.fields_by_name['data'].message_type = _LSCADYNAMICOBJECTPREDICTIONSHAPESERIALIZABLE
DESCRIPTOR.message_types_by_name['LscaDynamicObjectPredictionShapeSerializable'] = _LSCADYNAMICOBJECTPREDICTIONSHAPESERIALIZABLE
DESCRIPTOR.message_types_by_name['LscaDynamicObjectPredictionShapeSerializable_array_port'] = _LSCADYNAMICOBJECTPREDICTIONSHAPESERIALIZABLE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LscaDynamicObjectPredictionShapeSerializable = _reflection.GeneratedProtocolMessageType('LscaDynamicObjectPredictionShapeSerializable', (_message.Message,), {
  'DESCRIPTOR' : _LSCADYNAMICOBJECTPREDICTIONSHAPESERIALIZABLE,
  '__module__' : 'mf_lsca.lsca_dynamic_object_prediction_shape_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_dynamic_object_prediction_shape_serializable.LscaDynamicObjectPredictionShapeSerializable)
  })
_sym_db.RegisterMessage(LscaDynamicObjectPredictionShapeSerializable)

LscaDynamicObjectPredictionShapeSerializable_array_port = _reflection.GeneratedProtocolMessageType('LscaDynamicObjectPredictionShapeSerializable_array_port', (_message.Message,), {
  'DESCRIPTOR' : _LSCADYNAMICOBJECTPREDICTIONSHAPESERIALIZABLE_ARRAY_PORT,
  '__module__' : 'mf_lsca.lsca_dynamic_object_prediction_shape_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_dynamic_object_prediction_shape_serializable.LscaDynamicObjectPredictionShapeSerializable_array_port)
  })
_sym_db.RegisterMessage(LscaDynamicObjectPredictionShapeSerializable_array_port)


# @@protoc_insertion_point(module_scope)
