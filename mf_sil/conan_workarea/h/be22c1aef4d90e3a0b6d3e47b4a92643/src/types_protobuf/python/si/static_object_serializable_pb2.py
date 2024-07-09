# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/static_object_serializable.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from si import object_trend_pb2 as si_dot_object__trend__pb2
from si import static_obj_shape_serializable_pb2 as si_dot_static__obj__shape__serializable__pb2
from si import static_obj_heigth_type_pb2 as si_dot_static__obj__heigth__type__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/static_object_serializable.proto',
  package='pb.si.static_object_serializable',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n#si/static_object_serializable.proto\x12 pb.si.static_object_serializable\x1a\x15si/object_trend.proto\x1a&si/static_obj_shape_serializable.proto\x1a\x1fsi/static_obj_heigth_type.proto\"\xd7\x03\n\x18StaticObjectSerializable\x12\x14\n\x0brefObjID_nu\x18\xd8\x19 \x01(\r\x12\x1b\n\x12\x65xistenceProb_perc\x18\xf8\x12 \x01(\r\x12\x1a\n\x11objAgeInCycles_nu\x18\xcb\x06 \x01(\r\x12%\n\x1cobjMeasLastUpdateInCycles_nu\x18\xe7\x07 \x01(\r\x12&\n\x1dobjTrendLastUpdateInCycles_nu\x18\xf2\x0b \x01(\r\x12\x35\n\x0bobjTrend_nu\x18\xbb\x1d \x01(\x0e\x32\x1f.pb.si.object_trend.ObjectTrend\x12\x19\n\x10readFromNVRAM_nu\x18\xd7\t \x01(\x08\x12T\n\nobjShape_m\x18\x95\x08 \x01(\x0b\x32?.pb.si.static_obj_shape_serializable.StaticObjShapeSerializable\x12M\n\x11objHeightClass_nu\x18\xce\x1b \x01(\x0e\x32\x31.pb.si.static_obj_heigth_type.StaticObjHeigthType\x12&\n\x1dobjHeightClassConfidence_perc\x18\xb3\x1b \x01(\r\"p\n#StaticObjectSerializable_array_port\x12I\n\x04\x64\x61ta\x18\x9b\n \x03(\x0b\x32:.pb.si.static_object_serializable.StaticObjectSerializable'
  ,
  dependencies=[si_dot_object__trend__pb2.DESCRIPTOR,si_dot_static__obj__shape__serializable__pb2.DESCRIPTOR,si_dot_static__obj__heigth__type__pb2.DESCRIPTOR,])




_STATICOBJECTSERIALIZABLE = _descriptor.Descriptor(
  name='StaticObjectSerializable',
  full_name='pb.si.static_object_serializable.StaticObjectSerializable',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='refObjID_nu', full_name='pb.si.static_object_serializable.StaticObjectSerializable.refObjID_nu', index=0,
      number=3288, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='existenceProb_perc', full_name='pb.si.static_object_serializable.StaticObjectSerializable.existenceProb_perc', index=1,
      number=2424, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='objAgeInCycles_nu', full_name='pb.si.static_object_serializable.StaticObjectSerializable.objAgeInCycles_nu', index=2,
      number=843, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='objMeasLastUpdateInCycles_nu', full_name='pb.si.static_object_serializable.StaticObjectSerializable.objMeasLastUpdateInCycles_nu', index=3,
      number=999, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='objTrendLastUpdateInCycles_nu', full_name='pb.si.static_object_serializable.StaticObjectSerializable.objTrendLastUpdateInCycles_nu', index=4,
      number=1522, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='objTrend_nu', full_name='pb.si.static_object_serializable.StaticObjectSerializable.objTrend_nu', index=5,
      number=3771, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='readFromNVRAM_nu', full_name='pb.si.static_object_serializable.StaticObjectSerializable.readFromNVRAM_nu', index=6,
      number=1239, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='objShape_m', full_name='pb.si.static_object_serializable.StaticObjectSerializable.objShape_m', index=7,
      number=1045, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='objHeightClass_nu', full_name='pb.si.static_object_serializable.StaticObjectSerializable.objHeightClass_nu', index=8,
      number=3534, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='objHeightClassConfidence_perc', full_name='pb.si.static_object_serializable.StaticObjectSerializable.objHeightClassConfidence_perc', index=9,
      number=3507, type=13, cpp_type=3, label=1,
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
  serialized_start=170,
  serialized_end=641,
)


_STATICOBJECTSERIALIZABLE_ARRAY_PORT = _descriptor.Descriptor(
  name='StaticObjectSerializable_array_port',
  full_name='pb.si.static_object_serializable.StaticObjectSerializable_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.static_object_serializable.StaticObjectSerializable_array_port.data', index=0,
      number=1307, type=11, cpp_type=10, label=3,
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
  serialized_start=643,
  serialized_end=755,
)

_STATICOBJECTSERIALIZABLE.fields_by_name['objTrend_nu'].enum_type = si_dot_object__trend__pb2._OBJECTTREND
_STATICOBJECTSERIALIZABLE.fields_by_name['objShape_m'].message_type = si_dot_static__obj__shape__serializable__pb2._STATICOBJSHAPESERIALIZABLE
_STATICOBJECTSERIALIZABLE.fields_by_name['objHeightClass_nu'].enum_type = si_dot_static__obj__heigth__type__pb2._STATICOBJHEIGTHTYPE
_STATICOBJECTSERIALIZABLE_ARRAY_PORT.fields_by_name['data'].message_type = _STATICOBJECTSERIALIZABLE
DESCRIPTOR.message_types_by_name['StaticObjectSerializable'] = _STATICOBJECTSERIALIZABLE
DESCRIPTOR.message_types_by_name['StaticObjectSerializable_array_port'] = _STATICOBJECTSERIALIZABLE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

StaticObjectSerializable = _reflection.GeneratedProtocolMessageType('StaticObjectSerializable', (_message.Message,), {
  'DESCRIPTOR' : _STATICOBJECTSERIALIZABLE,
  '__module__' : 'si.static_object_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.static_object_serializable.StaticObjectSerializable)
  })
_sym_db.RegisterMessage(StaticObjectSerializable)

StaticObjectSerializable_array_port = _reflection.GeneratedProtocolMessageType('StaticObjectSerializable_array_port', (_message.Message,), {
  'DESCRIPTOR' : _STATICOBJECTSERIALIZABLE_ARRAY_PORT,
  '__module__' : 'si.static_object_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.static_object_serializable.StaticObjectSerializable_array_port)
  })
_sym_db.RegisterMessage(StaticObjectSerializable_array_port)


# @@protoc_insertion_point(module_scope)