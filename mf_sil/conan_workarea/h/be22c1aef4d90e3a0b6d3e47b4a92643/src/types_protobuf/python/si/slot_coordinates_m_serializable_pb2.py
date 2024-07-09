# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/slot_coordinates_m_serializable.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from cml import vec2_df_pod_pb2 as cml_dot_vec2__df__pod__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/slot_coordinates_m_serializable.proto',
  package='pb.si.slot_coordinates_m_serializable',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n(si/slot_coordinates_m_serializable.proto\x12%pb.si.slot_coordinates_m_serializable\x1a\x15\x63ml/vec2_df_pod.proto\"d\n\x1dSlotCoordinates_mSerializable\x12\x13\n\nactualSize\x18\xc1\x17 \x01(\r\x12.\n\x05\x61rray\x18\xb9\x18 \x03(\x0b\x32\x1e.pb.cml.vec2_df_pod.Vec2Df_POD\"\x7f\n(SlotCoordinates_mSerializable_array_port\x12S\n\x04\x64\x61ta\x18\x92\r \x03(\x0b\x32\x44.pb.si.slot_coordinates_m_serializable.SlotCoordinates_mSerializable'
  ,
  dependencies=[cml_dot_vec2__df__pod__pb2.DESCRIPTOR,])




_SLOTCOORDINATES_MSERIALIZABLE = _descriptor.Descriptor(
  name='SlotCoordinates_mSerializable',
  full_name='pb.si.slot_coordinates_m_serializable.SlotCoordinates_mSerializable',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='actualSize', full_name='pb.si.slot_coordinates_m_serializable.SlotCoordinates_mSerializable.actualSize', index=0,
      number=3009, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='array', full_name='pb.si.slot_coordinates_m_serializable.SlotCoordinates_mSerializable.array', index=1,
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
  serialized_start=106,
  serialized_end=206,
)


_SLOTCOORDINATES_MSERIALIZABLE_ARRAY_PORT = _descriptor.Descriptor(
  name='SlotCoordinates_mSerializable_array_port',
  full_name='pb.si.slot_coordinates_m_serializable.SlotCoordinates_mSerializable_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.slot_coordinates_m_serializable.SlotCoordinates_mSerializable_array_port.data', index=0,
      number=1682, type=11, cpp_type=10, label=3,
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
  serialized_start=208,
  serialized_end=335,
)

_SLOTCOORDINATES_MSERIALIZABLE.fields_by_name['array'].message_type = cml_dot_vec2__df__pod__pb2._VEC2DF_POD
_SLOTCOORDINATES_MSERIALIZABLE_ARRAY_PORT.fields_by_name['data'].message_type = _SLOTCOORDINATES_MSERIALIZABLE
DESCRIPTOR.message_types_by_name['SlotCoordinates_mSerializable'] = _SLOTCOORDINATES_MSERIALIZABLE
DESCRIPTOR.message_types_by_name['SlotCoordinates_mSerializable_array_port'] = _SLOTCOORDINATES_MSERIALIZABLE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SlotCoordinates_mSerializable = _reflection.GeneratedProtocolMessageType('SlotCoordinates_mSerializable', (_message.Message,), {
  'DESCRIPTOR' : _SLOTCOORDINATES_MSERIALIZABLE,
  '__module__' : 'si.slot_coordinates_m_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.slot_coordinates_m_serializable.SlotCoordinates_mSerializable)
  })
_sym_db.RegisterMessage(SlotCoordinates_mSerializable)

SlotCoordinates_mSerializable_array_port = _reflection.GeneratedProtocolMessageType('SlotCoordinates_mSerializable_array_port', (_message.Message,), {
  'DESCRIPTOR' : _SLOTCOORDINATES_MSERIALIZABLE_ARRAY_PORT,
  '__module__' : 'si.slot_coordinates_m_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.slot_coordinates_m_serializable.SlotCoordinates_mSerializable_array_port)
  })
_sym_db.RegisterMessage(SlotCoordinates_mSerializable_array_port)


# @@protoc_insertion_point(module_scope)