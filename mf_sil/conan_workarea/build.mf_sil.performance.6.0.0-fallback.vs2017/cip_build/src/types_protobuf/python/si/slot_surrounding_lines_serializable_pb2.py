# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/slot_surrounding_lines_serializable.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from si import slot_surrounding_lines_pb2 as si_dot_slot__surrounding__lines__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/slot_surrounding_lines_serializable.proto',
  package='pb.si.slot_surrounding_lines_serializable',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n,si/slot_surrounding_lines_serializable.proto\x12)pb.si.slot_surrounding_lines_serializable\x1a\x1fsi/slot_surrounding_lines.proto\"{\n SlotSurroundingLinesSerializable\x12\x13\n\nactualSize\x18\xc1\x17 \x01(\r\x12\x42\n\x05\x61rray\x18\xe7\n \x03(\x0b\x32\x32.pb.si.slot_surrounding_lines.SlotSurroundingLines\"\x89\x01\n+SlotSurroundingLinesSerializable_array_port\x12Z\n\x04\x64\x61ta\x18\xcf\x1d \x03(\x0b\x32K.pb.si.slot_surrounding_lines_serializable.SlotSurroundingLinesSerializable'
  ,
  dependencies=[si_dot_slot__surrounding__lines__pb2.DESCRIPTOR,])




_SLOTSURROUNDINGLINESSERIALIZABLE = _descriptor.Descriptor(
  name='SlotSurroundingLinesSerializable',
  full_name='pb.si.slot_surrounding_lines_serializable.SlotSurroundingLinesSerializable',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='actualSize', full_name='pb.si.slot_surrounding_lines_serializable.SlotSurroundingLinesSerializable.actualSize', index=0,
      number=3009, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='array', full_name='pb.si.slot_surrounding_lines_serializable.SlotSurroundingLinesSerializable.array', index=1,
      number=1383, type=11, cpp_type=10, label=3,
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
  serialized_start=124,
  serialized_end=247,
)


_SLOTSURROUNDINGLINESSERIALIZABLE_ARRAY_PORT = _descriptor.Descriptor(
  name='SlotSurroundingLinesSerializable_array_port',
  full_name='pb.si.slot_surrounding_lines_serializable.SlotSurroundingLinesSerializable_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.slot_surrounding_lines_serializable.SlotSurroundingLinesSerializable_array_port.data', index=0,
      number=3791, type=11, cpp_type=10, label=3,
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
  serialized_start=250,
  serialized_end=387,
)

_SLOTSURROUNDINGLINESSERIALIZABLE.fields_by_name['array'].message_type = si_dot_slot__surrounding__lines__pb2._SLOTSURROUNDINGLINES
_SLOTSURROUNDINGLINESSERIALIZABLE_ARRAY_PORT.fields_by_name['data'].message_type = _SLOTSURROUNDINGLINESSERIALIZABLE
DESCRIPTOR.message_types_by_name['SlotSurroundingLinesSerializable'] = _SLOTSURROUNDINGLINESSERIALIZABLE
DESCRIPTOR.message_types_by_name['SlotSurroundingLinesSerializable_array_port'] = _SLOTSURROUNDINGLINESSERIALIZABLE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SlotSurroundingLinesSerializable = _reflection.GeneratedProtocolMessageType('SlotSurroundingLinesSerializable', (_message.Message,), {
  'DESCRIPTOR' : _SLOTSURROUNDINGLINESSERIALIZABLE,
  '__module__' : 'si.slot_surrounding_lines_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.slot_surrounding_lines_serializable.SlotSurroundingLinesSerializable)
  })
_sym_db.RegisterMessage(SlotSurroundingLinesSerializable)

SlotSurroundingLinesSerializable_array_port = _reflection.GeneratedProtocolMessageType('SlotSurroundingLinesSerializable_array_port', (_message.Message,), {
  'DESCRIPTOR' : _SLOTSURROUNDINGLINESSERIALIZABLE_ARRAY_PORT,
  '__module__' : 'si.slot_surrounding_lines_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.slot_surrounding_lines_serializable.SlotSurroundingLinesSerializable_array_port)
  })
_sym_db.RegisterMessage(SlotSurroundingLinesSerializable_array_port)


# @@protoc_insertion_point(module_scope)
