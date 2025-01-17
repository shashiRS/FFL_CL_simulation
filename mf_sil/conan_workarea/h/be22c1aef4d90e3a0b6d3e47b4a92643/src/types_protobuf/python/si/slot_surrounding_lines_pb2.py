# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/slot_surrounding_lines.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from si import line_segment_serializable_pb2 as si_dot_line__segment__serializable__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/slot_surrounding_lines.proto',
  package='pb.si.slot_surrounding_lines',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1fsi/slot_surrounding_lines.proto\x12\x1cpb.si.slot_surrounding_lines\x1a\"si/line_segment_serializable.proto\"\xcb\x02\n\x14SlotSurroundingLines\x12K\n\x08leftLine\x18\xa4\x0f \x01(\x0b\x32\x38.pb.si.line_segment_serializable.LineSegmentSerializable\x12L\n\trightLine\x18\xcf\x1c \x01(\x0b\x32\x38.pb.si.line_segment_serializable.LineSegmentSerializable\x12K\n\x08roadLine\x18\xd4\x16 \x01(\x0b\x32\x38.pb.si.line_segment_serializable.LineSegmentSerializable\x12K\n\x08\x63urbLine\x18\xe7\x19 \x01(\x0b\x32\x38.pb.si.line_segment_serializable.LineSegmentSerializable\"d\n\x1fSlotSurroundingLines_array_port\x12\x41\n\x04\x64\x61ta\x18\xa5\x15 \x03(\x0b\x32\x32.pb.si.slot_surrounding_lines.SlotSurroundingLines'
  ,
  dependencies=[si_dot_line__segment__serializable__pb2.DESCRIPTOR,])




_SLOTSURROUNDINGLINES = _descriptor.Descriptor(
  name='SlotSurroundingLines',
  full_name='pb.si.slot_surrounding_lines.SlotSurroundingLines',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='leftLine', full_name='pb.si.slot_surrounding_lines.SlotSurroundingLines.leftLine', index=0,
      number=1956, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rightLine', full_name='pb.si.slot_surrounding_lines.SlotSurroundingLines.rightLine', index=1,
      number=3663, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='roadLine', full_name='pb.si.slot_surrounding_lines.SlotSurroundingLines.roadLine', index=2,
      number=2900, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='curbLine', full_name='pb.si.slot_surrounding_lines.SlotSurroundingLines.curbLine', index=3,
      number=3303, type=11, cpp_type=10, label=1,
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
  serialized_start=102,
  serialized_end=433,
)


_SLOTSURROUNDINGLINES_ARRAY_PORT = _descriptor.Descriptor(
  name='SlotSurroundingLines_array_port',
  full_name='pb.si.slot_surrounding_lines.SlotSurroundingLines_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.slot_surrounding_lines.SlotSurroundingLines_array_port.data', index=0,
      number=2725, type=11, cpp_type=10, label=3,
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
  serialized_start=435,
  serialized_end=535,
)

_SLOTSURROUNDINGLINES.fields_by_name['leftLine'].message_type = si_dot_line__segment__serializable__pb2._LINESEGMENTSERIALIZABLE
_SLOTSURROUNDINGLINES.fields_by_name['rightLine'].message_type = si_dot_line__segment__serializable__pb2._LINESEGMENTSERIALIZABLE
_SLOTSURROUNDINGLINES.fields_by_name['roadLine'].message_type = si_dot_line__segment__serializable__pb2._LINESEGMENTSERIALIZABLE
_SLOTSURROUNDINGLINES.fields_by_name['curbLine'].message_type = si_dot_line__segment__serializable__pb2._LINESEGMENTSERIALIZABLE
_SLOTSURROUNDINGLINES_ARRAY_PORT.fields_by_name['data'].message_type = _SLOTSURROUNDINGLINES
DESCRIPTOR.message_types_by_name['SlotSurroundingLines'] = _SLOTSURROUNDINGLINES
DESCRIPTOR.message_types_by_name['SlotSurroundingLines_array_port'] = _SLOTSURROUNDINGLINES_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SlotSurroundingLines = _reflection.GeneratedProtocolMessageType('SlotSurroundingLines', (_message.Message,), {
  'DESCRIPTOR' : _SLOTSURROUNDINGLINES,
  '__module__' : 'si.slot_surrounding_lines_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.slot_surrounding_lines.SlotSurroundingLines)
  })
_sym_db.RegisterMessage(SlotSurroundingLines)

SlotSurroundingLines_array_port = _reflection.GeneratedProtocolMessageType('SlotSurroundingLines_array_port', (_message.Message,), {
  'DESCRIPTOR' : _SLOTSURROUNDINGLINES_ARRAY_PORT,
  '__module__' : 'si.slot_surrounding_lines_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.slot_surrounding_lines.SlotSurroundingLines_array_port)
  })
_sym_db.RegisterMessage(SlotSurroundingLines_array_port)


# @@protoc_insertion_point(module_scope)
