# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/slot_dimension.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/slot_dimension.proto',
  package='pb.si.slot_dimension',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x17si/slot_dimension.proto\x12\x14pb.si.slot_dimension\"C\n\rSlotDimension\x12\x0e\n\x05min_m\x18\xa3\x02 \x01(\x02\x12\x12\n\ttypical_m\x18\xc3\t \x01(\x02\x12\x0e\n\x05max_m\x18\xa3\n \x01(\x02\"N\n\x18SlotDimension_array_port\x12\x32\n\x04\x64\x61ta\x18\xfa\x15 \x03(\x0b\x32#.pb.si.slot_dimension.SlotDimension'
)




_SLOTDIMENSION = _descriptor.Descriptor(
  name='SlotDimension',
  full_name='pb.si.slot_dimension.SlotDimension',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='min_m', full_name='pb.si.slot_dimension.SlotDimension.min_m', index=0,
      number=291, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='typical_m', full_name='pb.si.slot_dimension.SlotDimension.typical_m', index=1,
      number=1219, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='max_m', full_name='pb.si.slot_dimension.SlotDimension.max_m', index=2,
      number=1315, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
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
  serialized_start=49,
  serialized_end=116,
)


_SLOTDIMENSION_ARRAY_PORT = _descriptor.Descriptor(
  name='SlotDimension_array_port',
  full_name='pb.si.slot_dimension.SlotDimension_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.slot_dimension.SlotDimension_array_port.data', index=0,
      number=2810, type=11, cpp_type=10, label=3,
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
  serialized_start=118,
  serialized_end=196,
)

_SLOTDIMENSION_ARRAY_PORT.fields_by_name['data'].message_type = _SLOTDIMENSION
DESCRIPTOR.message_types_by_name['SlotDimension'] = _SLOTDIMENSION
DESCRIPTOR.message_types_by_name['SlotDimension_array_port'] = _SLOTDIMENSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SlotDimension = _reflection.GeneratedProtocolMessageType('SlotDimension', (_message.Message,), {
  'DESCRIPTOR' : _SLOTDIMENSION,
  '__module__' : 'si.slot_dimension_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.slot_dimension.SlotDimension)
  })
_sym_db.RegisterMessage(SlotDimension)

SlotDimension_array_port = _reflection.GeneratedProtocolMessageType('SlotDimension_array_port', (_message.Message,), {
  'DESCRIPTOR' : _SLOTDIMENSION_ARRAY_PORT,
  '__module__' : 'si.slot_dimension_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.slot_dimension.SlotDimension_array_port)
  })
_sym_db.RegisterMessage(SlotDimension_array_port)


# @@protoc_insertion_point(module_scope)
