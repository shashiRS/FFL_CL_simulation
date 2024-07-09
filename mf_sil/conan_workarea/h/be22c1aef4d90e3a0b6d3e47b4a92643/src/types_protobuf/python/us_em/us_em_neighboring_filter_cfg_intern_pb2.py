# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_em/us_em_neighboring_filter_cfg_intern.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_em/us_em_neighboring_filter_cfg_intern.proto',
  package='pb.us_em.us_em_neighboring_filter_cfg_intern',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n/us_em/us_em_neighboring_filter_cfg_intern.proto\x12,pb.us_em.us_em_neighboring_filter_cfg_intern\"\xc7\x02\n\x1eUsEmNeighboringFilterCfgIntern\x12\x16\n\rkeep_distance\x18\xa0\x14 \x01(\x02\x12\"\n\x19neighbor_region_threshold\x18\x84\x1a \x01(\x02\x12\x13\n\x0bkeep_cycles\x18Y \x01(\r\x12\x1b\n\x12keep_buffer_length\x18\xa3\x19 \x01(\r\x12!\n\x18neighbor_count_threshold\x18\xa0\x02 \x01(\r\x12&\n\x1dneighbor_count_threshold_only\x18\xc9\x08 \x01(\r\x12!\n\x18neighbor_paths_threshold\x18\xe3\x16 \x01(\r\x12\x1e\n\x15use_absolute_distance\x18\x9a\x1e \x01(\x08\x12)\n weight_neighbors_by_dir_variance\x18\xf0\x01 \x01(\x08\"\x88\x01\n)UsEmNeighboringFilterCfgIntern_array_port\x12[\n\x04\x64\x61ta\x18\xb5\x1c \x03(\x0b\x32L.pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern'
)




_USEMNEIGHBORINGFILTERCFGINTERN = _descriptor.Descriptor(
  name='UsEmNeighboringFilterCfgIntern',
  full_name='pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='keep_distance', full_name='pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern.keep_distance', index=0,
      number=2592, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='neighbor_region_threshold', full_name='pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern.neighbor_region_threshold', index=1,
      number=3332, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='keep_cycles', full_name='pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern.keep_cycles', index=2,
      number=89, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='keep_buffer_length', full_name='pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern.keep_buffer_length', index=3,
      number=3235, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='neighbor_count_threshold', full_name='pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern.neighbor_count_threshold', index=4,
      number=288, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='neighbor_count_threshold_only', full_name='pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern.neighbor_count_threshold_only', index=5,
      number=1097, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='neighbor_paths_threshold', full_name='pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern.neighbor_paths_threshold', index=6,
      number=2915, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='use_absolute_distance', full_name='pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern.use_absolute_distance', index=7,
      number=3866, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='weight_neighbors_by_dir_variance', full_name='pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern.weight_neighbors_by_dir_variance', index=8,
      number=240, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
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
  serialized_start=98,
  serialized_end=425,
)


_USEMNEIGHBORINGFILTERCFGINTERN_ARRAY_PORT = _descriptor.Descriptor(
  name='UsEmNeighboringFilterCfgIntern_array_port',
  full_name='pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern_array_port.data', index=0,
      number=3637, type=11, cpp_type=10, label=3,
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
  serialized_start=428,
  serialized_end=564,
)

_USEMNEIGHBORINGFILTERCFGINTERN_ARRAY_PORT.fields_by_name['data'].message_type = _USEMNEIGHBORINGFILTERCFGINTERN
DESCRIPTOR.message_types_by_name['UsEmNeighboringFilterCfgIntern'] = _USEMNEIGHBORINGFILTERCFGINTERN
DESCRIPTOR.message_types_by_name['UsEmNeighboringFilterCfgIntern_array_port'] = _USEMNEIGHBORINGFILTERCFGINTERN_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsEmNeighboringFilterCfgIntern = _reflection.GeneratedProtocolMessageType('UsEmNeighboringFilterCfgIntern', (_message.Message,), {
  'DESCRIPTOR' : _USEMNEIGHBORINGFILTERCFGINTERN,
  '__module__' : 'us_em.us_em_neighboring_filter_cfg_intern_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern)
  })
_sym_db.RegisterMessage(UsEmNeighboringFilterCfgIntern)

UsEmNeighboringFilterCfgIntern_array_port = _reflection.GeneratedProtocolMessageType('UsEmNeighboringFilterCfgIntern_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USEMNEIGHBORINGFILTERCFGINTERN_ARRAY_PORT,
  '__module__' : 'us_em.us_em_neighboring_filter_cfg_intern_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_em.us_em_neighboring_filter_cfg_intern.UsEmNeighboringFilterCfgIntern_array_port)
  })
_sym_db.RegisterMessage(UsEmNeighboringFilterCfgIntern_array_port)


# @@protoc_insertion_point(module_scope)