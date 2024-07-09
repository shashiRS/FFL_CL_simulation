# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_trjctl/moco_t_long_lims.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_trjctl/moco_t_long_lims.proto',
  package='pb.ap_trjctl.moco_t_long_lims',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n ap_trjctl/moco_t_long_lims.proto\x12\x1dpb.ap_trjctl.moco_t_long_lims\"k\n\x0fMOCO_t_LongLims\x12\x15\n\x0clongAccelMax\x18\xa2\x1e \x01(\x02\x12\x15\n\x0clongAccelMin\x18\xfc\x08 \x01(\x02\x12\x14\n\x0blongJerkMax\x18\xa9\x0f \x01(\x02\x12\x14\n\x0blongJerkMin\x18\xf7\x19 \x01(\x02\"[\n\x1aMOCO_t_LongLims_array_port\x12=\n\x04\x64\x61ta\x18\x98\x1b \x03(\x0b\x32..pb.ap_trjctl.moco_t_long_lims.MOCO_t_LongLims'
)




_MOCO_T_LONGLIMS = _descriptor.Descriptor(
  name='MOCO_t_LongLims',
  full_name='pb.ap_trjctl.moco_t_long_lims.MOCO_t_LongLims',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='longAccelMax', full_name='pb.ap_trjctl.moco_t_long_lims.MOCO_t_LongLims.longAccelMax', index=0,
      number=3874, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='longAccelMin', full_name='pb.ap_trjctl.moco_t_long_lims.MOCO_t_LongLims.longAccelMin', index=1,
      number=1148, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='longJerkMax', full_name='pb.ap_trjctl.moco_t_long_lims.MOCO_t_LongLims.longJerkMax', index=2,
      number=1961, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='longJerkMin', full_name='pb.ap_trjctl.moco_t_long_lims.MOCO_t_LongLims.longJerkMin', index=3,
      number=3319, type=2, cpp_type=6, label=1,
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
  serialized_start=67,
  serialized_end=174,
)


_MOCO_T_LONGLIMS_ARRAY_PORT = _descriptor.Descriptor(
  name='MOCO_t_LongLims_array_port',
  full_name='pb.ap_trjctl.moco_t_long_lims.MOCO_t_LongLims_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_trjctl.moco_t_long_lims.MOCO_t_LongLims_array_port.data', index=0,
      number=3480, type=11, cpp_type=10, label=3,
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
  serialized_start=176,
  serialized_end=267,
)

_MOCO_T_LONGLIMS_ARRAY_PORT.fields_by_name['data'].message_type = _MOCO_T_LONGLIMS
DESCRIPTOR.message_types_by_name['MOCO_t_LongLims'] = _MOCO_T_LONGLIMS
DESCRIPTOR.message_types_by_name['MOCO_t_LongLims_array_port'] = _MOCO_T_LONGLIMS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MOCO_t_LongLims = _reflection.GeneratedProtocolMessageType('MOCO_t_LongLims', (_message.Message,), {
  'DESCRIPTOR' : _MOCO_T_LONGLIMS,
  '__module__' : 'ap_trjctl.moco_t_long_lims_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_trjctl.moco_t_long_lims.MOCO_t_LongLims)
  })
_sym_db.RegisterMessage(MOCO_t_LongLims)

MOCO_t_LongLims_array_port = _reflection.GeneratedProtocolMessageType('MOCO_t_LongLims_array_port', (_message.Message,), {
  'DESCRIPTOR' : _MOCO_T_LONGLIMS_ARRAY_PORT,
  '__module__' : 'ap_trjctl.moco_t_long_lims_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_trjctl.moco_t_long_lims.MOCO_t_LongLims_array_port)
  })
_sym_db.RegisterMessage(MOCO_t_LongLims_array_port)


# @@protoc_insertion_point(module_scope)
