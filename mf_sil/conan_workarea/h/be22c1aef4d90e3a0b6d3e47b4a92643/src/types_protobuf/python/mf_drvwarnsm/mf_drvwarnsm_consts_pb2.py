# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_drvwarnsm/mf_drvwarnsm_consts.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_drvwarnsm/mf_drvwarnsm_consts.proto',
  package='pb.mf_drvwarnsm.mf_drvwarnsm_consts',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n&mf_drvwarnsm/mf_drvwarnsm_consts.proto\x12#pb.mf_drvwarnsm.mf_drvwarnsm_consts\"o\n\x13MF_DRVWARNSM_Consts\x12\x13\n\nNUM_WHEELS\x18\xfa\x1a \x01(\r\x12\x19\n\x10NUM_SECTOR_SIDES\x18\x8f\x1a \x01(\r\x12(\n\x1fNUM_MTS_DEBUG_FREESPACE_APP_DWF\x18\xe6\x08 \x01(\r\"i\n\x1eMF_DRVWARNSM_Consts_array_port\x12G\n\x04\x64\x61ta\x18\x8c\x08 \x03(\x0b\x32\x38.pb.mf_drvwarnsm.mf_drvwarnsm_consts.MF_DRVWARNSM_Consts'
)




_MF_DRVWARNSM_CONSTS = _descriptor.Descriptor(
  name='MF_DRVWARNSM_Consts',
  full_name='pb.mf_drvwarnsm.mf_drvwarnsm_consts.MF_DRVWARNSM_Consts',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='NUM_WHEELS', full_name='pb.mf_drvwarnsm.mf_drvwarnsm_consts.MF_DRVWARNSM_Consts.NUM_WHEELS', index=0,
      number=3450, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='NUM_SECTOR_SIDES', full_name='pb.mf_drvwarnsm.mf_drvwarnsm_consts.MF_DRVWARNSM_Consts.NUM_SECTOR_SIDES', index=1,
      number=3343, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='NUM_MTS_DEBUG_FREESPACE_APP_DWF', full_name='pb.mf_drvwarnsm.mf_drvwarnsm_consts.MF_DRVWARNSM_Consts.NUM_MTS_DEBUG_FREESPACE_APP_DWF', index=2,
      number=1126, type=13, cpp_type=3, label=1,
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
  serialized_start=79,
  serialized_end=190,
)


_MF_DRVWARNSM_CONSTS_ARRAY_PORT = _descriptor.Descriptor(
  name='MF_DRVWARNSM_Consts_array_port',
  full_name='pb.mf_drvwarnsm.mf_drvwarnsm_consts.MF_DRVWARNSM_Consts_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_drvwarnsm.mf_drvwarnsm_consts.MF_DRVWARNSM_Consts_array_port.data', index=0,
      number=1036, type=11, cpp_type=10, label=3,
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
  serialized_start=192,
  serialized_end=297,
)

_MF_DRVWARNSM_CONSTS_ARRAY_PORT.fields_by_name['data'].message_type = _MF_DRVWARNSM_CONSTS
DESCRIPTOR.message_types_by_name['MF_DRVWARNSM_Consts'] = _MF_DRVWARNSM_CONSTS
DESCRIPTOR.message_types_by_name['MF_DRVWARNSM_Consts_array_port'] = _MF_DRVWARNSM_CONSTS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MF_DRVWARNSM_Consts = _reflection.GeneratedProtocolMessageType('MF_DRVWARNSM_Consts', (_message.Message,), {
  'DESCRIPTOR' : _MF_DRVWARNSM_CONSTS,
  '__module__' : 'mf_drvwarnsm.mf_drvwarnsm_consts_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_drvwarnsm.mf_drvwarnsm_consts.MF_DRVWARNSM_Consts)
  })
_sym_db.RegisterMessage(MF_DRVWARNSM_Consts)

MF_DRVWARNSM_Consts_array_port = _reflection.GeneratedProtocolMessageType('MF_DRVWARNSM_Consts_array_port', (_message.Message,), {
  'DESCRIPTOR' : _MF_DRVWARNSM_CONSTS_ARRAY_PORT,
  '__module__' : 'mf_drvwarnsm.mf_drvwarnsm_consts_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_drvwarnsm.mf_drvwarnsm_consts.MF_DRVWARNSM_Consts_array_port)
  })
_sym_db.RegisterMessage(MF_DRVWARNSM_Consts_array_port)


# @@protoc_insertion_point(module_scope)
