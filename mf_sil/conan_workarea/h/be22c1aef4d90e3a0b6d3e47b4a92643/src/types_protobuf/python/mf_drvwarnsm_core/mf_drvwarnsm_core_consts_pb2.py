# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_drvwarnsm_core/mf_drvwarnsm_core_consts.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_drvwarnsm_core/mf_drvwarnsm_core_consts.proto',
  package='pb.mf_drvwarnsm_core.mf_drvwarnsm_core_consts',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n0mf_drvwarnsm_core/mf_drvwarnsm_core_consts.proto\x12-pb.mf_drvwarnsm_core.mf_drvwarnsm_core_consts\"E\n\x18MF_DRVWARNSM_CORE_Consts\x12)\n NUM_MTS_DEBUG_FREESPACE_CORE_DWF\x18\xc7\x16 \x01(\r\"}\n#MF_DRVWARNSM_CORE_Consts_array_port\x12V\n\x04\x64\x61ta\x18\x82\x1e \x03(\x0b\x32G.pb.mf_drvwarnsm_core.mf_drvwarnsm_core_consts.MF_DRVWARNSM_CORE_Consts'
)




_MF_DRVWARNSM_CORE_CONSTS = _descriptor.Descriptor(
  name='MF_DRVWARNSM_CORE_Consts',
  full_name='pb.mf_drvwarnsm_core.mf_drvwarnsm_core_consts.MF_DRVWARNSM_CORE_Consts',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='NUM_MTS_DEBUG_FREESPACE_CORE_DWF', full_name='pb.mf_drvwarnsm_core.mf_drvwarnsm_core_consts.MF_DRVWARNSM_CORE_Consts.NUM_MTS_DEBUG_FREESPACE_CORE_DWF', index=0,
      number=2887, type=13, cpp_type=3, label=1,
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
  serialized_start=99,
  serialized_end=168,
)


_MF_DRVWARNSM_CORE_CONSTS_ARRAY_PORT = _descriptor.Descriptor(
  name='MF_DRVWARNSM_CORE_Consts_array_port',
  full_name='pb.mf_drvwarnsm_core.mf_drvwarnsm_core_consts.MF_DRVWARNSM_CORE_Consts_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_drvwarnsm_core.mf_drvwarnsm_core_consts.MF_DRVWARNSM_CORE_Consts_array_port.data', index=0,
      number=3842, type=11, cpp_type=10, label=3,
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
  serialized_start=170,
  serialized_end=295,
)

_MF_DRVWARNSM_CORE_CONSTS_ARRAY_PORT.fields_by_name['data'].message_type = _MF_DRVWARNSM_CORE_CONSTS
DESCRIPTOR.message_types_by_name['MF_DRVWARNSM_CORE_Consts'] = _MF_DRVWARNSM_CORE_CONSTS
DESCRIPTOR.message_types_by_name['MF_DRVWARNSM_CORE_Consts_array_port'] = _MF_DRVWARNSM_CORE_CONSTS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MF_DRVWARNSM_CORE_Consts = _reflection.GeneratedProtocolMessageType('MF_DRVWARNSM_CORE_Consts', (_message.Message,), {
  'DESCRIPTOR' : _MF_DRVWARNSM_CORE_CONSTS,
  '__module__' : 'mf_drvwarnsm_core.mf_drvwarnsm_core_consts_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_drvwarnsm_core.mf_drvwarnsm_core_consts.MF_DRVWARNSM_CORE_Consts)
  })
_sym_db.RegisterMessage(MF_DRVWARNSM_CORE_Consts)

MF_DRVWARNSM_CORE_Consts_array_port = _reflection.GeneratedProtocolMessageType('MF_DRVWARNSM_CORE_Consts_array_port', (_message.Message,), {
  'DESCRIPTOR' : _MF_DRVWARNSM_CORE_CONSTS_ARRAY_PORT,
  '__module__' : 'mf_drvwarnsm_core.mf_drvwarnsm_core_consts_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_drvwarnsm_core.mf_drvwarnsm_core_consts.MF_DRVWARNSM_CORE_Consts_array_port)
  })
_sym_db.RegisterMessage(MF_DRVWARNSM_CORE_Consts_array_port)


# @@protoc_insertion_point(module_scope)
