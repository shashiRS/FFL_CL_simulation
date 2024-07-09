# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_manager/mf_manager_consts.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_manager/mf_manager_consts.proto',
  package='pb.mf_manager.mf_manager_consts',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\"mf_manager/mf_manager_consts.proto\x12\x1fpb.mf_manager.mf_manager_consts\"e\n\x11MF_MANAGER_Consts\x12&\n\x1d\x41P_M_MAX_NUM_TRAJ_CTRL_POINTS\x18\x94\x06 \x01(\r\x12(\n\x1f\x41P_M_MAX_NUM_DRIVING_RESISTANCE\x18\xaf\x0f \x01(\r\"a\n\x1cMF_MANAGER_Consts_array_port\x12\x41\n\x04\x64\x61ta\x18\xfc\r \x03(\x0b\x32\x32.pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts'
)




_MF_MANAGER_CONSTS = _descriptor.Descriptor(
  name='MF_MANAGER_Consts',
  full_name='pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='AP_M_MAX_NUM_TRAJ_CTRL_POINTS', full_name='pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts.AP_M_MAX_NUM_TRAJ_CTRL_POINTS', index=0,
      number=788, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_M_MAX_NUM_DRIVING_RESISTANCE', full_name='pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts.AP_M_MAX_NUM_DRIVING_RESISTANCE', index=1,
      number=1967, type=13, cpp_type=3, label=1,
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
  serialized_start=71,
  serialized_end=172,
)


_MF_MANAGER_CONSTS_ARRAY_PORT = _descriptor.Descriptor(
  name='MF_MANAGER_Consts_array_port',
  full_name='pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts_array_port.data', index=0,
      number=1788, type=11, cpp_type=10, label=3,
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
  serialized_start=174,
  serialized_end=271,
)

_MF_MANAGER_CONSTS_ARRAY_PORT.fields_by_name['data'].message_type = _MF_MANAGER_CONSTS
DESCRIPTOR.message_types_by_name['MF_MANAGER_Consts'] = _MF_MANAGER_CONSTS
DESCRIPTOR.message_types_by_name['MF_MANAGER_Consts_array_port'] = _MF_MANAGER_CONSTS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MF_MANAGER_Consts = _reflection.GeneratedProtocolMessageType('MF_MANAGER_Consts', (_message.Message,), {
  'DESCRIPTOR' : _MF_MANAGER_CONSTS,
  '__module__' : 'mf_manager.mf_manager_consts_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts)
  })
_sym_db.RegisterMessage(MF_MANAGER_Consts)

MF_MANAGER_Consts_array_port = _reflection.GeneratedProtocolMessageType('MF_MANAGER_Consts_array_port', (_message.Message,), {
  'DESCRIPTOR' : _MF_MANAGER_CONSTS_ARRAY_PORT,
  '__module__' : 'mf_manager.mf_manager_consts_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts_array_port)
  })
_sym_db.RegisterMessage(MF_MANAGER_Consts_array_port)


# @@protoc_insertion_point(module_scope)