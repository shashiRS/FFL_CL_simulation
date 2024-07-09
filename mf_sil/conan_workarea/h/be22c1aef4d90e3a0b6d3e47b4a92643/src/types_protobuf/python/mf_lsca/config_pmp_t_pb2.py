# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_lsca/config_pmp_t.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_lsca/config_pmp_t.proto',
  package='pb.mf_lsca.config_pmp_t',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1amf_lsca/config_pmp_t.proto\x12\x17pb.mf_lsca.config_pmp_t\"\x96\x01\n\x0b\x63onfigPmp_t\x12\x16\n\rjerkModel_mps\x18\xcb\x0f \x01(\x02\x12\x17\n\x0esafetyMargin_m\x18\xf2\x14 \x01(\x02\x12\x17\n\x0esafetyMargin_s\x18\x8d\x13 \x01(\x02\x12\x1e\n\x15maxSpeedBackwards_mps\x18\xf7\x14 \x01(\x02\x12\x1d\n\x14maxSpeedForwards_mps\x18\xf3\x1d \x01(\x02\"M\n\x16\x63onfigPmp_t_array_port\x12\x33\n\x04\x64\x61ta\x18\xcf\x0f \x03(\x0b\x32$.pb.mf_lsca.config_pmp_t.configPmp_t'
)




_CONFIGPMP_T = _descriptor.Descriptor(
  name='configPmp_t',
  full_name='pb.mf_lsca.config_pmp_t.configPmp_t',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='jerkModel_mps', full_name='pb.mf_lsca.config_pmp_t.configPmp_t.jerkModel_mps', index=0,
      number=1995, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='safetyMargin_m', full_name='pb.mf_lsca.config_pmp_t.configPmp_t.safetyMargin_m', index=1,
      number=2674, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='safetyMargin_s', full_name='pb.mf_lsca.config_pmp_t.configPmp_t.safetyMargin_s', index=2,
      number=2445, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='maxSpeedBackwards_mps', full_name='pb.mf_lsca.config_pmp_t.configPmp_t.maxSpeedBackwards_mps', index=3,
      number=2679, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='maxSpeedForwards_mps', full_name='pb.mf_lsca.config_pmp_t.configPmp_t.maxSpeedForwards_mps', index=4,
      number=3827, type=2, cpp_type=6, label=1,
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
  serialized_start=56,
  serialized_end=206,
)


_CONFIGPMP_T_ARRAY_PORT = _descriptor.Descriptor(
  name='configPmp_t_array_port',
  full_name='pb.mf_lsca.config_pmp_t.configPmp_t_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_lsca.config_pmp_t.configPmp_t_array_port.data', index=0,
      number=1999, type=11, cpp_type=10, label=3,
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
  serialized_end=285,
)

_CONFIGPMP_T_ARRAY_PORT.fields_by_name['data'].message_type = _CONFIGPMP_T
DESCRIPTOR.message_types_by_name['configPmp_t'] = _CONFIGPMP_T
DESCRIPTOR.message_types_by_name['configPmp_t_array_port'] = _CONFIGPMP_T_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

configPmp_t = _reflection.GeneratedProtocolMessageType('configPmp_t', (_message.Message,), {
  'DESCRIPTOR' : _CONFIGPMP_T,
  '__module__' : 'mf_lsca.config_pmp_t_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.config_pmp_t.configPmp_t)
  })
_sym_db.RegisterMessage(configPmp_t)

configPmp_t_array_port = _reflection.GeneratedProtocolMessageType('configPmp_t_array_port', (_message.Message,), {
  'DESCRIPTOR' : _CONFIGPMP_T_ARRAY_PORT,
  '__module__' : 'mf_lsca.config_pmp_t_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.config_pmp_t.configPmp_t_array_port)
  })
_sym_db.RegisterMessage(configPmp_t_array_port)


# @@protoc_insertion_point(module_scope)
