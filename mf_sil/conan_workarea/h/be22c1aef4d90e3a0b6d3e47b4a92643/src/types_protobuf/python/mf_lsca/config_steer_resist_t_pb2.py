# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_lsca/config_steer_resist_t.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_lsca/config_steer_resist_t.proto',
  package='pb.mf_lsca.config_steer_resist_t',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n#mf_lsca/config_steer_resist_t.proto\x12 pb.mf_lsca.config_steer_resist_t\"/\n\x13\x63onfigSteerResist_t\x12\x18\n\x0fsteerMargin_deg\x18\xe9\x10 \x01(\x02\"f\n\x1e\x63onfigSteerResist_t_array_port\x12\x44\n\x04\x64\x61ta\x18\x82\x0f \x03(\x0b\x32\x35.pb.mf_lsca.config_steer_resist_t.configSteerResist_t'
)




_CONFIGSTEERRESIST_T = _descriptor.Descriptor(
  name='configSteerResist_t',
  full_name='pb.mf_lsca.config_steer_resist_t.configSteerResist_t',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='steerMargin_deg', full_name='pb.mf_lsca.config_steer_resist_t.configSteerResist_t.steerMargin_deg', index=0,
      number=2153, type=2, cpp_type=6, label=1,
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
  serialized_start=73,
  serialized_end=120,
)


_CONFIGSTEERRESIST_T_ARRAY_PORT = _descriptor.Descriptor(
  name='configSteerResist_t_array_port',
  full_name='pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port.data', index=0,
      number=1922, type=11, cpp_type=10, label=3,
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
  serialized_start=122,
  serialized_end=224,
)

_CONFIGSTEERRESIST_T_ARRAY_PORT.fields_by_name['data'].message_type = _CONFIGSTEERRESIST_T
DESCRIPTOR.message_types_by_name['configSteerResist_t'] = _CONFIGSTEERRESIST_T
DESCRIPTOR.message_types_by_name['configSteerResist_t_array_port'] = _CONFIGSTEERRESIST_T_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

configSteerResist_t = _reflection.GeneratedProtocolMessageType('configSteerResist_t', (_message.Message,), {
  'DESCRIPTOR' : _CONFIGSTEERRESIST_T,
  '__module__' : 'mf_lsca.config_steer_resist_t_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.config_steer_resist_t.configSteerResist_t)
  })
_sym_db.RegisterMessage(configSteerResist_t)

configSteerResist_t_array_port = _reflection.GeneratedProtocolMessageType('configSteerResist_t_array_port', (_message.Message,), {
  'DESCRIPTOR' : _CONFIGSTEERRESIST_T_ARRAY_PORT,
  '__module__' : 'mf_lsca.config_steer_resist_t_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.config_steer_resist_t.configSteerResist_t_array_port)
  })
_sym_db.RegisterMessage(configSteerResist_t_array_port)


# @@protoc_insertion_point(module_scope)
