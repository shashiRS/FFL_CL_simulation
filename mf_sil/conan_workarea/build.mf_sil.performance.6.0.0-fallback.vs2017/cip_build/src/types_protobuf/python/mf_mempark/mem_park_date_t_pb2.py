# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/mem_park_date_t.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/mem_park_date_t.proto',
  package='pb.mf_mempark.mem_park_date_t',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n mf_mempark/mem_park_date_t.proto\x12\x1dpb.mf_mempark.mem_park_date_t\"\x80\x01\n\rMemParkDate_t\x12\r\n\x04year\x18\xd1\x17 \x01(\r\x12\x0e\n\x05month\x18\xc7\x15 \x01(\r\x12\x0c\n\x03\x64\x61y\x18\xea\x05 \x01(\r\x12\r\n\x04hour\x18\xb4\x1d \x01(\r\x12\x0f\n\x06minute\x18\xae\x1f \x01(\r\x12\x0f\n\x06second\x18\x9f\x1e \x01(\r\x12\x11\n\x08timeZone\x18\x90\x1e \x01(\x11\"W\n\x18MemParkDate_t_array_port\x12;\n\x04\x64\x61ta\x18\xb7\x01 \x03(\x0b\x32,.pb.mf_mempark.mem_park_date_t.MemParkDate_t'
)




_MEMPARKDATE_T = _descriptor.Descriptor(
  name='MemParkDate_t',
  full_name='pb.mf_mempark.mem_park_date_t.MemParkDate_t',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='year', full_name='pb.mf_mempark.mem_park_date_t.MemParkDate_t.year', index=0,
      number=3025, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='month', full_name='pb.mf_mempark.mem_park_date_t.MemParkDate_t.month', index=1,
      number=2759, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='day', full_name='pb.mf_mempark.mem_park_date_t.MemParkDate_t.day', index=2,
      number=746, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='hour', full_name='pb.mf_mempark.mem_park_date_t.MemParkDate_t.hour', index=3,
      number=3764, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='minute', full_name='pb.mf_mempark.mem_park_date_t.MemParkDate_t.minute', index=4,
      number=4014, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='second', full_name='pb.mf_mempark.mem_park_date_t.MemParkDate_t.second', index=5,
      number=3871, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='timeZone', full_name='pb.mf_mempark.mem_park_date_t.MemParkDate_t.timeZone', index=6,
      number=3856, type=17, cpp_type=1, label=1,
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
  serialized_start=68,
  serialized_end=196,
)


_MEMPARKDATE_T_ARRAY_PORT = _descriptor.Descriptor(
  name='MemParkDate_t_array_port',
  full_name='pb.mf_mempark.mem_park_date_t.MemParkDate_t_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_mempark.mem_park_date_t.MemParkDate_t_array_port.data', index=0,
      number=183, type=11, cpp_type=10, label=3,
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
  serialized_start=198,
  serialized_end=285,
)

_MEMPARKDATE_T_ARRAY_PORT.fields_by_name['data'].message_type = _MEMPARKDATE_T
DESCRIPTOR.message_types_by_name['MemParkDate_t'] = _MEMPARKDATE_T
DESCRIPTOR.message_types_by_name['MemParkDate_t_array_port'] = _MEMPARKDATE_T_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MemParkDate_t = _reflection.GeneratedProtocolMessageType('MemParkDate_t', (_message.Message,), {
  'DESCRIPTOR' : _MEMPARKDATE_T,
  '__module__' : 'mf_mempark.mem_park_date_t_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.mem_park_date_t.MemParkDate_t)
  })
_sym_db.RegisterMessage(MemParkDate_t)

MemParkDate_t_array_port = _reflection.GeneratedProtocolMessageType('MemParkDate_t_array_port', (_message.Message,), {
  'DESCRIPTOR' : _MEMPARKDATE_T_ARRAY_PORT,
  '__module__' : 'mf_mempark.mem_park_date_t_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.mem_park_date_t.MemParkDate_t_array_port)
  })
_sym_db.RegisterMessage(MemParkDate_t_array_port)


# @@protoc_insertion_point(module_scope)
