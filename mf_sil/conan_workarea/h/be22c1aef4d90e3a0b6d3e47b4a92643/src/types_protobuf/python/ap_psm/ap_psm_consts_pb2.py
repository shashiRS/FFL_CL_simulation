# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_psm/ap_psm_consts.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_psm/ap_psm_consts.proto',
  package='pb.ap_psm.ap_psm_consts',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1a\x61p_psm/ap_psm_consts.proto\x12\x17pb.ap_psm.ap_psm_consts\"5\n\rAP_PSM_Consts\x12$\n\x1bNUM_MTS_DEBUG_FREESPACE_PSM\x18\xcf\x13 \x01(\r\"Q\n\x18\x41P_PSM_Consts_array_port\x12\x35\n\x04\x64\x61ta\x18\xad\r \x03(\x0b\x32&.pb.ap_psm.ap_psm_consts.AP_PSM_Consts'
)




_AP_PSM_CONSTS = _descriptor.Descriptor(
  name='AP_PSM_Consts',
  full_name='pb.ap_psm.ap_psm_consts.AP_PSM_Consts',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='NUM_MTS_DEBUG_FREESPACE_PSM', full_name='pb.ap_psm.ap_psm_consts.AP_PSM_Consts.NUM_MTS_DEBUG_FREESPACE_PSM', index=0,
      number=2511, type=13, cpp_type=3, label=1,
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
  serialized_start=55,
  serialized_end=108,
)


_AP_PSM_CONSTS_ARRAY_PORT = _descriptor.Descriptor(
  name='AP_PSM_Consts_array_port',
  full_name='pb.ap_psm.ap_psm_consts.AP_PSM_Consts_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_psm.ap_psm_consts.AP_PSM_Consts_array_port.data', index=0,
      number=1709, type=11, cpp_type=10, label=3,
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
  serialized_start=110,
  serialized_end=191,
)

_AP_PSM_CONSTS_ARRAY_PORT.fields_by_name['data'].message_type = _AP_PSM_CONSTS
DESCRIPTOR.message_types_by_name['AP_PSM_Consts'] = _AP_PSM_CONSTS
DESCRIPTOR.message_types_by_name['AP_PSM_Consts_array_port'] = _AP_PSM_CONSTS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

AP_PSM_Consts = _reflection.GeneratedProtocolMessageType('AP_PSM_Consts', (_message.Message,), {
  'DESCRIPTOR' : _AP_PSM_CONSTS,
  '__module__' : 'ap_psm.ap_psm_consts_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm.ap_psm_consts.AP_PSM_Consts)
  })
_sym_db.RegisterMessage(AP_PSM_Consts)

AP_PSM_Consts_array_port = _reflection.GeneratedProtocolMessageType('AP_PSM_Consts_array_port', (_message.Message,), {
  'DESCRIPTOR' : _AP_PSM_CONSTS_ARRAY_PORT,
  '__module__' : 'ap_psm.ap_psm_consts_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm.ap_psm_consts.AP_PSM_Consts_array_port)
  })
_sym_db.RegisterMessage(AP_PSM_Consts_array_port)


# @@protoc_insertion_point(module_scope)
