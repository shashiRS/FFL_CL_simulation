# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_common/fc_parksm_sys_func_params_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_common/fc_parksm_sys_func_params_interface_version.proto',
  package='pb.ap_common.fc_parksm_sys_func_params_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n;ap_common/fc_parksm_sys_func_params_interface_version.proto\x12\x38pb.ap_common.fc_parksm_sys_func_params_interface_version\"X\n*FC_PARKSM_Sys_Func_Params_InterfaceVersion\x12*\n!FC_PARKSM_Sys_Func_Params_VERSION\x18\xa9\x0b \x01(\r\"\xac\x01\n5FC_PARKSM_Sys_Func_Params_InterfaceVersion_array_port\x12s\n\x04\x64\x61ta\x18\xbc\x12 \x03(\x0b\x32\x64.pb.ap_common.fc_parksm_sys_func_params_interface_version.FC_PARKSM_Sys_Func_Params_InterfaceVersion'
)




_FC_PARKSM_SYS_FUNC_PARAMS_INTERFACEVERSION = _descriptor.Descriptor(
  name='FC_PARKSM_Sys_Func_Params_InterfaceVersion',
  full_name='pb.ap_common.fc_parksm_sys_func_params_interface_version.FC_PARKSM_Sys_Func_Params_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='FC_PARKSM_Sys_Func_Params_VERSION', full_name='pb.ap_common.fc_parksm_sys_func_params_interface_version.FC_PARKSM_Sys_Func_Params_InterfaceVersion.FC_PARKSM_Sys_Func_Params_VERSION', index=0,
      number=1449, type=13, cpp_type=3, label=1,
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
  serialized_start=121,
  serialized_end=209,
)


_FC_PARKSM_SYS_FUNC_PARAMS_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='FC_PARKSM_Sys_Func_Params_InterfaceVersion_array_port',
  full_name='pb.ap_common.fc_parksm_sys_func_params_interface_version.FC_PARKSM_Sys_Func_Params_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_common.fc_parksm_sys_func_params_interface_version.FC_PARKSM_Sys_Func_Params_InterfaceVersion_array_port.data', index=0,
      number=2364, type=11, cpp_type=10, label=3,
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
  serialized_start=212,
  serialized_end=384,
)

_FC_PARKSM_SYS_FUNC_PARAMS_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _FC_PARKSM_SYS_FUNC_PARAMS_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['FC_PARKSM_Sys_Func_Params_InterfaceVersion'] = _FC_PARKSM_SYS_FUNC_PARAMS_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['FC_PARKSM_Sys_Func_Params_InterfaceVersion_array_port'] = _FC_PARKSM_SYS_FUNC_PARAMS_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FC_PARKSM_Sys_Func_Params_InterfaceVersion = _reflection.GeneratedProtocolMessageType('FC_PARKSM_Sys_Func_Params_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _FC_PARKSM_SYS_FUNC_PARAMS_INTERFACEVERSION,
  '__module__' : 'ap_common.fc_parksm_sys_func_params_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_common.fc_parksm_sys_func_params_interface_version.FC_PARKSM_Sys_Func_Params_InterfaceVersion)
  })
_sym_db.RegisterMessage(FC_PARKSM_Sys_Func_Params_InterfaceVersion)

FC_PARKSM_Sys_Func_Params_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('FC_PARKSM_Sys_Func_Params_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _FC_PARKSM_SYS_FUNC_PARAMS_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'ap_common.fc_parksm_sys_func_params_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_common.fc_parksm_sys_func_params_interface_version.FC_PARKSM_Sys_Func_Params_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(FC_PARKSM_Sys_Func_Params_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
