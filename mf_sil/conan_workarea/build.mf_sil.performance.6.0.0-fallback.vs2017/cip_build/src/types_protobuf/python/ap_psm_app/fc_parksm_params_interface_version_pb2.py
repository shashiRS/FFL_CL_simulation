# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_psm_app/fc_parksm_params_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_psm_app/fc_parksm_params_interface_version.proto',
  package='pb.ap_psm_app.fc_parksm_params_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n3ap_psm_app/fc_parksm_params_interface_version.proto\x12\x30pb.ap_psm_app.fc_parksm_params_interface_version\"F\n!FC_PARKSM_Params_InterfaceVersion\x12!\n\x18\x46\x43_PARKSM_Params_VERSION\x18\x80\x11 \x01(\r\"\x92\x01\n,FC_PARKSM_Params_InterfaceVersion_array_port\x12\x62\n\x04\x64\x61ta\x18\x94\r \x03(\x0b\x32S.pb.ap_psm_app.fc_parksm_params_interface_version.FC_PARKSM_Params_InterfaceVersion'
)




_FC_PARKSM_PARAMS_INTERFACEVERSION = _descriptor.Descriptor(
  name='FC_PARKSM_Params_InterfaceVersion',
  full_name='pb.ap_psm_app.fc_parksm_params_interface_version.FC_PARKSM_Params_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='FC_PARKSM_Params_VERSION', full_name='pb.ap_psm_app.fc_parksm_params_interface_version.FC_PARKSM_Params_InterfaceVersion.FC_PARKSM_Params_VERSION', index=0,
      number=2176, type=13, cpp_type=3, label=1,
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
  serialized_start=105,
  serialized_end=175,
)


_FC_PARKSM_PARAMS_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='FC_PARKSM_Params_InterfaceVersion_array_port',
  full_name='pb.ap_psm_app.fc_parksm_params_interface_version.FC_PARKSM_Params_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_psm_app.fc_parksm_params_interface_version.FC_PARKSM_Params_InterfaceVersion_array_port.data', index=0,
      number=1684, type=11, cpp_type=10, label=3,
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
  serialized_start=178,
  serialized_end=324,
)

_FC_PARKSM_PARAMS_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _FC_PARKSM_PARAMS_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['FC_PARKSM_Params_InterfaceVersion'] = _FC_PARKSM_PARAMS_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['FC_PARKSM_Params_InterfaceVersion_array_port'] = _FC_PARKSM_PARAMS_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FC_PARKSM_Params_InterfaceVersion = _reflection.GeneratedProtocolMessageType('FC_PARKSM_Params_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _FC_PARKSM_PARAMS_INTERFACEVERSION,
  '__module__' : 'ap_psm_app.fc_parksm_params_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm_app.fc_parksm_params_interface_version.FC_PARKSM_Params_InterfaceVersion)
  })
_sym_db.RegisterMessage(FC_PARKSM_Params_InterfaceVersion)

FC_PARKSM_Params_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('FC_PARKSM_Params_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _FC_PARKSM_PARAMS_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'ap_psm_app.fc_parksm_params_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm_app.fc_parksm_params_interface_version.FC_PARKSM_Params_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(FC_PARKSM_Params_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
