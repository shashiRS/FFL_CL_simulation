# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_commonvehsigprovider/thsample_time_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_commonvehsigprovider/thsample_time_port_interface_version.proto',
  package='pb.ap_commonvehsigprovider.thsample_time_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\nBap_commonvehsigprovider/thsample_time_port_interface_version.proto\x12?pb.ap_commonvehsigprovider.thsample_time_port_interface_version\"F\n!THSampleTimePort_InterfaceVersion\x12!\n\x18THSampleTimePort_VERSION\x18\xfa\x1c \x01(\r\"\xa1\x01\n,THSampleTimePort_InterfaceVersion_array_port\x12q\n\x04\x64\x61ta\x18\xde\x1a \x03(\x0b\x32\x62.pb.ap_commonvehsigprovider.thsample_time_port_interface_version.THSampleTimePort_InterfaceVersion'
)




_THSAMPLETIMEPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='THSampleTimePort_InterfaceVersion',
  full_name='pb.ap_commonvehsigprovider.thsample_time_port_interface_version.THSampleTimePort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='THSampleTimePort_VERSION', full_name='pb.ap_commonvehsigprovider.thsample_time_port_interface_version.THSampleTimePort_InterfaceVersion.THSampleTimePort_VERSION', index=0,
      number=3706, type=13, cpp_type=3, label=1,
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
  serialized_start=135,
  serialized_end=205,
)


_THSAMPLETIMEPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='THSampleTimePort_InterfaceVersion_array_port',
  full_name='pb.ap_commonvehsigprovider.thsample_time_port_interface_version.THSampleTimePort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_commonvehsigprovider.thsample_time_port_interface_version.THSampleTimePort_InterfaceVersion_array_port.data', index=0,
      number=3422, type=11, cpp_type=10, label=3,
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
  serialized_end=369,
)

_THSAMPLETIMEPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _THSAMPLETIMEPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['THSampleTimePort_InterfaceVersion'] = _THSAMPLETIMEPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['THSampleTimePort_InterfaceVersion_array_port'] = _THSAMPLETIMEPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

THSampleTimePort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('THSampleTimePort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _THSAMPLETIMEPORT_INTERFACEVERSION,
  '__module__' : 'ap_commonvehsigprovider.thsample_time_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.thsample_time_port_interface_version.THSampleTimePort_InterfaceVersion)
  })
_sym_db.RegisterMessage(THSampleTimePort_InterfaceVersion)

THSampleTimePort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('THSampleTimePort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _THSAMPLETIMEPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'ap_commonvehsigprovider.thsample_time_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.thsample_time_port_interface_version.THSampleTimePort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(THSampleTimePort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
