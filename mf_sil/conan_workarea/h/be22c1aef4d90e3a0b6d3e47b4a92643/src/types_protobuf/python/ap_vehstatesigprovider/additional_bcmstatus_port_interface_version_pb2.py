# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_vehstatesigprovider/additional_bcmstatus_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_vehstatesigprovider/additional_bcmstatus_port_interface_version.proto',
  package='pb.ap_vehstatesigprovider.additional_bcmstatus_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\nHap_vehstatesigprovider/additional_bcmstatus_port_interface_version.proto\x12\x45pb.ap_vehstatesigprovider.additional_bcmstatus_port_interface_version\"T\n(AdditionalBCMStatusPort_InterfaceVersion\x12(\n\x1f\x41\x64\x64itionalBCMStatusPort_VERSION\x18\xed\x10 \x01(\r\"\xb5\x01\n3AdditionalBCMStatusPort_InterfaceVersion_array_port\x12~\n\x04\x64\x61ta\x18\xfb\x1f \x03(\x0b\x32o.pb.ap_vehstatesigprovider.additional_bcmstatus_port_interface_version.AdditionalBCMStatusPort_InterfaceVersion'
)




_ADDITIONALBCMSTATUSPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='AdditionalBCMStatusPort_InterfaceVersion',
  full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port_interface_version.AdditionalBCMStatusPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='AdditionalBCMStatusPort_VERSION', full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port_interface_version.AdditionalBCMStatusPort_InterfaceVersion.AdditionalBCMStatusPort_VERSION', index=0,
      number=2157, type=13, cpp_type=3, label=1,
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
  serialized_start=147,
  serialized_end=231,
)


_ADDITIONALBCMSTATUSPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='AdditionalBCMStatusPort_InterfaceVersion_array_port',
  full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port_interface_version.AdditionalBCMStatusPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port_interface_version.AdditionalBCMStatusPort_InterfaceVersion_array_port.data', index=0,
      number=4091, type=11, cpp_type=10, label=3,
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
  serialized_start=234,
  serialized_end=415,
)

_ADDITIONALBCMSTATUSPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _ADDITIONALBCMSTATUSPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['AdditionalBCMStatusPort_InterfaceVersion'] = _ADDITIONALBCMSTATUSPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['AdditionalBCMStatusPort_InterfaceVersion_array_port'] = _ADDITIONALBCMSTATUSPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

AdditionalBCMStatusPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('AdditionalBCMStatusPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _ADDITIONALBCMSTATUSPORT_INTERFACEVERSION,
  '__module__' : 'ap_vehstatesigprovider.additional_bcmstatus_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.additional_bcmstatus_port_interface_version.AdditionalBCMStatusPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(AdditionalBCMStatusPort_InterfaceVersion)

AdditionalBCMStatusPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('AdditionalBCMStatusPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _ADDITIONALBCMSTATUSPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'ap_vehstatesigprovider.additional_bcmstatus_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.additional_bcmstatus_port_interface_version.AdditionalBCMStatusPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(AdditionalBCMStatusPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
