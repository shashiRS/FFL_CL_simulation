# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_psm/parksmcore_status_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_psm/parksmcore_status_port_interface_version.proto',
  package='pb.ap_psm.parksmcore_status_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n5ap_psm/parksmcore_status_port_interface_version.proto\x12\x32pb.ap_psm.parksmcore_status_port_interface_version\"N\n%PARKSMCoreStatusPort_InterfaceVersion\x12%\n\x1cPARKSMCoreStatusPort_VERSION\x18\xda\x04 \x01(\r\"\x9c\x01\n0PARKSMCoreStatusPort_InterfaceVersion_array_port\x12h\n\x04\x64\x61ta\x18\xe8\x04 \x03(\x0b\x32Y.pb.ap_psm.parksmcore_status_port_interface_version.PARKSMCoreStatusPort_InterfaceVersion'
)




_PARKSMCORESTATUSPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='PARKSMCoreStatusPort_InterfaceVersion',
  full_name='pb.ap_psm.parksmcore_status_port_interface_version.PARKSMCoreStatusPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='PARKSMCoreStatusPort_VERSION', full_name='pb.ap_psm.parksmcore_status_port_interface_version.PARKSMCoreStatusPort_InterfaceVersion.PARKSMCoreStatusPort_VERSION', index=0,
      number=602, type=13, cpp_type=3, label=1,
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
  serialized_start=109,
  serialized_end=187,
)


_PARKSMCORESTATUSPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='PARKSMCoreStatusPort_InterfaceVersion_array_port',
  full_name='pb.ap_psm.parksmcore_status_port_interface_version.PARKSMCoreStatusPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_psm.parksmcore_status_port_interface_version.PARKSMCoreStatusPort_InterfaceVersion_array_port.data', index=0,
      number=616, type=11, cpp_type=10, label=3,
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
  serialized_start=190,
  serialized_end=346,
)

_PARKSMCORESTATUSPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _PARKSMCORESTATUSPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['PARKSMCoreStatusPort_InterfaceVersion'] = _PARKSMCORESTATUSPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['PARKSMCoreStatusPort_InterfaceVersion_array_port'] = _PARKSMCORESTATUSPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PARKSMCoreStatusPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('PARKSMCoreStatusPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _PARKSMCORESTATUSPORT_INTERFACEVERSION,
  '__module__' : 'ap_psm.parksmcore_status_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm.parksmcore_status_port_interface_version.PARKSMCoreStatusPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(PARKSMCoreStatusPort_InterfaceVersion)

PARKSMCoreStatusPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('PARKSMCoreStatusPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _PARKSMCORESTATUSPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'ap_psm.parksmcore_status_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm.parksmcore_status_port_interface_version.PARKSMCoreStatusPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(PARKSMCoreStatusPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)