# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_trjctl/lo_dmcctrl_request_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_trjctl/lo_dmcctrl_request_port_interface_version.proto',
  package='pb.ap_trjctl.lo_dmcctrl_request_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n9ap_trjctl/lo_dmcctrl_request_port_interface_version.proto\x12\x36pb.ap_trjctl.lo_dmcctrl_request_port_interface_version\"N\n%LoDMCCtrlRequestPort_InterfaceVersion\x12%\n\x1cLoDMCCtrlRequestPort_VERSION\x18\x99\x04 \x01(\r\"\xa0\x01\n0LoDMCCtrlRequestPort_InterfaceVersion_array_port\x12l\n\x04\x64\x61ta\x18\xc6\x04 \x03(\x0b\x32].pb.ap_trjctl.lo_dmcctrl_request_port_interface_version.LoDMCCtrlRequestPort_InterfaceVersion'
)




_LODMCCTRLREQUESTPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='LoDMCCtrlRequestPort_InterfaceVersion',
  full_name='pb.ap_trjctl.lo_dmcctrl_request_port_interface_version.LoDMCCtrlRequestPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='LoDMCCtrlRequestPort_VERSION', full_name='pb.ap_trjctl.lo_dmcctrl_request_port_interface_version.LoDMCCtrlRequestPort_InterfaceVersion.LoDMCCtrlRequestPort_VERSION', index=0,
      number=537, type=13, cpp_type=3, label=1,
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
  serialized_start=117,
  serialized_end=195,
)


_LODMCCTRLREQUESTPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='LoDMCCtrlRequestPort_InterfaceVersion_array_port',
  full_name='pb.ap_trjctl.lo_dmcctrl_request_port_interface_version.LoDMCCtrlRequestPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_trjctl.lo_dmcctrl_request_port_interface_version.LoDMCCtrlRequestPort_InterfaceVersion_array_port.data', index=0,
      number=582, type=11, cpp_type=10, label=3,
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
  serialized_end=358,
)

_LODMCCTRLREQUESTPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _LODMCCTRLREQUESTPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['LoDMCCtrlRequestPort_InterfaceVersion'] = _LODMCCTRLREQUESTPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['LoDMCCtrlRequestPort_InterfaceVersion_array_port'] = _LODMCCTRLREQUESTPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LoDMCCtrlRequestPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('LoDMCCtrlRequestPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _LODMCCTRLREQUESTPORT_INTERFACEVERSION,
  '__module__' : 'ap_trjctl.lo_dmcctrl_request_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_trjctl.lo_dmcctrl_request_port_interface_version.LoDMCCtrlRequestPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(LoDMCCtrlRequestPort_InterfaceVersion)

LoDMCCtrlRequestPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('LoDMCCtrlRequestPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _LODMCCTRLREQUESTPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'ap_trjctl.lo_dmcctrl_request_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_trjctl.lo_dmcctrl_request_port_interface_version.LoDMCCtrlRequestPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(LoDMCCtrlRequestPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
