# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: lsm_vedodo/odo_debug_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='lsm_vedodo/odo_debug_port_interface_version.proto',
  package='pb.lsm_vedodo.odo_debug_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n1lsm_vedodo/odo_debug_port_interface_version.proto\x12.pb.lsm_vedodo.odo_debug_port_interface_version\">\n\x1dOdoDebugPort_InterfaceVersion\x12\x1d\n\x14OdoDebugPort_VERSION\x18\xec\x19 \x01(\r\"\x88\x01\n(OdoDebugPort_InterfaceVersion_array_port\x12\\\n\x04\x64\x61ta\x18\xd2\x02 \x03(\x0b\x32M.pb.lsm_vedodo.odo_debug_port_interface_version.OdoDebugPort_InterfaceVersion'
)




_ODODEBUGPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='OdoDebugPort_InterfaceVersion',
  full_name='pb.lsm_vedodo.odo_debug_port_interface_version.OdoDebugPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='OdoDebugPort_VERSION', full_name='pb.lsm_vedodo.odo_debug_port_interface_version.OdoDebugPort_InterfaceVersion.OdoDebugPort_VERSION', index=0,
      number=3308, type=13, cpp_type=3, label=1,
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
  serialized_start=101,
  serialized_end=163,
)


_ODODEBUGPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='OdoDebugPort_InterfaceVersion_array_port',
  full_name='pb.lsm_vedodo.odo_debug_port_interface_version.OdoDebugPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.lsm_vedodo.odo_debug_port_interface_version.OdoDebugPort_InterfaceVersion_array_port.data', index=0,
      number=338, type=11, cpp_type=10, label=3,
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
  serialized_start=166,
  serialized_end=302,
)

_ODODEBUGPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _ODODEBUGPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['OdoDebugPort_InterfaceVersion'] = _ODODEBUGPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['OdoDebugPort_InterfaceVersion_array_port'] = _ODODEBUGPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

OdoDebugPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('OdoDebugPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _ODODEBUGPORT_INTERFACEVERSION,
  '__module__' : 'lsm_vedodo.odo_debug_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.lsm_vedodo.odo_debug_port_interface_version.OdoDebugPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(OdoDebugPort_InterfaceVersion)

OdoDebugPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('OdoDebugPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _ODODEBUGPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'lsm_vedodo.odo_debug_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.lsm_vedodo.odo_debug_port_interface_version.OdoDebugPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(OdoDebugPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
