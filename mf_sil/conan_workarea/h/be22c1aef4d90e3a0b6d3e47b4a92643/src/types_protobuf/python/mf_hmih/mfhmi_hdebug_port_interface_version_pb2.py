# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/mfhmi_hdebug_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/mfhmi_hdebug_port_interface_version.proto',
  package='pb.mf_hmih.mfhmi_hdebug_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n1mf_hmih/mfhmi_hdebug_port_interface_version.proto\x12.pb.mf_hmih.mfhmi_hdebug_port_interface_version\"D\n MFHmiHDebugPort_InterfaceVersion\x12 \n\x17MFHmiHDebugPort_VERSION\x18\x9a\x02 \x01(\r\"\x8e\x01\n+MFHmiHDebugPort_InterfaceVersion_array_port\x12_\n\x04\x64\x61ta\x18\x87\x1d \x03(\x0b\x32P.pb.mf_hmih.mfhmi_hdebug_port_interface_version.MFHmiHDebugPort_InterfaceVersion'
)




_MFHMIHDEBUGPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='MFHmiHDebugPort_InterfaceVersion',
  full_name='pb.mf_hmih.mfhmi_hdebug_port_interface_version.MFHmiHDebugPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='MFHmiHDebugPort_VERSION', full_name='pb.mf_hmih.mfhmi_hdebug_port_interface_version.MFHmiHDebugPort_InterfaceVersion.MFHmiHDebugPort_VERSION', index=0,
      number=282, type=13, cpp_type=3, label=1,
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
  serialized_end=169,
)


_MFHMIHDEBUGPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='MFHmiHDebugPort_InterfaceVersion_array_port',
  full_name='pb.mf_hmih.mfhmi_hdebug_port_interface_version.MFHmiHDebugPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_hmih.mfhmi_hdebug_port_interface_version.MFHmiHDebugPort_InterfaceVersion_array_port.data', index=0,
      number=3719, type=11, cpp_type=10, label=3,
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
  serialized_start=172,
  serialized_end=314,
)

_MFHMIHDEBUGPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _MFHMIHDEBUGPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['MFHmiHDebugPort_InterfaceVersion'] = _MFHMIHDEBUGPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['MFHmiHDebugPort_InterfaceVersion_array_port'] = _MFHMIHDEBUGPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MFHmiHDebugPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('MFHmiHDebugPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _MFHMIHDEBUGPORT_INTERFACEVERSION,
  '__module__' : 'mf_hmih.mfhmi_hdebug_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.mfhmi_hdebug_port_interface_version.MFHmiHDebugPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(MFHmiHDebugPort_InterfaceVersion)

MFHmiHDebugPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('MFHmiHDebugPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _MFHMIHDEBUGPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'mf_hmih.mfhmi_hdebug_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.mfhmi_hdebug_port_interface_version.MFHmiHDebugPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(MFHmiHDebugPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)