# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_hmitoap/remote_hmioutput_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_hmitoap/remote_hmioutput_port_interface_version.proto',
  package='pb.ap_hmitoap.remote_hmioutput_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n8ap_hmitoap/remote_hmioutput_port_interface_version.proto\x12\x35pb.ap_hmitoap.remote_hmioutput_port_interface_version\"L\n$RemoteHMIOutputPort_InterfaceVersion\x12$\n\x1bRemoteHMIOutputPort_VERSION\x18\xd7\x01 \x01(\r\"\x9d\x01\n/RemoteHMIOutputPort_InterfaceVersion_array_port\x12j\n\x04\x64\x61ta\x18\xfe\x03 \x03(\x0b\x32[.pb.ap_hmitoap.remote_hmioutput_port_interface_version.RemoteHMIOutputPort_InterfaceVersion'
)




_REMOTEHMIOUTPUTPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='RemoteHMIOutputPort_InterfaceVersion',
  full_name='pb.ap_hmitoap.remote_hmioutput_port_interface_version.RemoteHMIOutputPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='RemoteHMIOutputPort_VERSION', full_name='pb.ap_hmitoap.remote_hmioutput_port_interface_version.RemoteHMIOutputPort_InterfaceVersion.RemoteHMIOutputPort_VERSION', index=0,
      number=215, type=13, cpp_type=3, label=1,
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
  serialized_start=115,
  serialized_end=191,
)


_REMOTEHMIOUTPUTPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='RemoteHMIOutputPort_InterfaceVersion_array_port',
  full_name='pb.ap_hmitoap.remote_hmioutput_port_interface_version.RemoteHMIOutputPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_hmitoap.remote_hmioutput_port_interface_version.RemoteHMIOutputPort_InterfaceVersion_array_port.data', index=0,
      number=510, type=11, cpp_type=10, label=3,
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
  serialized_start=194,
  serialized_end=351,
)

_REMOTEHMIOUTPUTPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _REMOTEHMIOUTPUTPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['RemoteHMIOutputPort_InterfaceVersion'] = _REMOTEHMIOUTPUTPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['RemoteHMIOutputPort_InterfaceVersion_array_port'] = _REMOTEHMIOUTPUTPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

RemoteHMIOutputPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('RemoteHMIOutputPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _REMOTEHMIOUTPUTPORT_INTERFACEVERSION,
  '__module__' : 'ap_hmitoap.remote_hmioutput_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_hmitoap.remote_hmioutput_port_interface_version.RemoteHMIOutputPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(RemoteHMIOutputPort_InterfaceVersion)

RemoteHMIOutputPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('RemoteHMIOutputPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _REMOTEHMIOUTPUTPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'ap_hmitoap.remote_hmioutput_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_hmitoap.remote_hmioutput_port_interface_version.RemoteHMIOutputPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(RemoteHMIOutputPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
