# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: tce/tce_estimation_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='tce/tce_estimation_port_interface_version.proto',
  package='pb.tce.tce_estimation_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n/tce/tce_estimation_port_interface_version.proto\x12,pb.tce.tce_estimation_port_interface_version\"G\n\"TceEstimationPort_InterfaceVersion\x12!\n\x19TceEstimationPort_VERSION\x18G \x01(\r\"\x90\x01\n-TceEstimationPort_InterfaceVersion_array_port\x12_\n\x04\x64\x61ta\x18\x9a\x19 \x03(\x0b\x32P.pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion'
)




_TCEESTIMATIONPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='TceEstimationPort_InterfaceVersion',
  full_name='pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='TceEstimationPort_VERSION', full_name='pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion.TceEstimationPort_VERSION', index=0,
      number=71, type=13, cpp_type=3, label=1,
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
  serialized_start=97,
  serialized_end=168,
)


_TCEESTIMATIONPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='TceEstimationPort_InterfaceVersion_array_port',
  full_name='pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion_array_port.data', index=0,
      number=3226, type=11, cpp_type=10, label=3,
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
  serialized_start=171,
  serialized_end=315,
)

_TCEESTIMATIONPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _TCEESTIMATIONPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['TceEstimationPort_InterfaceVersion'] = _TCEESTIMATIONPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['TceEstimationPort_InterfaceVersion_array_port'] = _TCEESTIMATIONPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TceEstimationPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('TceEstimationPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _TCEESTIMATIONPORT_INTERFACEVERSION,
  '__module__' : 'tce.tce_estimation_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(TceEstimationPort_InterfaceVersion)

TceEstimationPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('TceEstimationPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _TCEESTIMATIONPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'tce.tce_estimation_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.tce.tce_estimation_port_interface_version.TceEstimationPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(TceEstimationPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)