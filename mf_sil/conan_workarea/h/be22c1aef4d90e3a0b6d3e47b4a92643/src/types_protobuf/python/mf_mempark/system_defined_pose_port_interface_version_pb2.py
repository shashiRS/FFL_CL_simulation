# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/system_defined_pose_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/system_defined_pose_port_interface_version.proto',
  package='pb.mf_mempark.system_defined_pose_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n;mf_mempark/system_defined_pose_port_interface_version.proto\x12\x38pb.mf_mempark.system_defined_pose_port_interface_version\"P\n&SystemDefinedPosePort_InterfaceVersion\x12&\n\x1dSystemDefinedPosePort_VERSION\x18\xfd\x07 \x01(\r\"\xa4\x01\n1SystemDefinedPosePort_InterfaceVersion_array_port\x12o\n\x04\x64\x61ta\x18\xb7\x14 \x03(\x0b\x32`.pb.mf_mempark.system_defined_pose_port_interface_version.SystemDefinedPosePort_InterfaceVersion'
)




_SYSTEMDEFINEDPOSEPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='SystemDefinedPosePort_InterfaceVersion',
  full_name='pb.mf_mempark.system_defined_pose_port_interface_version.SystemDefinedPosePort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='SystemDefinedPosePort_VERSION', full_name='pb.mf_mempark.system_defined_pose_port_interface_version.SystemDefinedPosePort_InterfaceVersion.SystemDefinedPosePort_VERSION', index=0,
      number=1021, type=13, cpp_type=3, label=1,
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
  serialized_end=201,
)


_SYSTEMDEFINEDPOSEPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='SystemDefinedPosePort_InterfaceVersion_array_port',
  full_name='pb.mf_mempark.system_defined_pose_port_interface_version.SystemDefinedPosePort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_mempark.system_defined_pose_port_interface_version.SystemDefinedPosePort_InterfaceVersion_array_port.data', index=0,
      number=2615, type=11, cpp_type=10, label=3,
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
  serialized_start=204,
  serialized_end=368,
)

_SYSTEMDEFINEDPOSEPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _SYSTEMDEFINEDPOSEPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['SystemDefinedPosePort_InterfaceVersion'] = _SYSTEMDEFINEDPOSEPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['SystemDefinedPosePort_InterfaceVersion_array_port'] = _SYSTEMDEFINEDPOSEPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SystemDefinedPosePort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('SystemDefinedPosePort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _SYSTEMDEFINEDPOSEPORT_INTERFACEVERSION,
  '__module__' : 'mf_mempark.system_defined_pose_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.system_defined_pose_port_interface_version.SystemDefinedPosePort_InterfaceVersion)
  })
_sym_db.RegisterMessage(SystemDefinedPosePort_InterfaceVersion)

SystemDefinedPosePort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('SystemDefinedPosePort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _SYSTEMDEFINEDPOSEPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'mf_mempark.system_defined_pose_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.system_defined_pose_port_interface_version.SystemDefinedPosePort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(SystemDefinedPosePort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
