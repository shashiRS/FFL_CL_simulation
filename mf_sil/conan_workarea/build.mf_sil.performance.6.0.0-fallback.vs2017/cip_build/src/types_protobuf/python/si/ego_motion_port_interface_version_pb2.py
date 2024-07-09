# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/ego_motion_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/ego_motion_port_interface_version.proto',
  package='pb.si.ego_motion_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n*si/ego_motion_port_interface_version.proto\x12\'pb.si.ego_motion_port_interface_version\"@\n\x1e\x45goMotionPort_InterfaceVersion\x12\x1e\n\x15\x45goMotionPort_VERSION\x18\xf2\x1c \x01(\r\"\x83\x01\n)EgoMotionPort_InterfaceVersion_array_port\x12V\n\x04\x64\x61ta\x18\xac\x10 \x03(\x0b\x32G.pb.si.ego_motion_port_interface_version.EgoMotionPort_InterfaceVersion'
)




_EGOMOTIONPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='EgoMotionPort_InterfaceVersion',
  full_name='pb.si.ego_motion_port_interface_version.EgoMotionPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='EgoMotionPort_VERSION', full_name='pb.si.ego_motion_port_interface_version.EgoMotionPort_InterfaceVersion.EgoMotionPort_VERSION', index=0,
      number=3698, type=13, cpp_type=3, label=1,
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
  serialized_start=87,
  serialized_end=151,
)


_EGOMOTIONPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='EgoMotionPort_InterfaceVersion_array_port',
  full_name='pb.si.ego_motion_port_interface_version.EgoMotionPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.ego_motion_port_interface_version.EgoMotionPort_InterfaceVersion_array_port.data', index=0,
      number=2092, type=11, cpp_type=10, label=3,
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
  serialized_start=154,
  serialized_end=285,
)

_EGOMOTIONPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _EGOMOTIONPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['EgoMotionPort_InterfaceVersion'] = _EGOMOTIONPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['EgoMotionPort_InterfaceVersion_array_port'] = _EGOMOTIONPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

EgoMotionPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('EgoMotionPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _EGOMOTIONPORT_INTERFACEVERSION,
  '__module__' : 'si.ego_motion_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.ego_motion_port_interface_version.EgoMotionPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(EgoMotionPort_InterfaceVersion)

EgoMotionPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('EgoMotionPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _EGOMOTIONPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'si.ego_motion_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.ego_motion_port_interface_version.EgoMotionPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(EgoMotionPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)