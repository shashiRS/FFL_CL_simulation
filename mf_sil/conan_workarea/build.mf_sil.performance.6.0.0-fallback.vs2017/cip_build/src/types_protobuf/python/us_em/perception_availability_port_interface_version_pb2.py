# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_em/perception_availability_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_em/perception_availability_port_interface_version.proto',
  package='pb.us_em.perception_availability_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n:us_em/perception_availability_port_interface_version.proto\x12\x37pb.us_em.perception_availability_port_interface_version\"Z\n+PerceptionAvailabilityPort_InterfaceVersion\x12+\n\"PerceptionAvailabilityPort_VERSION\x18\xf3\x17 \x01(\r\"\xad\x01\n6PerceptionAvailabilityPort_InterfaceVersion_array_port\x12s\n\x04\x64\x61ta\x18\xc9\x04 \x03(\x0b\x32\x64.pb.us_em.perception_availability_port_interface_version.PerceptionAvailabilityPort_InterfaceVersion'
)




_PERCEPTIONAVAILABILITYPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='PerceptionAvailabilityPort_InterfaceVersion',
  full_name='pb.us_em.perception_availability_port_interface_version.PerceptionAvailabilityPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='PerceptionAvailabilityPort_VERSION', full_name='pb.us_em.perception_availability_port_interface_version.PerceptionAvailabilityPort_InterfaceVersion.PerceptionAvailabilityPort_VERSION', index=0,
      number=3059, type=13, cpp_type=3, label=1,
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
  serialized_start=119,
  serialized_end=209,
)


_PERCEPTIONAVAILABILITYPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='PerceptionAvailabilityPort_InterfaceVersion_array_port',
  full_name='pb.us_em.perception_availability_port_interface_version.PerceptionAvailabilityPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_em.perception_availability_port_interface_version.PerceptionAvailabilityPort_InterfaceVersion_array_port.data', index=0,
      number=585, type=11, cpp_type=10, label=3,
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
  serialized_start=212,
  serialized_end=385,
)

_PERCEPTIONAVAILABILITYPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _PERCEPTIONAVAILABILITYPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['PerceptionAvailabilityPort_InterfaceVersion'] = _PERCEPTIONAVAILABILITYPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['PerceptionAvailabilityPort_InterfaceVersion_array_port'] = _PERCEPTIONAVAILABILITYPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PerceptionAvailabilityPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('PerceptionAvailabilityPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _PERCEPTIONAVAILABILITYPORT_INTERFACEVERSION,
  '__module__' : 'us_em.perception_availability_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_em.perception_availability_port_interface_version.PerceptionAvailabilityPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(PerceptionAvailabilityPort_InterfaceVersion)

PerceptionAvailabilityPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('PerceptionAvailabilityPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _PERCEPTIONAVAILABILITYPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'us_em.perception_availability_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_em.perception_availability_port_interface_version.PerceptionAvailabilityPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(PerceptionAvailabilityPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)