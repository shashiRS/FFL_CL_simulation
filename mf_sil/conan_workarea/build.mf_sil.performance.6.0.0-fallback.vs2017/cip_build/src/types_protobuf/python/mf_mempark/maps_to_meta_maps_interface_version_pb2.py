# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/maps_to_meta_maps_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/maps_to_meta_maps_interface_version.proto',
  package='pb.mf_mempark.maps_to_meta_maps_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n4mf_mempark/maps_to_meta_maps_interface_version.proto\x12\x31pb.mf_mempark.maps_to_meta_maps_interface_version\"B\n\x1fMapsToMetaMaps_InterfaceVersion\x12\x1f\n\x16MapsToMetaMaps_VERSION\x18\xeb\x14 \x01(\r\"\x8f\x01\n*MapsToMetaMaps_InterfaceVersion_array_port\x12\x61\n\x04\x64\x61ta\x18\xfc\x1e \x03(\x0b\x32R.pb.mf_mempark.maps_to_meta_maps_interface_version.MapsToMetaMaps_InterfaceVersion'
)




_MAPSTOMETAMAPS_INTERFACEVERSION = _descriptor.Descriptor(
  name='MapsToMetaMaps_InterfaceVersion',
  full_name='pb.mf_mempark.maps_to_meta_maps_interface_version.MapsToMetaMaps_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='MapsToMetaMaps_VERSION', full_name='pb.mf_mempark.maps_to_meta_maps_interface_version.MapsToMetaMaps_InterfaceVersion.MapsToMetaMaps_VERSION', index=0,
      number=2667, type=13, cpp_type=3, label=1,
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
  serialized_start=107,
  serialized_end=173,
)


_MAPSTOMETAMAPS_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='MapsToMetaMaps_InterfaceVersion_array_port',
  full_name='pb.mf_mempark.maps_to_meta_maps_interface_version.MapsToMetaMaps_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_mempark.maps_to_meta_maps_interface_version.MapsToMetaMaps_InterfaceVersion_array_port.data', index=0,
      number=3964, type=11, cpp_type=10, label=3,
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
  serialized_start=176,
  serialized_end=319,
)

_MAPSTOMETAMAPS_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _MAPSTOMETAMAPS_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['MapsToMetaMaps_InterfaceVersion'] = _MAPSTOMETAMAPS_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['MapsToMetaMaps_InterfaceVersion_array_port'] = _MAPSTOMETAMAPS_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MapsToMetaMaps_InterfaceVersion = _reflection.GeneratedProtocolMessageType('MapsToMetaMaps_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _MAPSTOMETAMAPS_INTERFACEVERSION,
  '__module__' : 'mf_mempark.maps_to_meta_maps_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.maps_to_meta_maps_interface_version.MapsToMetaMaps_InterfaceVersion)
  })
_sym_db.RegisterMessage(MapsToMetaMaps_InterfaceVersion)

MapsToMetaMaps_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('MapsToMetaMaps_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _MAPSTOMETAMAPS_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'mf_mempark.maps_to_meta_maps_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.maps_to_meta_maps_interface_version.MapsToMetaMaps_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(MapsToMetaMaps_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)