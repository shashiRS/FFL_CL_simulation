# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_vehstatesigprovider/convertible_top_status_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_vehstatesigprovider/convertible_top_status_port_interface_version.proto',
  package='pb.ap_vehstatesigprovider.convertible_top_status_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\nJap_vehstatesigprovider/convertible_top_status_port_interface_version.proto\x12Gpb.ap_vehstatesigprovider.convertible_top_status_port_interface_version\"U\n)ConvertibleTopStatusPort_InterfaceVersion\x12(\n ConvertibleTopStatusPort_VERSION\x18[ \x01(\r\"\xba\x01\n4ConvertibleTopStatusPort_InterfaceVersion_array_port\x12\x81\x01\n\x04\x64\x61ta\x18\xd5\x07 \x03(\x0b\x32r.pb.ap_vehstatesigprovider.convertible_top_status_port_interface_version.ConvertibleTopStatusPort_InterfaceVersion'
)




_CONVERTIBLETOPSTATUSPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='ConvertibleTopStatusPort_InterfaceVersion',
  full_name='pb.ap_vehstatesigprovider.convertible_top_status_port_interface_version.ConvertibleTopStatusPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ConvertibleTopStatusPort_VERSION', full_name='pb.ap_vehstatesigprovider.convertible_top_status_port_interface_version.ConvertibleTopStatusPort_InterfaceVersion.ConvertibleTopStatusPort_VERSION', index=0,
      number=91, type=13, cpp_type=3, label=1,
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
  serialized_start=151,
  serialized_end=236,
)


_CONVERTIBLETOPSTATUSPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='ConvertibleTopStatusPort_InterfaceVersion_array_port',
  full_name='pb.ap_vehstatesigprovider.convertible_top_status_port_interface_version.ConvertibleTopStatusPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_vehstatesigprovider.convertible_top_status_port_interface_version.ConvertibleTopStatusPort_InterfaceVersion_array_port.data', index=0,
      number=981, type=11, cpp_type=10, label=3,
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
  serialized_start=239,
  serialized_end=425,
)

_CONVERTIBLETOPSTATUSPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _CONVERTIBLETOPSTATUSPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['ConvertibleTopStatusPort_InterfaceVersion'] = _CONVERTIBLETOPSTATUSPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['ConvertibleTopStatusPort_InterfaceVersion_array_port'] = _CONVERTIBLETOPSTATUSPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ConvertibleTopStatusPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('ConvertibleTopStatusPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _CONVERTIBLETOPSTATUSPORT_INTERFACEVERSION,
  '__module__' : 'ap_vehstatesigprovider.convertible_top_status_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.convertible_top_status_port_interface_version.ConvertibleTopStatusPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(ConvertibleTopStatusPort_InterfaceVersion)

ConvertibleTopStatusPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('ConvertibleTopStatusPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _CONVERTIBLETOPSTATUSPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'ap_vehstatesigprovider.convertible_top_status_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.convertible_top_status_port_interface_version.ConvertibleTopStatusPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(ConvertibleTopStatusPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
