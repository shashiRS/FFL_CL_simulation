# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/visu_input_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/visu_input_port_interface_version.proto',
  package='pb.mf_hmih.visu_input_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n/mf_hmih/visu_input_port_interface_version.proto\x12,pb.mf_hmih.visu_input_port_interface_version\"@\n\x1eVisuInputPort_InterfaceVersion\x12\x1e\n\x15VisuInputPort_VERSION\x18\xc9\x0e \x01(\r\"\x88\x01\n)VisuInputPort_InterfaceVersion_array_port\x12[\n\x04\x64\x61ta\x18\x85\x0f \x03(\x0b\x32L.pb.mf_hmih.visu_input_port_interface_version.VisuInputPort_InterfaceVersion'
)




_VISUINPUTPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='VisuInputPort_InterfaceVersion',
  full_name='pb.mf_hmih.visu_input_port_interface_version.VisuInputPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='VisuInputPort_VERSION', full_name='pb.mf_hmih.visu_input_port_interface_version.VisuInputPort_InterfaceVersion.VisuInputPort_VERSION', index=0,
      number=1865, type=13, cpp_type=3, label=1,
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
  serialized_end=161,
)


_VISUINPUTPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='VisuInputPort_InterfaceVersion_array_port',
  full_name='pb.mf_hmih.visu_input_port_interface_version.VisuInputPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_hmih.visu_input_port_interface_version.VisuInputPort_InterfaceVersion_array_port.data', index=0,
      number=1925, type=11, cpp_type=10, label=3,
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
  serialized_start=164,
  serialized_end=300,
)

_VISUINPUTPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _VISUINPUTPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['VisuInputPort_InterfaceVersion'] = _VISUINPUTPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['VisuInputPort_InterfaceVersion_array_port'] = _VISUINPUTPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

VisuInputPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('VisuInputPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _VISUINPUTPORT_INTERFACEVERSION,
  '__module__' : 'mf_hmih.visu_input_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.visu_input_port_interface_version.VisuInputPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(VisuInputPort_InterfaceVersion)

VisuInputPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('VisuInputPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _VISUINPUTPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'mf_hmih.visu_input_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.visu_input_port_interface_version.VisuInputPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(VisuInputPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
