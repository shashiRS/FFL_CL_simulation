# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_tp/driven_path_data_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_tp/driven_path_data_port_interface_version.proto',
  package='pb.ap_tp.driven_path_data_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n3ap_tp/driven_path_data_port_interface_version.proto\x12\x30pb.ap_tp.driven_path_data_port_interface_version\"J\n#DrivenPathDataPort_InterfaceVersion\x12#\n\x1a\x44rivenPathDataPort_VERSION\x18\xfa\x02 \x01(\r\"\x96\x01\n.DrivenPathDataPort_InterfaceVersion_array_port\x12\x64\n\x04\x64\x61ta\x18\xc8\x07 \x03(\x0b\x32U.pb.ap_tp.driven_path_data_port_interface_version.DrivenPathDataPort_InterfaceVersion'
)




_DRIVENPATHDATAPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='DrivenPathDataPort_InterfaceVersion',
  full_name='pb.ap_tp.driven_path_data_port_interface_version.DrivenPathDataPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='DrivenPathDataPort_VERSION', full_name='pb.ap_tp.driven_path_data_port_interface_version.DrivenPathDataPort_InterfaceVersion.DrivenPathDataPort_VERSION', index=0,
      number=378, type=13, cpp_type=3, label=1,
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
  serialized_start=105,
  serialized_end=179,
)


_DRIVENPATHDATAPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='DrivenPathDataPort_InterfaceVersion_array_port',
  full_name='pb.ap_tp.driven_path_data_port_interface_version.DrivenPathDataPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_tp.driven_path_data_port_interface_version.DrivenPathDataPort_InterfaceVersion_array_port.data', index=0,
      number=968, type=11, cpp_type=10, label=3,
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
  serialized_start=182,
  serialized_end=332,
)

_DRIVENPATHDATAPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _DRIVENPATHDATAPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['DrivenPathDataPort_InterfaceVersion'] = _DRIVENPATHDATAPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['DrivenPathDataPort_InterfaceVersion_array_port'] = _DRIVENPATHDATAPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

DrivenPathDataPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('DrivenPathDataPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _DRIVENPATHDATAPORT_INTERFACEVERSION,
  '__module__' : 'ap_tp.driven_path_data_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.driven_path_data_port_interface_version.DrivenPathDataPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(DrivenPathDataPort_InterfaceVersion)

DrivenPathDataPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('DrivenPathDataPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _DRIVENPATHDATAPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'ap_tp.driven_path_data_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.driven_path_data_port_interface_version.DrivenPathDataPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(DrivenPathDataPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
