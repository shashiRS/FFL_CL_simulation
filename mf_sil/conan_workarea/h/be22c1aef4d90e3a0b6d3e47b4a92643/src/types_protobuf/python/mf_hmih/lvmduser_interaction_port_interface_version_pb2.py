# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/lvmduser_interaction_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/lvmduser_interaction_port_interface_version.proto',
  package='pb.mf_hmih.lvmduser_interaction_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n9mf_hmih/lvmduser_interaction_port_interface_version.proto\x12\x36pb.mf_hmih.lvmduser_interaction_port_interface_version\"T\n(LVMDUserInteractionPort_InterfaceVersion\x12(\n\x1fLVMDUserInteractionPort_VERSION\x18\xd0\x0e \x01(\r\"\xa6\x01\n3LVMDUserInteractionPort_InterfaceVersion_array_port\x12o\n\x04\x64\x61ta\x18\xe5\x11 \x03(\x0b\x32`.pb.mf_hmih.lvmduser_interaction_port_interface_version.LVMDUserInteractionPort_InterfaceVersion'
)




_LVMDUSERINTERACTIONPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='LVMDUserInteractionPort_InterfaceVersion',
  full_name='pb.mf_hmih.lvmduser_interaction_port_interface_version.LVMDUserInteractionPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='LVMDUserInteractionPort_VERSION', full_name='pb.mf_hmih.lvmduser_interaction_port_interface_version.LVMDUserInteractionPort_InterfaceVersion.LVMDUserInteractionPort_VERSION', index=0,
      number=1872, type=13, cpp_type=3, label=1,
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
  serialized_start=117,
  serialized_end=201,
)


_LVMDUSERINTERACTIONPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='LVMDUserInteractionPort_InterfaceVersion_array_port',
  full_name='pb.mf_hmih.lvmduser_interaction_port_interface_version.LVMDUserInteractionPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_hmih.lvmduser_interaction_port_interface_version.LVMDUserInteractionPort_InterfaceVersion_array_port.data', index=0,
      number=2277, type=11, cpp_type=10, label=3,
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
  serialized_end=370,
)

_LVMDUSERINTERACTIONPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _LVMDUSERINTERACTIONPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['LVMDUserInteractionPort_InterfaceVersion'] = _LVMDUSERINTERACTIONPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['LVMDUserInteractionPort_InterfaceVersion_array_port'] = _LVMDUSERINTERACTIONPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LVMDUserInteractionPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('LVMDUserInteractionPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _LVMDUSERINTERACTIONPORT_INTERFACEVERSION,
  '__module__' : 'mf_hmih.lvmduser_interaction_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.lvmduser_interaction_port_interface_version.LVMDUserInteractionPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(LVMDUserInteractionPort_InterfaceVersion)

LVMDUserInteractionPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('LVMDUserInteractionPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _LVMDUSERINTERACTIONPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'mf_hmih.lvmduser_interaction_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.lvmduser_interaction_port_interface_version.LVMDUserInteractionPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(LVMDUserInteractionPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
