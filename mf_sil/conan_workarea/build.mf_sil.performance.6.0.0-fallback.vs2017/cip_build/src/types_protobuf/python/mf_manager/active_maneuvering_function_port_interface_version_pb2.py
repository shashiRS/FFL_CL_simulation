# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_manager/active_maneuvering_function_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_manager/active_maneuvering_function_port_interface_version.proto',
  package='pb.mf_manager.active_maneuvering_function_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\nCmf_manager/active_maneuvering_function_port_interface_version.proto\x12@pb.mf_manager.active_maneuvering_function_port_interface_version\"`\n.ActiveManeuveringFunctionPort_InterfaceVersion\x12.\n%ActiveManeuveringFunctionPort_VERSION\x18\xe2\x01 \x01(\r\"\xbc\x01\n9ActiveManeuveringFunctionPort_InterfaceVersion_array_port\x12\x7f\n\x04\x64\x61ta\x18\xd1\n \x03(\x0b\x32p.pb.mf_manager.active_maneuvering_function_port_interface_version.ActiveManeuveringFunctionPort_InterfaceVersion'
)




_ACTIVEMANEUVERINGFUNCTIONPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='ActiveManeuveringFunctionPort_InterfaceVersion',
  full_name='pb.mf_manager.active_maneuvering_function_port_interface_version.ActiveManeuveringFunctionPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ActiveManeuveringFunctionPort_VERSION', full_name='pb.mf_manager.active_maneuvering_function_port_interface_version.ActiveManeuveringFunctionPort_InterfaceVersion.ActiveManeuveringFunctionPort_VERSION', index=0,
      number=226, type=13, cpp_type=3, label=1,
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
  serialized_start=137,
  serialized_end=233,
)


_ACTIVEMANEUVERINGFUNCTIONPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='ActiveManeuveringFunctionPort_InterfaceVersion_array_port',
  full_name='pb.mf_manager.active_maneuvering_function_port_interface_version.ActiveManeuveringFunctionPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_manager.active_maneuvering_function_port_interface_version.ActiveManeuveringFunctionPort_InterfaceVersion_array_port.data', index=0,
      number=1361, type=11, cpp_type=10, label=3,
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
  serialized_start=236,
  serialized_end=424,
)

_ACTIVEMANEUVERINGFUNCTIONPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _ACTIVEMANEUVERINGFUNCTIONPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['ActiveManeuveringFunctionPort_InterfaceVersion'] = _ACTIVEMANEUVERINGFUNCTIONPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['ActiveManeuveringFunctionPort_InterfaceVersion_array_port'] = _ACTIVEMANEUVERINGFUNCTIONPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ActiveManeuveringFunctionPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('ActiveManeuveringFunctionPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _ACTIVEMANEUVERINGFUNCTIONPORT_INTERFACEVERSION,
  '__module__' : 'mf_manager.active_maneuvering_function_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_manager.active_maneuvering_function_port_interface_version.ActiveManeuveringFunctionPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(ActiveManeuveringFunctionPort_InterfaceVersion)

ActiveManeuveringFunctionPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('ActiveManeuveringFunctionPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _ACTIVEMANEUVERINGFUNCTIONPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'mf_manager.active_maneuvering_function_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_manager.active_maneuvering_function_port_interface_version.ActiveManeuveringFunctionPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(ActiveManeuveringFunctionPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)