# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_drvwarnsm/drv_warn_status_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_drvwarnsm/drv_warn_status_port_interface_version.proto',
  package='pb.mf_drvwarnsm.drv_warn_status_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n9mf_drvwarnsm/drv_warn_status_port_interface_version.proto\x12\x36pb.mf_drvwarnsm.drv_warn_status_port_interface_version\"H\n\"DrvWarnStatusPort_InterfaceVersion\x12\"\n\x19\x44rvWarnStatusPort_VERSION\x18\xb8\x1a \x01(\r\"\x9a\x01\n-DrvWarnStatusPort_InterfaceVersion_array_port\x12i\n\x04\x64\x61ta\x18\xcb\x11 \x03(\x0b\x32Z.pb.mf_drvwarnsm.drv_warn_status_port_interface_version.DrvWarnStatusPort_InterfaceVersion'
)




_DRVWARNSTATUSPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='DrvWarnStatusPort_InterfaceVersion',
  full_name='pb.mf_drvwarnsm.drv_warn_status_port_interface_version.DrvWarnStatusPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='DrvWarnStatusPort_VERSION', full_name='pb.mf_drvwarnsm.drv_warn_status_port_interface_version.DrvWarnStatusPort_InterfaceVersion.DrvWarnStatusPort_VERSION', index=0,
      number=3384, type=13, cpp_type=3, label=1,
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
  serialized_end=189,
)


_DRVWARNSTATUSPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='DrvWarnStatusPort_InterfaceVersion_array_port',
  full_name='pb.mf_drvwarnsm.drv_warn_status_port_interface_version.DrvWarnStatusPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_drvwarnsm.drv_warn_status_port_interface_version.DrvWarnStatusPort_InterfaceVersion_array_port.data', index=0,
      number=2251, type=11, cpp_type=10, label=3,
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
  serialized_start=192,
  serialized_end=346,
)

_DRVWARNSTATUSPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _DRVWARNSTATUSPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['DrvWarnStatusPort_InterfaceVersion'] = _DRVWARNSTATUSPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['DrvWarnStatusPort_InterfaceVersion_array_port'] = _DRVWARNSTATUSPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

DrvWarnStatusPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('DrvWarnStatusPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _DRVWARNSTATUSPORT_INTERFACEVERSION,
  '__module__' : 'mf_drvwarnsm.drv_warn_status_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_drvwarnsm.drv_warn_status_port_interface_version.DrvWarnStatusPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(DrvWarnStatusPort_InterfaceVersion)

DrvWarnStatusPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('DrvWarnStatusPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _DRVWARNSTATUSPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'mf_drvwarnsm.drv_warn_status_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_drvwarnsm.drv_warn_status_port_interface_version.DrvWarnStatusPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(DrvWarnStatusPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)