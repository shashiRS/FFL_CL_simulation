# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_drvwarnsm_core/fc_drv_warn_smcore_params_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_drvwarnsm_core/fc_drv_warn_smcore_params_interface_version.proto',
  package='pb.mf_drvwarnsm_core.fc_drv_warn_smcore_params_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\nCmf_drvwarnsm_core/fc_drv_warn_smcore_params_interface_version.proto\x12@pb.mf_drvwarnsm_core.fc_drv_warn_smcore_params_interface_version\"T\n(FC_DrvWarnSMCore_Params_InterfaceVersion\x12(\n\x1f\x46\x43_DrvWarnSMCore_Params_VERSION\x18\x93\x06 \x01(\r\"\xb0\x01\n3FC_DrvWarnSMCore_Params_InterfaceVersion_array_port\x12y\n\x04\x64\x61ta\x18\xe6\x11 \x03(\x0b\x32j.pb.mf_drvwarnsm_core.fc_drv_warn_smcore_params_interface_version.FC_DrvWarnSMCore_Params_InterfaceVersion'
)




_FC_DRVWARNSMCORE_PARAMS_INTERFACEVERSION = _descriptor.Descriptor(
  name='FC_DrvWarnSMCore_Params_InterfaceVersion',
  full_name='pb.mf_drvwarnsm_core.fc_drv_warn_smcore_params_interface_version.FC_DrvWarnSMCore_Params_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='FC_DrvWarnSMCore_Params_VERSION', full_name='pb.mf_drvwarnsm_core.fc_drv_warn_smcore_params_interface_version.FC_DrvWarnSMCore_Params_InterfaceVersion.FC_DrvWarnSMCore_Params_VERSION', index=0,
      number=787, type=13, cpp_type=3, label=1,
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
  serialized_end=221,
)


_FC_DRVWARNSMCORE_PARAMS_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='FC_DrvWarnSMCore_Params_InterfaceVersion_array_port',
  full_name='pb.mf_drvwarnsm_core.fc_drv_warn_smcore_params_interface_version.FC_DrvWarnSMCore_Params_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_drvwarnsm_core.fc_drv_warn_smcore_params_interface_version.FC_DrvWarnSMCore_Params_InterfaceVersion_array_port.data', index=0,
      number=2278, type=11, cpp_type=10, label=3,
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
  serialized_start=224,
  serialized_end=400,
)

_FC_DRVWARNSMCORE_PARAMS_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _FC_DRVWARNSMCORE_PARAMS_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['FC_DrvWarnSMCore_Params_InterfaceVersion'] = _FC_DRVWARNSMCORE_PARAMS_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['FC_DrvWarnSMCore_Params_InterfaceVersion_array_port'] = _FC_DRVWARNSMCORE_PARAMS_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FC_DrvWarnSMCore_Params_InterfaceVersion = _reflection.GeneratedProtocolMessageType('FC_DrvWarnSMCore_Params_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _FC_DRVWARNSMCORE_PARAMS_INTERFACEVERSION,
  '__module__' : 'mf_drvwarnsm_core.fc_drv_warn_smcore_params_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_drvwarnsm_core.fc_drv_warn_smcore_params_interface_version.FC_DrvWarnSMCore_Params_InterfaceVersion)
  })
_sym_db.RegisterMessage(FC_DrvWarnSMCore_Params_InterfaceVersion)

FC_DrvWarnSMCore_Params_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('FC_DrvWarnSMCore_Params_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _FC_DRVWARNSMCORE_PARAMS_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'mf_drvwarnsm_core.fc_drv_warn_smcore_params_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_drvwarnsm_core.fc_drv_warn_smcore_params_interface_version.FC_DrvWarnSMCore_Params_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(FC_DrvWarnSMCore_Params_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
