# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_common/vehicle_params_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_common/vehicle_params_interface_version.proto',
  package='pb.ap_common.vehicle_params_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n0ap_common/vehicle_params_interface_version.proto\x12-pb.ap_common.vehicle_params_interface_version\"B\n\x1fVehicle_Params_InterfaceVersion\x12\x1f\n\x16Vehicle_Params_VERSION\x18\xdf\x18 \x01(\r\"\x8b\x01\n*Vehicle_Params_InterfaceVersion_array_port\x12]\n\x04\x64\x61ta\x18\xe2\x0e \x03(\x0b\x32N.pb.ap_common.vehicle_params_interface_version.Vehicle_Params_InterfaceVersion'
)




_VEHICLE_PARAMS_INTERFACEVERSION = _descriptor.Descriptor(
  name='Vehicle_Params_InterfaceVersion',
  full_name='pb.ap_common.vehicle_params_interface_version.Vehicle_Params_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='Vehicle_Params_VERSION', full_name='pb.ap_common.vehicle_params_interface_version.Vehicle_Params_InterfaceVersion.Vehicle_Params_VERSION', index=0,
      number=3167, type=13, cpp_type=3, label=1,
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
  serialized_start=99,
  serialized_end=165,
)


_VEHICLE_PARAMS_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='Vehicle_Params_InterfaceVersion_array_port',
  full_name='pb.ap_common.vehicle_params_interface_version.Vehicle_Params_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_common.vehicle_params_interface_version.Vehicle_Params_InterfaceVersion_array_port.data', index=0,
      number=1890, type=11, cpp_type=10, label=3,
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
  serialized_start=168,
  serialized_end=307,
)

_VEHICLE_PARAMS_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _VEHICLE_PARAMS_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['Vehicle_Params_InterfaceVersion'] = _VEHICLE_PARAMS_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['Vehicle_Params_InterfaceVersion_array_port'] = _VEHICLE_PARAMS_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Vehicle_Params_InterfaceVersion = _reflection.GeneratedProtocolMessageType('Vehicle_Params_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _VEHICLE_PARAMS_INTERFACEVERSION,
  '__module__' : 'ap_common.vehicle_params_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_common.vehicle_params_interface_version.Vehicle_Params_InterfaceVersion)
  })
_sym_db.RegisterMessage(Vehicle_Params_InterfaceVersion)

Vehicle_Params_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('Vehicle_Params_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _VEHICLE_PARAMS_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'ap_common.vehicle_params_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_common.vehicle_params_interface_version.Vehicle_Params_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(Vehicle_Params_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)