# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/parking_trajectory_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/parking_trajectory_interface_version.proto',
  package='pb.mf_mempark.parking_trajectory_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n5mf_mempark/parking_trajectory_interface_version.proto\x12\x32pb.mf_mempark.parking_trajectory_interface_version\"H\n\"ParkingTrajectory_InterfaceVersion\x12\"\n\x19ParkingTrajectory_VERSION\x18\x97\x04 \x01(\r\"\x96\x01\n-ParkingTrajectory_InterfaceVersion_array_port\x12\x65\n\x04\x64\x61ta\x18\xd3\x0c \x03(\x0b\x32V.pb.mf_mempark.parking_trajectory_interface_version.ParkingTrajectory_InterfaceVersion'
)




_PARKINGTRAJECTORY_INTERFACEVERSION = _descriptor.Descriptor(
  name='ParkingTrajectory_InterfaceVersion',
  full_name='pb.mf_mempark.parking_trajectory_interface_version.ParkingTrajectory_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ParkingTrajectory_VERSION', full_name='pb.mf_mempark.parking_trajectory_interface_version.ParkingTrajectory_InterfaceVersion.ParkingTrajectory_VERSION', index=0,
      number=535, type=13, cpp_type=3, label=1,
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
  serialized_start=109,
  serialized_end=181,
)


_PARKINGTRAJECTORY_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='ParkingTrajectory_InterfaceVersion_array_port',
  full_name='pb.mf_mempark.parking_trajectory_interface_version.ParkingTrajectory_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_mempark.parking_trajectory_interface_version.ParkingTrajectory_InterfaceVersion_array_port.data', index=0,
      number=1619, type=11, cpp_type=10, label=3,
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
  serialized_start=184,
  serialized_end=334,
)

_PARKINGTRAJECTORY_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _PARKINGTRAJECTORY_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['ParkingTrajectory_InterfaceVersion'] = _PARKINGTRAJECTORY_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['ParkingTrajectory_InterfaceVersion_array_port'] = _PARKINGTRAJECTORY_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ParkingTrajectory_InterfaceVersion = _reflection.GeneratedProtocolMessageType('ParkingTrajectory_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGTRAJECTORY_INTERFACEVERSION,
  '__module__' : 'mf_mempark.parking_trajectory_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.parking_trajectory_interface_version.ParkingTrajectory_InterfaceVersion)
  })
_sym_db.RegisterMessage(ParkingTrajectory_InterfaceVersion)

ParkingTrajectory_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('ParkingTrajectory_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGTRAJECTORY_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'mf_mempark.parking_trajectory_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.parking_trajectory_interface_version.ParkingTrajectory_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(ParkingTrajectory_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)