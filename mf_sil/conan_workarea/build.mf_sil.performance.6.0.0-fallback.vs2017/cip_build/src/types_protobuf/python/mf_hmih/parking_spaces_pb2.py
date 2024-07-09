# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/parking_spaces.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from mf_hmih import parking_space_pb2 as mf__hmih_dot_parking__space__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/parking_spaces.proto',
  package='pb.mf_hmih.parking_spaces',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1cmf_hmih/parking_spaces.proto\x12\x19pb.mf_hmih.parking_spaces\x1a\x1bmf_hmih/parking_space.proto\"\xec\x01\n\rParkingSpaces\x12\x35\n\x04left\x18\xc2\x08 \x01(\x0b\x32&.pb.mf_hmih.parking_space.ParkingSpace\x12\x35\n\x05right\x18\x35 \x01(\x0b\x32&.pb.mf_hmih.parking_space.ParkingSpace\x12\x36\n\x05\x66ront\x18\xd6\x17 \x01(\x0b\x32&.pb.mf_hmih.parking_space.ParkingSpace\x12\x35\n\x04rear\x18\xee\t \x01(\x0b\x32&.pb.mf_hmih.parking_space.ParkingSpace\"S\n\x18ParkingSpaces_array_port\x12\x37\n\x04\x64\x61ta\x18\xfa\x01 \x03(\x0b\x32(.pb.mf_hmih.parking_spaces.ParkingSpaces'
  ,
  dependencies=[mf__hmih_dot_parking__space__pb2.DESCRIPTOR,])




_PARKINGSPACES = _descriptor.Descriptor(
  name='ParkingSpaces',
  full_name='pb.mf_hmih.parking_spaces.ParkingSpaces',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='left', full_name='pb.mf_hmih.parking_spaces.ParkingSpaces.left', index=0,
      number=1090, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='right', full_name='pb.mf_hmih.parking_spaces.ParkingSpaces.right', index=1,
      number=53, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='front', full_name='pb.mf_hmih.parking_spaces.ParkingSpaces.front', index=2,
      number=3030, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rear', full_name='pb.mf_hmih.parking_spaces.ParkingSpaces.rear', index=3,
      number=1262, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
  serialized_start=89,
  serialized_end=325,
)


_PARKINGSPACES_ARRAY_PORT = _descriptor.Descriptor(
  name='ParkingSpaces_array_port',
  full_name='pb.mf_hmih.parking_spaces.ParkingSpaces_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_hmih.parking_spaces.ParkingSpaces_array_port.data', index=0,
      number=250, type=11, cpp_type=10, label=3,
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
  serialized_start=327,
  serialized_end=410,
)

_PARKINGSPACES.fields_by_name['left'].message_type = mf__hmih_dot_parking__space__pb2._PARKINGSPACE
_PARKINGSPACES.fields_by_name['right'].message_type = mf__hmih_dot_parking__space__pb2._PARKINGSPACE
_PARKINGSPACES.fields_by_name['front'].message_type = mf__hmih_dot_parking__space__pb2._PARKINGSPACE
_PARKINGSPACES.fields_by_name['rear'].message_type = mf__hmih_dot_parking__space__pb2._PARKINGSPACE
_PARKINGSPACES_ARRAY_PORT.fields_by_name['data'].message_type = _PARKINGSPACES
DESCRIPTOR.message_types_by_name['ParkingSpaces'] = _PARKINGSPACES
DESCRIPTOR.message_types_by_name['ParkingSpaces_array_port'] = _PARKINGSPACES_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ParkingSpaces = _reflection.GeneratedProtocolMessageType('ParkingSpaces', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGSPACES,
  '__module__' : 'mf_hmih.parking_spaces_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.parking_spaces.ParkingSpaces)
  })
_sym_db.RegisterMessage(ParkingSpaces)

ParkingSpaces_array_port = _reflection.GeneratedProtocolMessageType('ParkingSpaces_array_port', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGSPACES_ARRAY_PORT,
  '__module__' : 'mf_hmih.parking_spaces_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.parking_spaces.ParkingSpaces_array_port)
  })
_sym_db.RegisterMessage(ParkingSpaces_array_port)


# @@protoc_insertion_point(module_scope)
