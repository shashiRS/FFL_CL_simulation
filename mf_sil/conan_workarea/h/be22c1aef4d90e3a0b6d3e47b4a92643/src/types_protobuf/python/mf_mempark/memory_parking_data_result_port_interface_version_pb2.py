# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/memory_parking_data_result_port_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/memory_parking_data_result_port_interface_version.proto',
  package='pb.mf_mempark.memory_parking_data_result_port_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\nBmf_mempark/memory_parking_data_result_port_interface_version.proto\x12?pb.mf_mempark.memory_parking_data_result_port_interface_version\"\\\n,MemoryParkingDataResultPort_InterfaceVersion\x12,\n#MemoryParkingDataResultPort_VERSION\x18\xa0\x08 \x01(\r\"\xb7\x01\n7MemoryParkingDataResultPort_InterfaceVersion_array_port\x12|\n\x04\x64\x61ta\x18\x82\x1b \x03(\x0b\x32m.pb.mf_mempark.memory_parking_data_result_port_interface_version.MemoryParkingDataResultPort_InterfaceVersion'
)




_MEMORYPARKINGDATARESULTPORT_INTERFACEVERSION = _descriptor.Descriptor(
  name='MemoryParkingDataResultPort_InterfaceVersion',
  full_name='pb.mf_mempark.memory_parking_data_result_port_interface_version.MemoryParkingDataResultPort_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='MemoryParkingDataResultPort_VERSION', full_name='pb.mf_mempark.memory_parking_data_result_port_interface_version.MemoryParkingDataResultPort_InterfaceVersion.MemoryParkingDataResultPort_VERSION', index=0,
      number=1056, type=13, cpp_type=3, label=1,
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
  serialized_start=135,
  serialized_end=227,
)


_MEMORYPARKINGDATARESULTPORT_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='MemoryParkingDataResultPort_InterfaceVersion_array_port',
  full_name='pb.mf_mempark.memory_parking_data_result_port_interface_version.MemoryParkingDataResultPort_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_mempark.memory_parking_data_result_port_interface_version.MemoryParkingDataResultPort_InterfaceVersion_array_port.data', index=0,
      number=3458, type=11, cpp_type=10, label=3,
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
  serialized_start=230,
  serialized_end=413,
)

_MEMORYPARKINGDATARESULTPORT_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _MEMORYPARKINGDATARESULTPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['MemoryParkingDataResultPort_InterfaceVersion'] = _MEMORYPARKINGDATARESULTPORT_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['MemoryParkingDataResultPort_InterfaceVersion_array_port'] = _MEMORYPARKINGDATARESULTPORT_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MemoryParkingDataResultPort_InterfaceVersion = _reflection.GeneratedProtocolMessageType('MemoryParkingDataResultPort_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _MEMORYPARKINGDATARESULTPORT_INTERFACEVERSION,
  '__module__' : 'mf_mempark.memory_parking_data_result_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.memory_parking_data_result_port_interface_version.MemoryParkingDataResultPort_InterfaceVersion)
  })
_sym_db.RegisterMessage(MemoryParkingDataResultPort_InterfaceVersion)

MemoryParkingDataResultPort_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('MemoryParkingDataResultPort_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _MEMORYPARKINGDATARESULTPORT_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'mf_mempark.memory_parking_data_result_port_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.memory_parking_data_result_port_interface_version.MemoryParkingDataResultPort_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(MemoryParkingDataResultPort_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)
