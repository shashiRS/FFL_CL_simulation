# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/trajectory_meta_data.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from mf_mempark import mem_park_date_t_pb2 as mf__mempark_dot_mem__park__date__t__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/trajectory_meta_data.proto',
  package='pb.mf_mempark.trajectory_meta_data',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n%mf_mempark/trajectory_meta_data.proto\x12\"pb.mf_mempark.trajectory_meta_data\x1a mf_mempark/mem_park_date_t.proto\"t\n\x12TrajectoryMetaData\x12I\n\x12trajectorySaveDate\x18\xc1\x0c \x01(\x0b\x32,.pb.mf_mempark.mem_park_date_t.MemParkDate_t\x12\x13\n\negoVehicle\x18\x85\x05 \x01(\r\"f\n\x1dTrajectoryMetaData_array_port\x12\x45\n\x04\x64\x61ta\x18\x9b\x14 \x03(\x0b\x32\x36.pb.mf_mempark.trajectory_meta_data.TrajectoryMetaData'
  ,
  dependencies=[mf__mempark_dot_mem__park__date__t__pb2.DESCRIPTOR,])




_TRAJECTORYMETADATA = _descriptor.Descriptor(
  name='TrajectoryMetaData',
  full_name='pb.mf_mempark.trajectory_meta_data.TrajectoryMetaData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='trajectorySaveDate', full_name='pb.mf_mempark.trajectory_meta_data.TrajectoryMetaData.trajectorySaveDate', index=0,
      number=1601, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='egoVehicle', full_name='pb.mf_mempark.trajectory_meta_data.TrajectoryMetaData.egoVehicle', index=1,
      number=645, type=13, cpp_type=3, label=1,
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
  serialized_start=111,
  serialized_end=227,
)


_TRAJECTORYMETADATA_ARRAY_PORT = _descriptor.Descriptor(
  name='TrajectoryMetaData_array_port',
  full_name='pb.mf_mempark.trajectory_meta_data.TrajectoryMetaData_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_mempark.trajectory_meta_data.TrajectoryMetaData_array_port.data', index=0,
      number=2587, type=11, cpp_type=10, label=3,
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
  serialized_start=229,
  serialized_end=331,
)

_TRAJECTORYMETADATA.fields_by_name['trajectorySaveDate'].message_type = mf__mempark_dot_mem__park__date__t__pb2._MEMPARKDATE_T
_TRAJECTORYMETADATA_ARRAY_PORT.fields_by_name['data'].message_type = _TRAJECTORYMETADATA
DESCRIPTOR.message_types_by_name['TrajectoryMetaData'] = _TRAJECTORYMETADATA
DESCRIPTOR.message_types_by_name['TrajectoryMetaData_array_port'] = _TRAJECTORYMETADATA_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TrajectoryMetaData = _reflection.GeneratedProtocolMessageType('TrajectoryMetaData', (_message.Message,), {
  'DESCRIPTOR' : _TRAJECTORYMETADATA,
  '__module__' : 'mf_mempark.trajectory_meta_data_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.trajectory_meta_data.TrajectoryMetaData)
  })
_sym_db.RegisterMessage(TrajectoryMetaData)

TrajectoryMetaData_array_port = _reflection.GeneratedProtocolMessageType('TrajectoryMetaData_array_port', (_message.Message,), {
  'DESCRIPTOR' : _TRAJECTORYMETADATA_ARRAY_PORT,
  '__module__' : 'mf_mempark.trajectory_meta_data_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.trajectory_meta_data.TrajectoryMetaData_array_port)
  })
_sym_db.RegisterMessage(TrajectoryMetaData_array_port)


# @@protoc_insertion_point(module_scope)
