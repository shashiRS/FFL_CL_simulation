# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/trajectory_point.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from lsm_geoml import pose_pod_pb2 as lsm__geoml_dot_pose__pod__pb2
from ap_commonvehsigprovider import gpsdata_pb2 as ap__commonvehsigprovider_dot_gpsdata__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/trajectory_point.proto',
  package='pb.mf_mempark.trajectory_point',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n!mf_mempark/trajectory_point.proto\x12\x1epb.mf_mempark.trajectory_point\x1a\x18lsm_geoml/pose_pod.proto\x1a%ap_commonvehsigprovider/gpsdata.proto\"\xaf\x01\n\x0fTrajectoryPoint\x12\x10\n\x07pointID\x18\x8b\x01 \x01(\r\x12\x33\n\tpointPose\x18\xdb\x10 \x01(\x0b\x32\x1f.pb.lsm_geoml.pose_pod.Pose_POD\x12?\n\tpointGNSS\x18\xd0\x02 \x01(\x0b\x32+.pb.ap_commonvehsigprovider.gpsdata.GPSData\x12\x14\n\x0bisGNSSValid\x18\xa2\x11 \x01(\x08\"\\\n\x1aTrajectoryPoint_array_port\x12>\n\x04\x64\x61ta\x18\xd0\x15 \x03(\x0b\x32/.pb.mf_mempark.trajectory_point.TrajectoryPoint'
  ,
  dependencies=[lsm__geoml_dot_pose__pod__pb2.DESCRIPTOR,ap__commonvehsigprovider_dot_gpsdata__pb2.DESCRIPTOR,])




_TRAJECTORYPOINT = _descriptor.Descriptor(
  name='TrajectoryPoint',
  full_name='pb.mf_mempark.trajectory_point.TrajectoryPoint',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='pointID', full_name='pb.mf_mempark.trajectory_point.TrajectoryPoint.pointID', index=0,
      number=139, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pointPose', full_name='pb.mf_mempark.trajectory_point.TrajectoryPoint.pointPose', index=1,
      number=2139, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pointGNSS', full_name='pb.mf_mempark.trajectory_point.TrajectoryPoint.pointGNSS', index=2,
      number=336, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isGNSSValid', full_name='pb.mf_mempark.trajectory_point.TrajectoryPoint.isGNSSValid', index=3,
      number=2210, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
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
  serialized_end=310,
)


_TRAJECTORYPOINT_ARRAY_PORT = _descriptor.Descriptor(
  name='TrajectoryPoint_array_port',
  full_name='pb.mf_mempark.trajectory_point.TrajectoryPoint_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_mempark.trajectory_point.TrajectoryPoint_array_port.data', index=0,
      number=2768, type=11, cpp_type=10, label=3,
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
  serialized_start=312,
  serialized_end=404,
)

_TRAJECTORYPOINT.fields_by_name['pointPose'].message_type = lsm__geoml_dot_pose__pod__pb2._POSE_POD
_TRAJECTORYPOINT.fields_by_name['pointGNSS'].message_type = ap__commonvehsigprovider_dot_gpsdata__pb2._GPSDATA
_TRAJECTORYPOINT_ARRAY_PORT.fields_by_name['data'].message_type = _TRAJECTORYPOINT
DESCRIPTOR.message_types_by_name['TrajectoryPoint'] = _TRAJECTORYPOINT
DESCRIPTOR.message_types_by_name['TrajectoryPoint_array_port'] = _TRAJECTORYPOINT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TrajectoryPoint = _reflection.GeneratedProtocolMessageType('TrajectoryPoint', (_message.Message,), {
  'DESCRIPTOR' : _TRAJECTORYPOINT,
  '__module__' : 'mf_mempark.trajectory_point_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.trajectory_point.TrajectoryPoint)
  })
_sym_db.RegisterMessage(TrajectoryPoint)

TrajectoryPoint_array_port = _reflection.GeneratedProtocolMessageType('TrajectoryPoint_array_port', (_message.Message,), {
  'DESCRIPTOR' : _TRAJECTORYPOINT_ARRAY_PORT,
  '__module__' : 'mf_mempark.trajectory_point_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.trajectory_point.TrajectoryPoint_array_port)
  })
_sym_db.RegisterMessage(TrajectoryPoint_array_port)


# @@protoc_insertion_point(module_scope)