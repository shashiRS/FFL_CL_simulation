# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/parking_trajectory.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from lsm_geoml import pose_pod_pb2 as lsm__geoml_dot_pose__pod__pb2
from mf_mempark import trajectory_point_pb2 as mf__mempark_dot_trajectory__point__pb2
from mf_mempark import trajectory_meta_data_pb2 as mf__mempark_dot_trajectory__meta__data__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/parking_trajectory.proto',
  package='pb.mf_mempark.parking_trajectory',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n#mf_mempark/parking_trajectory.proto\x12 pb.mf_mempark.parking_trajectory\x1a\x17\x65\x63o/signal_header.proto\x1a\x18lsm_geoml/pose_pod.proto\x1a!mf_mempark/trajectory_point.proto\x1a%mf_mempark/trajectory_meta_data.proto\"\xa5\x03\n\x11ParkingTrajectory\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x14\n\x0ctrajectoryID\x18V \x01(\r\x12\x33\n\tstartPose\x18\xce\x19 \x01(\x0b\x32\x1f.pb.lsm_geoml.pose_pod.Pose_POD\x12\x31\n\x07\x65ndPose\x18\xf2\x14 \x01(\x0b\x32\x1f.pb.lsm_geoml.pose_pod.Pose_POD\x12\x1b\n\x12numValidTrajPoints\x18\xdc\x19 \x01(\r\x12\x46\n\x0clistOfPoints\x18\xf2\x1a \x03(\x0b\x32/.pb.mf_mempark.trajectory_point.TrajectoryPoint\x12\x0f\n\x06slotID\x18\xd0\x01 \x01(\r\x12I\n\x08metaData\x18\x88\x18 \x01(\x0b\x32\x36.pb.mf_mempark.trajectory_meta_data.TrajectoryMetaData\"b\n\x1cParkingTrajectory_array_port\x12\x42\n\x04\x64\x61ta\x18\xb0\x08 \x03(\x0b\x32\x33.pb.mf_mempark.parking_trajectory.ParkingTrajectory'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,lsm__geoml_dot_pose__pod__pb2.DESCRIPTOR,mf__mempark_dot_trajectory__point__pb2.DESCRIPTOR,mf__mempark_dot_trajectory__meta__data__pb2.DESCRIPTOR,])




_PARKINGTRAJECTORY = _descriptor.Descriptor(
  name='ParkingTrajectory',
  full_name='pb.mf_mempark.parking_trajectory.ParkingTrajectory',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.mf_mempark.parking_trajectory.ParkingTrajectory.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.mf_mempark.parking_trajectory.ParkingTrajectory.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='trajectoryID', full_name='pb.mf_mempark.parking_trajectory.ParkingTrajectory.trajectoryID', index=2,
      number=86, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='startPose', full_name='pb.mf_mempark.parking_trajectory.ParkingTrajectory.startPose', index=3,
      number=3278, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='endPose', full_name='pb.mf_mempark.parking_trajectory.ParkingTrajectory.endPose', index=4,
      number=2674, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numValidTrajPoints', full_name='pb.mf_mempark.parking_trajectory.ParkingTrajectory.numValidTrajPoints', index=5,
      number=3292, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='listOfPoints', full_name='pb.mf_mempark.parking_trajectory.ParkingTrajectory.listOfPoints', index=6,
      number=3442, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='slotID', full_name='pb.mf_mempark.parking_trajectory.ParkingTrajectory.slotID', index=7,
      number=208, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='metaData', full_name='pb.mf_mempark.parking_trajectory.ParkingTrajectory.metaData', index=8,
      number=3080, type=11, cpp_type=10, label=1,
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
  serialized_start=199,
  serialized_end=620,
)


_PARKINGTRAJECTORY_ARRAY_PORT = _descriptor.Descriptor(
  name='ParkingTrajectory_array_port',
  full_name='pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port.data', index=0,
      number=1072, type=11, cpp_type=10, label=3,
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
  serialized_start=622,
  serialized_end=720,
)

_PARKINGTRAJECTORY.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_PARKINGTRAJECTORY.fields_by_name['startPose'].message_type = lsm__geoml_dot_pose__pod__pb2._POSE_POD
_PARKINGTRAJECTORY.fields_by_name['endPose'].message_type = lsm__geoml_dot_pose__pod__pb2._POSE_POD
_PARKINGTRAJECTORY.fields_by_name['listOfPoints'].message_type = mf__mempark_dot_trajectory__point__pb2._TRAJECTORYPOINT
_PARKINGTRAJECTORY.fields_by_name['metaData'].message_type = mf__mempark_dot_trajectory__meta__data__pb2._TRAJECTORYMETADATA
_PARKINGTRAJECTORY_ARRAY_PORT.fields_by_name['data'].message_type = _PARKINGTRAJECTORY
DESCRIPTOR.message_types_by_name['ParkingTrajectory'] = _PARKINGTRAJECTORY
DESCRIPTOR.message_types_by_name['ParkingTrajectory_array_port'] = _PARKINGTRAJECTORY_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ParkingTrajectory = _reflection.GeneratedProtocolMessageType('ParkingTrajectory', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGTRAJECTORY,
  '__module__' : 'mf_mempark.parking_trajectory_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.parking_trajectory.ParkingTrajectory)
  })
_sym_db.RegisterMessage(ParkingTrajectory)

ParkingTrajectory_array_port = _reflection.GeneratedProtocolMessageType('ParkingTrajectory_array_port', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGTRAJECTORY_ARRAY_PORT,
  '__module__' : 'mf_mempark.parking_trajectory_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port)
  })
_sym_db.RegisterMessage(ParkingTrajectory_array_port)


# @@protoc_insertion_point(module_scope)
