# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/parking_target_poses.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from mf_hmih import parking_target_pose_pb2 as mf__hmih_dot_parking__target__pose__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/parking_target_poses.proto',
  package='pb.mf_hmih.parking_target_poses',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\"mf_hmih/parking_target_poses.proto\x12\x1fpb.mf_hmih.parking_target_poses\x1a!mf_hmih/parking_target_pose.proto\"\x80\x01\n\x12ParkingTargetPoses\x12 \n\x17numValidParkingPoses_nu\x18\xa5\x0b \x01(\r\x12H\n\x0cparkingPoses\x18\xda\x1a \x03(\x0b\x32\x31.pb.mf_hmih.parking_target_pose.ParkingTargetPose\"c\n\x1dParkingTargetPoses_array_port\x12\x42\n\x04\x64\x61ta\x18\x92\x1f \x03(\x0b\x32\x33.pb.mf_hmih.parking_target_poses.ParkingTargetPoses'
  ,
  dependencies=[mf__hmih_dot_parking__target__pose__pb2.DESCRIPTOR,])




_PARKINGTARGETPOSES = _descriptor.Descriptor(
  name='ParkingTargetPoses',
  full_name='pb.mf_hmih.parking_target_poses.ParkingTargetPoses',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='numValidParkingPoses_nu', full_name='pb.mf_hmih.parking_target_poses.ParkingTargetPoses.numValidParkingPoses_nu', index=0,
      number=1445, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='parkingPoses', full_name='pb.mf_hmih.parking_target_poses.ParkingTargetPoses.parkingPoses', index=1,
      number=3418, type=11, cpp_type=10, label=3,
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
  serialized_start=107,
  serialized_end=235,
)


_PARKINGTARGETPOSES_ARRAY_PORT = _descriptor.Descriptor(
  name='ParkingTargetPoses_array_port',
  full_name='pb.mf_hmih.parking_target_poses.ParkingTargetPoses_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_hmih.parking_target_poses.ParkingTargetPoses_array_port.data', index=0,
      number=3986, type=11, cpp_type=10, label=3,
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
  serialized_start=237,
  serialized_end=336,
)

_PARKINGTARGETPOSES.fields_by_name['parkingPoses'].message_type = mf__hmih_dot_parking__target__pose__pb2._PARKINGTARGETPOSE
_PARKINGTARGETPOSES_ARRAY_PORT.fields_by_name['data'].message_type = _PARKINGTARGETPOSES
DESCRIPTOR.message_types_by_name['ParkingTargetPoses'] = _PARKINGTARGETPOSES
DESCRIPTOR.message_types_by_name['ParkingTargetPoses_array_port'] = _PARKINGTARGETPOSES_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ParkingTargetPoses = _reflection.GeneratedProtocolMessageType('ParkingTargetPoses', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGTARGETPOSES,
  '__module__' : 'mf_hmih.parking_target_poses_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.parking_target_poses.ParkingTargetPoses)
  })
_sym_db.RegisterMessage(ParkingTargetPoses)

ParkingTargetPoses_array_port = _reflection.GeneratedProtocolMessageType('ParkingTargetPoses_array_port', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGTARGETPOSES_ARRAY_PORT,
  '__module__' : 'mf_hmih.parking_target_poses_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.parking_target_poses.ParkingTargetPoses_array_port)
  })
_sym_db.RegisterMessage(ParkingTargetPoses_array_port)


# @@protoc_insertion_point(module_scope)
