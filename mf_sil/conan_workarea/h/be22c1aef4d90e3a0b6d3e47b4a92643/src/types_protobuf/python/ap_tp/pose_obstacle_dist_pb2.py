# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_tp/pose_obstacle_dist.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_tp/pose_obstacle_dist.proto',
  package='pb.ap_tp.pose_obstacle_dist',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1e\x61p_tp/pose_obstacle_dist.proto\x12\x1bpb.ap_tp.pose_obstacle_dist\"\xea\x01\n\x10PoseObstacleDist\x12\x18\n\x0f\x66rontObstDist_m\x18\xe5\x03 \x01(\x02\x12\x1b\n\x12\x66rontObstDistValid\x18\xa2\x0e \x01(\x08\x12\x17\n\x0erearObstDist_m\x18\xd5\x1a \x01(\x02\x12\x1a\n\x11rearObstDistValid\x18\x94\x15 \x01(\x08\x12\x17\n\x0eleftObstDist_m\x18\x91\x04 \x01(\x02\x12\x1a\n\x11leftObstDistValid\x18\xb9\x04 \x01(\x08\x12\x18\n\x0frightObstDist_m\x18\x86\t \x01(\x02\x12\x1b\n\x12rightObstDistValid\x18\xa3\x15 \x01(\x08\"[\n\x1bPoseObstacleDist_array_port\x12<\n\x04\x64\x61ta\x18\xe2\x0c \x03(\x0b\x32-.pb.ap_tp.pose_obstacle_dist.PoseObstacleDist'
)




_POSEOBSTACLEDIST = _descriptor.Descriptor(
  name='PoseObstacleDist',
  full_name='pb.ap_tp.pose_obstacle_dist.PoseObstacleDist',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='frontObstDist_m', full_name='pb.ap_tp.pose_obstacle_dist.PoseObstacleDist.frontObstDist_m', index=0,
      number=485, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='frontObstDistValid', full_name='pb.ap_tp.pose_obstacle_dist.PoseObstacleDist.frontObstDistValid', index=1,
      number=1826, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rearObstDist_m', full_name='pb.ap_tp.pose_obstacle_dist.PoseObstacleDist.rearObstDist_m', index=2,
      number=3413, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rearObstDistValid', full_name='pb.ap_tp.pose_obstacle_dist.PoseObstacleDist.rearObstDistValid', index=3,
      number=2708, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='leftObstDist_m', full_name='pb.ap_tp.pose_obstacle_dist.PoseObstacleDist.leftObstDist_m', index=4,
      number=529, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='leftObstDistValid', full_name='pb.ap_tp.pose_obstacle_dist.PoseObstacleDist.leftObstDistValid', index=5,
      number=569, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rightObstDist_m', full_name='pb.ap_tp.pose_obstacle_dist.PoseObstacleDist.rightObstDist_m', index=6,
      number=1158, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rightObstDistValid', full_name='pb.ap_tp.pose_obstacle_dist.PoseObstacleDist.rightObstDistValid', index=7,
      number=2723, type=8, cpp_type=7, label=1,
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
  serialized_start=64,
  serialized_end=298,
)


_POSEOBSTACLEDIST_ARRAY_PORT = _descriptor.Descriptor(
  name='PoseObstacleDist_array_port',
  full_name='pb.ap_tp.pose_obstacle_dist.PoseObstacleDist_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_tp.pose_obstacle_dist.PoseObstacleDist_array_port.data', index=0,
      number=1634, type=11, cpp_type=10, label=3,
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
  serialized_start=300,
  serialized_end=391,
)

_POSEOBSTACLEDIST_ARRAY_PORT.fields_by_name['data'].message_type = _POSEOBSTACLEDIST
DESCRIPTOR.message_types_by_name['PoseObstacleDist'] = _POSEOBSTACLEDIST
DESCRIPTOR.message_types_by_name['PoseObstacleDist_array_port'] = _POSEOBSTACLEDIST_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PoseObstacleDist = _reflection.GeneratedProtocolMessageType('PoseObstacleDist', (_message.Message,), {
  'DESCRIPTOR' : _POSEOBSTACLEDIST,
  '__module__' : 'ap_tp.pose_obstacle_dist_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.pose_obstacle_dist.PoseObstacleDist)
  })
_sym_db.RegisterMessage(PoseObstacleDist)

PoseObstacleDist_array_port = _reflection.GeneratedProtocolMessageType('PoseObstacleDist_array_port', (_message.Message,), {
  'DESCRIPTOR' : _POSEOBSTACLEDIST_ARRAY_PORT,
  '__module__' : 'ap_tp.pose_obstacle_dist_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.pose_obstacle_dist.PoseObstacleDist_array_port)
  })
_sym_db.RegisterMessage(PoseObstacleDist_array_port)


# @@protoc_insertion_point(module_scope)
