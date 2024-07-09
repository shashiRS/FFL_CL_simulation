# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_manager/mfmplanned_traj.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_manager/mfmplanned_traj.proto',
  package='pb.mf_manager.mfmplanned_traj',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n mf_manager/mfmplanned_traj.proto\x12\x1dpb.mf_manager.mfmplanned_traj\"\xa7\x01\n\x0eMFMPlannedTraj\x12\x15\n\x0cxTrajRAReq_m\x18\x91\x0e \x01(\x02\x12\x15\n\x0cyTrajRAReq_m\x18\xe4\x08 \x01(\x02\x12\x13\n\nyawReq_rad\x18\xe3\n \x01(\x02\x12\x15\n\x0c\x63rvRAReq_1pm\x18\xb4\r \x01(\x02\x12\x1c\n\x13\x64istanceToStopReq_m\x18\xfa\x19 \x01(\x02\x12\x1d\n\x14velocityLimitReq_mps\x18\xd1\x13 \x01(\x02\"Y\n\x19MFMPlannedTraj_array_port\x12<\n\x04\x64\x61ta\x18\x96\x17 \x03(\x0b\x32-.pb.mf_manager.mfmplanned_traj.MFMPlannedTraj'
)




_MFMPLANNEDTRAJ = _descriptor.Descriptor(
  name='MFMPlannedTraj',
  full_name='pb.mf_manager.mfmplanned_traj.MFMPlannedTraj',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='xTrajRAReq_m', full_name='pb.mf_manager.mfmplanned_traj.MFMPlannedTraj.xTrajRAReq_m', index=0,
      number=1809, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='yTrajRAReq_m', full_name='pb.mf_manager.mfmplanned_traj.MFMPlannedTraj.yTrajRAReq_m', index=1,
      number=1124, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='yawReq_rad', full_name='pb.mf_manager.mfmplanned_traj.MFMPlannedTraj.yawReq_rad', index=2,
      number=1379, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='crvRAReq_1pm', full_name='pb.mf_manager.mfmplanned_traj.MFMPlannedTraj.crvRAReq_1pm', index=3,
      number=1716, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='distanceToStopReq_m', full_name='pb.mf_manager.mfmplanned_traj.MFMPlannedTraj.distanceToStopReq_m', index=4,
      number=3322, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='velocityLimitReq_mps', full_name='pb.mf_manager.mfmplanned_traj.MFMPlannedTraj.velocityLimitReq_mps', index=5,
      number=2513, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
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
  serialized_start=68,
  serialized_end=235,
)


_MFMPLANNEDTRAJ_ARRAY_PORT = _descriptor.Descriptor(
  name='MFMPlannedTraj_array_port',
  full_name='pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port.data', index=0,
      number=2966, type=11, cpp_type=10, label=3,
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
  serialized_end=326,
)

_MFMPLANNEDTRAJ_ARRAY_PORT.fields_by_name['data'].message_type = _MFMPLANNEDTRAJ
DESCRIPTOR.message_types_by_name['MFMPlannedTraj'] = _MFMPLANNEDTRAJ
DESCRIPTOR.message_types_by_name['MFMPlannedTraj_array_port'] = _MFMPLANNEDTRAJ_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MFMPlannedTraj = _reflection.GeneratedProtocolMessageType('MFMPlannedTraj', (_message.Message,), {
  'DESCRIPTOR' : _MFMPLANNEDTRAJ,
  '__module__' : 'mf_manager.mfmplanned_traj_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj)
  })
_sym_db.RegisterMessage(MFMPlannedTraj)

MFMPlannedTraj_array_port = _reflection.GeneratedProtocolMessageType('MFMPlannedTraj_array_port', (_message.Message,), {
  'DESCRIPTOR' : _MFMPLANNEDTRAJ_ARRAY_PORT,
  '__module__' : 'mf_manager.mfmplanned_traj_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port)
  })
_sym_db.RegisterMessage(MFMPlannedTraj_array_port)


# @@protoc_insertion_point(module_scope)
