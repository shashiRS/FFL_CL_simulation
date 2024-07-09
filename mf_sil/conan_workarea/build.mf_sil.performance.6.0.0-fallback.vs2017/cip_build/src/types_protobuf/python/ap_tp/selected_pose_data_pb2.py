# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_tp/selected_pose_data.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ap_tp import pose_selection_status_pb2 as ap__tp_dot_pose__selection__status__pb2
from ap_tp import pose_reached_status_pb2 as ap__tp_dot_pose__reached__status__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_tp/selected_pose_data.proto',
  package='pb.ap_tp.selected_pose_data',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1e\x61p_tp/selected_pose_data.proto\x12\x1bpb.ap_tp.selected_pose_data\x1a!ap_tp/pose_selection_status.proto\x1a\x1f\x61p_tp/pose_reached_status.proto\"\xc6\x01\n\x10SelectedPoseData\x12M\n\x0fselectionStatus\x18\xa2\n \x01(\x0e\x32\x33.pb.ap_tp.pose_selection_status.PoseSelectionStatus\x12G\n\rreachedStatus\x18\x93\x07 \x01(\x0e\x32/.pb.ap_tp.pose_reached_status.PoseReachedStatus\x12\x1a\n\x11\x64istanceToStart_m\x18\xaa\x14 \x01(\x02\"[\n\x1bSelectedPoseData_array_port\x12<\n\x04\x64\x61ta\x18\xf1\x05 \x03(\x0b\x32-.pb.ap_tp.selected_pose_data.SelectedPoseData'
  ,
  dependencies=[ap__tp_dot_pose__selection__status__pb2.DESCRIPTOR,ap__tp_dot_pose__reached__status__pb2.DESCRIPTOR,])




_SELECTEDPOSEDATA = _descriptor.Descriptor(
  name='SelectedPoseData',
  full_name='pb.ap_tp.selected_pose_data.SelectedPoseData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='selectionStatus', full_name='pb.ap_tp.selected_pose_data.SelectedPoseData.selectionStatus', index=0,
      number=1314, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='reachedStatus', full_name='pb.ap_tp.selected_pose_data.SelectedPoseData.reachedStatus', index=1,
      number=915, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='distanceToStart_m', full_name='pb.ap_tp.selected_pose_data.SelectedPoseData.distanceToStart_m', index=2,
      number=2602, type=2, cpp_type=6, label=1,
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
  serialized_start=132,
  serialized_end=330,
)


_SELECTEDPOSEDATA_ARRAY_PORT = _descriptor.Descriptor(
  name='SelectedPoseData_array_port',
  full_name='pb.ap_tp.selected_pose_data.SelectedPoseData_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_tp.selected_pose_data.SelectedPoseData_array_port.data', index=0,
      number=753, type=11, cpp_type=10, label=3,
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
  serialized_start=332,
  serialized_end=423,
)

_SELECTEDPOSEDATA.fields_by_name['selectionStatus'].enum_type = ap__tp_dot_pose__selection__status__pb2._POSESELECTIONSTATUS
_SELECTEDPOSEDATA.fields_by_name['reachedStatus'].enum_type = ap__tp_dot_pose__reached__status__pb2._POSEREACHEDSTATUS
_SELECTEDPOSEDATA_ARRAY_PORT.fields_by_name['data'].message_type = _SELECTEDPOSEDATA
DESCRIPTOR.message_types_by_name['SelectedPoseData'] = _SELECTEDPOSEDATA
DESCRIPTOR.message_types_by_name['SelectedPoseData_array_port'] = _SELECTEDPOSEDATA_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SelectedPoseData = _reflection.GeneratedProtocolMessageType('SelectedPoseData', (_message.Message,), {
  'DESCRIPTOR' : _SELECTEDPOSEDATA,
  '__module__' : 'ap_tp.selected_pose_data_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.selected_pose_data.SelectedPoseData)
  })
_sym_db.RegisterMessage(SelectedPoseData)

SelectedPoseData_array_port = _reflection.GeneratedProtocolMessageType('SelectedPoseData_array_port', (_message.Message,), {
  'DESCRIPTOR' : _SELECTEDPOSEDATA_ARRAY_PORT,
  '__module__' : 'ap_tp.selected_pose_data_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.selected_pose_data.SelectedPoseData_array_port)
  })
_sym_db.RegisterMessage(SelectedPoseData_array_port)


# @@protoc_insertion_point(module_scope)
