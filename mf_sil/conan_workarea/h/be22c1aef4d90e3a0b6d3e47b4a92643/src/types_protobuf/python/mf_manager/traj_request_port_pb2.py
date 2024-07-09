# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_manager/traj_request_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from mf_manager import mfmplanned_traj_pb2 as mf__manager_dot_mfmplanned__traj__pb2
from mf_manager import mfmplanned_traj_type_pb2 as mf__manager_dot_mfmplanned__traj__type__pb2
from mf_manager import mfmdriving_resistance_pb2 as mf__manager_dot_mfmdriving__resistance__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_manager/traj_request_port.proto',
  package='pb.mf_manager.traj_request_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\"mf_manager/traj_request_port.proto\x12\x1fpb.mf_manager.traj_request_port\x1a\x17\x65\x63o/signal_header.proto\x1a mf_manager/mfmplanned_traj.proto\x1a%mf_manager/mfmplanned_traj_type.proto\x1a&mf_manager/mfmdriving_resistance.proto\"\xfd\x03\n\x0fTrajRequestPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x43\n\x0bplannedTraj\x18\xd0\x1b \x03(\x0b\x32-.pb.mf_manager.mfmplanned_traj.MFMPlannedTraj\x12L\n\x0btrajType_nu\x18\x8a\r \x01(\x0e\x32\x36.pb.mf_manager.mfmplanned_traj_type.MFMPlannedTrajType\x12\x1e\n\x15numValidCtrlPoints_nu\x18\xd2\x06 \x01(\r\x12\x1d\n\x14\x64rivingForwardReq_nu\x18\xcb\x18 \x01(\x08\x12\x14\n\x0ctrajValid_nu\x18} \x01(\x08\x12\x1d\n\x14newSegmentStarted_nu\x18\xdf\n \x01(\x08\x12\x19\n\x10isLastSegment_nu\x18\xb8\x0c \x01(\x08\x12\x1e\n\x15stepInTrajAfterIdx_nu\x18\xcf\n \x01(\r\x12U\n\x11\x64rivingResistance\x18\xd3\x17 \x03(\x0b\x32\x39.pb.mf_manager.mfmdriving_resistance.MFMDrivingResistance\"]\n\x1aTrajRequestPort_array_port\x12?\n\x04\x64\x61ta\x18\x8f\x03 \x03(\x0b\x32\x30.pb.mf_manager.traj_request_port.TrajRequestPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,mf__manager_dot_mfmplanned__traj__pb2.DESCRIPTOR,mf__manager_dot_mfmplanned__traj__type__pb2.DESCRIPTOR,mf__manager_dot_mfmdriving__resistance__pb2.DESCRIPTOR,])




_TRAJREQUESTPORT = _descriptor.Descriptor(
  name='TrajRequestPort',
  full_name='pb.mf_manager.traj_request_port.TrajRequestPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.mf_manager.traj_request_port.TrajRequestPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.mf_manager.traj_request_port.TrajRequestPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='plannedTraj', full_name='pb.mf_manager.traj_request_port.TrajRequestPort.plannedTraj', index=2,
      number=3536, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='trajType_nu', full_name='pb.mf_manager.traj_request_port.TrajRequestPort.trajType_nu', index=3,
      number=1674, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numValidCtrlPoints_nu', full_name='pb.mf_manager.traj_request_port.TrajRequestPort.numValidCtrlPoints_nu', index=4,
      number=850, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='drivingForwardReq_nu', full_name='pb.mf_manager.traj_request_port.TrajRequestPort.drivingForwardReq_nu', index=5,
      number=3147, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='trajValid_nu', full_name='pb.mf_manager.traj_request_port.TrajRequestPort.trajValid_nu', index=6,
      number=125, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='newSegmentStarted_nu', full_name='pb.mf_manager.traj_request_port.TrajRequestPort.newSegmentStarted_nu', index=7,
      number=1375, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isLastSegment_nu', full_name='pb.mf_manager.traj_request_port.TrajRequestPort.isLastSegment_nu', index=8,
      number=1592, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='stepInTrajAfterIdx_nu', full_name='pb.mf_manager.traj_request_port.TrajRequestPort.stepInTrajAfterIdx_nu', index=9,
      number=1359, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='drivingResistance', full_name='pb.mf_manager.traj_request_port.TrajRequestPort.drivingResistance', index=10,
      number=3027, type=11, cpp_type=10, label=3,
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
  serialized_start=210,
  serialized_end=719,
)


_TRAJREQUESTPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='TrajRequestPort_array_port',
  full_name='pb.mf_manager.traj_request_port.TrajRequestPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_manager.traj_request_port.TrajRequestPort_array_port.data', index=0,
      number=399, type=11, cpp_type=10, label=3,
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
  serialized_start=721,
  serialized_end=814,
)

_TRAJREQUESTPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_TRAJREQUESTPORT.fields_by_name['plannedTraj'].message_type = mf__manager_dot_mfmplanned__traj__pb2._MFMPLANNEDTRAJ
_TRAJREQUESTPORT.fields_by_name['trajType_nu'].enum_type = mf__manager_dot_mfmplanned__traj__type__pb2._MFMPLANNEDTRAJTYPE
_TRAJREQUESTPORT.fields_by_name['drivingResistance'].message_type = mf__manager_dot_mfmdriving__resistance__pb2._MFMDRIVINGRESISTANCE
_TRAJREQUESTPORT_ARRAY_PORT.fields_by_name['data'].message_type = _TRAJREQUESTPORT
DESCRIPTOR.message_types_by_name['TrajRequestPort'] = _TRAJREQUESTPORT
DESCRIPTOR.message_types_by_name['TrajRequestPort_array_port'] = _TRAJREQUESTPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TrajRequestPort = _reflection.GeneratedProtocolMessageType('TrajRequestPort', (_message.Message,), {
  'DESCRIPTOR' : _TRAJREQUESTPORT,
  '__module__' : 'mf_manager.traj_request_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_manager.traj_request_port.TrajRequestPort)
  })
_sym_db.RegisterMessage(TrajRequestPort)

TrajRequestPort_array_port = _reflection.GeneratedProtocolMessageType('TrajRequestPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _TRAJREQUESTPORT_ARRAY_PORT,
  '__module__' : 'mf_manager.traj_request_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_manager.traj_request_port.TrajRequestPort_array_port)
  })
_sym_db.RegisterMessage(TrajRequestPort_array_port)


# @@protoc_insertion_point(module_scope)
