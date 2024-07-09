# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_tp/traj_plan_visu_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from ap_tp import planned_geometric_path_output_pb2 as ap__tp_dot_planned__geometric__path__output__pb2
from ap_tp import driving_resistance_pb2 as ap__tp_dot_driving__resistance__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_tp/traj_plan_visu_port.proto',
  package='pb.ap_tp.traj_plan_visu_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1f\x61p_tp/traj_plan_visu_port.proto\x12\x1cpb.ap_tp.traj_plan_visu_port\x1a\x17\x65\x63o/signal_header.proto\x1a)ap_tp/planned_geometric_path_output.proto\x1a\x1e\x61p_tp/driving_resistance.proto\"\x9e\x03\n\x10TrajPlanVisuPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x1a\n\x11\x63urrentPoseIdx_nu\x18\xb9\x0f \x01(\r\x12\x19\n\x10numValidPoses_nu\x18\x89\x05 \x01(\r\x12\x19\n\x10numValidSegments\x18\xef\n \x01(\r\x12\x1a\n\x11plannedPathXPos_m\x18\xd6\x10 \x03(\x02\x12\x1a\n\x11plannedPathYPos_m\x18\xf6\x1b \x03(\x02\x12\x61\n\x14plannedGeometricPath\x18\x88\x0f \x03(\x0b\x32\x42.pb.ap_tp.planned_geometric_path_output.PlannedGeometricPathOutput\x12J\n\x11\x64rivingResistance\x18\x94\x14 \x03(\x0b\x32..pb.ap_tp.driving_resistance.DrivingResistance\"[\n\x1bTrajPlanVisuPort_array_port\x12<\n\x04\x64\x61ta\x18\x45 \x03(\x0b\x32..pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,ap__tp_dot_planned__geometric__path__output__pb2.DESCRIPTOR,ap__tp_dot_driving__resistance__pb2.DESCRIPTOR,])




_TRAJPLANVISUPORT = _descriptor.Descriptor(
  name='TrajPlanVisuPort',
  full_name='pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='currentPoseIdx_nu', full_name='pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort.currentPoseIdx_nu', index=2,
      number=1977, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numValidPoses_nu', full_name='pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort.numValidPoses_nu', index=3,
      number=649, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numValidSegments', full_name='pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort.numValidSegments', index=4,
      number=1391, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='plannedPathXPos_m', full_name='pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort.plannedPathXPos_m', index=5,
      number=2134, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='plannedPathYPos_m', full_name='pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort.plannedPathYPos_m', index=6,
      number=3574, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='plannedGeometricPath', full_name='pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort.plannedGeometricPath', index=7,
      number=1928, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='drivingResistance', full_name='pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort.drivingResistance', index=8,
      number=2580, type=11, cpp_type=10, label=3,
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
  serialized_start=166,
  serialized_end=580,
)


_TRAJPLANVISUPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='TrajPlanVisuPort_array_port',
  full_name='pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort_array_port.data', index=0,
      number=69, type=11, cpp_type=10, label=3,
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
  serialized_start=582,
  serialized_end=673,
)

_TRAJPLANVISUPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_TRAJPLANVISUPORT.fields_by_name['plannedGeometricPath'].message_type = ap__tp_dot_planned__geometric__path__output__pb2._PLANNEDGEOMETRICPATHOUTPUT
_TRAJPLANVISUPORT.fields_by_name['drivingResistance'].message_type = ap__tp_dot_driving__resistance__pb2._DRIVINGRESISTANCE
_TRAJPLANVISUPORT_ARRAY_PORT.fields_by_name['data'].message_type = _TRAJPLANVISUPORT
DESCRIPTOR.message_types_by_name['TrajPlanVisuPort'] = _TRAJPLANVISUPORT
DESCRIPTOR.message_types_by_name['TrajPlanVisuPort_array_port'] = _TRAJPLANVISUPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TrajPlanVisuPort = _reflection.GeneratedProtocolMessageType('TrajPlanVisuPort', (_message.Message,), {
  'DESCRIPTOR' : _TRAJPLANVISUPORT,
  '__module__' : 'ap_tp.traj_plan_visu_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort)
  })
_sym_db.RegisterMessage(TrajPlanVisuPort)

TrajPlanVisuPort_array_port = _reflection.GeneratedProtocolMessageType('TrajPlanVisuPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _TRAJPLANVISUPORT_ARRAY_PORT,
  '__module__' : 'ap_tp.traj_plan_visu_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort_array_port)
  })
_sym_db.RegisterMessage(TrajPlanVisuPort_array_port)


# @@protoc_insertion_point(module_scope)
