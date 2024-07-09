# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/visu_input_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from mf_hmih import visu_hmidata_pb2 as mf__hmih_dot_visu__hmidata__pb2
from ap_hmitoap import screen_types_pb2 as ap__hmitoap_dot_screen__types__pb2
from ap_tp import traj_plan_visu_port_pb2 as ap__tp_dot_traj__plan__visu__port__pb2
from mf_hmih import parking_target_poses_pb2 as mf__hmih_dot_parking__target__poses__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/visu_input_port.proto',
  package='pb.mf_hmih.visu_input_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1dmf_hmih/visu_input_port.proto\x12\x1apb.mf_hmih.visu_input_port\x1a\x17\x65\x63o/signal_header.proto\x1a\x1amf_hmih/visu_hmidata.proto\x1a\x1d\x61p_hmitoap/screen_types.proto\x1a\x1f\x61p_tp/traj_plan_visu_port.proto\x1a\"mf_hmih/parking_target_poses.proto\"\xc4\x03\n\rVisuInputPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x1c\n\x13\x64istanceToStopReq_m\x18\xfa\x19 \x01(\x02\x12:\n\x0bvisuHMIData\x18\x99\n \x01(\x0b\x32$.pb.mf_hmih.visu_hmidata.VisuHMIData\x12K\n\x19HmiOutUserActScreenReq_u8\x18\xf2\x04 \x01(\x0e\x32\'.pb.ap_hmitoap.screen_types.ScreenTypes\x12L\n\x13trajPlanVisuPort_nu\x18\xbd\x01 \x01(\x0b\x32..pb.ap_tp.traj_plan_visu_port.TrajPlanVisuPort\x12N\n\x10parkingPosesVisu\x18\x90\x15 \x01(\x0b\x32\x33.pb.mf_hmih.parking_target_poses.ParkingTargetPoses\x12\x1b\n\x12\x64riverSelection_nu\x18\xf4\x08 \x01(\x08\"T\n\x18VisuInputPort_array_port\x12\x38\n\x04\x64\x61ta\x18\xe1\r \x03(\x0b\x32).pb.mf_hmih.visu_input_port.VisuInputPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,mf__hmih_dot_visu__hmidata__pb2.DESCRIPTOR,ap__hmitoap_dot_screen__types__pb2.DESCRIPTOR,ap__tp_dot_traj__plan__visu__port__pb2.DESCRIPTOR,mf__hmih_dot_parking__target__poses__pb2.DESCRIPTOR,])




_VISUINPUTPORT = _descriptor.Descriptor(
  name='VisuInputPort',
  full_name='pb.mf_hmih.visu_input_port.VisuInputPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.mf_hmih.visu_input_port.VisuInputPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.mf_hmih.visu_input_port.VisuInputPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='distanceToStopReq_m', full_name='pb.mf_hmih.visu_input_port.VisuInputPort.distanceToStopReq_m', index=2,
      number=3322, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='visuHMIData', full_name='pb.mf_hmih.visu_input_port.VisuInputPort.visuHMIData', index=3,
      number=1305, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='HmiOutUserActScreenReq_u8', full_name='pb.mf_hmih.visu_input_port.VisuInputPort.HmiOutUserActScreenReq_u8', index=4,
      number=626, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='trajPlanVisuPort_nu', full_name='pb.mf_hmih.visu_input_port.VisuInputPort.trajPlanVisuPort_nu', index=5,
      number=189, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='parkingPosesVisu', full_name='pb.mf_hmih.visu_input_port.VisuInputPort.parkingPosesVisu', index=6,
      number=2704, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='driverSelection_nu', full_name='pb.mf_hmih.visu_input_port.VisuInputPort.driverSelection_nu', index=7,
      number=1140, type=8, cpp_type=7, label=1,
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
  serialized_start=215,
  serialized_end=667,
)


_VISUINPUTPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='VisuInputPort_array_port',
  full_name='pb.mf_hmih.visu_input_port.VisuInputPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_hmih.visu_input_port.VisuInputPort_array_port.data', index=0,
      number=1761, type=11, cpp_type=10, label=3,
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
  serialized_start=669,
  serialized_end=753,
)

_VISUINPUTPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_VISUINPUTPORT.fields_by_name['visuHMIData'].message_type = mf__hmih_dot_visu__hmidata__pb2._VISUHMIDATA
_VISUINPUTPORT.fields_by_name['HmiOutUserActScreenReq_u8'].enum_type = ap__hmitoap_dot_screen__types__pb2._SCREENTYPES
_VISUINPUTPORT.fields_by_name['trajPlanVisuPort_nu'].message_type = ap__tp_dot_traj__plan__visu__port__pb2._TRAJPLANVISUPORT
_VISUINPUTPORT.fields_by_name['parkingPosesVisu'].message_type = mf__hmih_dot_parking__target__poses__pb2._PARKINGTARGETPOSES
_VISUINPUTPORT_ARRAY_PORT.fields_by_name['data'].message_type = _VISUINPUTPORT
DESCRIPTOR.message_types_by_name['VisuInputPort'] = _VISUINPUTPORT
DESCRIPTOR.message_types_by_name['VisuInputPort_array_port'] = _VISUINPUTPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

VisuInputPort = _reflection.GeneratedProtocolMessageType('VisuInputPort', (_message.Message,), {
  'DESCRIPTOR' : _VISUINPUTPORT,
  '__module__' : 'mf_hmih.visu_input_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.visu_input_port.VisuInputPort)
  })
_sym_db.RegisterMessage(VisuInputPort)

VisuInputPort_array_port = _reflection.GeneratedProtocolMessageType('VisuInputPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _VISUINPUTPORT_ARRAY_PORT,
  '__module__' : 'mf_hmih.visu_input_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.visu_input_port.VisuInputPort_array_port)
  })
_sym_db.RegisterMessage(VisuInputPort_array_port)


# @@protoc_insertion_point(module_scope)
