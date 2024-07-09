# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_psm/traj_ctrl_request_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from ap_psm import driving_mode_trc_pb2 as ap__psm_dot_driving__mode__trc__pb2
from ap_psm import motion_control_request_type_pb2 as ap__psm_dot_motion__control__request__type__pb2
from ap_psm import driver_in_out_request_type_pb2 as ap__psm_dot_driver__in__out__request__type__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_psm/traj_ctrl_request_port.proto',
  package='pb.ap_psm.traj_ctrl_request_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n#ap_psm/traj_ctrl_request_port.proto\x12 pb.ap_psm.traj_ctrl_request_port\x1a\x17\x65\x63o/signal_header.proto\x1a\x1d\x61p_psm/driving_mode_trc.proto\x1a(ap_psm/motion_control_request_type.proto\x1a\'ap_psm/driver_in_out_request_type.proto\"\xb5\x03\n\x13TrajCtrlRequestPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x46\n\x11\x64rivingModeReq_nu\x18\xb2\x03 \x01(\x0e\x32*.pb.ap_psm.driving_mode_trc.DrivingModeTRC\x12\x1a\n\x11trajCtrlActive_nu\x18\xd8\t \x01(\x08\x12\x1e\n\x15\x65mergencyBrakeRequest\x18\xc7\x18 \x01(\x08\x12\x65\n\x1bMotionControlRequestType_nu\x18\xea\x10 \x01(\x0e\x32?.pb.ap_psm.motion_control_request_type.MotionControlRequestType\x12`\n\x19\x64riverInOutRequestType_nu\x18\xd9\x04 \x01(\x0e\x32<.pb.ap_psm.driver_in_out_request_type.DriverInOutRequestType\"f\n\x1eTrajCtrlRequestPort_array_port\x12\x44\n\x04\x64\x61ta\x18\xa5\x14 \x03(\x0b\x32\x35.pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,ap__psm_dot_driving__mode__trc__pb2.DESCRIPTOR,ap__psm_dot_motion__control__request__type__pb2.DESCRIPTOR,ap__psm_dot_driver__in__out__request__type__pb2.DESCRIPTOR,])




_TRAJCTRLREQUESTPORT = _descriptor.Descriptor(
  name='TrajCtrlRequestPort',
  full_name='pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='drivingModeReq_nu', full_name='pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort.drivingModeReq_nu', index=2,
      number=434, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='trajCtrlActive_nu', full_name='pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort.trajCtrlActive_nu', index=3,
      number=1240, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='emergencyBrakeRequest', full_name='pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort.emergencyBrakeRequest', index=4,
      number=3143, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='MotionControlRequestType_nu', full_name='pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort.MotionControlRequestType_nu', index=5,
      number=2154, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='driverInOutRequestType_nu', full_name='pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort.driverInOutRequestType_nu', index=6,
      number=601, type=14, cpp_type=8, label=1,
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
  serialized_start=213,
  serialized_end=650,
)


_TRAJCTRLREQUESTPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='TrajCtrlRequestPort_array_port',
  full_name='pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port.data', index=0,
      number=2597, type=11, cpp_type=10, label=3,
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
  serialized_start=652,
  serialized_end=754,
)

_TRAJCTRLREQUESTPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_TRAJCTRLREQUESTPORT.fields_by_name['drivingModeReq_nu'].enum_type = ap__psm_dot_driving__mode__trc__pb2._DRIVINGMODETRC
_TRAJCTRLREQUESTPORT.fields_by_name['MotionControlRequestType_nu'].enum_type = ap__psm_dot_motion__control__request__type__pb2._MOTIONCONTROLREQUESTTYPE
_TRAJCTRLREQUESTPORT.fields_by_name['driverInOutRequestType_nu'].enum_type = ap__psm_dot_driver__in__out__request__type__pb2._DRIVERINOUTREQUESTTYPE
_TRAJCTRLREQUESTPORT_ARRAY_PORT.fields_by_name['data'].message_type = _TRAJCTRLREQUESTPORT
DESCRIPTOR.message_types_by_name['TrajCtrlRequestPort'] = _TRAJCTRLREQUESTPORT
DESCRIPTOR.message_types_by_name['TrajCtrlRequestPort_array_port'] = _TRAJCTRLREQUESTPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TrajCtrlRequestPort = _reflection.GeneratedProtocolMessageType('TrajCtrlRequestPort', (_message.Message,), {
  'DESCRIPTOR' : _TRAJCTRLREQUESTPORT,
  '__module__' : 'ap_psm.traj_ctrl_request_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort)
  })
_sym_db.RegisterMessage(TrajCtrlRequestPort)

TrajCtrlRequestPort_array_port = _reflection.GeneratedProtocolMessageType('TrajCtrlRequestPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _TRAJCTRLREQUESTPORT_ARRAY_PORT,
  '__module__' : 'ap_psm.traj_ctrl_request_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port)
  })
_sym_db.RegisterMessage(TrajCtrlRequestPort_array_port)


# @@protoc_insertion_point(module_scope)