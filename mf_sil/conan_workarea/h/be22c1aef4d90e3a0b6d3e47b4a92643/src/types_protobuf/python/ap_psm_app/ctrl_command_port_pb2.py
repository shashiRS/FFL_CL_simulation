# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_psm_app/ctrl_command_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from ap_psm_app import std_request_pb2 as ap__psm__app_dot_std__request__pb2
from ap_psm_app import ppcparking_mode_pb2 as ap__psm__app_dot_ppcparking__mode__pb2
from ap_psm_app import degrade_type_pb2 as ap__psm__app_dot_degrade__type__pb2
from ap_psm_app import motion_control_request_type_pb2 as ap__psm__app_dot_motion__control__request__type__pb2
from ap_psm_app import driver_in_out_request_type_pb2 as ap__psm__app_dot_driver__in__out__request__type__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_psm_app/ctrl_command_port.proto',
  package='pb.ap_psm_app.ctrl_command_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\"ap_psm_app/ctrl_command_port.proto\x12\x1fpb.ap_psm_app.ctrl_command_port\x1a\x17\x65\x63o/signal_header.proto\x1a\x1c\x61p_psm_app/std_request.proto\x1a ap_psm_app/ppcparking_mode.proto\x1a\x1d\x61p_psm_app/degrade_type.proto\x1a,ap_psm_app/motion_control_request_type.proto\x1a+ap_psm_app/driver_in_out_request_type.proto\"\xa9\x05\n\x0f\x43trlCommandPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12=\n\rstdRequest_nu\x18\xa7\x1d \x01(\x0e\x32%.pb.ap_psm_app.std_request.StdRequest\x12I\n\x11ppcParkingMode_nu\x18\xbf\n \x01(\x0e\x32-.pb.ap_psm_app.ppcparking_mode.PPCParkingMode\x12@\n\x0e\x64\x65gradeType_nu\x18\x9d\r \x01(\x0e\x32\'.pb.ap_psm_app.degrade_type.DegradeType\x12 \n\x17selectedTargetPoseId_nu\x18\xee\x07 \x01(\r\x12\x1a\n\x11\x65mergencyBrake_nu\x18\xa1\x05 \x01(\x08\x12 \n\x17\x63urrentSelectedMpSlotId\x18\x8c\x17 \x01(\r\x12#\n\x1a\x64\x65leteMemorizedParkingData\x18\xdb\t \x01(\x08\x12!\n\x19storeMemorizedParkingData\x18\x36 \x01(\x08\x12i\n\x1bmotionControlRequestType_nu\x18\xac\r \x01(\x0e\x32\x43.pb.ap_psm_app.motion_control_request_type.MotionControlRequestType\x12\x64\n\x19\x64riverInOutRequestType_nu\x18\x8c\n \x01(\x0e\x32@.pb.ap_psm_app.driver_in_out_request_type.DriverInOutRequestType\"]\n\x1a\x43trlCommandPort_array_port\x12?\n\x04\x64\x61ta\x18\xbe\x11 \x03(\x0b\x32\x30.pb.ap_psm_app.ctrl_command_port.CtrlCommandPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,ap__psm__app_dot_std__request__pb2.DESCRIPTOR,ap__psm__app_dot_ppcparking__mode__pb2.DESCRIPTOR,ap__psm__app_dot_degrade__type__pb2.DESCRIPTOR,ap__psm__app_dot_motion__control__request__type__pb2.DESCRIPTOR,ap__psm__app_dot_driver__in__out__request__type__pb2.DESCRIPTOR,])




_CTRLCOMMANDPORT = _descriptor.Descriptor(
  name='CtrlCommandPort',
  full_name='pb.ap_psm_app.ctrl_command_port.CtrlCommandPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_psm_app.ctrl_command_port.CtrlCommandPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_psm_app.ctrl_command_port.CtrlCommandPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='stdRequest_nu', full_name='pb.ap_psm_app.ctrl_command_port.CtrlCommandPort.stdRequest_nu', index=2,
      number=3751, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ppcParkingMode_nu', full_name='pb.ap_psm_app.ctrl_command_port.CtrlCommandPort.ppcParkingMode_nu', index=3,
      number=1343, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='degradeType_nu', full_name='pb.ap_psm_app.ctrl_command_port.CtrlCommandPort.degradeType_nu', index=4,
      number=1693, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='selectedTargetPoseId_nu', full_name='pb.ap_psm_app.ctrl_command_port.CtrlCommandPort.selectedTargetPoseId_nu', index=5,
      number=1006, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='emergencyBrake_nu', full_name='pb.ap_psm_app.ctrl_command_port.CtrlCommandPort.emergencyBrake_nu', index=6,
      number=673, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='currentSelectedMpSlotId', full_name='pb.ap_psm_app.ctrl_command_port.CtrlCommandPort.currentSelectedMpSlotId', index=7,
      number=2956, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='deleteMemorizedParkingData', full_name='pb.ap_psm_app.ctrl_command_port.CtrlCommandPort.deleteMemorizedParkingData', index=8,
      number=1243, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='storeMemorizedParkingData', full_name='pb.ap_psm_app.ctrl_command_port.CtrlCommandPort.storeMemorizedParkingData', index=9,
      number=54, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='motionControlRequestType_nu', full_name='pb.ap_psm_app.ctrl_command_port.CtrlCommandPort.motionControlRequestType_nu', index=10,
      number=1708, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='driverInOutRequestType_nu', full_name='pb.ap_psm_app.ctrl_command_port.CtrlCommandPort.driverInOutRequestType_nu', index=11,
      number=1292, type=14, cpp_type=8, label=1,
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
  serialized_start=283,
  serialized_end=964,
)


_CTRLCOMMANDPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='CtrlCommandPort_array_port',
  full_name='pb.ap_psm_app.ctrl_command_port.CtrlCommandPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_psm_app.ctrl_command_port.CtrlCommandPort_array_port.data', index=0,
      number=2238, type=11, cpp_type=10, label=3,
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
  serialized_start=966,
  serialized_end=1059,
)

_CTRLCOMMANDPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_CTRLCOMMANDPORT.fields_by_name['stdRequest_nu'].enum_type = ap__psm__app_dot_std__request__pb2._STDREQUEST
_CTRLCOMMANDPORT.fields_by_name['ppcParkingMode_nu'].enum_type = ap__psm__app_dot_ppcparking__mode__pb2._PPCPARKINGMODE
_CTRLCOMMANDPORT.fields_by_name['degradeType_nu'].enum_type = ap__psm__app_dot_degrade__type__pb2._DEGRADETYPE
_CTRLCOMMANDPORT.fields_by_name['motionControlRequestType_nu'].enum_type = ap__psm__app_dot_motion__control__request__type__pb2._MOTIONCONTROLREQUESTTYPE
_CTRLCOMMANDPORT.fields_by_name['driverInOutRequestType_nu'].enum_type = ap__psm__app_dot_driver__in__out__request__type__pb2._DRIVERINOUTREQUESTTYPE
_CTRLCOMMANDPORT_ARRAY_PORT.fields_by_name['data'].message_type = _CTRLCOMMANDPORT
DESCRIPTOR.message_types_by_name['CtrlCommandPort'] = _CTRLCOMMANDPORT
DESCRIPTOR.message_types_by_name['CtrlCommandPort_array_port'] = _CTRLCOMMANDPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

CtrlCommandPort = _reflection.GeneratedProtocolMessageType('CtrlCommandPort', (_message.Message,), {
  'DESCRIPTOR' : _CTRLCOMMANDPORT,
  '__module__' : 'ap_psm_app.ctrl_command_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm_app.ctrl_command_port.CtrlCommandPort)
  })
_sym_db.RegisterMessage(CtrlCommandPort)

CtrlCommandPort_array_port = _reflection.GeneratedProtocolMessageType('CtrlCommandPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _CTRLCOMMANDPORT_ARRAY_PORT,
  '__module__' : 'ap_psm_app.ctrl_command_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm_app.ctrl_command_port.CtrlCommandPort_array_port)
  })
_sym_db.RegisterMessage(CtrlCommandPort_array_port)


# @@protoc_insertion_point(module_scope)
