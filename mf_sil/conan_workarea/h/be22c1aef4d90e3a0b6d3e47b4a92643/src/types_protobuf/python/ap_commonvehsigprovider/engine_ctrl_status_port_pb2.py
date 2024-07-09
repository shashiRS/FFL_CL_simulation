# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_commonvehsigprovider/engine_ctrl_status_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from ap_commonvehsigprovider import start_stop_status_pb2 as ap__commonvehsigprovider_dot_start__stop__status__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_commonvehsigprovider/engine_ctrl_status_port.proto',
  package='pb.ap_commonvehsigprovider.engine_ctrl_status_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n5ap_commonvehsigprovider/engine_ctrl_status_port.proto\x12\x32pb.ap_commonvehsigprovider.engine_ctrl_status_port\x1a\x17\x65\x63o/signal_header.proto\x1a/ap_commonvehsigprovider/start_stop_status.proto\"\x8e\x03\n\x14\x45ngineCtrlStatusPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x19\n\x10throttlePos_perc\x18\xec\x0b \x01(\x02\x12 \n\x17throttleGradient_percps\x18\xdb\x02 \x01(\x02\x12\x16\n\raxleTorque_nm\x18\xa5\x0b \x01(\x02\x12\x14\n\x0b\x65ngineOn_nu\x18\x9a\x04 \x01(\x08\x12Z\n\x12startStopStatus_nu\x18\x9c\x05 \x01(\x0e\x32=.pb.ap_commonvehsigprovider.start_stop_status.StartStopStatus\x12\x1f\n\x16remoteStartPossible_nu\x18\x90\x04 \x01(\x08\x12\x1a\n\x11throttlePos_QF_nu\x18\xf9\x1c \x01(\x08\x12\x1f\n\x16throttleGradient_QF_nu\x18\xc0\x04 \x01(\x08\"z\n\x1f\x45ngineCtrlStatusPort_array_port\x12W\n\x04\x64\x61ta\x18\x82\n \x03(\x0b\x32H.pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,ap__commonvehsigprovider_dot_start__stop__status__pb2.DESCRIPTOR,])




_ENGINECTRLSTATUSPORT = _descriptor.Descriptor(
  name='EngineCtrlStatusPort',
  full_name='pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='throttlePos_perc', full_name='pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort.throttlePos_perc', index=2,
      number=1516, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='throttleGradient_percps', full_name='pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort.throttleGradient_percps', index=3,
      number=347, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='axleTorque_nm', full_name='pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort.axleTorque_nm', index=4,
      number=1445, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='engineOn_nu', full_name='pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort.engineOn_nu', index=5,
      number=538, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='startStopStatus_nu', full_name='pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort.startStopStatus_nu', index=6,
      number=668, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='remoteStartPossible_nu', full_name='pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort.remoteStartPossible_nu', index=7,
      number=528, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='throttlePos_QF_nu', full_name='pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort.throttlePos_QF_nu', index=8,
      number=3705, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='throttleGradient_QF_nu', full_name='pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort.throttleGradient_QF_nu', index=9,
      number=576, type=8, cpp_type=7, label=1,
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
  serialized_start=184,
  serialized_end=582,
)


_ENGINECTRLSTATUSPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='EngineCtrlStatusPort_array_port',
  full_name='pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort_array_port.data', index=0,
      number=1282, type=11, cpp_type=10, label=3,
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
  serialized_start=584,
  serialized_end=706,
)

_ENGINECTRLSTATUSPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_ENGINECTRLSTATUSPORT.fields_by_name['startStopStatus_nu'].enum_type = ap__commonvehsigprovider_dot_start__stop__status__pb2._STARTSTOPSTATUS
_ENGINECTRLSTATUSPORT_ARRAY_PORT.fields_by_name['data'].message_type = _ENGINECTRLSTATUSPORT
DESCRIPTOR.message_types_by_name['EngineCtrlStatusPort'] = _ENGINECTRLSTATUSPORT
DESCRIPTOR.message_types_by_name['EngineCtrlStatusPort_array_port'] = _ENGINECTRLSTATUSPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

EngineCtrlStatusPort = _reflection.GeneratedProtocolMessageType('EngineCtrlStatusPort', (_message.Message,), {
  'DESCRIPTOR' : _ENGINECTRLSTATUSPORT,
  '__module__' : 'ap_commonvehsigprovider.engine_ctrl_status_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort)
  })
_sym_db.RegisterMessage(EngineCtrlStatusPort)

EngineCtrlStatusPort_array_port = _reflection.GeneratedProtocolMessageType('EngineCtrlStatusPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _ENGINECTRLSTATUSPORT_ARRAY_PORT,
  '__module__' : 'ap_commonvehsigprovider.engine_ctrl_status_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.engine_ctrl_status_port.EngineCtrlStatusPort_array_port)
  })
_sym_db.RegisterMessage(EngineCtrlStatusPort_array_port)


# @@protoc_insertion_point(module_scope)
