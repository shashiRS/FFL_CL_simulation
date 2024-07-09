# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_manager/la_ctrl_request_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from mf_manager import la_ctrl_request_type_pb2 as mf__manager_dot_la__ctrl__request__type__pb2
from mf_manager import la_ctrl_request_source_pb2 as mf__manager_dot_la__ctrl__request__source__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_manager/la_ctrl_request_port.proto',
  package='pb.mf_manager.la_ctrl_request_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n%mf_manager/la_ctrl_request_port.proto\x12\"pb.mf_manager.la_ctrl_request_port\x1a\x17\x65\x63o/signal_header.proto\x1a%mf_manager/la_ctrl_request_type.proto\x1a\'mf_manager/la_ctrl_request_source.proto\"\xa2\x03\n\x11LaCtrlRequestPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x17\n\x0e\x61\x63tivateLaCtrl\x18\x9f\x02 \x01(\x08\x12Q\n\x11laCtrlRequestType\x18\xd8\x1a \x01(\x0e\x32\x35.pb.mf_manager.la_ctrl_request_type.LaCtrlRequestType\x12\x1d\n\x14steerAngReqFront_rad\x18\xf2\x1b \x01(\x02\x12\x1c\n\x13steerAngReqRear_rad\x18\xdc\x0b \x01(\x02\x12\x1a\n\x11steerTorqueReq_Nm\x18\x9b\x0f \x01(\x02\x12\x19\n\x10\x63urvatureReq_1pm\x18\xd7\x17 \x01(\x02\x12Z\n\x16laCtrlRequestSource_nu\x18\x89\x16 \x01(\x0e\x32\x39.pb.mf_manager.la_ctrl_request_source.laCtrlRequestSource\"d\n\x1cLaCtrlRequestPort_array_port\x12\x44\n\x04\x64\x61ta\x18\xf3\x10 \x03(\x0b\x32\x35.pb.mf_manager.la_ctrl_request_port.LaCtrlRequestPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,mf__manager_dot_la__ctrl__request__type__pb2.DESCRIPTOR,mf__manager_dot_la__ctrl__request__source__pb2.DESCRIPTOR,])




_LACTRLREQUESTPORT = _descriptor.Descriptor(
  name='LaCtrlRequestPort',
  full_name='pb.mf_manager.la_ctrl_request_port.LaCtrlRequestPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.mf_manager.la_ctrl_request_port.LaCtrlRequestPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.mf_manager.la_ctrl_request_port.LaCtrlRequestPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='activateLaCtrl', full_name='pb.mf_manager.la_ctrl_request_port.LaCtrlRequestPort.activateLaCtrl', index=2,
      number=287, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='laCtrlRequestType', full_name='pb.mf_manager.la_ctrl_request_port.LaCtrlRequestPort.laCtrlRequestType', index=3,
      number=3416, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='steerAngReqFront_rad', full_name='pb.mf_manager.la_ctrl_request_port.LaCtrlRequestPort.steerAngReqFront_rad', index=4,
      number=3570, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='steerAngReqRear_rad', full_name='pb.mf_manager.la_ctrl_request_port.LaCtrlRequestPort.steerAngReqRear_rad', index=5,
      number=1500, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='steerTorqueReq_Nm', full_name='pb.mf_manager.la_ctrl_request_port.LaCtrlRequestPort.steerTorqueReq_Nm', index=6,
      number=1947, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='curvatureReq_1pm', full_name='pb.mf_manager.la_ctrl_request_port.LaCtrlRequestPort.curvatureReq_1pm', index=7,
      number=3031, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='laCtrlRequestSource_nu', full_name='pb.mf_manager.la_ctrl_request_port.LaCtrlRequestPort.laCtrlRequestSource_nu', index=8,
      number=2825, type=14, cpp_type=8, label=1,
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
  serialized_start=183,
  serialized_end=601,
)


_LACTRLREQUESTPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='LaCtrlRequestPort_array_port',
  full_name='pb.mf_manager.la_ctrl_request_port.LaCtrlRequestPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_manager.la_ctrl_request_port.LaCtrlRequestPort_array_port.data', index=0,
      number=2163, type=11, cpp_type=10, label=3,
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
  serialized_start=603,
  serialized_end=703,
)

_LACTRLREQUESTPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_LACTRLREQUESTPORT.fields_by_name['laCtrlRequestType'].enum_type = mf__manager_dot_la__ctrl__request__type__pb2._LACTRLREQUESTTYPE
_LACTRLREQUESTPORT.fields_by_name['laCtrlRequestSource_nu'].enum_type = mf__manager_dot_la__ctrl__request__source__pb2._LACTRLREQUESTSOURCE
_LACTRLREQUESTPORT_ARRAY_PORT.fields_by_name['data'].message_type = _LACTRLREQUESTPORT
DESCRIPTOR.message_types_by_name['LaCtrlRequestPort'] = _LACTRLREQUESTPORT
DESCRIPTOR.message_types_by_name['LaCtrlRequestPort_array_port'] = _LACTRLREQUESTPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LaCtrlRequestPort = _reflection.GeneratedProtocolMessageType('LaCtrlRequestPort', (_message.Message,), {
  'DESCRIPTOR' : _LACTRLREQUESTPORT,
  '__module__' : 'mf_manager.la_ctrl_request_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_manager.la_ctrl_request_port.LaCtrlRequestPort)
  })
_sym_db.RegisterMessage(LaCtrlRequestPort)

LaCtrlRequestPort_array_port = _reflection.GeneratedProtocolMessageType('LaCtrlRequestPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _LACTRLREQUESTPORT_ARRAY_PORT,
  '__module__' : 'mf_manager.la_ctrl_request_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_manager.la_ctrl_request_port.LaCtrlRequestPort_array_port)
  })
_sym_db.RegisterMessage(LaCtrlRequestPort_array_port)


# @@protoc_insertion_point(module_scope)