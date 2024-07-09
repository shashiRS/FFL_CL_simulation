# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_manager/lo_ctrl_request_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from mf_manager import lo_ctrl_request_type_pb2 as mf__manager_dot_lo__ctrl__request__type__pb2
from mf_manager import activate_lo_ctrl_type_pb2 as mf__manager_dot_activate__lo__ctrl__type__pb2
from mf_manager import directed_lo_ctrl_request_pb2 as mf__manager_dot_directed__lo__ctrl__request__pb2
from mf_manager import lo_ctrl_request_source_pb2 as mf__manager_dot_lo__ctrl__request__source__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_manager/lo_ctrl_request_port.proto',
  package='pb.mf_manager.lo_ctrl_request_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n%mf_manager/lo_ctrl_request_port.proto\x12\"pb.mf_manager.lo_ctrl_request_port\x1a\x17\x65\x63o/signal_header.proto\x1a%mf_manager/lo_ctrl_request_type.proto\x1a&mf_manager/activate_lo_ctrl_type.proto\x1a)mf_manager/directed_lo_ctrl_request.proto\x1a\'mf_manager/lo_ctrl_request_source.proto\"\xed\x05\n\x11LoCtrlRequestPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x17\n\x0e\x61\x63tivateLoCtrl\x18\xb7\x1d \x01(\x08\x12Q\n\x11loCtrlRequestType\x18\xc0\x04 \x01(\x0e\x32\x35.pb.mf_manager.lo_ctrl_request_type.LoCtrlRequestType\x12V\n\x15\x61\x63tivateLoCtrlType_nu\x18\x31 \x01(\x0e\x32\x37.pb.mf_manager.activate_lo_ctrl_type.ActivateLoCtrlType\x12\x1b\n\x12secureInStandstill\x18\xb6\x14 \x01(\x08\x12\x1b\n\x12\x63omfortStopRequest\x18\xe7\x19 \x01(\x08\x12[\n\x13\x64istanceToStopReq_m\x18\x82\x1f \x01(\x0b\x32=.pb.mf_manager.directed_lo_ctrl_request.DirectedLoCtrlRequest\x12W\n\x0fvelocityReq_mps\x18\xa0\x10 \x01(\x0b\x32=.pb.mf_manager.directed_lo_ctrl_request.DirectedLoCtrlRequest\x12\\\n\x14\x61\x63\x63\x65lerationReq_mps2\x18\xbf\r \x01(\x0b\x32=.pb.mf_manager.directed_lo_ctrl_request.DirectedLoCtrlRequest\x12Z\n\x16loCtrlRequestSource_nu\x18\xf7\x14 \x01(\x0e\x32\x39.pb.mf_manager.lo_ctrl_request_source.loCtrlRequestSource\x12\x17\n\x0eremoteSelfTest\x18\xc1\x12 \x01(\x08\"d\n\x1cLoCtrlRequestPort_array_port\x12\x44\n\x04\x64\x61ta\x18\x85\x17 \x03(\x0b\x32\x35.pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,mf__manager_dot_lo__ctrl__request__type__pb2.DESCRIPTOR,mf__manager_dot_activate__lo__ctrl__type__pb2.DESCRIPTOR,mf__manager_dot_directed__lo__ctrl__request__pb2.DESCRIPTOR,mf__manager_dot_lo__ctrl__request__source__pb2.DESCRIPTOR,])




_LOCTRLREQUESTPORT = _descriptor.Descriptor(
  name='LoCtrlRequestPort',
  full_name='pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='activateLoCtrl', full_name='pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort.activateLoCtrl', index=2,
      number=3767, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='loCtrlRequestType', full_name='pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort.loCtrlRequestType', index=3,
      number=576, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='activateLoCtrlType_nu', full_name='pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort.activateLoCtrlType_nu', index=4,
      number=49, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='secureInStandstill', full_name='pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort.secureInStandstill', index=5,
      number=2614, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='comfortStopRequest', full_name='pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort.comfortStopRequest', index=6,
      number=3303, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='distanceToStopReq_m', full_name='pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort.distanceToStopReq_m', index=7,
      number=3970, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='velocityReq_mps', full_name='pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort.velocityReq_mps', index=8,
      number=2080, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='accelerationReq_mps2', full_name='pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort.accelerationReq_mps2', index=9,
      number=1727, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='loCtrlRequestSource_nu', full_name='pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort.loCtrlRequestSource_nu', index=10,
      number=2679, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='remoteSelfTest', full_name='pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort.remoteSelfTest', index=11,
      number=2369, type=8, cpp_type=7, label=1,
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
  serialized_start=266,
  serialized_end=1015,
)


_LOCTRLREQUESTPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='LoCtrlRequestPort_array_port',
  full_name='pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort_array_port.data', index=0,
      number=2949, type=11, cpp_type=10, label=3,
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
  serialized_start=1017,
  serialized_end=1117,
)

_LOCTRLREQUESTPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_LOCTRLREQUESTPORT.fields_by_name['loCtrlRequestType'].enum_type = mf__manager_dot_lo__ctrl__request__type__pb2._LOCTRLREQUESTTYPE
_LOCTRLREQUESTPORT.fields_by_name['activateLoCtrlType_nu'].enum_type = mf__manager_dot_activate__lo__ctrl__type__pb2._ACTIVATELOCTRLTYPE
_LOCTRLREQUESTPORT.fields_by_name['distanceToStopReq_m'].message_type = mf__manager_dot_directed__lo__ctrl__request__pb2._DIRECTEDLOCTRLREQUEST
_LOCTRLREQUESTPORT.fields_by_name['velocityReq_mps'].message_type = mf__manager_dot_directed__lo__ctrl__request__pb2._DIRECTEDLOCTRLREQUEST
_LOCTRLREQUESTPORT.fields_by_name['accelerationReq_mps2'].message_type = mf__manager_dot_directed__lo__ctrl__request__pb2._DIRECTEDLOCTRLREQUEST
_LOCTRLREQUESTPORT.fields_by_name['loCtrlRequestSource_nu'].enum_type = mf__manager_dot_lo__ctrl__request__source__pb2._LOCTRLREQUESTSOURCE
_LOCTRLREQUESTPORT_ARRAY_PORT.fields_by_name['data'].message_type = _LOCTRLREQUESTPORT
DESCRIPTOR.message_types_by_name['LoCtrlRequestPort'] = _LOCTRLREQUESTPORT
DESCRIPTOR.message_types_by_name['LoCtrlRequestPort_array_port'] = _LOCTRLREQUESTPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LoCtrlRequestPort = _reflection.GeneratedProtocolMessageType('LoCtrlRequestPort', (_message.Message,), {
  'DESCRIPTOR' : _LOCTRLREQUESTPORT,
  '__module__' : 'mf_manager.lo_ctrl_request_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort)
  })
_sym_db.RegisterMessage(LoCtrlRequestPort)

LoCtrlRequestPort_array_port = _reflection.GeneratedProtocolMessageType('LoCtrlRequestPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _LOCTRLREQUESTPORT_ARRAY_PORT,
  '__module__' : 'mf_manager.lo_ctrl_request_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_manager.lo_ctrl_request_port.LoCtrlRequestPort_array_port)
  })
_sym_db.RegisterMessage(LoCtrlRequestPort_array_port)


# @@protoc_insertion_point(module_scope)