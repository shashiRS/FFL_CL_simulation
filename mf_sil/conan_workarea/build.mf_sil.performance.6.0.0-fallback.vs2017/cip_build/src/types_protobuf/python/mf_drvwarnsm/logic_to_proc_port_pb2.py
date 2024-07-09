# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_drvwarnsm/logic_to_proc_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from mf_drvwarnsm import whl_warning_type_pb2 as mf__drvwarnsm_dot_whl__warning__type__pb2
from mf_drvwarnsm import drv_tube_display_req_pb2 as mf__drvwarnsm_dot_drv__tube__display__req__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_drvwarnsm/logic_to_proc_port.proto',
  package='pb.mf_drvwarnsm.logic_to_proc_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n%mf_drvwarnsm/logic_to_proc_port.proto\x12\"pb.mf_drvwarnsm.logic_to_proc_port\x1a\x17\x65\x63o/signal_header.proto\x1a#mf_drvwarnsm/whl_warning_type.proto\x1a\'mf_drvwarnsm/drv_tube_display_req.proto\"\xa4\x02\n\x0fLogicToProcPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12L\n\x11whlWarningType_nu\x18\xaa\n \x03(\x0e\x32\x30.pb.mf_drvwarnsm.whl_warning_type.WhlWarningType\x12\x18\n\x0fsideResetReq_nu\x18\xec\x04 \x03(\x08\x12V\n\x14\x64rvTubeDisplayReq_nu\x18\xd5\x19 \x01(\x0e\x32\x37.pb.mf_drvwarnsm.drv_tube_display_req.DrvTubeDisplayReq\"`\n\x1aLogicToProcPort_array_port\x12\x42\n\x04\x64\x61ta\x18\x89\x11 \x03(\x0b\x32\x33.pb.mf_drvwarnsm.logic_to_proc_port.LogicToProcPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,mf__drvwarnsm_dot_whl__warning__type__pb2.DESCRIPTOR,mf__drvwarnsm_dot_drv__tube__display__req__pb2.DESCRIPTOR,])




_LOGICTOPROCPORT = _descriptor.Descriptor(
  name='LogicToProcPort',
  full_name='pb.mf_drvwarnsm.logic_to_proc_port.LogicToProcPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.mf_drvwarnsm.logic_to_proc_port.LogicToProcPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.mf_drvwarnsm.logic_to_proc_port.LogicToProcPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='whlWarningType_nu', full_name='pb.mf_drvwarnsm.logic_to_proc_port.LogicToProcPort.whlWarningType_nu', index=2,
      number=1322, type=14, cpp_type=8, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sideResetReq_nu', full_name='pb.mf_drvwarnsm.logic_to_proc_port.LogicToProcPort.sideResetReq_nu', index=3,
      number=620, type=8, cpp_type=7, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='drvTubeDisplayReq_nu', full_name='pb.mf_drvwarnsm.logic_to_proc_port.LogicToProcPort.drvTubeDisplayReq_nu', index=4,
      number=3285, type=14, cpp_type=8, label=1,
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
  serialized_start=181,
  serialized_end=473,
)


_LOGICTOPROCPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='LogicToProcPort_array_port',
  full_name='pb.mf_drvwarnsm.logic_to_proc_port.LogicToProcPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_drvwarnsm.logic_to_proc_port.LogicToProcPort_array_port.data', index=0,
      number=2185, type=11, cpp_type=10, label=3,
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
  serialized_start=475,
  serialized_end=571,
)

_LOGICTOPROCPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_LOGICTOPROCPORT.fields_by_name['whlWarningType_nu'].enum_type = mf__drvwarnsm_dot_whl__warning__type__pb2._WHLWARNINGTYPE
_LOGICTOPROCPORT.fields_by_name['drvTubeDisplayReq_nu'].enum_type = mf__drvwarnsm_dot_drv__tube__display__req__pb2._DRVTUBEDISPLAYREQ
_LOGICTOPROCPORT_ARRAY_PORT.fields_by_name['data'].message_type = _LOGICTOPROCPORT
DESCRIPTOR.message_types_by_name['LogicToProcPort'] = _LOGICTOPROCPORT
DESCRIPTOR.message_types_by_name['LogicToProcPort_array_port'] = _LOGICTOPROCPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LogicToProcPort = _reflection.GeneratedProtocolMessageType('LogicToProcPort', (_message.Message,), {
  'DESCRIPTOR' : _LOGICTOPROCPORT,
  '__module__' : 'mf_drvwarnsm.logic_to_proc_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_drvwarnsm.logic_to_proc_port.LogicToProcPort)
  })
_sym_db.RegisterMessage(LogicToProcPort)

LogicToProcPort_array_port = _reflection.GeneratedProtocolMessageType('LogicToProcPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _LOGICTOPROCPORT_ARRAY_PORT,
  '__module__' : 'mf_drvwarnsm.logic_to_proc_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_drvwarnsm.logic_to_proc_port.LogicToProcPort_array_port)
  })
_sym_db.RegisterMessage(LogicToProcPort_array_port)


# @@protoc_insertion_point(module_scope)