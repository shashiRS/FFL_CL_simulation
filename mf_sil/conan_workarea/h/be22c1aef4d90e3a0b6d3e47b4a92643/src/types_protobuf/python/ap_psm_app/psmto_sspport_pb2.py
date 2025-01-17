# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_psm_app/psmto_sspport.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from ap_psm_app import engine_req_pb2 as ap__psm__app_dot_engine__req__pb2
from ap_psm_app import bcmreq_pb2 as ap__psm__app_dot_bcmreq__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_psm_app/psmto_sspport.proto',
  package='pb.ap_psm_app.psmto_sspport',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1e\x61p_psm_app/psmto_sspport.proto\x12\x1bpb.ap_psm_app.psmto_sspport\x1a\x17\x65\x63o/signal_header.proto\x1a\x1b\x61p_psm_app/engine_req.proto\x1a\x17\x61p_psm_app/bcmreq.proto\"\xc9\x01\n\x0cPSMToSSPPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x37\n\tengineReq\x18\xe8\x17 \x01(\x0b\x32#.pb.ap_psm_app.engine_req.EngineReq\x12-\n\x06\x62\x63mReq\x18\xd7\x17 \x01(\x0b\x32\x1c.pb.ap_psm_app.bcmreq.BCMReq\"S\n\x17PSMToSSPPort_array_port\x12\x38\n\x04\x64\x61ta\x18\x9f\x10 \x03(\x0b\x32).pb.ap_psm_app.psmto_sspport.PSMToSSPPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,ap__psm__app_dot_engine__req__pb2.DESCRIPTOR,ap__psm__app_dot_bcmreq__pb2.DESCRIPTOR,])




_PSMTOSSPPORT = _descriptor.Descriptor(
  name='PSMToSSPPort',
  full_name='pb.ap_psm_app.psmto_sspport.PSMToSSPPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_psm_app.psmto_sspport.PSMToSSPPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_psm_app.psmto_sspport.PSMToSSPPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='engineReq', full_name='pb.ap_psm_app.psmto_sspport.PSMToSSPPort.engineReq', index=2,
      number=3048, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bcmReq', full_name='pb.ap_psm_app.psmto_sspport.PSMToSSPPort.bcmReq', index=3,
      number=3031, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
  serialized_start=143,
  serialized_end=344,
)


_PSMTOSSPPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='PSMToSSPPort_array_port',
  full_name='pb.ap_psm_app.psmto_sspport.PSMToSSPPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_psm_app.psmto_sspport.PSMToSSPPort_array_port.data', index=0,
      number=2079, type=11, cpp_type=10, label=3,
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
  serialized_start=346,
  serialized_end=429,
)

_PSMTOSSPPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_PSMTOSSPPORT.fields_by_name['engineReq'].message_type = ap__psm__app_dot_engine__req__pb2._ENGINEREQ
_PSMTOSSPPORT.fields_by_name['bcmReq'].message_type = ap__psm__app_dot_bcmreq__pb2._BCMREQ
_PSMTOSSPPORT_ARRAY_PORT.fields_by_name['data'].message_type = _PSMTOSSPPORT
DESCRIPTOR.message_types_by_name['PSMToSSPPort'] = _PSMTOSSPPORT
DESCRIPTOR.message_types_by_name['PSMToSSPPort_array_port'] = _PSMTOSSPPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PSMToSSPPort = _reflection.GeneratedProtocolMessageType('PSMToSSPPort', (_message.Message,), {
  'DESCRIPTOR' : _PSMTOSSPPORT,
  '__module__' : 'ap_psm_app.psmto_sspport_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm_app.psmto_sspport.PSMToSSPPort)
  })
_sym_db.RegisterMessage(PSMToSSPPort)

PSMToSSPPort_array_port = _reflection.GeneratedProtocolMessageType('PSMToSSPPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _PSMTOSSPPORT_ARRAY_PORT,
  '__module__' : 'ap_psm_app.psmto_sspport_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm_app.psmto_sspport.PSMToSSPPort_array_port)
  })
_sym_db.RegisterMessage(PSMToSSPPort_array_port)


# @@protoc_insertion_point(module_scope)
