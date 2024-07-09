# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_lsca/lsca_brake_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from mf_lsca import lsca_brake_mode_pb2 as mf__lsca_dot_lsca__brake__mode__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_lsca/lsca_brake_port.proto',
  package='pb.mf_lsca.lsca_brake_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1dmf_lsca/lsca_brake_port.proto\x12\x1apb.mf_lsca.lsca_brake_port\x1a\x17\x65\x63o/signal_header.proto\x1a\x1dmf_lsca/lsca_brake_mode.proto\"\xdd\x01\n\rLscaBrakePort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x19\n\x10\x64istanceToStop_m\x18\xf0\x03 \x01(\x02\x12\x1b\n\x13holdInStandstill_nu\x18L \x01(\x08\x12\x41\n\x0brequestMode\x18\xaa\x05 \x01(\x0e\x32+.pb.mf_lsca.lsca_brake_mode.LSCA_BRAKE_MODE\"T\n\x18LscaBrakePort_array_port\x12\x38\n\x04\x64\x61ta\x18\xc4\x1b \x03(\x0b\x32).pb.mf_lsca.lsca_brake_port.LscaBrakePort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,mf__lsca_dot_lsca__brake__mode__pb2.DESCRIPTOR,])




_LSCABRAKEPORT = _descriptor.Descriptor(
  name='LscaBrakePort',
  full_name='pb.mf_lsca.lsca_brake_port.LscaBrakePort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.mf_lsca.lsca_brake_port.LscaBrakePort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.mf_lsca.lsca_brake_port.LscaBrakePort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='distanceToStop_m', full_name='pb.mf_lsca.lsca_brake_port.LscaBrakePort.distanceToStop_m', index=2,
      number=496, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='holdInStandstill_nu', full_name='pb.mf_lsca.lsca_brake_port.LscaBrakePort.holdInStandstill_nu', index=3,
      number=76, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='requestMode', full_name='pb.mf_lsca.lsca_brake_port.LscaBrakePort.requestMode', index=4,
      number=682, type=14, cpp_type=8, label=1,
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
  serialized_start=118,
  serialized_end=339,
)


_LSCABRAKEPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='LscaBrakePort_array_port',
  full_name='pb.mf_lsca.lsca_brake_port.LscaBrakePort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_lsca.lsca_brake_port.LscaBrakePort_array_port.data', index=0,
      number=3524, type=11, cpp_type=10, label=3,
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
  serialized_start=341,
  serialized_end=425,
)

_LSCABRAKEPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_LSCABRAKEPORT.fields_by_name['requestMode'].enum_type = mf__lsca_dot_lsca__brake__mode__pb2._LSCA_BRAKE_MODE
_LSCABRAKEPORT_ARRAY_PORT.fields_by_name['data'].message_type = _LSCABRAKEPORT
DESCRIPTOR.message_types_by_name['LscaBrakePort'] = _LSCABRAKEPORT
DESCRIPTOR.message_types_by_name['LscaBrakePort_array_port'] = _LSCABRAKEPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LscaBrakePort = _reflection.GeneratedProtocolMessageType('LscaBrakePort', (_message.Message,), {
  'DESCRIPTOR' : _LSCABRAKEPORT,
  '__module__' : 'mf_lsca.lsca_brake_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_brake_port.LscaBrakePort)
  })
_sym_db.RegisterMessage(LscaBrakePort)

LscaBrakePort_array_port = _reflection.GeneratedProtocolMessageType('LscaBrakePort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _LSCABRAKEPORT_ARRAY_PORT,
  '__module__' : 'mf_lsca.lsca_brake_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.lsca_brake_port.LscaBrakePort_array_port)
  })
_sym_db.RegisterMessage(LscaBrakePort_array_port)


# @@protoc_insertion_point(module_scope)
