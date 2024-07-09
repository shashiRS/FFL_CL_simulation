# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_ladmc/la_dmcstatus_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from ap_ladmc import la_dmcsystem_state_pb2 as ap__ladmc_dot_la__dmcsystem__state__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_ladmc/la_dmcstatus_port.proto',
  package='pb.ap_ladmc.la_dmcstatus_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n ap_ladmc/la_dmcstatus_port.proto\x12\x1dpb.ap_ladmc.la_dmcstatus_port\x1a\x17\x65\x63o/signal_header.proto\x1a!ap_ladmc/la_dmcsystem_state.proto\"\x97\x02\n\x0fLaDMCStatusPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x1d\n\x15handSteeringTorque_Nm\x18\x38 \x01(\x02\x12!\n\x18handSteeringTorque_QF_nu\x18\xcb\x16 \x01(\x08\x12P\n\x13laDMCSystemState_nu\x18\x9c\xf1\xfa\x03 \x01(\x0e\x32\x30.pb.ap_ladmc.la_dmcsystem_state.LaDMCSystemState\x12\x1d\n\x15\x64riverIntervention_nu\x18\x62 \x01(\x08\"[\n\x1aLaDMCStatusPort_array_port\x12=\n\x04\x64\x61ta\x18\xd4\x1f \x03(\x0b\x32..pb.ap_ladmc.la_dmcstatus_port.LaDMCStatusPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,ap__ladmc_dot_la__dmcsystem__state__pb2.DESCRIPTOR,])




_LADMCSTATUSPORT = _descriptor.Descriptor(
  name='LaDMCStatusPort',
  full_name='pb.ap_ladmc.la_dmcstatus_port.LaDMCStatusPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_ladmc.la_dmcstatus_port.LaDMCStatusPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_ladmc.la_dmcstatus_port.LaDMCStatusPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='handSteeringTorque_Nm', full_name='pb.ap_ladmc.la_dmcstatus_port.LaDMCStatusPort.handSteeringTorque_Nm', index=2,
      number=56, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='handSteeringTorque_QF_nu', full_name='pb.ap_ladmc.la_dmcstatus_port.LaDMCStatusPort.handSteeringTorque_QF_nu', index=3,
      number=2891, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='laDMCSystemState_nu', full_name='pb.ap_ladmc.la_dmcstatus_port.LaDMCStatusPort.laDMCSystemState_nu', index=4,
      number=8304796, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='driverIntervention_nu', full_name='pb.ap_ladmc.la_dmcstatus_port.LaDMCStatusPort.driverIntervention_nu', index=5,
      number=98, type=8, cpp_type=7, label=1,
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
  serialized_start=128,
  serialized_end=407,
)


_LADMCSTATUSPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='LaDMCStatusPort_array_port',
  full_name='pb.ap_ladmc.la_dmcstatus_port.LaDMCStatusPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_ladmc.la_dmcstatus_port.LaDMCStatusPort_array_port.data', index=0,
      number=4052, type=11, cpp_type=10, label=3,
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
  serialized_start=409,
  serialized_end=500,
)

_LADMCSTATUSPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_LADMCSTATUSPORT.fields_by_name['laDMCSystemState_nu'].enum_type = ap__ladmc_dot_la__dmcsystem__state__pb2._LADMCSYSTEMSTATE
_LADMCSTATUSPORT_ARRAY_PORT.fields_by_name['data'].message_type = _LADMCSTATUSPORT
DESCRIPTOR.message_types_by_name['LaDMCStatusPort'] = _LADMCSTATUSPORT
DESCRIPTOR.message_types_by_name['LaDMCStatusPort_array_port'] = _LADMCSTATUSPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LaDMCStatusPort = _reflection.GeneratedProtocolMessageType('LaDMCStatusPort', (_message.Message,), {
  'DESCRIPTOR' : _LADMCSTATUSPORT,
  '__module__' : 'ap_ladmc.la_dmcstatus_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_ladmc.la_dmcstatus_port.LaDMCStatusPort)
  })
_sym_db.RegisterMessage(LaDMCStatusPort)

LaDMCStatusPort_array_port = _reflection.GeneratedProtocolMessageType('LaDMCStatusPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _LADMCSTATUSPORT_ARRAY_PORT,
  '__module__' : 'ap_ladmc.la_dmcstatus_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_ladmc.la_dmcstatus_port.LaDMCStatusPort_array_port)
  })
_sym_db.RegisterMessage(LaDMCStatusPort_array_port)


# @@protoc_insertion_point(module_scope)