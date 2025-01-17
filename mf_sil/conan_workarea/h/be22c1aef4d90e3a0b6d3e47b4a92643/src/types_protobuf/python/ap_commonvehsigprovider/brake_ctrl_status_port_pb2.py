# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_commonvehsigprovider/brake_ctrl_status_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_commonvehsigprovider/brake_ctrl_status_port.proto',
  package='pb.ap_commonvehsigprovider.brake_ctrl_status_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n4ap_commonvehsigprovider/brake_ctrl_status_port.proto\x12\x31pb.ap_commonvehsigprovider.brake_ctrl_status_port\x1a\x17\x65\x63o/signal_header.proto\"\x83\x01\n\x13\x42rakeCtrlStatusPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x19\n\x10vehicleBraked_nu\x18\xf3\x0f \x01(\x08\"w\n\x1e\x42rakeCtrlStatusPort_array_port\x12U\n\x04\x64\x61ta\x18\xe8\x0b \x03(\x0b\x32\x46.pb.ap_commonvehsigprovider.brake_ctrl_status_port.BrakeCtrlStatusPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,])




_BRAKECTRLSTATUSPORT = _descriptor.Descriptor(
  name='BrakeCtrlStatusPort',
  full_name='pb.ap_commonvehsigprovider.brake_ctrl_status_port.BrakeCtrlStatusPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_commonvehsigprovider.brake_ctrl_status_port.BrakeCtrlStatusPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_commonvehsigprovider.brake_ctrl_status_port.BrakeCtrlStatusPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='vehicleBraked_nu', full_name='pb.ap_commonvehsigprovider.brake_ctrl_status_port.BrakeCtrlStatusPort.vehicleBraked_nu', index=2,
      number=2035, type=8, cpp_type=7, label=1,
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
  serialized_start=133,
  serialized_end=264,
)


_BRAKECTRLSTATUSPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='BrakeCtrlStatusPort_array_port',
  full_name='pb.ap_commonvehsigprovider.brake_ctrl_status_port.BrakeCtrlStatusPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_commonvehsigprovider.brake_ctrl_status_port.BrakeCtrlStatusPort_array_port.data', index=0,
      number=1512, type=11, cpp_type=10, label=3,
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
  serialized_start=266,
  serialized_end=385,
)

_BRAKECTRLSTATUSPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_BRAKECTRLSTATUSPORT_ARRAY_PORT.fields_by_name['data'].message_type = _BRAKECTRLSTATUSPORT
DESCRIPTOR.message_types_by_name['BrakeCtrlStatusPort'] = _BRAKECTRLSTATUSPORT
DESCRIPTOR.message_types_by_name['BrakeCtrlStatusPort_array_port'] = _BRAKECTRLSTATUSPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

BrakeCtrlStatusPort = _reflection.GeneratedProtocolMessageType('BrakeCtrlStatusPort', (_message.Message,), {
  'DESCRIPTOR' : _BRAKECTRLSTATUSPORT,
  '__module__' : 'ap_commonvehsigprovider.brake_ctrl_status_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.brake_ctrl_status_port.BrakeCtrlStatusPort)
  })
_sym_db.RegisterMessage(BrakeCtrlStatusPort)

BrakeCtrlStatusPort_array_port = _reflection.GeneratedProtocolMessageType('BrakeCtrlStatusPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _BRAKECTRLSTATUSPORT_ARRAY_PORT,
  '__module__' : 'ap_commonvehsigprovider.brake_ctrl_status_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.brake_ctrl_status_port.BrakeCtrlStatusPort_array_port)
  })
_sym_db.RegisterMessage(BrakeCtrlStatusPort_array_port)


# @@protoc_insertion_point(module_scope)
