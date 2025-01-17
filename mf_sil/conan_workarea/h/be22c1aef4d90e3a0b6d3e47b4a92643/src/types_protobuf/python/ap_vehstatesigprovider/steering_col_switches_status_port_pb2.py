# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_vehstatesigprovider/steering_col_switches_status_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_vehstatesigprovider/steering_col_switches_status_port.proto',
  package='pb.ap_vehstatesigprovider.steering_col_switches_status_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n>ap_vehstatesigprovider/steering_col_switches_status_port.proto\x12;pb.ap_vehstatesigprovider.steering_col_switches_status_port\x1a\x17\x65\x63o/signal_header.proto\"\x9f\x01\n\x1dSteeringColSwitchesStatusPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x14\n\x0b\x63trlLeft_nu\x18\xe4\x05 \x01(\x08\x12\x15\n\x0c\x63trlRight_nu\x18\xd7\x1f \x01(\x08\"\x95\x01\n(SteeringColSwitchesStatusPort_array_port\x12i\n\x04\x64\x61ta\x18\xa4\x03 \x03(\x0b\x32Z.pb.ap_vehstatesigprovider.steering_col_switches_status_port.SteeringColSwitchesStatusPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,])




_STEERINGCOLSWITCHESSTATUSPORT = _descriptor.Descriptor(
  name='SteeringColSwitchesStatusPort',
  full_name='pb.ap_vehstatesigprovider.steering_col_switches_status_port.SteeringColSwitchesStatusPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_vehstatesigprovider.steering_col_switches_status_port.SteeringColSwitchesStatusPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_vehstatesigprovider.steering_col_switches_status_port.SteeringColSwitchesStatusPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ctrlLeft_nu', full_name='pb.ap_vehstatesigprovider.steering_col_switches_status_port.SteeringColSwitchesStatusPort.ctrlLeft_nu', index=2,
      number=740, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ctrlRight_nu', full_name='pb.ap_vehstatesigprovider.steering_col_switches_status_port.SteeringColSwitchesStatusPort.ctrlRight_nu', index=3,
      number=4055, type=8, cpp_type=7, label=1,
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
  serialized_start=153,
  serialized_end=312,
)


_STEERINGCOLSWITCHESSTATUSPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='SteeringColSwitchesStatusPort_array_port',
  full_name='pb.ap_vehstatesigprovider.steering_col_switches_status_port.SteeringColSwitchesStatusPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_vehstatesigprovider.steering_col_switches_status_port.SteeringColSwitchesStatusPort_array_port.data', index=0,
      number=420, type=11, cpp_type=10, label=3,
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
  serialized_start=315,
  serialized_end=464,
)

_STEERINGCOLSWITCHESSTATUSPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_STEERINGCOLSWITCHESSTATUSPORT_ARRAY_PORT.fields_by_name['data'].message_type = _STEERINGCOLSWITCHESSTATUSPORT
DESCRIPTOR.message_types_by_name['SteeringColSwitchesStatusPort'] = _STEERINGCOLSWITCHESSTATUSPORT
DESCRIPTOR.message_types_by_name['SteeringColSwitchesStatusPort_array_port'] = _STEERINGCOLSWITCHESSTATUSPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SteeringColSwitchesStatusPort = _reflection.GeneratedProtocolMessageType('SteeringColSwitchesStatusPort', (_message.Message,), {
  'DESCRIPTOR' : _STEERINGCOLSWITCHESSTATUSPORT,
  '__module__' : 'ap_vehstatesigprovider.steering_col_switches_status_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.steering_col_switches_status_port.SteeringColSwitchesStatusPort)
  })
_sym_db.RegisterMessage(SteeringColSwitchesStatusPort)

SteeringColSwitchesStatusPort_array_port = _reflection.GeneratedProtocolMessageType('SteeringColSwitchesStatusPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _STEERINGCOLSWITCHESSTATUSPORT_ARRAY_PORT,
  '__module__' : 'ap_vehstatesigprovider.steering_col_switches_status_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.steering_col_switches_status_port.SteeringColSwitchesStatusPort_array_port)
  })
_sym_db.RegisterMessage(SteeringColSwitchesStatusPort_array_port)


# @@protoc_insertion_point(module_scope)
