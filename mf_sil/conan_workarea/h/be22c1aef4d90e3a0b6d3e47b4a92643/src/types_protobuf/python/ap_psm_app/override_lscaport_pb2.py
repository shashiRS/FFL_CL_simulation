# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_psm_app/override_lscaport.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from ap_common import driving_direction_pb2 as ap__common_dot_driving__direction__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_psm_app/override_lscaport.proto',
  package='pb.ap_psm_app.override_lscaport',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\"ap_psm_app/override_lscaport.proto\x12\x1fpb.ap_psm_app.override_lscaport\x1a\x17\x65\x63o/signal_header.proto\x1a!ap_common/driving_direction.proto\"\xd3\x01\n\x10OverrideLSCAPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x1d\n\x14ignoreDistanceLSCA_m\x18\xfd\x15 \x01(\x02\x12M\n\x12ignoreDirection_nu\x18\xcb\x05 \x01(\x0e\x32\x30.pb.ap_common.driving_direction.DrivingDirection\"_\n\x1bOverrideLSCAPort_array_port\x12@\n\x04\x64\x61ta\x18\xb8\r \x03(\x0b\x32\x31.pb.ap_psm_app.override_lscaport.OverrideLSCAPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,ap__common_dot_driving__direction__pb2.DESCRIPTOR,])




_OVERRIDELSCAPORT = _descriptor.Descriptor(
  name='OverrideLSCAPort',
  full_name='pb.ap_psm_app.override_lscaport.OverrideLSCAPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_psm_app.override_lscaport.OverrideLSCAPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_psm_app.override_lscaport.OverrideLSCAPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ignoreDistanceLSCA_m', full_name='pb.ap_psm_app.override_lscaport.OverrideLSCAPort.ignoreDistanceLSCA_m', index=2,
      number=2813, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ignoreDirection_nu', full_name='pb.ap_psm_app.override_lscaport.OverrideLSCAPort.ignoreDirection_nu', index=3,
      number=715, type=14, cpp_type=8, label=1,
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
  serialized_start=132,
  serialized_end=343,
)


_OVERRIDELSCAPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='OverrideLSCAPort_array_port',
  full_name='pb.ap_psm_app.override_lscaport.OverrideLSCAPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_psm_app.override_lscaport.OverrideLSCAPort_array_port.data', index=0,
      number=1720, type=11, cpp_type=10, label=3,
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
  serialized_start=345,
  serialized_end=440,
)

_OVERRIDELSCAPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_OVERRIDELSCAPORT.fields_by_name['ignoreDirection_nu'].enum_type = ap__common_dot_driving__direction__pb2._DRIVINGDIRECTION
_OVERRIDELSCAPORT_ARRAY_PORT.fields_by_name['data'].message_type = _OVERRIDELSCAPORT
DESCRIPTOR.message_types_by_name['OverrideLSCAPort'] = _OVERRIDELSCAPORT
DESCRIPTOR.message_types_by_name['OverrideLSCAPort_array_port'] = _OVERRIDELSCAPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

OverrideLSCAPort = _reflection.GeneratedProtocolMessageType('OverrideLSCAPort', (_message.Message,), {
  'DESCRIPTOR' : _OVERRIDELSCAPORT,
  '__module__' : 'ap_psm_app.override_lscaport_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm_app.override_lscaport.OverrideLSCAPort)
  })
_sym_db.RegisterMessage(OverrideLSCAPort)

OverrideLSCAPort_array_port = _reflection.GeneratedProtocolMessageType('OverrideLSCAPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _OVERRIDELSCAPORT_ARRAY_PORT,
  '__module__' : 'ap_psm_app.override_lscaport_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm_app.override_lscaport.OverrideLSCAPort_array_port)
  })
_sym_db.RegisterMessage(OverrideLSCAPort_array_port)


# @@protoc_insertion_point(module_scope)
