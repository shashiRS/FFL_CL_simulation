# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: hpsd/health_vector_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from hpsd import health_vector_pb2 as hpsd_dot_health__vector__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='hpsd/health_vector_port.proto',
  package='pb.hpsd.health_vector_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1dhpsd/health_vector_port.proto\x12\x1apb.hpsd.health_vector_port\x1a\x17\x65\x63o/signal_header.proto\x1a\x18hpsd/health_vector.proto\"\xa1\x01\n\x10HealthVectorPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12:\n\x0chealthVector\x18\xfd\x0b \x01(\x0b\x32#.pb.hpsd.health_vector.HealthVector\"Y\n\x1bHealthVectorPort_array_port\x12:\n\x04\x64\x61ta\x18\x42 \x03(\x0b\x32,.pb.hpsd.health_vector_port.HealthVectorPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,hpsd_dot_health__vector__pb2.DESCRIPTOR,])




_HEALTHVECTORPORT = _descriptor.Descriptor(
  name='HealthVectorPort',
  full_name='pb.hpsd.health_vector_port.HealthVectorPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.hpsd.health_vector_port.HealthVectorPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.hpsd.health_vector_port.HealthVectorPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='healthVector', full_name='pb.hpsd.health_vector_port.HealthVectorPort.healthVector', index=2,
      number=1533, type=11, cpp_type=10, label=1,
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
  serialized_start=113,
  serialized_end=274,
)


_HEALTHVECTORPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='HealthVectorPort_array_port',
  full_name='pb.hpsd.health_vector_port.HealthVectorPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.hpsd.health_vector_port.HealthVectorPort_array_port.data', index=0,
      number=66, type=11, cpp_type=10, label=3,
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
  serialized_start=276,
  serialized_end=365,
)

_HEALTHVECTORPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_HEALTHVECTORPORT.fields_by_name['healthVector'].message_type = hpsd_dot_health__vector__pb2._HEALTHVECTOR
_HEALTHVECTORPORT_ARRAY_PORT.fields_by_name['data'].message_type = _HEALTHVECTORPORT
DESCRIPTOR.message_types_by_name['HealthVectorPort'] = _HEALTHVECTORPORT
DESCRIPTOR.message_types_by_name['HealthVectorPort_array_port'] = _HEALTHVECTORPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

HealthVectorPort = _reflection.GeneratedProtocolMessageType('HealthVectorPort', (_message.Message,), {
  'DESCRIPTOR' : _HEALTHVECTORPORT,
  '__module__' : 'hpsd.health_vector_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.hpsd.health_vector_port.HealthVectorPort)
  })
_sym_db.RegisterMessage(HealthVectorPort)

HealthVectorPort_array_port = _reflection.GeneratedProtocolMessageType('HealthVectorPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _HEALTHVECTORPORT_ARRAY_PORT,
  '__module__' : 'hpsd.health_vector_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.hpsd.health_vector_port.HealthVectorPort_array_port)
  })
_sym_db.RegisterMessage(HealthVectorPort_array_port)


# @@protoc_insertion_point(module_scope)