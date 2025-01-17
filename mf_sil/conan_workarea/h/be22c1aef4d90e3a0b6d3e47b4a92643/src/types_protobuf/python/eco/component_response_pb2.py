# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: eco/component_response.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from eco import comp_op_state_pb2 as eco_dot_comp__op__state__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='eco/component_response.proto',
  package='pb.eco.component_response',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1c\x65\x63o/component_response.proto\x12\x19pb.eco.component_response\x1a\x17\x65\x63o/signal_header.proto\x1a\x17\x65\x63o/comp_op_state.proto\"\xb2\x01\n\x11\x43omponentResponse\x12\x36\n\tsigHeader\x18\xaa\x0b \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x12\n\terrorCode\x18\xa7\x04 \x01(\x04\x12\x1c\n\x13\x66unctionalErrorCode\x18\xbc\n \x03(\x04\x12\x33\n\x07opState\x18\xef\x1e \x01(\x0e\x32!.pb.eco.comp_op_state.CompOpState\"[\n\x1c\x43omponentResponse_array_port\x12;\n\x04\x64\x61ta\x18\xc5\x08 \x03(\x0b\x32,.pb.eco.component_response.ComponentResponse'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,eco_dot_comp__op__state__pb2.DESCRIPTOR,])




_COMPONENTRESPONSE = _descriptor.Descriptor(
  name='ComponentResponse',
  full_name='pb.eco.component_response.ComponentResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='sigHeader', full_name='pb.eco.component_response.ComponentResponse.sigHeader', index=0,
      number=1450, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='errorCode', full_name='pb.eco.component_response.ComponentResponse.errorCode', index=1,
      number=551, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='functionalErrorCode', full_name='pb.eco.component_response.ComponentResponse.functionalErrorCode', index=2,
      number=1340, type=4, cpp_type=4, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='opState', full_name='pb.eco.component_response.ComponentResponse.opState', index=3,
      number=3951, type=14, cpp_type=8, label=1,
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
  serialized_start=110,
  serialized_end=288,
)


_COMPONENTRESPONSE_ARRAY_PORT = _descriptor.Descriptor(
  name='ComponentResponse_array_port',
  full_name='pb.eco.component_response.ComponentResponse_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.eco.component_response.ComponentResponse_array_port.data', index=0,
      number=1093, type=11, cpp_type=10, label=3,
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
  serialized_start=290,
  serialized_end=381,
)

_COMPONENTRESPONSE.fields_by_name['sigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_COMPONENTRESPONSE.fields_by_name['opState'].enum_type = eco_dot_comp__op__state__pb2._COMPOPSTATE
_COMPONENTRESPONSE_ARRAY_PORT.fields_by_name['data'].message_type = _COMPONENTRESPONSE
DESCRIPTOR.message_types_by_name['ComponentResponse'] = _COMPONENTRESPONSE
DESCRIPTOR.message_types_by_name['ComponentResponse_array_port'] = _COMPONENTRESPONSE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ComponentResponse = _reflection.GeneratedProtocolMessageType('ComponentResponse', (_message.Message,), {
  'DESCRIPTOR' : _COMPONENTRESPONSE,
  '__module__' : 'eco.component_response_pb2'
  # @@protoc_insertion_point(class_scope:pb.eco.component_response.ComponentResponse)
  })
_sym_db.RegisterMessage(ComponentResponse)

ComponentResponse_array_port = _reflection.GeneratedProtocolMessageType('ComponentResponse_array_port', (_message.Message,), {
  'DESCRIPTOR' : _COMPONENTRESPONSE_ARRAY_PORT,
  '__module__' : 'eco.component_response_pb2'
  # @@protoc_insertion_point(class_scope:pb.eco.component_response.ComponentResponse_array_port)
  })
_sym_db.RegisterMessage(ComponentResponse_array_port)


# @@protoc_insertion_point(module_scope)
