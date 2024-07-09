# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_processing/us_processing_filtered_echo_list.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from us_processing import us_processing_filtered_echo_pb2 as us__processing_dot_us__processing__filtered__echo__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_processing/us_processing_filtered_echo_list.proto',
  package='pb.us_processing.us_processing_filtered_echo_list',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n4us_processing/us_processing_filtered_echo_list.proto\x12\x31pb.us_processing.us_processing_filtered_echo_list\x1a\x17\x65\x63o/signal_header.proto\x1a/us_processing/us_processing_filtered_echo.proto\"\xde\x01\n\x1cUsProcessingFilteredEchoList\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x12\n\tnumEchoes\x18\xd5\r \x01(\r\x12W\n\x06\x65\x63hoes\x18\xd9\x1a \x03(\x0b\x32\x46.pb.us_processing.us_processing_filtered_echo.UsProcessingFilteredEcho\"\x89\x01\n\'UsProcessingFilteredEchoList_array_port\x12^\n\x04\x64\x61ta\x18\xf2\x0f \x03(\x0b\x32O.pb.us_processing.us_processing_filtered_echo_list.UsProcessingFilteredEchoList'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,us__processing_dot_us__processing__filtered__echo__pb2.DESCRIPTOR,])




_USPROCESSINGFILTEREDECHOLIST = _descriptor.Descriptor(
  name='UsProcessingFilteredEchoList',
  full_name='pb.us_processing.us_processing_filtered_echo_list.UsProcessingFilteredEchoList',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.us_processing.us_processing_filtered_echo_list.UsProcessingFilteredEchoList.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.us_processing.us_processing_filtered_echo_list.UsProcessingFilteredEchoList.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numEchoes', full_name='pb.us_processing.us_processing_filtered_echo_list.UsProcessingFilteredEchoList.numEchoes', index=2,
      number=1749, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='echoes', full_name='pb.us_processing.us_processing_filtered_echo_list.UsProcessingFilteredEchoList.echoes', index=3,
      number=3417, type=11, cpp_type=10, label=3,
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
  serialized_start=182,
  serialized_end=404,
)


_USPROCESSINGFILTEREDECHOLIST_ARRAY_PORT = _descriptor.Descriptor(
  name='UsProcessingFilteredEchoList_array_port',
  full_name='pb.us_processing.us_processing_filtered_echo_list.UsProcessingFilteredEchoList_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_processing.us_processing_filtered_echo_list.UsProcessingFilteredEchoList_array_port.data', index=0,
      number=2034, type=11, cpp_type=10, label=3,
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
  serialized_start=407,
  serialized_end=544,
)

_USPROCESSINGFILTEREDECHOLIST.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_USPROCESSINGFILTEREDECHOLIST.fields_by_name['echoes'].message_type = us__processing_dot_us__processing__filtered__echo__pb2._USPROCESSINGFILTEREDECHO
_USPROCESSINGFILTEREDECHOLIST_ARRAY_PORT.fields_by_name['data'].message_type = _USPROCESSINGFILTEREDECHOLIST
DESCRIPTOR.message_types_by_name['UsProcessingFilteredEchoList'] = _USPROCESSINGFILTEREDECHOLIST
DESCRIPTOR.message_types_by_name['UsProcessingFilteredEchoList_array_port'] = _USPROCESSINGFILTEREDECHOLIST_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsProcessingFilteredEchoList = _reflection.GeneratedProtocolMessageType('UsProcessingFilteredEchoList', (_message.Message,), {
  'DESCRIPTOR' : _USPROCESSINGFILTEREDECHOLIST,
  '__module__' : 'us_processing.us_processing_filtered_echo_list_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_filtered_echo_list.UsProcessingFilteredEchoList)
  })
_sym_db.RegisterMessage(UsProcessingFilteredEchoList)

UsProcessingFilteredEchoList_array_port = _reflection.GeneratedProtocolMessageType('UsProcessingFilteredEchoList_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USPROCESSINGFILTEREDECHOLIST_ARRAY_PORT,
  '__module__' : 'us_processing.us_processing_filtered_echo_list_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_filtered_echo_list.UsProcessingFilteredEchoList_array_port)
  })
_sym_db.RegisterMessage(UsProcessingFilteredEchoList_array_port)


# @@protoc_insertion_point(module_scope)