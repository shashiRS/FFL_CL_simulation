# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_processing/us_processing_point_list.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from us_processing import us_processing_point_pb2 as us__processing_dot_us__processing__point__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_processing/us_processing_point_list.proto',
  package='pb.us_processing.us_processing_point_list',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n,us_processing/us_processing_point_list.proto\x12)pb.us_processing.us_processing_point_list\x1a\x17\x65\x63o/signal_header.proto\x1a\'us_processing/us_processing_point.proto\"\xcd\x01\n\x15UsProcessingPointList\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x17\n\x0enumberOfPoints\x18\xda\x19 \x01(\r\x12H\n\x06points\x18\xca\x05 \x03(\x0b\x32\x37.pb.us_processing.us_processing_point.UsProcessingPoint\"s\n UsProcessingPointList_array_port\x12O\n\x04\x64\x61ta\x18\xb6\x19 \x03(\x0b\x32@.pb.us_processing.us_processing_point_list.UsProcessingPointList'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,us__processing_dot_us__processing__point__pb2.DESCRIPTOR,])




_USPROCESSINGPOINTLIST = _descriptor.Descriptor(
  name='UsProcessingPointList',
  full_name='pb.us_processing.us_processing_point_list.UsProcessingPointList',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.us_processing.us_processing_point_list.UsProcessingPointList.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.us_processing.us_processing_point_list.UsProcessingPointList.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numberOfPoints', full_name='pb.us_processing.us_processing_point_list.UsProcessingPointList.numberOfPoints', index=2,
      number=3290, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='points', full_name='pb.us_processing.us_processing_point_list.UsProcessingPointList.points', index=3,
      number=714, type=11, cpp_type=10, label=3,
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
  serialized_start=158,
  serialized_end=363,
)


_USPROCESSINGPOINTLIST_ARRAY_PORT = _descriptor.Descriptor(
  name='UsProcessingPointList_array_port',
  full_name='pb.us_processing.us_processing_point_list.UsProcessingPointList_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_processing.us_processing_point_list.UsProcessingPointList_array_port.data', index=0,
      number=3254, type=11, cpp_type=10, label=3,
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
  serialized_start=365,
  serialized_end=480,
)

_USPROCESSINGPOINTLIST.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_USPROCESSINGPOINTLIST.fields_by_name['points'].message_type = us__processing_dot_us__processing__point__pb2._USPROCESSINGPOINT
_USPROCESSINGPOINTLIST_ARRAY_PORT.fields_by_name['data'].message_type = _USPROCESSINGPOINTLIST
DESCRIPTOR.message_types_by_name['UsProcessingPointList'] = _USPROCESSINGPOINTLIST
DESCRIPTOR.message_types_by_name['UsProcessingPointList_array_port'] = _USPROCESSINGPOINTLIST_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsProcessingPointList = _reflection.GeneratedProtocolMessageType('UsProcessingPointList', (_message.Message,), {
  'DESCRIPTOR' : _USPROCESSINGPOINTLIST,
  '__module__' : 'us_processing.us_processing_point_list_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_point_list.UsProcessingPointList)
  })
_sym_db.RegisterMessage(UsProcessingPointList)

UsProcessingPointList_array_port = _reflection.GeneratedProtocolMessageType('UsProcessingPointList_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USPROCESSINGPOINTLIST_ARRAY_PORT,
  '__module__' : 'us_processing.us_processing_point_list_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_point_list.UsProcessingPointList_array_port)
  })
_sym_db.RegisterMessage(UsProcessingPointList_array_port)


# @@protoc_insertion_point(module_scope)
