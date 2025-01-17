# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_whlprotectproc/whpproc_output_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from mf_whlprotectproc import whl_warning_level_pb2 as mf__whlprotectproc_dot_whl__warning__level__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_whlprotectproc/whpproc_output_port.proto',
  package='pb.mf_whlprotectproc.whpproc_output_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n+mf_whlprotectproc/whpproc_output_port.proto\x12(pb.mf_whlprotectproc.whpproc_output_port\x1a\x17\x65\x63o/signal_header.proto\x1a)mf_whlprotectproc/whl_warning_level.proto\"\xf8\x01\n\x11WHPProcOutputPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12T\n\x12whlWarningLevel_nu\x18\xc0\x1a \x03(\x0e\x32\x37.pb.mf_whlprotectproc.whl_warning_level.WhlWarningLevel\x12\x1d\n\x14whlWarningPresent_nu\x18\xe0\x03 \x01(\x08\x12\x1b\n\x12processingError_nu\x18\x81\x15 \x01(\x08\"j\n\x1cWHPProcOutputPort_array_port\x12J\n\x04\x64\x61ta\x18\xe8\x01 \x03(\x0b\x32;.pb.mf_whlprotectproc.whpproc_output_port.WHPProcOutputPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,mf__whlprotectproc_dot_whl__warning__level__pb2.DESCRIPTOR,])




_WHPPROCOUTPUTPORT = _descriptor.Descriptor(
  name='WHPProcOutputPort',
  full_name='pb.mf_whlprotectproc.whpproc_output_port.WHPProcOutputPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.mf_whlprotectproc.whpproc_output_port.WHPProcOutputPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.mf_whlprotectproc.whpproc_output_port.WHPProcOutputPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='whlWarningLevel_nu', full_name='pb.mf_whlprotectproc.whpproc_output_port.WHPProcOutputPort.whlWarningLevel_nu', index=2,
      number=3392, type=14, cpp_type=8, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='whlWarningPresent_nu', full_name='pb.mf_whlprotectproc.whpproc_output_port.WHPProcOutputPort.whlWarningPresent_nu', index=3,
      number=480, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='processingError_nu', full_name='pb.mf_whlprotectproc.whpproc_output_port.WHPProcOutputPort.processingError_nu', index=4,
      number=2689, type=8, cpp_type=7, label=1,
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
  serialized_start=158,
  serialized_end=406,
)


_WHPPROCOUTPUTPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='WHPProcOutputPort_array_port',
  full_name='pb.mf_whlprotectproc.whpproc_output_port.WHPProcOutputPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_whlprotectproc.whpproc_output_port.WHPProcOutputPort_array_port.data', index=0,
      number=232, type=11, cpp_type=10, label=3,
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
  serialized_start=408,
  serialized_end=514,
)

_WHPPROCOUTPUTPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_WHPPROCOUTPUTPORT.fields_by_name['whlWarningLevel_nu'].enum_type = mf__whlprotectproc_dot_whl__warning__level__pb2._WHLWARNINGLEVEL
_WHPPROCOUTPUTPORT_ARRAY_PORT.fields_by_name['data'].message_type = _WHPPROCOUTPUTPORT
DESCRIPTOR.message_types_by_name['WHPProcOutputPort'] = _WHPPROCOUTPUTPORT
DESCRIPTOR.message_types_by_name['WHPProcOutputPort_array_port'] = _WHPPROCOUTPUTPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

WHPProcOutputPort = _reflection.GeneratedProtocolMessageType('WHPProcOutputPort', (_message.Message,), {
  'DESCRIPTOR' : _WHPPROCOUTPUTPORT,
  '__module__' : 'mf_whlprotectproc.whpproc_output_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_whlprotectproc.whpproc_output_port.WHPProcOutputPort)
  })
_sym_db.RegisterMessage(WHPProcOutputPort)

WHPProcOutputPort_array_port = _reflection.GeneratedProtocolMessageType('WHPProcOutputPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _WHPPROCOUTPUTPORT_ARRAY_PORT,
  '__module__' : 'mf_whlprotectproc.whpproc_output_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_whlprotectproc.whpproc_output_port.WHPProcOutputPort_array_port)
  })
_sym_db.RegisterMessage(WHPProcOutputPort_array_port)


# @@protoc_insertion_point(module_scope)
