# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: tce/tce_pers_data_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='tce/tce_pers_data_port.proto',
  package='pb.tce.tce_pers_data_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1ctce/tce_pers_data_port.proto\x12\x19pb.tce.tce_pers_data_port\x1a\x17\x65\x63o/signal_header.proto\"\xa4\x03\n\x0fTcePersDataPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x19\n\x10tireCircFL_0p1mm\x18\xd5\x14 \x01(\r\x12\x19\n\x10tireCircFR_0p1mm\x18\xa6\x1f \x01(\r\x12\x19\n\x10tireCircRL_0p1mm\x18\x8c\x11 \x01(\r\x12\x19\n\x10tireCircRR_0p1mm\x18\xff\x1a \x01(\r\x12\x1d\n\x14tireCircStdvFL_0p1mm\x18\xbe\x1f \x01(\r\x12\x1d\n\x14tireCircStdvFR_0p1mm\x18\xcd\x14 \x01(\r\x12\x1d\n\x14tireCircStdvRL_0p1mm\x18\xe7\x1a \x01(\r\x12\x1d\n\x14tireCircStdvRR_0p1mm\x18\x94\x11 \x01(\r\x12\x1d\n\x14rearTrackWidth_0p1mm\x18\x8c\n \x01(\r\x12!\n\x18rearTrackWidthStdv_0p1mm\x18\xee\x1f \x01(\r\x12\x14\n\x0b\x64\x61taChanged\x18\xcf\x07 \x01(\x08\"W\n\x1aTcePersDataPort_array_port\x12\x39\n\x04\x64\x61ta\x18\xb2\t \x03(\x0b\x32*.pb.tce.tce_pers_data_port.TcePersDataPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,])




_TCEPERSDATAPORT = _descriptor.Descriptor(
  name='TcePersDataPort',
  full_name='pb.tce.tce_pers_data_port.TcePersDataPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.tce.tce_pers_data_port.TcePersDataPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.tce.tce_pers_data_port.TcePersDataPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircFL_0p1mm', full_name='pb.tce.tce_pers_data_port.TcePersDataPort.tireCircFL_0p1mm', index=2,
      number=2645, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircFR_0p1mm', full_name='pb.tce.tce_pers_data_port.TcePersDataPort.tireCircFR_0p1mm', index=3,
      number=4006, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircRL_0p1mm', full_name='pb.tce.tce_pers_data_port.TcePersDataPort.tireCircRL_0p1mm', index=4,
      number=2188, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircRR_0p1mm', full_name='pb.tce.tce_pers_data_port.TcePersDataPort.tireCircRR_0p1mm', index=5,
      number=3455, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircStdvFL_0p1mm', full_name='pb.tce.tce_pers_data_port.TcePersDataPort.tireCircStdvFL_0p1mm', index=6,
      number=4030, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircStdvFR_0p1mm', full_name='pb.tce.tce_pers_data_port.TcePersDataPort.tireCircStdvFR_0p1mm', index=7,
      number=2637, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircStdvRL_0p1mm', full_name='pb.tce.tce_pers_data_port.TcePersDataPort.tireCircStdvRL_0p1mm', index=8,
      number=3431, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircStdvRR_0p1mm', full_name='pb.tce.tce_pers_data_port.TcePersDataPort.tireCircStdvRR_0p1mm', index=9,
      number=2196, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rearTrackWidth_0p1mm', full_name='pb.tce.tce_pers_data_port.TcePersDataPort.rearTrackWidth_0p1mm', index=10,
      number=1292, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rearTrackWidthStdv_0p1mm', full_name='pb.tce.tce_pers_data_port.TcePersDataPort.rearTrackWidthStdv_0p1mm', index=11,
      number=4078, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='dataChanged', full_name='pb.tce.tce_pers_data_port.TcePersDataPort.dataChanged', index=12,
      number=975, type=8, cpp_type=7, label=1,
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
  serialized_start=85,
  serialized_end=505,
)


_TCEPERSDATAPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='TcePersDataPort_array_port',
  full_name='pb.tce.tce_pers_data_port.TcePersDataPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.tce.tce_pers_data_port.TcePersDataPort_array_port.data', index=0,
      number=1202, type=11, cpp_type=10, label=3,
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
  serialized_start=507,
  serialized_end=594,
)

_TCEPERSDATAPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_TCEPERSDATAPORT_ARRAY_PORT.fields_by_name['data'].message_type = _TCEPERSDATAPORT
DESCRIPTOR.message_types_by_name['TcePersDataPort'] = _TCEPERSDATAPORT
DESCRIPTOR.message_types_by_name['TcePersDataPort_array_port'] = _TCEPERSDATAPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TcePersDataPort = _reflection.GeneratedProtocolMessageType('TcePersDataPort', (_message.Message,), {
  'DESCRIPTOR' : _TCEPERSDATAPORT,
  '__module__' : 'tce.tce_pers_data_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.tce.tce_pers_data_port.TcePersDataPort)
  })
_sym_db.RegisterMessage(TcePersDataPort)

TcePersDataPort_array_port = _reflection.GeneratedProtocolMessageType('TcePersDataPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _TCEPERSDATAPORT_ARRAY_PORT,
  '__module__' : 'tce.tce_pers_data_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.tce.tce_pers_data_port.TcePersDataPort_array_port)
  })
_sym_db.RegisterMessage(TcePersDataPort_array_port)


# @@protoc_insertion_point(module_scope)
