# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: tce/tce_estimation_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='tce/tce_estimation_port.proto',
  package='pb.tce.tce_estimation_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1dtce/tce_estimation_port.proto\x12\x1apb.tce.tce_estimation_port\x1a\x17\x65\x63o/signal_header.proto\"\xee\x03\n\x11TceEstimationPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x15\n\x0ctireCircFL_m\x18\xb3\n \x01(\x02\x12\x19\n\x10tireCircStdvFL_m\x18\x9f\x0c \x01(\x02\x12\x15\n\x0ctireCircFR_m\x18\xd1\x1a \x01(\x02\x12\x19\n\x10tireCircStdvFR_m\x18\xfd\x1c \x01(\x02\x12\x15\n\x0ctireCircRL_m\x18\xe5\x08 \x01(\x02\x12\x19\n\x10tireCircStdvRL_m\x18\xc9\x0e \x01(\x02\x12\x15\n\x0ctireCircRR_m\x18\x87\x18 \x01(\x02\x12\x19\n\x10tireCircStdvRR_m\x18\xab\x1e \x01(\x02\x12\x19\n\x10rearTrackWidth_m\x18\xdf\x07 \x01(\x02\x12\x1d\n\x14rearTrackWidthStdv_m\x18\x85\x05 \x01(\x02\x12\x18\n\x0ftireCircFLValid\x18\xaf\x01 \x01(\x08\x12\x18\n\x0ftireCircFRValid\x18\x88\x15 \x01(\x08\x12\x18\n\x0ftireCircRLValid\x18\x92\x0c \x01(\x08\x12\x18\n\x0ftireCircRRValid\x18\xb5\x18 \x01(\x08\x12\x1c\n\x13rearTrackWidthValid\x18\xcd\x0e \x01(\x08\"\\\n\x1cTceEstimationPort_array_port\x12<\n\x04\x64\x61ta\x18\x93\x07 \x03(\x0b\x32-.pb.tce.tce_estimation_port.TceEstimationPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,])




_TCEESTIMATIONPORT = _descriptor.Descriptor(
  name='TceEstimationPort',
  full_name='pb.tce.tce_estimation_port.TceEstimationPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.tce.tce_estimation_port.TceEstimationPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.tce.tce_estimation_port.TceEstimationPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircFL_m', full_name='pb.tce.tce_estimation_port.TceEstimationPort.tireCircFL_m', index=2,
      number=1331, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircStdvFL_m', full_name='pb.tce.tce_estimation_port.TceEstimationPort.tireCircStdvFL_m', index=3,
      number=1567, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircFR_m', full_name='pb.tce.tce_estimation_port.TceEstimationPort.tireCircFR_m', index=4,
      number=3409, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircStdvFR_m', full_name='pb.tce.tce_estimation_port.TceEstimationPort.tireCircStdvFR_m', index=5,
      number=3709, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircRL_m', full_name='pb.tce.tce_estimation_port.TceEstimationPort.tireCircRL_m', index=6,
      number=1125, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircStdvRL_m', full_name='pb.tce.tce_estimation_port.TceEstimationPort.tireCircStdvRL_m', index=7,
      number=1865, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircRR_m', full_name='pb.tce.tce_estimation_port.TceEstimationPort.tireCircRR_m', index=8,
      number=3079, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircStdvRR_m', full_name='pb.tce.tce_estimation_port.TceEstimationPort.tireCircStdvRR_m', index=9,
      number=3883, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rearTrackWidth_m', full_name='pb.tce.tce_estimation_port.TceEstimationPort.rearTrackWidth_m', index=10,
      number=991, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rearTrackWidthStdv_m', full_name='pb.tce.tce_estimation_port.TceEstimationPort.rearTrackWidthStdv_m', index=11,
      number=645, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircFLValid', full_name='pb.tce.tce_estimation_port.TceEstimationPort.tireCircFLValid', index=12,
      number=175, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircFRValid', full_name='pb.tce.tce_estimation_port.TceEstimationPort.tireCircFRValid', index=13,
      number=2696, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircRLValid', full_name='pb.tce.tce_estimation_port.TceEstimationPort.tireCircRLValid', index=14,
      number=1554, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tireCircRRValid', full_name='pb.tce.tce_estimation_port.TceEstimationPort.tireCircRRValid', index=15,
      number=3125, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rearTrackWidthValid', full_name='pb.tce.tce_estimation_port.TceEstimationPort.rearTrackWidthValid', index=16,
      number=1869, type=8, cpp_type=7, label=1,
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
  serialized_start=87,
  serialized_end=581,
)


_TCEESTIMATIONPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='TceEstimationPort_array_port',
  full_name='pb.tce.tce_estimation_port.TceEstimationPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.tce.tce_estimation_port.TceEstimationPort_array_port.data', index=0,
      number=915, type=11, cpp_type=10, label=3,
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
  serialized_start=583,
  serialized_end=675,
)

_TCEESTIMATIONPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_TCEESTIMATIONPORT_ARRAY_PORT.fields_by_name['data'].message_type = _TCEESTIMATIONPORT
DESCRIPTOR.message_types_by_name['TceEstimationPort'] = _TCEESTIMATIONPORT
DESCRIPTOR.message_types_by_name['TceEstimationPort_array_port'] = _TCEESTIMATIONPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TceEstimationPort = _reflection.GeneratedProtocolMessageType('TceEstimationPort', (_message.Message,), {
  'DESCRIPTOR' : _TCEESTIMATIONPORT,
  '__module__' : 'tce.tce_estimation_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.tce.tce_estimation_port.TceEstimationPort)
  })
_sym_db.RegisterMessage(TceEstimationPort)

TceEstimationPort_array_port = _reflection.GeneratedProtocolMessageType('TceEstimationPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _TCEESTIMATIONPORT_ARRAY_PORT,
  '__module__' : 'tce.tce_estimation_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.tce.tce_estimation_port.TceEstimationPort_array_port)
  })
_sym_db.RegisterMessage(TceEstimationPort_array_port)


# @@protoc_insertion_point(module_scope)
