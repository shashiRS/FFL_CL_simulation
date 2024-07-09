# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_drv/us_drv_debug_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from us_drv import us_drv_debug_port_bus_pb2 as us__drv_dot_us__drv__debug__port__bus__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_drv/us_drv_debug_port.proto',
  package='pb.us_drv.us_drv_debug_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1eus_drv/us_drv_debug_port.proto\x12\x1bpb.us_drv.us_drv_debug_port\x1a\x17\x65\x63o/signal_header.proto\x1a\"us_drv/us_drv_debug_port_bus.proto\"\xf6\x03\n\x0eUsDrvDebugPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x18\n\x0fspiCommandCount\x18\xad\x06 \x01(\r\x12\x19\n\x10spiCrcErrorCount\x18\xf4\x12 \x01(\r\x12\x19\n\x10\x65rrorStatusCount\x18\xc1\r \x01(\r\x12\x1c\n\x13pdcmCountTotalFront\x18\x82\x10 \x01(\r\x12!\n\x18pdcmErrorCountTotalFront\x18\xef\x07 \x01(\r\x12\x1b\n\x12\x63rmCountTotalFront\x18\xa0\x10 \x01(\r\x12 \n\x17\x63rmErrorCountTotalFront\x18\xad\x07 \x01(\r\x12\x1b\n\x12pdcmCountTotalRear\x18\xf7\x0b \x01(\r\x12 \n\x17pdcmErrorCountTotalRear\x18\xe5\x19 \x01(\r\x12\x1a\n\x11\x63rmCountTotalRear\x18\xfb\x02 \x01(\r\x12\x1f\n\x16\x63rmErrorCountTotalRear\x18\xb5\x1c \x01(\r\x12\x45\n\x08\x62usDebug\x18\xbf\x08 \x03(\x0b\x32\x32.pb.us_drv.us_drv_debug_port_bus.UsDrvDebugPortBus\"W\n\x19UsDrvDebugPort_array_port\x12:\n\x04\x64\x61ta\x18\xdf\x0c \x03(\x0b\x32+.pb.us_drv.us_drv_debug_port.UsDrvDebugPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,us__drv_dot_us__drv__debug__port__bus__pb2.DESCRIPTOR,])




_USDRVDEBUGPORT = _descriptor.Descriptor(
  name='UsDrvDebugPort',
  full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='spiCommandCount', full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort.spiCommandCount', index=2,
      number=813, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='spiCrcErrorCount', full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort.spiCrcErrorCount', index=3,
      number=2420, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='errorStatusCount', full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort.errorStatusCount', index=4,
      number=1729, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pdcmCountTotalFront', full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort.pdcmCountTotalFront', index=5,
      number=2050, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pdcmErrorCountTotalFront', full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort.pdcmErrorCountTotalFront', index=6,
      number=1007, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='crmCountTotalFront', full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort.crmCountTotalFront', index=7,
      number=2080, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='crmErrorCountTotalFront', full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort.crmErrorCountTotalFront', index=8,
      number=941, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pdcmCountTotalRear', full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort.pdcmCountTotalRear', index=9,
      number=1527, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pdcmErrorCountTotalRear', full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort.pdcmErrorCountTotalRear', index=10,
      number=3301, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='crmCountTotalRear', full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort.crmCountTotalRear', index=11,
      number=379, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='crmErrorCountTotalRear', full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort.crmErrorCountTotalRear', index=12,
      number=3637, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='busDebug', full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort.busDebug', index=13,
      number=1087, type=11, cpp_type=10, label=3,
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
  serialized_start=125,
  serialized_end=627,
)


_USDRVDEBUGPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='UsDrvDebugPort_array_port',
  full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_drv.us_drv_debug_port.UsDrvDebugPort_array_port.data', index=0,
      number=1631, type=11, cpp_type=10, label=3,
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
  serialized_start=629,
  serialized_end=716,
)

_USDRVDEBUGPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_USDRVDEBUGPORT.fields_by_name['busDebug'].message_type = us__drv_dot_us__drv__debug__port__bus__pb2._USDRVDEBUGPORTBUS
_USDRVDEBUGPORT_ARRAY_PORT.fields_by_name['data'].message_type = _USDRVDEBUGPORT
DESCRIPTOR.message_types_by_name['UsDrvDebugPort'] = _USDRVDEBUGPORT
DESCRIPTOR.message_types_by_name['UsDrvDebugPort_array_port'] = _USDRVDEBUGPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsDrvDebugPort = _reflection.GeneratedProtocolMessageType('UsDrvDebugPort', (_message.Message,), {
  'DESCRIPTOR' : _USDRVDEBUGPORT,
  '__module__' : 'us_drv.us_drv_debug_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_debug_port.UsDrvDebugPort)
  })
_sym_db.RegisterMessage(UsDrvDebugPort)

UsDrvDebugPort_array_port = _reflection.GeneratedProtocolMessageType('UsDrvDebugPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USDRVDEBUGPORT_ARRAY_PORT,
  '__module__' : 'us_drv.us_drv_debug_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_debug_port.UsDrvDebugPort_array_port)
  })
_sym_db.RegisterMessage(UsDrvDebugPort_array_port)


# @@protoc_insertion_point(module_scope)