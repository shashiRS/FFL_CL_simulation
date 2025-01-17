# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_drv/us_drv_crminput.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from us_drv import us_drv_crmdata_pb2 as us__drv_dot_us__drv__crmdata__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_drv/us_drv_crminput.proto',
  package='pb.us_drv.us_drv_crminput',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1cus_drv/us_drv_crminput.proto\x12\x19pb.us_drv.us_drv_crminput\x1a\x17\x65\x63o/signal_header.proto\x1a\x1bus_drv/us_drv_crmdata.proto\"\xd6\x01\n\rUsDrvCRMInput\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x10\n\x08sequence\x18^ \x01(\r\x12\x0e\n\x05\x66lags\x18\xaf\t \x01(\r\x12\x13\n\nbusBitmask\x18\xd8\x07 \x01(\r\x12;\n\ncrmCommand\x18\xb5\x08 \x01(\x0b\x32&.pb.us_drv.us_drv_crmdata.UsDrvCRMData\"S\n\x18UsDrvCRMInput_array_port\x12\x37\n\x04\x64\x61ta\x18\xc9\x15 \x03(\x0b\x32(.pb.us_drv.us_drv_crminput.UsDrvCRMInput'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,us__drv_dot_us__drv__crmdata__pb2.DESCRIPTOR,])




_USDRVCRMINPUT = _descriptor.Descriptor(
  name='UsDrvCRMInput',
  full_name='pb.us_drv.us_drv_crminput.UsDrvCRMInput',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.us_drv.us_drv_crminput.UsDrvCRMInput.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.us_drv.us_drv_crminput.UsDrvCRMInput.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sequence', full_name='pb.us_drv.us_drv_crminput.UsDrvCRMInput.sequence', index=2,
      number=94, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='flags', full_name='pb.us_drv.us_drv_crminput.UsDrvCRMInput.flags', index=3,
      number=1199, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='busBitmask', full_name='pb.us_drv.us_drv_crminput.UsDrvCRMInput.busBitmask', index=4,
      number=984, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='crmCommand', full_name='pb.us_drv.us_drv_crminput.UsDrvCRMInput.crmCommand', index=5,
      number=1077, type=11, cpp_type=10, label=1,
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
  serialized_start=114,
  serialized_end=328,
)


_USDRVCRMINPUT_ARRAY_PORT = _descriptor.Descriptor(
  name='UsDrvCRMInput_array_port',
  full_name='pb.us_drv.us_drv_crminput.UsDrvCRMInput_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_drv.us_drv_crminput.UsDrvCRMInput_array_port.data', index=0,
      number=2761, type=11, cpp_type=10, label=3,
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
  serialized_start=330,
  serialized_end=413,
)

_USDRVCRMINPUT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_USDRVCRMINPUT.fields_by_name['crmCommand'].message_type = us__drv_dot_us__drv__crmdata__pb2._USDRVCRMDATA
_USDRVCRMINPUT_ARRAY_PORT.fields_by_name['data'].message_type = _USDRVCRMINPUT
DESCRIPTOR.message_types_by_name['UsDrvCRMInput'] = _USDRVCRMINPUT
DESCRIPTOR.message_types_by_name['UsDrvCRMInput_array_port'] = _USDRVCRMINPUT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsDrvCRMInput = _reflection.GeneratedProtocolMessageType('UsDrvCRMInput', (_message.Message,), {
  'DESCRIPTOR' : _USDRVCRMINPUT,
  '__module__' : 'us_drv.us_drv_crminput_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_crminput.UsDrvCRMInput)
  })
_sym_db.RegisterMessage(UsDrvCRMInput)

UsDrvCRMInput_array_port = _reflection.GeneratedProtocolMessageType('UsDrvCRMInput_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USDRVCRMINPUT_ARRAY_PORT,
  '__module__' : 'us_drv.us_drv_crminput_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_crminput.UsDrvCRMInput_array_port)
  })
_sym_db.RegisterMessage(UsDrvCRMInput_array_port)


# @@protoc_insertion_point(module_scope)
